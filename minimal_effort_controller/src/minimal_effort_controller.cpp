#include <minimal_effort_controller/minimal_effort_controller.h>

namespace minimal_effort_controller {

MinimalEffortController::MinimalEffortController()
    : Base::EffortControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MinimalEffortController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);
  auto_declare<double>("nullspace_stiffness", 0.0);
  auto_declare<bool>("use_feedforward_torque", false);

  constexpr double default_joint_stiff = 100.0;

  for (size_t i = 1; i <= 7; i++) {
    auto_declare<double>("stiffness.joint" + std::to_string(i),
                         default_joint_stiff);
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MinimalEffortController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link =
      get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link
                            << " is not part of the kinematic chain from "
                            << Base::m_robot_base_link << " to "
                            << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
  }

  // Set stiffness
  m_joint_stiffness = ctrl::VectorND::Zero(Base::m_joint_number);
  for (size_t i = 1; i <= Base::m_joint_number; i++) {
    m_joint_stiffness(i - 1) =
        get_node()
            ->get_parameter("stiffness.joint" + std::to_string(i))
            .as_double();
  }

  // Set damping
  m_joint_damping = ctrl::VectorND::Zero(Base::m_joint_number);

  m_joint_damping = 2 * m_joint_stiffness.cwiseSqrt();

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Joint stiffness: " << m_joint_stiffness.transpose());
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Joint damping: " << m_joint_damping.transpose());
  // Set nullspace stiffness
  m_null_space_stiffness =
      get_node()->get_parameter("nullspace_stiffness").as_double();
  RCLCPP_INFO(get_node()->get_logger(), "Postural task stiffness: %f",
              m_null_space_stiffness);
              
  m_use_feedforward_torque =
      get_node()->get_parameter("use_feedforward_torque").as_bool();
  RCLCPP_INFO(get_node()->get_logger(), "Use feedforward torque: %s",
              m_use_feedforward_torque ? "true" : "false");

  // Set nullspace damping
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

  // Set the identity matrix with dimension of the joint space
  m_identity = ctrl::MatrixND::Identity(m_joint_number, m_joint_number);

  // m_target_wrench_subscriber =
  //     get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //         get_node()->get_name() + std::string("/target_wrench"), 10,
  //         std::bind(&MinimalEffortController::targetWrenchCallback, this,
  //                   std::placeholders::_1));

  // m_ft_sensor_wrench_subscriber =
  //   get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
  //     get_node()->get_name() + std::string("/ft_sensor_wrench"),
  //     10,
  //     std::bind(&MinimalEffortController::ftSensorWrenchCallback, this,
  //     std::placeholders::_1));

  m_target_joint_subscriber =
      get_node()->create_subscription<sensor_msgs::msg::JointState>(
        get_node()->get_name() + std::string("/target_joint"), 10,
        std::bind(&MinimalEffortController::targetJointCallback, this,
        std::placeholders::_1));

  m_data_publisher = get_node()->create_publisher<debug_msg::msg::Debug>(
      get_node()->get_name() + std::string("/data"), 1);

  RCLCPP_INFO(get_node()->get_logger(), "Finished MinimalEffortController on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MinimalEffortController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set the target frame to the current frame
  m_target_frame = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished MinimalEffortController on_activate");

  m_q_desired = Base::m_joint_positions.data;
  m_q_starting_pose = Base::m_joint_positions.data;
  m_tau_ff = ctrl::VectorND::Zero(Base::m_joint_number);
  m_q_dot_desired = ctrl::VectorND::Zero(Base::m_joint_number);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MinimalEffortController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  Base::writeJointEffortCmds();
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished MinimalEffortController on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type MinimalEffortController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  // Compute the torque to applay at the joints
  ctrl::VectorND tau_tot = computeTorque();

  // Saturation of the torque
  Base::computeJointEffortCmds(tau_tot);

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D MinimalEffortController::computeMotionError() {
  // Compute the cartesian error between the current and the target frame

  // Transformation from target -> current corresponds to error = target -
  // current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = error_kdl.M.GetRotAngle(rot_axis);  // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle = std::clamp(angle, -max_angle, max_angle);
  distance = std::clamp(distance, -max_distance, max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error.head<3>() << error_kdl.p.x(), error_kdl.p.y(), error_kdl.p.z();
  error.tail<3>() << rot_axis(0), rot_axis(1), rot_axis(2);

  return error;
}

ctrl::VectorND MinimalEffortController::computeTorque() {
  // Compute the inverse kinematics
  // Base::computeIKSolution(m_target_frame, m_q_desired);

  // Compute the jacobian
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions,
                                      Base::m_jacobian);

  // Compute the pseudo-inverse of the jacobian
  ctrl::MatrixND jac = Base::m_jacobian.data;

  // Redefine joints velocities in Eigen format
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  ctrl::VectorND tau_task(Base::m_joint_number);

  // Compute the task joint torques
  Eigen::VectorXd stiffness_torque =
      m_joint_stiffness.cwiseProduct((m_q_desired - q));
  Eigen::VectorXd damping_torque = -m_joint_damping.cwiseProduct(q_dot-m_q_dot_desired);
  tau_task = stiffness_torque + damping_torque;

  ctrl::VectorND tau = tau_task;
  if (m_use_feedforward_torque) {
    tau = tau + m_tau_ff;
  }

  // KDL::JntArray tau_coriolis(Base::m_joint_number),
  //     tau_gravity(Base::m_joint_number);

  // if (m_compensate_gravity) {
  //   Base::m_dyn_solver->JntToGravity(Base::m_joint_positions, tau_gravity);
  //   tau = tau + tau_gravity.data;
  // }
  // if (m_compensate_coriolis) {
  //   Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions,
  //                                     Base::m_joint_velocities, tau_coriolis);
  //   tau = tau + tau_coriolis.data;
  // }
  // TODO add nullspace projector
#if DEBUG
  for (int i = 0; i < 7; i++) {
    debug_msg.stiffness_torque[i] = stiffness_torque(i);
    debug_msg.damping_torque[i] = damping_torque(i);
    debug_msg.coriolis_torque[i] = tau_coriolis(i);
    // debug_msg.nullspace_torque[i] = tau_null(i);
  }
  m_data_publisher->publish(debug_msg);
#endif

  return tau;
}

// void MinimalEffortController::targetWrenchCallback(
//     const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
//   // Parse the target wrench
//   m_target_wrench[0] = wrench->wrench.force.x;
//   m_target_wrench[1] = wrench->wrench.force.y;
//   m_target_wrench[2] = wrench->wrench.force.z;
//   m_target_wrench[3] = wrench->wrench.torque.x;
//   m_target_wrench[4] = wrench->wrench.torque.y;
//   m_target_wrench[5] = wrench->wrench.torque.z;

//   // Check if the wrench is given in the base frame
//   if (wrench->header.frame_id != Base::m_robot_base_link) {
//     // Transform the wrench to the base frame
//     m_target_wrench =
//         Base::displayInBaseLink(m_target_wrench, wrench->header.frame_id);
//   }
// }

void MinimalEffortController::targetJointCallback(
    const sensor_msgs::msg::JointState::SharedPtr target) {
  if (target->position.size() != static_cast<size_t>(Base::m_joint_number)) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "Received joint target of wrong size (%zu vs %zu)",
        target->position.size(), Base::m_joint_number);
    return;
  }
  for (size_t i = 0; i < target->position.size(); ++i) {
    m_q_desired(i) = target->position[i];
  }

  // Feedforward torque is optional -- if the publisher doesn't fill
  // `effort`, treat it as zero rather than rejecting the whole message.
  if (target->effort.empty()) {
    m_tau_ff.setZero();
  } else if (target->effort.size() != static_cast<size_t>(Base::m_joint_number)) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "Received joint target with wrong effort size (%zu vs %zu), ignoring effort",
        target->effort.size(), Base::m_joint_number);
    m_tau_ff.setZero();
  } else {
    for (size_t i = 0; i < target->effort.size(); ++i) {
      m_tau_ff(i) = target->effort[i];
    }
  }

  // Desired velocity is optional too -- default to zero if not provided,
  // so damping falls back to plain -damping*q_dot when no velocity target
  // is given (equivalent to the original behavior).
  if (target->velocity.empty()) {
    m_q_dot_desired.setZero();
  } else if (target->velocity.size() != static_cast<size_t>(Base::m_joint_number)) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "Received joint target with wrong velocity size (%zu vs %zu), ignoring velocity",
        target->velocity.size(), Base::m_joint_number);
    m_q_dot_desired.setZero();
  } else {
    for (size_t i = 0; i < target->velocity.size(); ++i) {
      m_q_dot_desired(i) = target->velocity[i];
    }
  }
}
}  // namespace minimal_effort_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(minimal_effort_controller::MinimalEffortController,
                       controller_interface::ControllerInterface)
