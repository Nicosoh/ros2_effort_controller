#include <cartesian_impedance_controller/cartesian_impedance_controller.h>

// Franka semantic model component (assumes you added the include in your .h too)
// #include "franka_semantic_components/franka_robot_model.hpp"

namespace cartesian_impedance_controller {



CartesianImpedanceController::CartesianImpedanceController()
    : Base::EffortControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // ---------------- Parameters (existing) ----------------
  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);
  auto_declare<double>("nullspace_stiffness", 0.0);
  auto_declare<bool>("compensate_dJdq", false);
  auto_declare<std::vector<double>>("nullspace_desired_configuration", std::vector<double>());

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);
  auto_declare<double>("max_impedance_force", 70.0);  // TODO

  // Force control gains
  auto_declare<double>("force_control.k_p", 1.0);
  auto_declare<double>("force_control.k_d", 0.0);
  auto_declare<double>("force_control.k_i", 0.0);

  // ---------------- Franka model parameter ----------------
  // Used to select the correct franka model/state interface namespace: e.g. "fr3"
  auto_declare<std::string>("robot_type", "fr3");

  // Read immediately (same pattern as Franka example controller)
  if (!get_node()->get_parameter("robot_type", robot_type_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to get robot_type parameter");
    get_node()->shutdown();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  // Keep Base state interfaces (DO NOT change how you get joint state/command interfaces)
  auto cfg = Base::state_interface_configuration();

  // Append Franka model/state interfaces needed by FrankaRobotModel.
  // IMPORTANT: do not rely on franka_robot_model_ existing here (controller manager can query
  // interfaces before on_configure). We therefore instantiate a temporary model just to list names.
  controller_interface::InterfaceConfiguration franka_cfg;
  franka_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  franka_semantic_components::FrankaRobotModel tmp_model(
      robot_type_ + "/" + k_robot_model_interface_name,
      robot_type_ + "/" + k_robot_state_interface_name);

  for (const auto& name : tmp_model.get_state_interface_names()) {
    cfg.names.push_back(name);
  }
  return cfg;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State& previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // ---------------- Franka model component ----------------
  // Create persistent model component instance (same as Franka example)
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(
          robot_type_ + "/" + k_robot_model_interface_name,
          robot_type_ + "/" + k_robot_state_interface_name));

  // ---------------- Existing configuration ----------------

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link << " is not part of the kinematic chain from "
                                             << Base::m_robot_base_link << " to "
                                             << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Force control gains
  m_k_p = get_node()->get_parameter("force_control.k_p").as_double();
  m_k_d = get_node()->get_parameter("force_control.k_d").as_double();
  m_k_i = get_node()->get_parameter("force_control.k_i").as_double();

  // Set stiffness
  ctrl::Vector6D tmp;
  tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  m_cartesian_stiffness = tmp.asDiagonal();

  // Set damping (critical damping as a baseline)
  tmp[0] = 2 * sqrt(tmp[0]);
  tmp[1] = 2 * sqrt(tmp[1]);
  tmp[2] = 2 * sqrt(tmp[2]);
  tmp[3] = 2 * sqrt(tmp[3]);
  tmp[4] = 2 * sqrt(tmp[4]);
  tmp[5] = 2 * sqrt(tmp[5]);

  m_cartesian_damping = tmp.asDiagonal();

  m_max_impendance_force = get_node()->get_parameter("max_impedance_force").as_double();  // TODO

  // Set nullspace stiffness/damping
  m_null_space_stiffness = get_node()->get_parameter("nullspace_stiffness").as_double();
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

  m_compensate_dJdq = get_node()->get_parameter("compensate_dJdq").as_bool();
  RCLCPP_INFO(get_node()->get_logger(), "Compensate dJdq: %d", m_compensate_dJdq);

  // Identity matrix in joint space
  m_identity = ctrl::MatrixND::Identity(m_joint_number, m_joint_number);

  // ---------------- Subscribers/publishers (kept except Jacobian/Mass topics) ----------------
  m_target_wrench_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/target_wrench"), 10,
          std::bind(&CartesianImpedanceController::targetWrenchCallback, this,
                    std::placeholders::_1));

  m_ft_sensor_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/ft_sensor_wrench"), 10,
          std::bind(&CartesianImpedanceController::ftSensorWrenchCallback, this,
                    std::placeholders::_1));

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 1,
          std::bind(&CartesianImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));

  m_data_publisher = get_node()->create_publisher<debug_msg::msg::Debug>(
      get_node()->get_name() + std::string("/data"), 1);

  m_data_impedance_publisher =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          get_node()->get_name() + std::string("/data_impedance"), 1);

  // ---------------- REMOVED ----------------
  // Jacobian/Mass subscribers from topics were removed by design:
  // - m_jacobian_subscriber
  // - m_mass_matrix_subscriber

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State& previous_state) {
  Base::on_activate(previous_state);

  // Assign Franka model state interfaces
  if (franka_robot_model_) {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  } else {
    RCLCPP_FATAL(get_node()->get_logger(), "FrankaRobotModel not initialised (nullptr) on_activate");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Update joint states
  Base::updateJointStates();

  // Compute forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set target to current
  m_target_frame = m_current_frame;
  m_target_frame_old = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  m_error_old = ctrl::Vector6D::Zero();
  m_error_dot_old = ctrl::Vector6D::Zero();
  m_target_velocity = ctrl::Vector6D::Zero();
  m_last_time_target_frame_received = get_node()->now();

  m_target_wrench = ctrl::Vector6D::Zero();
  m_ft_sensor_wrench = ctrl::Vector6D::Zero();
  m_wrench_error = ctrl::Vector6D::Zero();
  m_wrench_error_dot = ctrl::Vector6D::Zero();
  m_wrench_error_integral = ctrl::Vector6D::Zero();

  m_last_time_target_wrench_received = get_node()->now();

  q_ref_ = Base::m_joint_positions.data;            // stato corrente
  dq_ref_.setZero();
  grad_filt_.setZero();
  grad_counter_ = 0;

  // Optional: initialise buffers with Franka model outputs
  {
    const std::array<double, 49> mass_array = franka_robot_model_->getMassMatrix();
    m_mass_matrix = ctrl::MatrixND(Base::m_joint_number, Base::m_joint_number);
    for (size_t col = 0; col < Base::m_joint_number; ++col) {
      for (size_t row = 0; row < Base::m_joint_number; ++row) {
        m_mass_matrix(row, col) = mass_array[col * Base::m_joint_number + row];
      }
    }

    const std::array<double, 42> jac_array =
        franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    m_jacobian = ctrl::MatrixND(6, Base::m_joint_number);
    for (size_t col = 0; col < Base::m_joint_number; ++col) {
      for (size_t row = 0; row < 6; ++row) {
        m_jacobian(row, col) = jac_array[col * 6 + row];
      }
    }
  }

  if (m_null_space_stiffness > 0.0) {
    std::vector<double> nullspace_config =
        get_node()->get_parameter("nullspace_desired_configuration").as_double_array();
    if (nullspace_config.empty()) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Null space configuration is empty, taking current joint positions");
      for (size_t i = 0; i < Base::m_joint_number; ++i) {
        nullspace_config.push_back(Base::m_joint_positions(i));
      }
    } else if (nullspace_config.size() != Base::m_joint_number) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Null space configuration size does not match joint number: %zu != %zu",
                   nullspace_config.size(), Base::m_joint_number);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    m_q_ns = ctrl::VectorND::Zero(Base::m_joint_number);
    for (size_t i = 0; i < Base::m_joint_number; ++i) {
      m_q_ns(i) = nullspace_config[i];
    }

    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Postural task stiffness: " << m_null_space_stiffness
                                                   << " for configuration: "
                                                   << m_q_ns.transpose());
  }

#if LOGGING
  m_logger = XBot::MatLogger2::MakeLogger("/tmp/cart_impedance_log");
  m_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
#endif

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  // Stop drifting by sending zero joint torques (via your base machinery)
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  Base::writeJointEffortCmds();

  if (franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }

  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
CartesianImpedanceController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  m_period_sec = period.seconds();
  // Update joint states (keep your current method)
  Base::updateJointStates();

  // Compute torque
  ctrl::VectorND tau_tot = computeTorque();

  // Saturate/limit using your base class
  Base::computeJointEffortCmds(tau_tot);

  // Write commands
  Base::writeJointEffortCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianImpedanceController::computeMotionError() {
  // target -> current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Rodrigues vector
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = error_kdl.M.GetRotAngle(rot_axis);
  double distance = error_kdl.p.Normalize();

  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle = std::clamp(angle, -max_angle, max_angle);
  distance = std::clamp(distance, -max_distance, max_distance);

  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  ctrl::Vector6D error;
  error.head<3>() << error_kdl.p.x(), error_kdl.p.y(), error_kdl.p.z();
  error.tail<3>() << rot_axis(0), rot_axis(1), rot_axis(2);
  return error;
}

ctrl::VectorND CartesianImpedanceController::computeTorque() {
  // Joint states in Eigen
  ctrl::VectorND q = Base::m_joint_positions.data;
  ctrl::VectorND q_dot = Base::m_joint_velocities.data;
  
  // Print joint positions
  // RCLCPP_INFO_STREAM_THROTTLE(
  //     get_node()->get_logger(), *get_node()->get_clock(), 5000,
  //     "Joint positions: " << q.transpose() << "\n");
  // FK (used by your motion error + gravity compensation in FT callback)
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);
  Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions, Base::m_jacobian);

  // ---------------- Franka model: Jacobian and Mass matrix ----------------
  // Jacobian of EE expressed in base frame (6x7)
  const std::array<double, 42> jac_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  ctrl::MatrixND jac(6, Base::m_joint_number);
  for (size_t col = 0; col < Base::m_joint_number; ++col) {
    for (size_t row = 0; row < 6; ++row) {
      jac(row, col) = jac_array[col * 6 + row];
    }
  }

  // Mass matrix (7x7)
  const std::array<double, 49> mass_array = franka_robot_model_->getMassMatrix();
  ctrl::MatrixND M(Base::m_joint_number, Base::m_joint_number);
  for (size_t col = 0; col < Base::m_joint_number; ++col) {
    for (size_t row = 0; row < Base::m_joint_number; ++row) {
      M(row, col) = mass_array[col * Base::m_joint_number + row];
    }
  }

  // Optionally store for logging/debug
  m_jacobian = jac;
  m_mass_matrix = M;

  // ---------------- Existing torque computation ----------------
  ctrl::MatrixND jac_tran_pseudo_inverse;
  pseudoInverse(jac.transpose(), &jac_tran_pseudo_inverse);

  ctrl::MatrixND Lambda;
  pseudoInverse(jac * M.inverse() * jac.transpose(), &Lambda);

  // Motion error
  ctrl::Vector6D motion_error = computeMotionError();

  ctrl::VectorND tau_task(Base::m_joint_number), tau_null(Base::m_joint_number),
      tau_ext(Base::m_joint_number), tau(Base::m_joint_number);
  tau.setZero();
  tau_task.setZero();
  tau_null.setZero();
  tau_ext.setZero();

  ctrl::Matrix6D selection_matrix = ctrl::Matrix6D::Identity();
  selection_matrix(2, 2) = 0.0;  // No Z translation impedance
  // jac: 6x7 Jacobian (tool or EE). M: 7x7 mass matrix.
  // q, q_dot are 7x1.

  ctrl::MatrixND J_pinv;  
  pseudoInverse(jac, &J_pinv);

  // ---- Dynamic-consistent nullspace projector in torque space ----
  // N_tau = I - J^T * Lambda * J * M^{-1}
  ctrl::MatrixND N_tau =
      m_identity - jac.transpose() * Lambda * jac * M.inverse();

  // ---- (Optional but useful) velocity-level nullspace projector ----
  // N_v = I - J^+ J
  ctrl::MatrixND N_v =
      m_identity - J_pinv * jac;

  // =====================
  // Joint reference update in nullspace (each cycle)
  // =====================

  // dt from your period:
  const double dt = std::clamp(m_period_sec, 1e-4, 5e-3);

  const int grad_update_every = 10;
  const double delta = 1e-4;
  const double alpha = 0.05;
  const double k_m = 1.4;      // tune
  const double grad_clip = 25.0;
  const double dq0_max = 0.3;  // tune
  const double Kq = 35.0;
  const double Dq = 2 * sqrt(Kq);

  // ---- gradient update ----
  if (++grad_counter_ % grad_update_every == 0) {
    Eigen::Matrix<double,7,1> grad =
        computeGradForceCapabilityToolZ_FD(Base::m_joint_positions, delta);

    for (int i = 0; i < 7; ++i) {
      grad(i) = std::clamp(grad(i), -grad_clip, grad_clip);
    }
    grad_filt_ = (1.0 - alpha) * grad_filt_ + alpha * grad;
  }

  // dq0 = k_m * grad
  ctrl::VectorND dq0 = ctrl::VectorND::Zero(Base::m_joint_number);
  for (int i = 0; i < 7; ++i) dq0(i) = k_m * grad_filt_(i);

  // clamp dq0
  for (int i = 0; i < 7; ++i) dq0(i) = std::clamp(dq0(i), -dq0_max, dq0_max);

  // project + integrate
  dq_ref_ = N_v * dq0;
  q_ref_ += dq_ref_ * dt;

  // PD torque in joint space
  ctrl::VectorND tau_jref =
      Kq * (q_ref_ - q) + Dq * (dq_ref_ - q_dot);
  // RCLCPP_INFO_STREAM_THROTTLE(
  //     get_node()->get_logger(), *get_node()->get_clock(), 5000,
  //     "q_ref: " << q_ref_.transpose() << "\n"
  //                << "dq_ref: " << dq_ref_.transpose() << "\n" <<
  //     "q" << q.transpose() << "\n"
  //                << "q_dot: " << q_dot.transpose() << "\n");
  // project torque into dynamic nullspace
  ctrl::VectorND tau_null_ref = N_tau * tau_jref;
  // Rotate selection matrix in the base frame
  selection_matrix = Base::displayInBaseLink(
      selection_matrix, Base::m_end_effector_link);
  // Stiffness/damping in base link
  ctrl::Matrix6D base_link_stiffness =
      Base::displayInBaseLink(m_cartesian_stiffness, Base::m_end_effector_link);

  ctrl::Matrix6D K_d = selection_matrix * base_link_stiffness;

  // Your damping shaping (kept)
  ctrl::Matrix6D D_d = selection_matrix * compute_correct_damping(Lambda, base_link_stiffness, std::sqrt(2.0) / 2.0);

  // Task-space impedance torque
  ctrl::VectorND stiffness_torque = jac.transpose() * (K_d * motion_error);
  ctrl::VectorND damping_torque = jac.transpose() * (D_d * (-jac * q_dot));
  tau_task = stiffness_torque + damping_torque;

  // Gravity/Coriolis compensation via your existing solvers (kept)
  KDL::JntArray tau_coriolis(Base::m_joint_number), tau_gravity(Base::m_joint_number);

  if (m_compensate_gravity) {
    Base::m_dyn_solver->JntToGravity(Base::m_joint_positions, tau_gravity);
    tau = tau + tau_gravity.data;
  }
  if (m_compensate_coriolis) {
    Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions, Base::m_joint_velocities, tau_coriolis);
    tau = tau + tau_coriolis.data;
  }

  // Jacobian derivative * qdot compensation (kept exactly as you had, but fixed the shadowing bug)
  if (m_compensate_dJdq) {
    KDL::JntArrayVel q_in(Base::m_joint_positions, Base::m_joint_velocities);
    KDL::Twist jac_dot_q_dot;
    Base::m_jnt_to_jac_dot_solver->JntToJacDot(q_in, jac_dot_q_dot);

    Eigen::VectorXd jac_dot_q_dot_eigen(6);
    jac_dot_q_dot_eigen.head(3) << jac_dot_q_dot.vel.x(), jac_dot_q_dot.vel.y(), jac_dot_q_dot.vel.z();
    jac_dot_q_dot_eigen.tail(3) << jac_dot_q_dot.rot.x(), jac_dot_q_dot.rot.y(), jac_dot_q_dot.rot.z();

    Eigen::VectorXd j_tran_lambda_jdot_qdot = jac.transpose() * Lambda * jac_dot_q_dot_eigen;
    tau = tau + j_tran_lambda_jdot_qdot;
  }

  // Nullspace torque (kept)
  if (m_null_space_stiffness > 1e-6) {
    tau_null =
        (m_identity - jac.transpose() * Lambda * jac * M.inverse()) *
        (m_null_space_stiffness * (-q + m_q_ns) - m_null_space_damping * q_dot);

    RCLCPP_INFO_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Nullspace residual torque: "
            << Lambda * jac * M.inverse() * tau_null << "\n");
  } else {
    tau_null = ctrl::VectorND::Zero(Base::m_joint_number);
  }

#if DEBUG
  debug_msg::msg::Debug debug_msg;
  Eigen::VectorXd Force = K_d * motion_error - D_d * (jac * q_dot);
  for (int i = 0; i < 7; i++) {
    debug_msg.stiffness_torque[i] = stiffness_torque(i);
    debug_msg.damping_torque[i] = damping_torque(i);
    debug_msg.coriolis_torque[i] = tau_coriolis(i);
    debug_msg.nullspace_torque[i] = tau_null(i);
    if (i < 6) {
      debug_msg.impedance_force[i] = Force(i);
    }
  }
  m_data_publisher->publish(debug_msg);
#endif

#if LOGGING
  Eigen::VectorXd Force = K_d * motion_error - D_d * (jac * q_dot);
  auto compute_condition_number = [](const Eigen::MatrixXd& matrix) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
    Eigen::VectorXd singular_values = svd.singularValues();
    return singular_values(0) / singular_values(singular_values.size() - 1);
  };
  m_logger->add("condition_number mass", compute_condition_number(M));
  m_logger->add("condition_number jac", compute_condition_number(jac));
  for (int i = 0; i < 7; i++) {
    m_logger->add("stiffness_" + std::to_string(i), stiffness_torque(i));
    m_logger->add("damping_" + std::to_string(i), damping_torque(i));
    m_logger->add("coriolis_" + std::to_string(i), tau_coriolis(i));
    m_logger->add("nullspace_" + std::to_string(i), tau_null(i));
    if (i < 6) {
      m_logger->add("impedance_force_" + std::to_string(i), Force(i));
    }
  }
#endif

  // Condition number warning (kept)
  auto compute_condition_number = [](const Eigen::MatrixXd& matrix) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
    Eigen::VectorXd singular_values = svd.singularValues();
    return singular_values(0) / singular_values(singular_values.size() - 1);
  };
  if (compute_condition_number(jac) > 20.0) {
    RCLCPP_WARN_STREAM_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "High condition number of the Jacobian: " << compute_condition_number(jac));
  }
  ctrl::Vector6D selected_force = ctrl::Vector6D::Zero();
  // External wrench tracking -> torque (kept)
  if (m_target_wrench.norm() > 0.00001) {
    ctrl::Matrix6D direction_selector = ctrl::Matrix6D::Zero();
    direction_selector(2, 2) = 1.0;

    selected_force =
        direction_selector *
        (m_target_wrench + m_k_p * m_wrench_error + m_k_d * m_wrench_error_dot +
         m_k_i * m_wrench_error_integral);

    auto F_b = Base::displayInBaseLink(selected_force, m_end_effector_link);
    tau_ext = jac.transpose() * F_b;
  } else {
    tau_ext = ctrl::VectorND::Zero(Base::m_joint_number);
  }

  const double tau_ns_max = 5.0; // Nm (start small)
  for (int i=0;i<7;i++) tau_null_ref(i) = std::clamp(tau_null_ref(i), -tau_ns_max, tau_ns_max);

  // Sum torques
  tau += tau_task + tau_null_ref + tau_ext;

  // Publish error (kept)
  // ================= SANITY CHECK PRINTS =================
  // ----- 1) Velocity nullspace leakage: J * dq_ref -----
  Eigen::VectorXd xdot_leak = jac * dq_ref_;
  double leak_vel = xdot_leak.norm();

  // Compare before projection
  Eigen::VectorXd xdot_before = jac * dq0;
  double leak_before = xdot_before.norm();

  // ----- 2) Torque nullspace leakage: J * M^{-1} * tau_null -----
  Eigen::VectorXd xdd_leak = jac * M.inverse() * tau_null_ref;
  double leak_tau = xdd_leak.norm();

  // ----- 3) Projector idempotence: N_tau^2 ≈ N_tau -----
  // Eigen::MatrixXd N_tau =
  //     m_identity - jac.transpose() * Lambda * jac * M.inverse();

  double proj_err = (N_tau * N_tau - N_tau).norm();

  // ----- 4) Objective directional derivative -----
  double wdot_pred = grad_filt_.dot(dq_ref_);

  // ----- 5) Pose error norm -----
  double pose_err = motion_error.norm();
  
  // double w_now = computeGradForceCapabilityToolZ_FD(Base::m_joint_positions);
  // ----- PRINT -----
  // RCLCPP_INFO_STREAM(
  //     get_node()->get_logger(),
  //     "\n=== NULLSPACE SANITY ===\n"
  //     << "w(q):               " << w_now << "\n"
  //     << "||J*dq0||:          " << leak_before << "\n"
  //     << "||J*dq_ref||:       " << leak_vel << "\n"
  //     << "||J*M^-1*tau_null||:" << leak_tau << "\n"
  //     << "proj_err:           " << proj_err << "\n"
  //     << "grad·dq_ref:        " << wdot_pred << "\n"
  //     << "||pose_error||:     " << pose_err << "\n");


  static std_msgs::msg::Float64MultiArray impedance_message;
  impedance_message.data = {
    // w_now,
    // leak_before,
    // leak_vel,
    // leak_tau,
    // proj_err,
    // wdot_pred,
    // pose_err, 
    tau_task(0),
    tau_task(1),
    tau_task(2),
    tau_task(3),
    tau_task(4),
    tau_task(5),
    tau_task(6),
    tau_ext(0),
    tau_ext(1),
    tau_ext(2),
    tau_ext(3),
    tau_ext(4),
    tau_ext(5),
    tau_ext(6),
    tau_null_ref(0),
    tau_null_ref(1),
    tau_null_ref(2),
    tau_null_ref(3),
    tau_null_ref(4),
    tau_null_ref(5),
    tau_null_ref(6),
    selected_force(2)
  };
  m_data_impedance_publisher->publish(impedance_message);

  return tau;
}

void CartesianImpedanceController::targetWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  // Parse target wrench
  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;

  // If not in EE frame, transform into EE frame interpretation (kept as you had)
  if (wrench->header.frame_id != Base::m_end_effector_link) {
    m_target_wrench = Base::displayInTipLink(m_target_wrench, wrench->header.frame_id);
  }

  double last_update =
      (get_node()->now() - m_last_time_target_wrench_received).seconds();

  if (last_update > 0.0012) {
    usable_force = false;
    m_wrench_error = m_target_wrench + m_ft_sensor_wrench;
  } else {
    usable_force = true;

    auto wrench_error_old = m_wrench_error;
    auto old_wrench_error_dot = m_wrench_error_dot;

    m_wrench_error = 0.88 * m_wrench_error + 0.12 * (m_target_wrench + m_ft_sensor_wrench);
    m_wrench_error_dot =
        0.95 * m_wrench_error_dot + 0.05 * (m_wrench_error - wrench_error_old) / 0.001;
    m_wrench_error_integral = m_wrench_error_integral + m_wrench_error * 0.001;

    // anti wind-up
    double integral_limit = 6.0;
    for (int i = 0; i < 6; i++) {
      if (m_wrench_error_integral(i) > integral_limit) {
        m_wrench_error_integral(i) = integral_limit;
      } else if (m_wrench_error_integral(i) < -integral_limit) {
        m_wrench_error_integral(i) = -integral_limit;
      }
    }

    // limit jerk on derivative
    double max_wrench_error_dot = 1.0;  // [N/s]
    for (int i = 0; i < 6; i++) {
      if (m_wrench_error_dot(i) - old_wrench_error_dot(i) > max_wrench_error_dot) {
        m_wrench_error_dot(i) = old_wrench_error_dot(i) + max_wrench_error_dot;
      } else if (m_wrench_error_dot(i) - old_wrench_error_dot(i) < -max_wrench_error_dot) {
        m_wrench_error_dot(i) = old_wrench_error_dot(i) - max_wrench_error_dot;
      }
    }
  }

  m_last_time_target_wrench_received = get_node()->now();
}

void CartesianImpedanceController::ftSensorWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z)) {
    auto& clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  m_ft_sensor_wrench[0] = wrench->wrench.force.x;
  m_ft_sensor_wrench[1] = wrench->wrench.force.y;
  m_ft_sensor_wrench[2] = wrench->wrench.force.z;
  m_ft_sensor_wrench[3] = wrench->wrench.torque.x;
  m_ft_sensor_wrench[4] = wrench->wrench.torque.y;
  m_ft_sensor_wrench[5] = wrench->wrench.torque.z;

  // ---------------- Gravity compensation (kept) ----------------
  double m_mass = 0.135617;  // [kg]
  if (m_mass > 0.0) {
    KDL::Vector gravity_base(0.0, 0.0, -9.8067);  // [m/s^2]
    KDL::Vector F_offset(0.0, 0.0, m_mass * 9.8067);  // [N]

    KDL::Rotation R_ee = m_current_frame.M;
    KDL::Vector gravity_sensor = R_ee.Inverse() * gravity_base;
    KDL::Vector F_gravity = m_mass * gravity_sensor;

    m_ft_sensor_wrench[0] -= F_gravity.x();
    m_ft_sensor_wrench[1] -= F_gravity.y();
    m_ft_sensor_wrench[2] -= F_gravity.z() - F_offset.z();
  }
}

void CartesianImpedanceController::targetFrameCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr target)
{
  if (target->header.frame_id != Base::m_robot_base_link) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), clock, 3000,
      "Got target pose in wrong reference frame. Expected: %s but got %s",
      Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  // --- NORMALISE QUATERNION ---
  Eigen::Quaterniond q(
    target->pose.orientation.w,
    target->pose.orientation.x,
    target->pose.orientation.y,
    target->pose.orientation.z
  );
  q.normalize();

  m_target_frame = KDL::Frame(
    KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()),
    KDL::Vector(
      target->pose.position.x,
      target->pose.position.y,
      target->pose.position.z)
  );

  m_target_velocity.setZero();
  m_last_time_target_frame_received = get_node()->now();
}

double CartesianImpedanceController::computeForceCapabilityToolZ_fromKDL(double eps = 1e-8)
{
  // Jv from KDL Jacobian (6x7 -> top 3 rows)
  Eigen::Matrix<double,3,7> Jv;
  for (int c = 0; c < 7; ++c) {
    Jv(0,c) = Base::m_jacobian(0,c);
    Jv(1,c) = Base::m_jacobian(1,c);
    Jv(2,c) = Base::m_jacobian(2,c);
  }

  Eigen::Matrix3d A = Jv * Jv.transpose();
  A += eps * Eigen::Matrix3d::Identity();

  // tool z axis in base: d_base = R_base_tool * e_z
  KDL::Vector ez(0.0, 0.0, 1.0);
  KDL::Vector d_kdl = m_current_frame.M * ez;

  Eigen::Vector3d d_base(d_kdl.x(), d_kdl.y(), d_kdl.z());

  double denom = (d_base.transpose() * A.inverse() * d_base)(0,0);
  denom = std::max(denom, 1e-12);
  return 1.0 / std::sqrt(denom);
}

Eigen::Matrix<double,7,1> CartesianImpedanceController::computeGradForceCapabilityToolZ_FD(
    const KDL::JntArray& q_current,
    double delta = 1e-4)
{
  Eigen::Matrix<double,7,1> grad;
  grad.setZero();

  // We'll reuse local temp containers
  KDL::JntArray q_p = q_current;
  KDL::JntArray q_m = q_current;

  KDL::Frame frame_tmp;
  KDL::Jacobian jac_tmp(7);

  auto eval_w_at = [&](const KDL::JntArray& q_test) -> double {
    // FK
    Base::m_fk_solver->JntToCart(q_test, frame_tmp);

    // Jacobian
    Base::m_jnt_to_jac_solver->JntToJac(q_test, jac_tmp);

    // Convert jac_tmp into Base::m_jacobian-like access by temporary assignment
    // (or compute directly from jac_tmp)
    // Here compute directly:

    Eigen::Matrix<double,3,7> Jv;
    for (int c = 0; c < 7; ++c) {
      Jv(0,c) = jac_tmp(0,c);
      Jv(1,c) = jac_tmp(1,c);
      Jv(2,c) = jac_tmp(2,c);
    }

    Eigen::Matrix3d A = Jv * Jv.transpose();
    const double eps = 1e-8;
    A += eps * Eigen::Matrix3d::Identity();

    KDL::Vector ez(0.0, 0.0, 1.0);
    KDL::Vector d_kdl = frame_tmp.M * ez;
    Eigen::Vector3d d_base(d_kdl.x(), d_kdl.y(), d_kdl.z());

    double denom = (d_base.transpose() * A.inverse() * d_base)(0,0);
    denom = std::max(denom, 1e-12);
    return 1.0 / std::sqrt(denom);
  };

  for (int i = 0; i < 7; ++i) {
    q_p = q_current;
    q_m = q_current;

    q_p(i) += delta;
    q_m(i) -= delta;

    const double wp = eval_w_at(q_p);
    const double wm = eval_w_at(q_m);

    grad(i) = (wp - wm) / (2.0 * delta);
  }

  return grad;
}

}  // namespace cartesian_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
