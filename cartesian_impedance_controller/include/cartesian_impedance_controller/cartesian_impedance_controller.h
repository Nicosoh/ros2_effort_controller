#ifndef EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Franka model semantic component
#include "franka_semantic_components/franka_robot_model.hpp"
#include <franka/robot.h>  // for franka::Frame

#define DEBUG 0
#if LOGGING
#include <matlogger2/matlogger2.h>
#endif

namespace cartesian_impedance_controller {

/**
 * @brief Cartesian impedance controller with optional wrench regulation.
 *
 * NOTE: Jacobian and Mass matrix are obtained from FrankaRobotModel (no topics).
 * Joint command/state interfaces are still handled by EffortControllerBase unchanged.
 */
class CartesianImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
 public:
  CartesianImpedanceController();

  using Base = effort_controller_base::EffortControllerBase;
  using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

  LifecycleNodeInterface::CallbackReturn on_init() override;

  LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // IMPORTANT: we keep Base joint interfaces and only append Franka model/state interfaces.
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  ctrl::VectorND computeTorque();

  // --- existing public fields you already exposed (kept) ---
  ctrl::Matrix6D m_cartesian_stiffness;
  ctrl::Matrix6D m_cartesian_damping;
  double m_null_space_stiffness{0.0};
  double m_null_space_damping{0.0};
  double m_max_impendance_force{0.0};

  ctrl::Vector6D m_target_wrench;
  ctrl::MatrixND m_jacobian;
  ctrl::MatrixND m_mass_matrix;
  double m_period_sec{0.001};

  rclcpp::Time m_last_time_target_wrench_received;
  bool usable_force{false};

 private:
  // ---- Franka model interface (NEW) ----
  std::string robot_type_{"fr3"};

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  // ---- internal helpers/callbacks ----
  ctrl::Vector6D compensateGravity();

  void targetWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

  void ftSensorWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

  void targetFrameCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr target);

  Eigen::Matrix<double,7,1> computeGradForceCapabilityToolZ_FD(
    const KDL::JntArray& q_current,
    double delta);

  double computeForceCapabilityToolZ_fromKDL(double eps);

  ctrl::Vector6D computeMotionError();

  // ---- ROS interfaces ----
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_target_wrench_subscriber;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_ft_sensor_subscriber;

  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_data_impedance_publisher;

#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
#endif

  // ---- controller state ----
  KDL::Frame m_target_frame;
  KDL::Frame m_target_frame_old;
  KDL::Frame m_current_frame;

  ctrl::Vector6D m_ft_sensor_wrench;
  ctrl::Vector6D m_wrench_error;
  ctrl::Vector6D m_wrench_error_dot;
  ctrl::Vector6D m_wrench_error_integral;

  ctrl::Vector6D m_error_old;
  ctrl::Vector6D m_error_dot_old;
  ctrl::Vector6D m_error_dot_dot_old;

  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;
  Eigen::Matrix<double,7,1> q_ref_, dq_ref_, grad_filt_;
  int grad_counter_ = 0;


  // Force control gains
  double m_k_p{0.0};
  double m_k_d{0.0};
  double m_k_i{0.0};

  KDL::JntArray m_null_space;

  ctrl::MatrixND m_identity;
  ctrl::VectorND m_q_ns;  // nullspace desired configuration

  ctrl::Vector6D m_target_velocity;

  double m_vel_old{0.0};
  double current_acc_j0{0.0};

  bool m_compensate_dJdq{false};

  /**
   * Allow users to choose whether to specify their target wrenches in the
   * end-effector frame (= true) or the base frame (= false).
   */
  bool m_hand_frame_control{true};

  rclcpp::Time m_last_time_target_frame_received;
};

}  // namespace cartesian_impedance_controller

#endif  // EFFORT_IMPEDANCE_CONTROLLER_H_INCLUDED