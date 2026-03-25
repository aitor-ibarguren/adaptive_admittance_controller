// Copyright (c) 2026 Aitor Ibarguren
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ADMITTANCE_CONTROLLER__ADAPTIVE_ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADAPTIVE_ADMITTANCE_CONTROLLER_HPP_

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Msgs
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// ROS2 Control
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "semantic_components/force_torque_sensor.hpp"

// Eigen
#include "Eigen/Geometry"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_eigen_kdl/tf2_eigen_kdl.hpp"

// Real Time Tools
#include "realtime_tools/realtime_publisher.hpp"

// Parameters
#include "adaptive_admittance_controller/adaptive_admittance_controller_params.hpp"

// Kinematics
#include "urdf/model.h"
// KDL
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"

// Admittance control law
#include "adaptive_admittance_controller/admittance_control_law_impl.hpp"

// Msgs
#include "adaptive_admittance_controller_msgs/msg/admittance_params.hpp"
#include "adaptive_admittance_controller_msgs/msg/feedback.hpp"
#include "adaptive_admittance_controller_msgs/msg/tool_params.hpp"

template <typename T>
using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

namespace admittance_controller
{

class AdaptiveAdmittanceController : public controller_interface::ChainableControllerInterface
{
public:
  AdaptiveAdmittanceController();

  // Command interface
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // State interface
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Update function
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  bool on_set_chained_mode(bool chained_mode) override;

  // Lifecycle
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Robot vars
  size_t dof_;
  size_t num_cmd_joints_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_joint_limits_, upper_joint_limits_;
  Eigen::VectorXd vel_joint_limits_;

  std::string world_link_, base_link_, tip_link_, ft_sensor_frame_;

  // Robot state
  Eigen::VectorXd joint_positions_, joint_velocities_;
  Eigen::VectorXd reference_joint_positions_, reference_joint_velocities_;

  Eigen::Isometry3d base_link_H_tip_, base_link_H_tip_ref_;

  // Robot commands
  Eigen::VectorXd joint_position_commands_, joint_velocity_commands_;
  // Prev robot commands (open loop)
  Eigen::VectorXd joint_position_commands_prev_, joint_velocity_commands_prev_;

  bool has_position_command_interface_ = false;

  // internal reference values
  std::vector<std::reference_wrapper<double>> position_reference_;
  std::vector<std::reference_wrapper<double>> velocity_reference_;

  // Parameter handlers
  adaptive_admittance_controller::Params params_;
  std::shared_ptr<adaptive_admittance_controller::ParamListener> param_listener_;

  // force torque sensor
  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  Eigen::VectorXd wrench_, clean_wrench_, wrench_bias_;

  Eigen::Isometry3d tip_H_ft_sensor_;

  // Low-pass filter
  double filter_alpha_;
  Eigen::VectorXd filtered_wrench_, prev_filtered_wrench_;

  // Tool
  double tool_mass_;
  Eigen::Vector3d tool_cog_;

  Eigen::Isometry3d flange_H_compliance_frame_;

  // Admittance params
  Eigen::Vector3d gravity_;

  AdmittanceParams admittance_params_;

  std::shared_ptr<admittance_controller::AdmittanceControlLaw> admittance_control_law_;

  // Control vars
  bool open_loop_;

  // Feedback vars
  bool feedback_active_;
  adaptive_admittance_controller_msgs::msg::Feedback feedback_msg_;

  // KDL
  KDL::JntArray q_;
  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // Publishers
  rclcpp::Publisher<adaptive_admittance_controller_msgs::msg::Feedback>::SharedPtr feedback_pub_;
  realtime_tools::RealtimePublisher<adaptive_admittance_controller_msgs::msg::Feedback>::SharedPtr
    feedback_pub_rt_;
  // Subscribers
  rclcpp::Subscription<adaptive_admittance_controller_msgs::msg::AdmittanceParams>::SharedPtr
    admittance_params_subs_;
  rclcpp::Subscription<adaptive_admittance_controller_msgs::msg::ToolParams>::SharedPtr
    tool_params_subs_;

  // Callbacks
  void admittance_params_callback(
    const adaptive_admittance_controller_msgs::msg::AdmittanceParams::SharedPtr msg);
  void tool_params_callback(
    const adaptive_admittance_controller_msgs::msg::ToolParams::SharedPtr msg);

  // Helper functions
  bool get_joint_limits(const std::vector<std::string> & joint_names);
  bool get_kinematics(const std::string & base_link, const std::string & tip_link);
  bool getTransform(
    const KDL::Tree & tree, const std::string & from, const std::string & to,
    Eigen::Isometry3d & result);
  void read_joint_state(Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities);
  void read_reference_joint_state(
    Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities);
  void read_wrench(Eigen::VectorXd & wrench);
  Eigen::Isometry3d get_tip_pose(const Eigen::VectorXd & joint_positions);

  void publish_feedback(
    const Eigen::VectorXd & received_wrench, const Eigen::VectorXd & filtered_wrench,
    const Eigen::VectorXd & wrench_in_compliance_frame,
    const Eigen::VectorXd & twist_in_compliance_frame);

  // Wrench helper functions
  Eigen::VectorXd clear_tool_wrench(
    const Eigen::VectorXd & measured_wrench, double tool_mass, const Eigen::Vector3d & tool_cog,
    const Eigen::Isometry3d & tip_pose, const Eigen::Isometry3d & tip_H_ft_sensor);
  Eigen::VectorXd change_wrench_frame(
    const Eigen::VectorXd & wrench, const Eigen::Isometry3d & new_frame);

  // Twist helper functions
  Eigen::VectorXd change_twist_reference(
    const Eigen::VectorXd & twist, const Eigen::Isometry3d & twist_reference);
  Eigen::VectorXd move_twist(const Eigen::VectorXd & twist, const Eigen::Vector3d & q);

  // Kinematics
  Eigen::VectorXd calculate_next_joint_positions(
    const Eigen::VectorXd & joint_positions, const Eigen::VectorXd & twist, double dt);

  template <int Rows, int Cols>
  Eigen::Matrix<double, Cols, Rows> dampedPseudoInverse(
    const Eigen::Matrix<double, Rows, Cols> & mat, double damping)
  {
    using MatrixType = Eigen::Matrix<double, Rows, Cols>;
    using SVDType = Eigen::JacobiSVD<MatrixType>;

    SVDType svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const auto & S = svd.singularValues();

    // Compute damped inverse of singular values
    Eigen::VectorXd S_inv = S.array() / (S.array().square() + damping * damping);

    // Build diagonal matrix directly
    Eigen::MatrixXd D = S_inv.asDiagonal();

    return svd.matrixV() * D * svd.matrixU().adjoint();
  }
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADAPTIVE_ADMITTANCE_CONTROLLER_HPP_