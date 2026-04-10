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

#include "adaptive_admittance_controller/adaptive_admittance_controller.hpp"

namespace admittance_controller
{
AdaptiveAdmittanceController::AdaptiveAdmittanceController()
: controller_interface::ChainableControllerInterface(), dof_(0), num_cmd_joints_(0)
{
}

controller_interface::CallbackReturn AdaptiveAdmittanceController::on_init()
{
  // Initialize the parameter handler
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<adaptive_admittance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
AdaptiveAdmittanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Check degrees of freedom
  if (dof_ == 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Degrees of freedom MUST valid positive (actual DOF %lu)", dof_);
    throw std::runtime_error("Invalid degrees of freedom");
  }

  // Reserve space for command interfaces
  conf.names.reserve(dof_ * params_.command_interfaces.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration
AdaptiveAdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Check degrees of freedom
  if (dof_ == 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Degrees of freedom MUST valid positive (actual DOF %lu)", dof_);
    throw std::runtime_error("Invalid degrees of freedom");
  }

  // Reserve space for state interfaces
  conf.names.reserve(dof_ * params_.state_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  auto ft_interfaces = force_torque_sensor_->get_state_interface_names();
  conf.names.insert(conf.names.end(), ft_interfaces.begin(), ft_interfaces.end());

  return conf;
}

controller_interface::CallbackReturn AdaptiveAdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Prepare the controller for activation.
  RCLCPP_INFO(get_node()->get_logger(), "Configuring AdaptiveAdmittanceController");

  // Update the dynamic map parameters TODO: What is this for??
  param_listener_->refresh_dynamic_parameters();

  // Get parameters from listener
  params_ = param_listener_->get_params();

  // Get DoF
  dof_ = params_.joints.size();

  // Get joint names
  joint_names_ = params_.joints;
  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints were specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get joint limits
  if (!get_joint_limits(joint_names_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error retrieving kinematic info from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Command interfaces
  has_position_command_interface_ =
    std::find(
      params_.command_interfaces.begin(), params_.command_interfaces.end(),
      hardware_interface::HW_IF_POSITION) != params_.command_interfaces.end();

  // Get open loop
  open_loop_ = params_.open_loop;

  /// Kinematic data
  // Get world, base and tip links
  world_link_ = params_.kinematics.world_link;
  base_link_ = params_.kinematics.base_link;
  tip_link_ = params_.kinematics.tip_link;

  // Set q
  q_ = KDL::JntArray(joint_names_.size());

  // Get kinematic info
  if (!get_kinematics(base_link_, tip_link_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error generating kinematic solvers from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Calculate gravity vector
  Eigen::Isometry3d world_H_base_link;

  if (!getTransform(tree_, world_link_, base_link_, world_H_base_link))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error retrieving transformation between world and base link");
    return controller_interface::CallbackReturn::ERROR;
  }

  gravity_ = -9.80665 * world_H_base_link.rotation().col(2);

  /// F/T sensor
  ft_sensor_frame_ = params_.ft_sensor.frame_id;
  if (!getTransform(tree_, tip_link_, ft_sensor_frame_, tip_H_ft_sensor_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error retrieving transformation between tip link and FT sensor");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Initialize FTS semantic semantic_component
  RCLCPP_INFO(get_node()->get_logger(), "ft_sensor.name %s", params_.ft_sensor.name.c_str());
  force_torque_sensor_ =
    std::make_unique<semantic_components::ForceTorqueSensor>(params_.ft_sensor.name);
  // Filter
  filter_alpha_ = params_.ft_sensor.filter_alpha;

  /// Tool
  tool_mass_ = params_.tool.mass;
  tool_cog_ = Eigen::Vector3d(params_.tool.CoG[0], params_.tool.CoG[1], params_.tool.CoG[2]);

  /// Admittance control
  // Compliance frame
  Eigen::Vector3d t(
    params_.admittance_control.compliance_frame.translation[0],
    params_.admittance_control.compliance_frame.translation[1],
    params_.admittance_control.compliance_frame.translation[2]);
  Eigen::Quaterniond q(
    params_.admittance_control.compliance_frame.rotation[3],
    params_.admittance_control.compliance_frame.rotation[0],
    params_.admittance_control.compliance_frame.rotation[1],
    params_.admittance_control.compliance_frame.rotation[2]);

  flange_H_compliance_frame_ = Eigen::Isometry3d::Identity();
  flange_H_compliance_frame_.translate(t);
  flange_H_compliance_frame_.rotate(q);

  // Admittance params
  // Stiffness
  admittance_params_.stiffness = Eigen::Map<Eigen::VectorXd>(
    params_.admittance_control.stiffness.data(), params_.admittance_control.stiffness.size());
  // Damping
  admittance_params_.damping = Eigen::Map<Eigen::VectorXd>(
    params_.admittance_control.damping.data(), params_.admittance_control.damping.size());
  // Mass
  admittance_params_.mass = Eigen::Map<Eigen::VectorXd>(
    params_.admittance_control.mass.data(), params_.admittance_control.mass.size());
  // Wrench command
  admittance_params_.wrench_command = Eigen::Map<Eigen::VectorXd>(
    params_.admittance_control.wrench_command.data(),
    params_.admittance_control.wrench_command.size());
  // Active axes
  admittance_params_.active_axes = params_.admittance_control.active_axes;
  // Ramp
  admittance_params_.ramp_active = params_.admittance_control.ramp_active;
  if (admittance_params_.ramp_active)
  {
    admittance_params_.ramp_time = params_.admittance_control.ramp_time;
    admittance_params_.transition_ramp_stiffness = Eigen::Map<Eigen::VectorXd>(
      params_.admittance_control.transition_ramp_stiffness.data(),
      params_.admittance_control.transition_ramp_stiffness.size());
    admittance_params_.transition_ramp_damping = Eigen::Map<Eigen::VectorXd>(
      params_.admittance_control.transition_ramp_damping.data(),
      params_.admittance_control.transition_ramp_damping.size());
    admittance_params_.transition_ramp_mass = Eigen::Map<Eigen::VectorXd>(
      params_.admittance_control.transition_ramp_mass.data(),
      params_.admittance_control.transition_ramp_mass.size());
  }

  // Feedback
  feedback_active_ = params_.feedback_active;

  if (feedback_active_)
  {
    // Tool params
    feedback_msg_.tool_params.mass = tool_mass_;
    feedback_msg_.tool_params.cog = tf2::toMsg(tool_cog_);

    // Admittance params
    feedback_msg_.admittance_params.compliance_frame = tf2::toMsg(flange_H_compliance_frame_);
    feedback_msg_.admittance_params.stiffness = params_.admittance_control.stiffness;
    feedback_msg_.admittance_params.damping = params_.admittance_control.damping;
    feedback_msg_.admittance_params.mass = params_.admittance_control.mass;
    tf2::toMsg(
      Eigen::Vector3d(admittance_params_.wrench_command.head<3>()),
      feedback_msg_.admittance_params.wrench_command.force);
    tf2::toMsg(
      Eigen::Vector3d(admittance_params_.wrench_command.tail<3>()),
      feedback_msg_.admittance_params.wrench_command.torque);
    feedback_msg_.admittance_params.active_axes = params_.admittance_control.active_axes;
  }

  // Log
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Open loop: %s", open_loop_ ? "True" : "False");
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    RCLCPP_INFO(get_node()->get_logger(), "╠═ Joint %s limits: ", joint_names_[i].c_str());
    RCLCPP_INFO(
      get_node()->get_logger(), "║  ├─ Lower: %f - Upper: %f", lower_joint_limits_[i],
      upper_joint_limits_[i]);
    RCLCPP_INFO(get_node()->get_logger(), "║  ╰─ Velocity: %f", vel_joint_limits_[i]);
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "╠═ Kinematic chain from '%s' to '%s'", base_link_.c_str(),
    tip_link_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(), "╠═ Gravity vector from '%s': [%f, %f, %f]", world_link_.c_str(),
    gravity_.x(), gravity_.y(), gravity_.z());
  RCLCPP_INFO(get_node()->get_logger(), "╠═ FT sensor");
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ Sensor frame '%s'", ft_sensor_frame_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ Filter alpha: %f", filter_alpha_);
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ╰─ Remove bias: %s",
    params_.ft_sensor.remove_bias ? "True" : "False");
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Tool");
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ Mass: %f", tool_mass_);
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ╰─ CoG:  [%f, %f, %f]", tool_cog_.x(), tool_cog_.y(),
    tool_cog_.z());
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Admittance control");
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Compliance frame - Translation:[%f, %f, %f]",
    params_.admittance_control.compliance_frame.translation[0],
    params_.admittance_control.compliance_frame.translation[1],
    params_.admittance_control.compliance_frame.translation[2]);
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Compliance frame - Rotation (xyzw):[%f, %f, %f, %f]",
    params_.admittance_control.compliance_frame.rotation[0],
    params_.admittance_control.compliance_frame.rotation[1],
    params_.admittance_control.compliance_frame.rotation[2],
    params_.admittance_control.compliance_frame.rotation[3]);
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Stiffness: [%f, %f, %f, %f, %f, %f]",
    admittance_params_.stiffness(0), admittance_params_.stiffness(1),
    admittance_params_.stiffness(2), admittance_params_.stiffness(3),
    admittance_params_.stiffness(4), admittance_params_.stiffness(5));
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Damping: [%f, %f, %f, %f, %f, %f]",
    admittance_params_.damping(0), admittance_params_.damping(1), admittance_params_.damping(2),
    admittance_params_.damping(3), admittance_params_.damping(4), admittance_params_.damping(5));
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Mass: [%f, %f, %f, %f, %f, %f]", admittance_params_.mass(0),
    admittance_params_.mass(1), admittance_params_.mass(2), admittance_params_.mass(3),
    admittance_params_.mass(4), admittance_params_.mass(5));
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Wrench command: [%f, %f, %f, %f, %f, %f]",
    admittance_params_.wrench_command(0), admittance_params_.wrench_command(1),
    admittance_params_.wrench_command(2), admittance_params_.wrench_command(3),
    admittance_params_.wrench_command(4), admittance_params_.wrench_command(5));
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Active axes: [%s, %s, %s, %s, %s, %s]",
    admittance_params_.active_axes[0] ? "True" : "False",
    admittance_params_.active_axes[1] ? "True" : "False",
    admittance_params_.active_axes[2] ? "True" : "False",
    admittance_params_.active_axes[3] ? "True" : "False",
    admittance_params_.active_axes[4] ? "True" : "False",
    admittance_params_.active_axes[5] ? "True" : "False");
  if (admittance_params_.ramp_active)
  {
    RCLCPP_INFO(get_node()->get_logger(), "║  ├─ Ramp active:  True");
    RCLCPP_INFO(get_node()->get_logger(), "║  ├─ Ramp time:  %f", admittance_params_.ramp_time);
    RCLCPP_INFO(
      get_node()->get_logger(), "║  ├─ Transition ramp stiffness: [%f, %f, %f, %f, %f, %f]",
      admittance_params_.transition_ramp_stiffness(0),
      admittance_params_.transition_ramp_stiffness(1),
      admittance_params_.transition_ramp_stiffness(2),
      admittance_params_.transition_ramp_stiffness(3),
      admittance_params_.transition_ramp_stiffness(4),
      admittance_params_.transition_ramp_stiffness(5));
    RCLCPP_INFO(
      get_node()->get_logger(), "║  ├─ Transition ramp damping: [%f, %f, %f, %f, %f, %f]",
      admittance_params_.transition_ramp_damping(0), admittance_params_.transition_ramp_damping(1),
      admittance_params_.transition_ramp_damping(2), admittance_params_.transition_ramp_damping(3),
      admittance_params_.transition_ramp_damping(4), admittance_params_.transition_ramp_damping(5));
    RCLCPP_INFO(
      get_node()->get_logger(), "║  ╰─ Transition ramp mass: [%f, %f, %f, %f, %f, %f]",
      admittance_params_.transition_ramp_mass(0), admittance_params_.transition_ramp_mass(1),
      admittance_params_.transition_ramp_mass(2), admittance_params_.transition_ramp_mass(3),
      admittance_params_.transition_ramp_mass(4), admittance_params_.transition_ramp_mass(5));
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "║  ╰─ Ramp active:  False");
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "╚═ Feedback: %s", feedback_active_ ? "ACTIVE" : "INACTIVE");

  // Create subscribers & publishers
  admittance_params_subs_ =
    get_node()->create_subscription<adaptive_admittance_controller_msgs::msg::AdmittanceParams>(
      std::string(get_node()->get_name()) + "/admittance_params", 1,
      std::bind(
        &AdaptiveAdmittanceController::admittance_params_callback, this, std::placeholders::_1));
  tool_params_subs_ =
    get_node()->create_subscription<adaptive_admittance_controller_msgs::msg::ToolParams>(
      std::string(get_node()->get_name()) + "/tool_params", 1,
      std::bind(&AdaptiveAdmittanceController::tool_params_callback, this, std::placeholders::_1));

  if (feedback_active_)
  {
    feedback_pub_ =
      get_node()->create_publisher<adaptive_admittance_controller_msgs::msg::Feedback>(
        std::string(get_node()->get_name()) + "/feedback", rclcpp::SystemDefaultsQoS());
    feedback_pub_rt_ = std::make_unique<
      realtime_tools::RealtimePublisher<adaptive_admittance_controller_msgs::msg::Feedback>>(
      feedback_pub_);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveAdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating AdaptiveAdmittanceController");

  // Initialize robot joint states
  joint_positions_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_ = Eigen::VectorXd::Zero(dof_);
  reference_joint_positions_ = Eigen::VectorXd::Zero(dof_);
  reference_joint_velocities_ = Eigen::VectorXd::Zero(dof_);

  wrench_ = Eigen::VectorXd::Zero(6);

  // Initialize joint commands
  joint_position_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_prev_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_prev_ = Eigen::VectorXd::Zero(dof_);

  // Verify hardware interfaces are correctly loaded
  if (reference_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No reference interfaces loaded");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces loaded");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get current joint positions and velocities
  read_joint_state(joint_positions_, joint_velocities_);
  // Get current reference joint positions and velocities
  read_reference_joint_state(reference_joint_positions_, reference_joint_velocities_);
  // Set commands to reference joint state
  joint_position_commands_ = reference_joint_positions_;
  joint_velocity_commands_ = reference_joint_velocities_;
  joint_position_commands_prev_ = reference_joint_positions_;
  joint_velocity_commands_prev_ = reference_joint_velocities_;

  // F/T sensor
  force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  // Calculate F/T sensor bias if required
  if (params_.ft_sensor.remove_bias)
  {
    // Get wrench
    read_wrench(wrench_);
    // Get tip pose
    base_link_H_tip_ = get_tip_pose(joint_positions_);

    // Get clean wrench
    wrench_bias_ =
      clear_tool_wrench(wrench_, tool_mass_, tool_cog_, base_link_H_tip_, tip_H_ft_sensor_);
  }
  else
  {
    wrench_bias_ = Eigen::VectorXd::Zero(6);
  }

  filtered_wrench_ = Eigen::VectorXd::Zero(6);
  prev_filtered_wrench_ = Eigen::VectorXd::Zero(6);

  // Admittance control law
  admittance_control_law_ = std::make_shared<AdmittanceControlLaw>(admittance_params_);
  update_admittance_params_ = false;

  /// Log
  // Joint positions
  std::ostringstream oss;
  for (double d : joint_positions_) oss << d << ' ';

  RCLCPP_INFO(get_node()->get_logger(), "Initial joint positions: [ %s]'", oss.str().c_str());

  // Reference joint positions
  oss.str("");
  oss.clear();
  for (double d : reference_joint_positions_) oss << d << ' ';

  RCLCPP_INFO(
    get_node()->get_logger(), "Initial reference joint positions: [ %s]'", oss.str().c_str());

  // F/T sensor bias (if option active)
  if (params_.ft_sensor.remove_bias)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "F/T sensor bias: [%f, %f, %f, %f, %f, %f]", wrench_bias_(0),
      wrench_bias_(1), wrench_bias_(2), wrench_bias_(3), wrench_bias_(4), wrench_bias_(5));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveAdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating AdaptiveAdmittanceController");

  // Set all values to zero
  joint_positions_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_prev_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_prev_ = Eigen::VectorXd::Zero(dof_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdaptiveAdmittanceController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating AdaptiveAdmittanceController");

  // Reset subscribers
  admittance_params_subs_.reset();
  tool_params_subs_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

bool AdaptiveAdmittanceController::on_set_chained_mode(bool chained_mode)
{
  if (chained_mode)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Controller entered chained mode");

    force_torque_sensor_.reset();
    force_torque_sensor_ =
      std::make_unique<semantic_components::ForceTorqueSensor>(params_.ft_sensor.name);

    force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Controller left chained mode");
  }

  return true;
}

Eigen::VectorXd AdaptiveAdmittanceController::change_twist_reference(
  const Eigen::VectorXd & twist, const Eigen::Isometry3d & twist_reference)
{
  // Declare new twist
  Eigen::VectorXd new_twist = Eigen::VectorXd::Zero(6);

  new_twist.head<3>() = twist_reference.rotation() * twist.head<3>();
  new_twist.tail<3>() = twist_reference.rotation() * twist.tail<3>();

  return new_twist;
}

Eigen::VectorXd AdaptiveAdmittanceController::move_twist(
  const Eigen::VectorXd & twist, const Eigen::Vector3d & q)
{
  // Declare new twist
  Eigen::VectorXd new_twist = Eigen::VectorXd::Zero(6);

  new_twist.head<3>() = twist.head<3>() + q.cross(twist.tail<3>());
  new_twist.tail<3>() = twist.tail<3>();

  return new_twist;
}

Eigen::VectorXd AdaptiveAdmittanceController::calculate_next_joint_positions(
  const Eigen::VectorXd & joint_positions, const Eigen::VectorXd & twist, double dt)
{
  Eigen::VectorXd next_joint_position = Eigen::VectorXd::Zero(6);

  // To KDL
  KDL::JntArray joint_positions_kdl(joint_positions.size());
  joint_positions_kdl.data = joint_positions;

  // Get Jacobian
  KDL::Jacobian jacobian(dof_);

  jnt_to_jac_solver_->JntToJac(joint_positions_kdl, jacobian);

  // Damped pseudo-inverse
  Eigen::MatrixXd jacobian_pseudo_inverse = dampedPseudoInverse(jacobian.data, 0.1);

  // Compute joint velocities
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(dof_);
  joint_velocities = jacobian_pseudo_inverse * twist;

  // Next joint positions
  next_joint_position = joint_positions + joint_velocities * dt;

  return next_joint_position;
}

controller_interface::return_type AdaptiveAdmittanceController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Update admittance params if required
  if (update_admittance_params_)
  {
    admittance_control_law_->update_admittance_parameters(admittance_params_);
    update_admittance_params_ = false;
  }

  // Get joint state
  if (open_loop_)
  {
    joint_positions_ = joint_position_commands_prev_;
    joint_velocities_ = joint_velocity_commands_prev_;
  }
  else
  {
    read_joint_state(joint_positions_, joint_velocities_);
  }
  // Get reference joint state
  read_reference_joint_state(reference_joint_positions_, reference_joint_velocities_);

  // Get wrench
  read_wrench(wrench_);

  // Get tip pose (reference & real)
  base_link_H_tip_ref_ = get_tip_pose(reference_joint_positions_);
  base_link_H_tip_ = get_tip_pose(joint_positions_);

  // Get clean wrench
  clean_wrench_ =
    clear_tool_wrench(wrench_, tool_mass_, tool_cog_, base_link_H_tip_, tip_H_ft_sensor_);

  // Remove bias (if option active)
  if (params_.ft_sensor.remove_bias) clean_wrench_ = clean_wrench_ - wrench_bias_;

  // Lowpass filter
  filtered_wrench_ = filter_alpha_ * clean_wrench_ + (1 - filter_alpha_) * prev_filtered_wrench_;

  prev_filtered_wrench_ = filtered_wrench_;

  // Wrench to compliance frame
  Eigen::VectorXd wrench_in_compliance_frame =
    change_wrench_frame(filtered_wrench_, tip_H_ft_sensor_.inverse() * flange_H_compliance_frame_);

  // Calculate twist based on admittance law
  Eigen::VectorXd twist_in_compliance_frame = admittance_control_law_->update(
    base_link_H_tip_ref_ * flange_H_compliance_frame_,
    base_link_H_tip_ * flange_H_compliance_frame_, wrench_in_compliance_frame, period.seconds());

  // Twist to base link
  Eigen::VectorXd twist_base_link_compliance_frame = change_twist_reference(
    twist_in_compliance_frame, base_link_H_tip_ * flange_H_compliance_frame_);

  // Transfer twist to tip link
  Eigen::VectorXd twist_base_link_tip = move_twist(
    twist_base_link_compliance_frame,
    (base_link_H_tip_ * flange_H_compliance_frame_).translation() - base_link_H_tip_.translation());

  // Get joint position commands
  joint_position_commands_ =
    calculate_next_joint_positions(joint_positions_, twist_base_link_tip, period.seconds());

  // Set joint position
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (has_position_command_interface_)
    {
      if (!command_interfaces_[i].set_value(static_cast<double>(joint_position_commands_(i))))
        RCLCPP_WARN(get_node()->get_logger(), "Error setting command for joint %zu", i);
    }
  }

  // Manage feedback
  if (feedback_active_)
  {
    // Update admittance parameters if ramp option active
    if (admittance_params_.ramp_active)
      admittance_control_law_->update_admittance_parameters_msg(feedback_msg_.admittance_params);

    // Publish
    publish_feedback(
      wrench_, filtered_wrench_, wrench_in_compliance_frame, twist_in_compliance_frame);
  }

  // Store joint commands
  joint_position_commands_prev_ = joint_position_commands_;
  joint_velocity_commands_prev_ = joint_velocity_commands_;

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
AdaptiveAdmittanceController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> chainable_command_interfaces;
  const auto num_chainable_interfaces =
    params_.chainable_command_interfaces.size() * params_.joints.size();

  // allocate dynamic memory
  chainable_command_interfaces.reserve(num_chainable_interfaces);
  reference_interfaces_.resize(num_chainable_interfaces, std::numeric_limits<double>::quiet_NaN());
  position_reference_ = {};
  velocity_reference_ = {};

  // assign reference interfaces
  auto index = 0ul;
  for (const auto & interface : params_.chainable_command_interfaces)
  {
    for (const auto & joint : params_.joints)
    {
      if (hardware_interface::HW_IF_POSITION == interface)
        position_reference_.emplace_back(reference_interfaces_[index]);
      else if (hardware_interface::HW_IF_VELOCITY == interface)
      {
        velocity_reference_.emplace_back(reference_interfaces_[index]);
      }
      const auto exported_prefix = std::string(get_node()->get_name()) + "/" + joint;
      chainable_command_interfaces.emplace_back(hardware_interface::CommandInterface(
        exported_prefix, interface, reference_interfaces_.data() + index));

      index++;
    }
  }

  return chainable_command_interfaces;
}

controller_interface::return_type AdaptiveAdmittanceController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

void AdaptiveAdmittanceController::admittance_params_callback(
  const adaptive_admittance_controller_msgs::msg::AdmittanceParams::SharedPtr msg)
{
  // Check if controller active
  auto state_id = this->get_node()->get_current_state().id();
  if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Can't accept admittance parameters: Controller is not active");
    return;
  }

  // Check arrays' size
  if (
    msg->stiffness.size() != 6 || msg->damping.size() != 6 || msg->mass.size() != 6 ||
    msg->active_axes.size() != 6)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Admittance parameter message incorrectly formatted: Check aray sizes");
  }

  // Admittance params
  // Compliance frame
  tf2::fromMsg(msg->compliance_frame, flange_H_compliance_frame_);
  // Stiffness
  admittance_params_.stiffness =
    Eigen::Map<Eigen::VectorXd>(msg->stiffness.data(), msg->stiffness.size());
  // Damping
  admittance_params_.damping =
    Eigen::Map<Eigen::VectorXd>(msg->damping.data(), msg->damping.size());
  // Mass
  admittance_params_.mass = Eigen::Map<Eigen::VectorXd>(msg->mass.data(), msg->mass.size());
  // Wrench command
  admittance_params_.wrench_command(0) = msg->wrench_command.force.x;
  admittance_params_.wrench_command(1) = msg->wrench_command.force.y;
  admittance_params_.wrench_command(2) = msg->wrench_command.force.z;
  admittance_params_.wrench_command(3) = msg->wrench_command.torque.x;
  admittance_params_.wrench_command(4) = msg->wrench_command.torque.y;
  admittance_params_.wrench_command(5) = msg->wrench_command.torque.z;
  // Active axes
  admittance_params_.active_axes = msg->active_axes;

  // Update admittance control law
  update_admittance_params_ = true;

  // Manage feedback
  if (feedback_active_) feedback_msg_.admittance_params = *msg;

  // Recalculate bias if required
  if (msg->recalculate_wrench_bias && params_.ft_sensor.remove_bias)
  {
    wrench_bias_ =
      clear_tool_wrench(wrench_, tool_mass_, tool_cog_, base_link_H_tip_, tip_H_ft_sensor_);

    RCLCPP_INFO(
      get_node()->get_logger(), "New F/T sensor bias: [%f, %f, %f, %f, %f, %f]", wrench_bias_(0),
      wrench_bias_(1), wrench_bias_(2), wrench_bias_(3), wrench_bias_(4), wrench_bias_(5));
  }
}

void AdaptiveAdmittanceController::tool_params_callback(
  const adaptive_admittance_controller_msgs::msg::ToolParams::SharedPtr msg)
{
  // Check if controller active
  auto state_id = this->get_node()->get_current_state().id();
  if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Can't accept admittance parameters: Controller is not active");
    return;
  }

  // Update tool data
  tool_mass_ = msg->mass;
  tf2::fromMsg(msg->cog, tool_cog_);

  // Manage feedback
  if (feedback_active_) feedback_msg_.tool_params = *msg;

  // Recalculate bias if required
  if (msg->recalculate_wrench_bias && params_.ft_sensor.remove_bias)
  {
    wrench_bias_ =
      clear_tool_wrench(wrench_, tool_mass_, tool_cog_, base_link_H_tip_, tip_H_ft_sensor_);

    RCLCPP_INFO(
      get_node()->get_logger(), "New F/T sensor bias: [%f, %f, %f, %f, %f, %f]", wrench_bias_(0),
      wrench_bias_(1), wrench_bias_(2), wrench_bias_(3), wrench_bias_(4), wrench_bias_(5));
  }
}

bool AdaptiveAdmittanceController::get_joint_limits(const std::vector<std::string> & joint_names)
{
  // Get URDF
  const std::string & urdf = get_robot_description();

  // Init model
  urdf::Model model;
  if (!model.initString(urdf))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed parsing URDF");
    return false;
  }

  // Define Eigen::VectorXd size
  lower_joint_limits_.resize(joint_names.size());
  upper_joint_limits_.resize(joint_names.size());
  vel_joint_limits_.resize(joint_names.size());

  // Get limits
  int idx = 0;
  for (auto & joint_name : joint_names)
  {
    // Get joint
    auto joint = model.getJoint(joint_name);
    if (!joint)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found on URDF", joint_name.c_str());
      return false;
    }

    // Get limits
    lower_joint_limits_[idx] = joint->limits->lower;
    upper_joint_limits_[idx] = joint->limits->upper;
    vel_joint_limits_[idx] = joint->limits->velocity;

    idx++;
  }

  return true;
}

bool AdaptiveAdmittanceController::get_kinematics(
  const std::string & base_link, const std::string & tip_link)
{
  // Get URDF
  const std::string & urdf = get_robot_description();

  // Init model
  if (!model_.initString(urdf))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed parsing URDF");
    return false;
  }

  // Init tree
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree");
    return false;
  }

  // Get kinematic chain
  if (!tree_.getChain(base_link, tip_link, chain_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to get KDL chain from '%s' to '%s'", base_link.c_str(),
      tip_link.c_str());
    return false;
  }

  // Check joint number
  if (chain_.getNrOfJoints() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Provided joint list (%d) and the number of joints of the KDL chain (%d) are not equal",
      (int)joint_names_.size(), chain_.getNrOfJoints());
    return false;
  }

  // Check if joint names are in the chain order
  int j_number = 0;
  for (size_t i = 0; i < chain_.getNrOfSegments(); i++)
  {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      // Check if same joint name
      if (joint.getName() != joint_names_[j_number])
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Joint number %d names ('%s' and '%s') are not equal", (int)i,
          joint.getName().c_str(), joint_names_[j_number].c_str());
        return false;
      }

      j_number++;
    }
  }

  // FK solver
  fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  jnt_to_jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);

  return true;
}

bool AdaptiveAdmittanceController::getTransform(
  const KDL::Tree & tree, const std::string & from, const std::string & to,
  Eigen::Isometry3d & result)
{
  // Get chain between
  KDL::Chain chain;

  if (!tree.getChain(from, to, chain)) return false;

  // Calculate transformation & verify all are fixed joints
  KDL::Frame frame = KDL::Frame::Identity();

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const KDL::Segment & seg = chain.getSegment(i);

    if (seg.getJoint().getType() != KDL::Joint::None)
    {
      return false;  // non-fixed joint found
    }

    frame = frame * seg.pose(0.0);
  }

  // To Eigen isometry
  tf2::transformKDLToEigen(frame, result);

  return true;
}

void AdaptiveAdmittanceController::read_joint_state(
  Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities)
{
  int idx_pos = 0;
  int idx_vel = 0;

  for (auto & state_interface : state_interfaces_)
  {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Get position
      const auto joint_position_opt = state_interface.get_optional();
      if (!joint_position_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_positions[idx_pos] = joint_position_opt.value();

        idx_pos++;
      }
    }
    else if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      // Get velocity
      const auto joint_vel_opt = state_interface.get_optional();
      if (!joint_vel_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_velocities[idx_vel] = joint_vel_opt.value();
        idx_vel++;
      }
    }
  }
}

void AdaptiveAdmittanceController::read_reference_joint_state(
  Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities)
{
  for (size_t i = 0; i < dof_; ++i)
  {
    for (const auto & interface : params_.chainable_command_interfaces)
    {
      // update position
      if (interface == hardware_interface::HW_IF_POSITION)
      {
        if (std::isnan(position_reference_[i]))
        {
          position_reference_[i].get() = joint_positions_(i);
        }
        joint_positions(i) = position_reference_[i].get();
      }

      // update velocity
      if (interface == hardware_interface::HW_IF_VELOCITY)
      {
        if (std::isnan(velocity_reference_[i]))
        {
          velocity_reference_[i].get() = joint_velocities_[i];
        }
        joint_velocities[i] = velocity_reference_[i];
      }
    }
  }
}

void AdaptiveAdmittanceController::read_wrench(Eigen::VectorXd & wrench)
{
  geometry_msgs::msg::Wrench w;

  // Read wrench from sensor
  force_torque_sensor_->get_values_as_message(w);

  // Check if there is any Nan to set wrench to zero
  if (
    std::isnan(w.force.x) || std::isnan(w.force.y) || std::isnan(w.force.z) ||
    std::isnan(w.torque.x) || std::isnan(w.torque.y) || std::isnan(w.torque.z))
  {
    w = geometry_msgs::msg::Wrench();
  }

  // Msg to Eigen
  wrench(0) = w.force.x;
  wrench(1) = w.force.y;
  wrench(2) = w.force.z;
  wrench(3) = w.torque.x;
  wrench(4) = w.torque.y;
  wrench(5) = w.torque.z;
}

Eigen::Isometry3d AdaptiveAdmittanceController::get_tip_pose(
  const Eigen::VectorXd & joint_positions)
{
  Eigen::Isometry3d tip_pose;

  // Insert joint positions
  for (int i = 0; i < joint_positions.size(); i++) q_(i) = joint_positions[i];

  // Get Cartesian pose
  KDL::Frame kdl_pose;
  fk_solver_->JntToCart(q_, kdl_pose);

  // KDL to Eigen
  // Translation
  tip_pose.translation() = Eigen::Vector3d(kdl_pose.p.x(), kdl_pose.p.y(), kdl_pose.p.z());
  // Rotation
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) tip_pose.linear()(i, j) = kdl_pose.M(i, j);

  return tip_pose;
}

void AdaptiveAdmittanceController::publish_feedback(
  const Eigen::VectorXd & received_wrench, const Eigen::VectorXd & filtered_wrench,
  const Eigen::VectorXd & wrench_in_compliance_frame,
  const Eigen::VectorXd & twist_in_compliance_frame)
{
  // Fill wrench data
  tf2::toMsg(Eigen::Vector3d(received_wrench.head<3>()), feedback_msg_.received_wrench.force);
  tf2::toMsg(Eigen::Vector3d(received_wrench.tail<3>()), feedback_msg_.received_wrench.torque);
  tf2::toMsg(Eigen::Vector3d(filtered_wrench.head<3>()), feedback_msg_.filtered_wrench.force);
  tf2::toMsg(Eigen::Vector3d(filtered_wrench.tail<3>()), feedback_msg_.filtered_wrench.torque);
  tf2::toMsg(
    Eigen::Vector3d(wrench_in_compliance_frame.head<3>()),
    feedback_msg_.wrench_in_compliance_frame.force);
  tf2::toMsg(
    Eigen::Vector3d(wrench_in_compliance_frame.tail<3>()),
    feedback_msg_.wrench_in_compliance_frame.torque);
  // Fill twist
  tf2::toMsg(
    Eigen::Vector3d(twist_in_compliance_frame.head<3>()),
    feedback_msg_.twist_in_compliance_frame.linear);
  tf2::toMsg(
    Eigen::Vector3d(twist_in_compliance_frame.tail<3>()),
    feedback_msg_.twist_in_compliance_frame.angular);

  // Publish
  feedback_pub_rt_->try_publish(feedback_msg_);
}

Eigen::VectorXd AdaptiveAdmittanceController::clear_tool_wrench(
  const Eigen::VectorXd & measured_wrench, double tool_mass, const Eigen::Vector3d & tool_cog,
  const Eigen::Isometry3d & tip_pose, const Eigen::Isometry3d & tip_H_ft_sensor)
{
  Eigen::VectorXd tool_wrench = Eigen::VectorXd::Zero(6);

  // Tool force vector in sensor frame
  tool_wrench.head<3>() = (tip_pose * tip_H_ft_sensor).rotation().inverse() *
                          (tool_mass * Eigen::Vector3d(0.0, 0.0, -9.80665));

  // Tool torque vector in sensor frame
  tool_wrench.tail<3>() = tool_cog.cross(tool_wrench.head<3>());

  return measured_wrench - tool_wrench;
}

Eigen::VectorXd AdaptiveAdmittanceController::change_wrench_frame(
  const Eigen::VectorXd & wrench, const Eigen::Isometry3d & new_frame)
{
  Eigen::VectorXd new_wrench = Eigen::VectorXd::Zero(6);

  // Change force frame
  new_wrench.head<3>() = new_frame.rotation().inverse() * wrench.head<3>();

  // Calculate new force's torque in new frame
  new_wrench.tail<3>() = new_frame.rotation().inverse() *
                         (wrench.tail<3>() - new_frame.translation().cross(wrench.head<3>()));

  return new_wrench;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdaptiveAdmittanceController,
  controller_interface::ChainableControllerInterface)
