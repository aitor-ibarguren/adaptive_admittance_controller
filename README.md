# Adaptive Admittance Controller

<p>
  <a href="https://github.com/aitor-ibarguren/adaptive_admittance_controller/actions/workflows/ros2_jazzy_ci.yml">
    <img src="https://github.com/aitor-ibarguren/adaptive_admittance_controller/actions/workflows/ros2_jazzy_ci.yml/badge.svg" alt="Build">
  </a>
</p>

The `admittance_controller/AdaptiveAdmittanceController` is a chainable ROS2 controller designed to implement admittance control based on the wrench received from a force/torque sensor. This controller extendens the `admittance_controller/AdmittanceController` controller, allowing the modification of its multiple parameters via topic, with some additional features for a smooth admittance parameter modification.

## Features

- Chainable ROS2 controller designed to be placed as the last controller (e.g., after a *joint trajectory controller*).
- Accepts admittance parameters (stiffness, damping, mass, wrench command, active axes, and compliance frame) as well as tool parameters (mass and center-of-gravity) through topics.
- Includes a low-pass filter to smooth the wrench values received from the force/torque sensor.
- During the admittance and tool paremeter modification, the recalibration of the force/torque sensor bias can be triggered.
- The admittance control law calculates a twist value, which is internally computed to generate joint positions using **KDL** to generate the Jacobian matrix and **Eigen** to calculate the pseudo-inverse using *SVD*.
- Allows an **open-loop** mode in which the previously commanded joint positions are used instead of the joint positions from the state interfaces, avoiding the injection of hardware feedback latency and transport delays into the command generation loop.
- The controller enables the smoothing of admittance parameter modifications by adding a ramp to modify the parameters gradually (e.g., reducing the stiffness from 5000 to 500 over 1 second). These ramps are also applied when the admittance controller axes are activated/deactivated.
- The controller includes the option to **enable a feedback topic** of the admittance parameters, and wrench and twist values are published.

## Configuration

The next lines show a snippet of the *YAML* file defining the configuration of the `admittance_controller/AdaptiveAdmittanceController` controller for a UR16e robot:

```yaml
controller_manager:
  ros__parameters:
    use_sim_time: true
    update_rate: 100  # Hz

    ur_adaptive_admittance_controller:
      type: admittance_controller/AdaptiveAdmittanceController

ur_adaptive_admittance_controller:
  ros__parameters:
    is_chainable: true
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    state_interfaces:
      - position
      - velocity
    command_interfaces:
      - position
    chainable_command_interfaces:
      - position
    open_loop: true
    feedback_active: false
    kinematics:
      world_link: world
      base_link: base_link
      tip_link: tool0
    ft_sensor:
      name: ur_tcp_fts_sensor
      frame_id: ur_tool0
      filter_alpha: 0.025
      remove_bias: true
    tool:
      mass: 2.0
      CoG: [0.0, 0.0, 0.050]
    admittance_control:
      compliance_frame:
        translation: [0.0, 0.0, 0.0]
        rotation: [0.0, 0.0, 0.0, 1.0]
      stiffness: [5000.0, 5000.0, 500.0, 5000.0, 5000.0, 5000.0]
      damping: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
      mass: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
      wrench_command: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      active_axes: [true, true, true, true, true, true]
      ramp_active: true
      ramp_time: 1.0
      transition_ramp_stiffness: [5000.0, 5000.0, 5000.0, 500.0, 500.0, 500.0]
      transition_ramp_damping: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
      transition_ramp_mass: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
```

Besides the typical *joints*, *command_interfaces*, *command_interfaces*, and *chainable_command_interfaces*, the controller includes the next parameters:

- **open_loop:** Enable an open-loop control (joint position commands from the previous update call are used to calculate the next commands).
- **feedback_active:**: Enable the feedback topic on `controller_ns/feedback` as `adaptive_admittance_controller_msgs::msg::Feedback`.
- **kinematics:**
  - **world_link:** World link, used to define the gravity vector (gravity points in the negative Z axis in the world).
  - **base_link:** Base link of the kinematic chain of the group.
  - **tip_link:** Tip link of the kinematic chain of the group.
- **ft_sensor:**
  - **name:** Force/torque sensor name as defined in ROS2 Control, loaded as `semantic_components::ForceTorqueSensor`.
  - **frame_id:** Link ID in the robot's URDF (the controller automatically extracts the transformation between the tip link of the group and the force/torque sensor).
  - **filter_alpha:** Low-pass filter alpha value between 0 and 1, used to smooth the force/torque sensor signal. The filtering is applied after removing the tool's weight component of the wrench vector. Specifically, the filtered wrench vector is calculated as $W_t = \alpha W^{sensor}_t + (1 - \alpha)W_{t-1}$.
  - **remove_bias:** This parameter allows activating the automatic sensor bias estimation and removal from the force/torque sensor signal. Specifically, during the controller activation, the sensor bias is estimated by removing the tool's weight component, $W^{bias} = W^{sensor} - W^{tool}$, assuming that there is not any force applied in the tool. Additionally, the admittance parameter modification messages also allow updating this bias vector.
- **tool:**
  - **mass:** Mass of the robot's end-effector.
  - **cog:** Center-of-gravity of the end-effector, expressed in the tip link's frame.
- **admittance_control:**
  - **compliance_frame:** Reference frame in which the system’s compliant behavior is defined and applied, expressed as a translation (XYZ) and rotation (quaternion XYZW).
  - **stiffness:** Stiffness values of the admittance control law.
  - **damping:** Damping values of the admittance control law.
  - **mass:** Mass values of the admittance control law.
  - **wrench_command:** The wrench command to be applied in the admittance control law (e.g., apply $10N$ of force with the tool in Z axis as `[0.0, 0.0, -10.0, 0.0, 0.0, 0.0]`).
  - **active_axes:** Active axis applied during admittance control law, allowing to *mute* some of the axes during the calculation. The axes are expressed in the compliance frame.
  - **ramp_active:** This parameter allows activating the smoothing of admittance parameter modification (stiffness, damping, and mass), modifying them gradually to avoid large accelerations that might lead to oscilations. Additionally, during axes' activation/deactivation, it also uses some transition values (more information below) to avoid the previously described large parameter changes.
  - **ramp_time:** The parameter defines (if the *ramp_active* is set to true) the time to reach the new admittance control parameters. In each controller update call, the admittance parameters will be gradually modified towards the set values.
  - **transition_ramp_stiffness:** During the activation or deactivation of axes (if the *ramp_active* is set to true), these stiffness values are used as the starting values (activation) or final values (deactivation) of the ramp. The main idea is to set high-stiffness values to ensure that the activation/deactivation is smooth and avoids large accelerations.
  - **transition_ramp_damping:** During the activation or deactivation of axes (if the *ramp_active* set to true), these damping values are used as the starting values (activation) or final values (deactivation) of the ramp.
  - **transition_ramp_mass:** During the activation or deactivation of axes (if the *ramp_active* set to true), these mass values are used as the starting values (activation) or final values (deactivation) of the ramp.

Within all the parameters of the *adaptive_admittance_controller*, the addition of the ramp in admittance control parameter modification is the main contribution, intended to facilitate the usage of the controller in applications where different force-based behaviours and profiles are required.

  > **⚠️ Important:** If you are not familiar with admittance/force control and its parametrization, the use of conservative *damping* and *filter_alpha* values is recommended during the initial executions in order to avoid oscillations and unstable behaviour. Additionally, the reading of the theoretical basis of force-based control is also recommended ([Wikipedia](https://en.wikipedia.org/wiki/Impedance_control)) for a deeper understanding of the controller's parameters.

## Messages

The controller provides two topics to modify the main parameters of the admittance control law:

- **Tool parameters:** Modification of the robot tool's mass and CoG through the custom `adaptive_admittance_controller_msgs::msg::ToolParams` in the `controller_ns/tool_params` topic. Additionally, the message also includes the *recalculate_wrench_bias* parameter to trigger the recalculation of the sensor bias.
- **Admittance parameters:** Modification of the admittance control values through the custom `adaptive_admittance_controller_msgs::msg::AdmittanceParams` in the `controller_ns/admittance_params` topic. It includes the stiffness, damping, mass, wrench command, and the active axes. Additionally, the message also includes the *recalculate_wrench_bias* parameter to trigger the recalculation of the sensor bias.

## Feedback

In order to enable the introspection of the internal control values when undesired behaviours such as instabilities and oscillations are found, the controller includes a feedback topic (activable through the *feedback_active* parameter). The custom feedback message adaptive_admittance_controller_msgs::msg::Feedback` includes the next values:

- **tool_params:** The used tool parameters as `adaptive_admittance_controller_msgs::msg::ToolParams`.
- **admittance_params:** The used admittance control parameters as `adaptive_admittance_controller_msgs::msg::AdmittanceParams`.
- **received_wrench:** The wrench values received from the sensor.
- **filtered_wrench:** The filtered wrench values, after removing the tool weight's component and applying the low-pass filter.
- **wrench_in_compliance_frame:** Wrench in compliance frame.
- **twist_in_compliance_frame:** The twist vector in compliance frame, calculated from the admittance control law, and used to generate the next joint positions.

## License

The *adaptive_admittance_controller* repository has an Apache 2.0 license, as found in the [LICENSE](LICENSE) file.
