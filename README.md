# Adaptive Admittance Controller

<p>
  <a href="https://github.com/aitor-ibarguren/adaptive_admittance_controller/actions/workflows/ros2_jazzy_ci.yml">
    <img src="https://github.com/aitor-ibarguren/adaptive_admittance_controller/actions/workflows/ros2_jazzy_ci.yml/badge.svg" alt="Build">
  </a>
</p>

The `admittance_controller/AdaptiveAdmittanceController` is a chainable ROS2 controller designed to implement admittance control based on the wrench received from a force/torque sensor. This controller extendens the `admittance_controller/AdmittanceController` controller allowing the modification of its multiple parameters via topic with some additional features for a smooth admittance parameter modification.

## Features

- Chainable ROS2 controller designed to be placed as the last controller (e.g., after a *joint trajectory controller*).
- Accepts admittance parameters (stiffness, damping, mass, wrench command, active axes, and compliance frame) as well as tool parameters (mass and center-of-gravity) through topics.
- Includes a low-pass filter to smooth the wrench values received from the force/torque sensor.
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

## License

The *adaptive_admittance_controller* repository has an Apache 2.0 license, as found in the [LICENSE](LICENSE) file.