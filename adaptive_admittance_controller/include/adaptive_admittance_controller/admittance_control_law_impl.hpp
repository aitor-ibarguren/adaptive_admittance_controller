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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROL_LAW_IMPL_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROL_LAW_IMPL_HPP_

// Eigen
#include "Eigen/Geometry"

namespace admittance_controller
{
struct AdmittanceParams
{
  // Control params
  Eigen::VectorXd stiffness;
  Eigen::VectorXd damping;
  Eigen::VectorXd mass;
  Eigen::VectorXd wrench_command;

  // Axis
  std::vector<bool> active_axes;

  // Ramp
  bool ramp_active;
  double ramp_time;

  Eigen::VectorXd transition_ramp_stiffness;
  Eigen::VectorXd transition_ramp_damping;
  Eigen::VectorXd transition_ramp_mass;
};

class AdmittanceControlLaw
{
public:
  AdmittanceControlLaw(const AdmittanceParams & admittance_params)
  {
    // Stiffness
    K_.setZero();
    K_.diagonal() = admittance_params.stiffness;
    // Damping
    D_.setZero();
    D_.diagonal() = admittance_params.damping;
    // Mass
    M_.setZero();
    M_.diagonal() = admittance_params.mass;
    // Mass inverse
    M_inv_ = M_.inverse();
    // Wrench command
    wrench_command_ = admittance_params.wrench_command;
    // Active axes
    active_axes_.setZero();
    for (size_t i = 0; i < 6; i++)
      active_axes_(i, i) = admittance_params.active_axes[i] ? 1.0 : 0.0;

    // Control values
    X_ = Eigen::VectorXd::Zero(6);
    admittance_acceleration_ = Eigen::VectorXd::Zero(6);
    admittance_velocity_ = Eigen::VectorXd::Zero(6);

    // Ramp
    ramp_active_ = admittance_params.ramp_active;
    if (ramp_active_)
    {
      ramp_time_ = admittance_params.ramp_time;

      transition_ramp_stiffness_ = admittance_params.transition_ramp_stiffness;
      transition_ramp_damping_ = admittance_params.transition_ramp_damping;
      transition_ramp_mass_ = admittance_params.transition_ramp_mass;

      ramp_goal_reached_ = true;
    }
  }

  void update_admittance_parameters(const AdmittanceParams & admittance_params)
  {
    if (ramp_active_)
    {
      // Ramp params
      target_admittance_params_ = admittance_params;
      ramp_goal_reached_ = false;
      ramp_time_start_ = std::chrono::system_clock::now();

      // Manage axes activation/deactivation ramp
      for (int i = 0; i < 6; i++)
      {
        // Check if change in axis
        if (target_admittance_params_.active_axes[i] != static_cast<bool>(active_axes_(i, i)))
        {
          // Activation - Start from transition values
          if (target_admittance_params_.active_axes[i])
          {
            K_(i, i) = transition_ramp_stiffness_(i);
            D_(i, i) = transition_ramp_damping_(i);
            M_(i, i) = transition_ramp_mass_(i);
            M_inv_ = M_.inverse();
          }
          else
          // Deactivation - End in transition values
          {
            target_admittance_params_.stiffness(i) = transition_ramp_stiffness_(i);
            target_admittance_params_.damping(i) = transition_ramp_damping_(i);
            target_admittance_params_.mass(i) = transition_ramp_mass_(i);
          }
        }
      }

      // Calculate difference to manage ramps
      stiffness_diff_ = target_admittance_params_.stiffness - K_.diagonal();
      damping_diff_ = target_admittance_params_.damping - D_.diagonal();
      mass_diff_ = target_admittance_params_.mass - M_.diagonal();

      // Wrench command
      wrench_command_ = admittance_params.wrench_command;
    }
    else
    {
      // Stiffness
      K_.setZero();
      K_.diagonal() = admittance_params.stiffness;
      // Damping
      D_.setZero();
      D_.diagonal() = admittance_params.damping;
      // Mass
      M_.setZero();
      M_.diagonal() = admittance_params.mass;
      // Mass inverse
      M_inv_ = M_.inverse();
      // Wrench command
      wrench_command_ = admittance_params.wrench_command;
      // Active axes
      active_axes_.setZero();
      for (size_t i = 0; i < 6; i++)
        active_axes_(i, i) = admittance_params.active_axes[i] ? 1.0 : 0.0;
    }
  }

  void manage_ramp()
  {
    // Calculate ramp time
    auto time_now = std::chrono::system_clock::now();
    double elapsed_ramp_time =
      (double)std::chrono::duration_cast<std::chrono::milliseconds>(time_now - ramp_time_start_)
        .count() /
      1000.0;

    // Calculate time factor (percentage of ramp time)
    double time_factor = std::min(elapsed_ramp_time / ramp_time_, 1.0);

    // Stiffness
    K_.diagonal() = target_admittance_params_.stiffness - stiffness_diff_ * (1.0 - time_factor);
    // Damping
    D_.diagonal() = target_admittance_params_.damping - damping_diff_ * (1.0 - time_factor);
    // Mass
    M_.diagonal() = target_admittance_params_.mass - mass_diff_ * (1.0 - time_factor);
    // Mass inverse
    M_inv_ = M_.inverse();

    // Check if ramp goal reached
    if (time_factor == 1.0)
    {
      // Active axes
      active_axes_.setZero();
      for (size_t i = 0; i < 6; i++)
        active_axes_(i, i) = target_admittance_params_.active_axes[i] ? 1.0 : 0.0;

      ramp_goal_reached_ = true;
    }
  }

  Eigen::VectorXd update(
    const Eigen::Isometry3d & base_H_compliance_frame_ref,
    const Eigen::Isometry3d & base_H_compliance_frame_current, const Eigen::VectorXd & wrench,
    double dt)
  {
    // Manage ramp if required
    if (ramp_active_ && !ramp_goal_reached_) manage_ramp();

    // Calculate X
    Eigen::Isometry3d diff =
      base_H_compliance_frame_ref.inverse() * base_H_compliance_frame_current;
    X_.head<3>() = diff.translation();
    Eigen::AngleAxisd aa(diff.rotation());
    X_.tail<3>() = aa.angle() * aa.axis();

    // Wrench
    Eigen::VectorXd wrench_error = active_axes_ * (wrench - wrench_command_);

    // Acceleration
    admittance_acceleration_ = M_inv_ * (wrench_error - D_ * admittance_velocity_ - K_ * X_);
    // Velocity
    admittance_velocity_ = admittance_velocity_ + admittance_acceleration_ * dt;

    // Return the admittance velocity (twist)
    return admittance_velocity_;
  }

private:
  // Admittance control params
  Eigen::VectorXd X_;
  Eigen::Matrix<double, 6, 6> K_;
  Eigen::Matrix<double, 6, 6> D_;
  Eigen::Matrix<double, 6, 6> M_;
  Eigen::Matrix<double, 6, 6> M_inv_;
  Eigen::VectorXd wrench_command_;
  Eigen::Matrix<double, 6, 6> active_axes_;

  Eigen::VectorXd admittance_velocity_;
  Eigen::VectorXd admittance_acceleration_;

  // Ramp
  bool ramp_active_;
  double ramp_time_;

  bool ramp_goal_reached_;
  AdmittanceParams target_admittance_params_;
  std::chrono::system_clock::time_point ramp_time_start_;

  Eigen::VectorXd current_stiffness, stiffness_diff_;
  Eigen::VectorXd current_damping, damping_diff_;
  Eigen::VectorXd current_mass, mass_diff_;

  Eigen::VectorXd transition_ramp_stiffness_;
  Eigen::VectorXd transition_ramp_damping_;
  Eigen::VectorXd transition_ramp_mass_;
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROL_LAW_IMPL_HPP_