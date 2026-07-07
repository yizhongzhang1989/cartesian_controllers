////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    IKSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/IKSolver.h>

#include <algorithm>
#include <functional>
#include <kdl/framevel.hpp>
#include <kdl/jntarrayvel.hpp>
#include <map>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"

namespace cartesian_controller_base
{
IKSolver::IKSolver() {}

IKSolver::~IKSolver() {}

const KDL::Frame & IKSolver::getEndEffectorPose() const { return m_end_effector_pose; }

const ctrl::Vector6D & IKSolver::getEndEffectorVel() const { return m_end_effector_vel; }

const KDL::JntArray & IKSolver::getPositions() const { return m_current_positions; }

bool IKSolver::setStartState(
  const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > &
    joint_pos_handles)
{
  // Copy into internal buffers.
  for (size_t i = 0; i < joint_pos_handles.size(); ++i)
  {
    // Interface type should be checked by the caller.
    // Add additional plausibility check just in case.
    if (joint_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      m_current_positions(i) = joint_pos_handles[i].get().get_value();
      m_current_velocities(i) = 0.0;
      m_current_accelerations(i) = 0.0;
      m_last_positions(i) = m_current_positions(i);
      m_last_velocities(i) = m_current_velocities(i);
      // Anchor the default null-space rest posture at the engagement pose, so
      // the redundant DOF is held where the arm started unless overridden.
      m_start_positions(i) = m_current_positions(i);
    }
    else
    {
      return false;
    }
  }
  return true;
}

void IKSolver::synchronizeJointPositions(
  const std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > &
    joint_pos_handles)
{
  for (size_t i = 0; i < joint_pos_handles.size(); ++i)
  {
    // Interface type should be checked by the caller.
    // Add additional plausibility check just in case.
    if (joint_pos_handles[i].get().get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      m_current_positions(i) = joint_pos_handles[i].get().get_value();
      m_last_positions(i) = m_current_positions(i);
    }
  }
}

bool IKSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh, const KDL::Chain & chain,
                    const KDL::JntArray & upper_pos_limits, const KDL::JntArray & lower_pos_limits)
{
  // Initialize
  m_handle = nh;
  m_chain = chain;
  m_number_joints = m_chain.getNrOfJoints();
  m_current_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_current_velocities.data = ctrl::VectorND::Zero(m_number_joints);
  m_current_accelerations.data = ctrl::VectorND::Zero(m_number_joints);
  m_last_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_last_velocities.data = ctrl::VectorND::Zero(m_number_joints);
  m_start_positions.data = ctrl::VectorND::Zero(m_number_joints);
  m_upper_pos_limits = upper_pos_limits;
  m_lower_pos_limits = lower_pos_limits;

  // Forward kinematics
  m_fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
  m_fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(m_chain));

  return true;
}

void IKSolver::updateKinematics()
{
  // Pose w. r. t. base
  m_fk_pos_solver->JntToCart(m_current_positions, m_end_effector_pose);

  // Absolute velocity w. r. t. base
  KDL::FrameVel vel;
  m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions, m_current_velocities), vel);
  m_end_effector_vel[0] = vel.deriv().vel.x();
  m_end_effector_vel[1] = vel.deriv().vel.y();
  m_end_effector_vel[2] = vel.deriv().vel.z();
  m_end_effector_vel[3] = vel.deriv().rot.x();
  m_end_effector_vel[4] = vel.deriv().rot.y();
  m_end_effector_vel[5] = vel.deriv().rot.z();
}

void IKSolver::applyJointLimits()
{
  for (int i = 0; i < m_number_joints; ++i)
  {
    if (std::isnan(m_lower_pos_limits(i)) || std::isnan(m_upper_pos_limits(i)))
    {
      // Joint marked as continuous.
      continue;
    }
    m_current_positions(i) =
      std::clamp(m_current_positions(i), m_lower_pos_limits(i), m_upper_pos_limits(i));
  }
}

void IKSolver::declareNullspaceParams()
{
  auto_declare<bool>("solver.nullspace.enabled", false);
  auto_declare<double>("solver.nullspace.stiffness", 1.0);
  auto_declare<double>("solver.nullspace.max_speed", 0.05);
  auto_declare<double>("solver.nullspace.pinv_lambda", 0.01);
  auto_declare<std::vector<double>>("solver.nullspace.rest_posture", std::vector<double>());
  auto_declare<std::vector<double>>("solver.nullspace.joint_weights", std::vector<double>());
}

void IKSolver::readNullspaceParams()
{
  m_ns_enabled = m_handle->get_parameter("solver.nullspace.enabled").as_bool();
  if (!m_ns_enabled)
  {
    return;  // stay in exact stock behaviour; skip the rest of the reads
  }
  m_ns_stiffness = m_handle->get_parameter("solver.nullspace.stiffness").as_double();
  m_ns_max_speed = m_handle->get_parameter("solver.nullspace.max_speed").as_double();
  m_ns_pinv_lambda = m_handle->get_parameter("solver.nullspace.pinv_lambda").as_double();
  m_ns_rest_posture = m_handle->get_parameter("solver.nullspace.rest_posture").as_double_array();
  m_ns_joint_weights = m_handle->get_parameter("solver.nullspace.joint_weights").as_double_array();
}

ctrl::VectorND IKSolver::computeNullspaceJointVelocity(const KDL::Jacobian & jacobian)
{
  const int n = m_number_joints;
  if (!m_ns_enabled)
  {
    return ctrl::VectorND::Zero(n);
  }

  // Resolve the rest posture: an explicit param of matching length, else the
  // configuration captured at engagement (setStartState()).
  ctrl::VectorND q_rest(n);
  if (static_cast<int>(m_ns_rest_posture.size()) == n)
  {
    for (int i = 0; i < n; ++i)
    {
      q_rest(i) = m_ns_rest_posture[i];
    }
  }
  else
  {
    q_rest = m_start_positions.data;
  }

  // Per-joint centering weights (default all ones; e.g. emphasise the elbow).
  ctrl::VectorND w = ctrl::VectorND::Ones(n);
  if (static_cast<int>(m_ns_joint_weights.size()) == n)
  {
    for (int i = 0; i < n; ++i)
    {
      w(i) = m_ns_joint_weights[i];
    }
  }

  // FIRST-ORDER posture-centering velocity in joint space (ported from
  // ikt_core, which applies dq_null = N * bias at the position level):
  //   v0 = Kp * W * (q_rest - q)
  // Being a velocity re-derived from the current configuration every cycle, it
  // carries no momentum and cannot accumulate/run away when the internal model
  // and the real robot diverge -- unlike a second-order (acceleration) form.
  ctrl::VectorND v0 =
    m_ns_stiffness * (w.array() * (q_rest - m_current_positions.data).array()).matrix();

  // Exact, singularity-safe null-space projector from the SVD of the Jacobian.
  //
  //   J = U S V^T   (6 x n),   {v_i} an orthonormal basis of joint space.
  //   N = sum_{i in null} v_i v_i^T  =  I - sum_{i: sigma_i > eps} v_i v_i^T
  //
  // Unlike the damped form  N = I - J^T (J J^T + lambda^2 I)^-1 J, this removes
  // ONLY the well-conditioned Cartesian task directions, so  J*(N v0) == 0 to
  // machine precision (no lambda^2 task-space leakage).  That decoupling is
  // what lets the posture bias be made arbitrarily strong for joint stability
  // WITHOUT dragging the end-effector off target -- the failure mode of the
  // damped projector, where a large null-space speed leaks ~lambda^2/sigma^2 of
  // itself into the task.  Directions with sigma_i <= eps (near a singularity)
  // are intentionally LEFT in the null space, so the redundant motion there is
  // simply not attempted rather than amplified -- keeping the step bounded.
  const auto & J = jacobian.data;  // 6 x n
  Eigen::JacobiSVD<ctrl::MatrixND> svd(J, Eigen::ComputeFullV);
  const ctrl::MatrixND & V = svd.matrixV();               // n x n (orthonormal)
  const ctrl::VectorND & sigma = svd.singularValues();    // min(6,n) values
  const double eps = std::max(m_ns_pinv_lambda, 1e-9);    // singular threshold
  ctrl::MatrixND N = ctrl::MatrixND::Identity(n, n);
  for (int i = 0; i < sigma.size(); ++i)
  {
    if (sigma(i) > eps)
    {
      N.noalias() -= V.col(i) * V.col(i).transpose();  // remove task direction
    }
  }
  ctrl::VectorND qdot_null = N * v0;

  // Safety saturation: bound the null-space joint speed so a mis-tuned gain or
  // an unusual posture error can never command a fast motion.
  if (m_ns_max_speed > 0.0)
  {
    const double nrm = qdot_null.norm();
    if (nrm > m_ns_max_speed)
    {
      qdot_null *= m_ns_max_speed / nrm;
    }
  }
  return qdot_null;
}

}  // namespace cartesian_controller_base
