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
/*!\file    ForwardDynamicsSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/03/24
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/ForwardDynamicsSolver.h>

#include <algorithm>
#include <cmath>
#include <kdl/framevel.hpp>
#include <kdl/jntarrayvel.hpp>
#include <map>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>

/**
 * \class cartesian_controller_base::ForwardDynamicsSolver
 *
 * Users may explicitly specify it with \a "forward_dynamics" as \a ik_solver
 * in their controllers.yaml configuration file for each controller:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *   ros__parameters:
 *     ik_solver: "forward_dynamics"
 *     ...
 *
 *     solver:
 *         ...
 *         forward_dynamics:
 *             link_mass: 0.5
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(cartesian_controller_base::ForwardDynamicsSolver,
                       cartesian_controller_base::IKSolver)

namespace cartesian_controller_base
{
ForwardDynamicsSolver::ForwardDynamicsSolver() {}

ForwardDynamicsSolver::~ForwardDynamicsSolver() {}

trajectory_msgs::msg::JointTrajectoryPoint ForwardDynamicsSolver::getJointControlCmds(
  rclcpp::Duration period, const ctrl::Vector6D & net_force)
{
  double link_mass = m_min.load();
  m_handle->get_parameter(m_params + ".link_mass", link_mass);
  m_min.store(link_mass);

  // Compute joint space inertia matrix with actualized link masses
  buildGenericModel();
  m_jnt_space_inertia_solver->JntToMass(m_current_positions, m_jnt_space_inertia);

  // Compute joint jacobian
  m_jnt_jacobian_solver->JntToJac(m_current_positions, m_jnt_jacobian);

  // Compute joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f) \f$
  m_current_accelerations.data =
    m_jnt_space_inertia.data.inverse() * m_jnt_jacobian.data.transpose() * net_force;

  // Symplectic (semi-implicit) Euler time integration.
  //
  // Update the velocity first (with the per-cycle 10 % global damping
  // against unwanted null-space motion; will cause exponential slow-down
  // without input), then propagate the position using that updated
  // velocity.  Compared with the textbook explicit Euler scheme used
  // upstream (q_new = q + v_old * dt, v_new = v_old + a * dt) this is
  // numerically stable for much higher Cartesian stiffness without
  // changing the steady-state behaviour: when the system has settled
  // (a == 0) both schemes give the same v_new = 0.9 * v_old, and any
  // bounded a produces the same first-order accuracy.  The benefit is
  // that the position update can no longer overshoot a high-frequency
  // mode that has just been reversed by a large a*dt, which is the
  // classic explicit-Euler instability when error_scale * P * K * dt^2
  // (stiffness loop) approaches unity.  See
  // https://en.wikipedia.org/wiki/Semi-implicit_Euler_method
  m_current_velocities.data =
    m_last_velocities.data + m_current_accelerations.data * period.seconds();
  m_current_velocities.data *= 0.9;

  // Redundancy resolution (7-DOF null-space constraint): a FIRST-ORDER posture
  // bias projected into the Cartesian task null space, so the redundant DOF
  // (e.g. the elbow swivel) is driven toward the rest posture without
  // disturbing the end-effector task.  It is re-derived from the measured
  // configuration every cycle and, crucially, is NOT integrated into the
  // persistent velocity state (m_last_velocities below), so it carries no
  // momentum and cannot accumulate / run away when the internal model and the
  // real robot diverge.  No-op when solver.nullspace.enabled is false.
  readNullspaceParams();
  const ctrl::VectorND qdot_null = computeNullspaceJointVelocity(m_jnt_jacobian);

  // Propagate the position with the task velocity PLUS the null-space bias.
  m_current_positions.data =
    m_last_positions.data + (m_current_velocities.data + qdot_null) * period.seconds();

  // Make sure positions stay in allowed margins
  applyJointLimits();

  // Apply results
  trajectory_msgs::msg::JointTrajectoryPoint control_cmd;
  for (int i = 0; i < m_number_joints; ++i)
  {
    control_cmd.positions.push_back(m_current_positions(i));
    control_cmd.velocities.push_back(m_current_velocities(i) + qdot_null(i));

    // Accelerations should be left empty. Those values will be interpreted
    // by most hardware joint drivers as max. tolerated values. As a
    // consequence, the robot will move very slowly.
  }
  control_cmd.time_from_start = period;  // valid for this duration

  // Update for the next cycle.  Persist the TASK velocity only -- the
  // first-order null-space bias must not accumulate.
  m_last_positions = m_current_positions;
  m_last_velocities = m_current_velocities;

  return control_cmd;
}

bool ForwardDynamicsSolver::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
                                 const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
                                 const KDL::JntArray & lower_pos_limits)
{
  IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

  if (!buildGenericModel())
  {
    RCLCPP_ERROR(nh->get_logger(), "Something went wrong in setting up the internal model.");
    return false;
  }

  // Forward dynamics
  m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
  m_jnt_space_inertia_solver.reset(new KDL::ChainDynParam(m_chain, KDL::Vector::Zero()));
  m_jnt_jacobian.resize(m_number_joints);
  m_jnt_space_inertia.resize(m_number_joints);

  // Set the initial value if provided at runtime, else use default value.
  const double link_mass = auto_declare(m_params + ".link_mass", 0.1);
  if (!std::isfinite(link_mass) || link_mass <= 0.0)
  {
    RCLCPP_ERROR(nh->get_logger(), "%s.link_mass must be finite and greater than zero",
                 m_params.c_str());
    return false;
  }
  m_min.store(link_mass);

  // Declare the shared null-space redundancy-resolution parameters
  // (solver.nullspace.*).  Disabled by default -> stock behaviour unchanged.
  declareNullspaceParams();

  RCLCPP_INFO(nh->get_logger(), "Forward dynamics solver initialized");
  RCLCPP_INFO(nh->get_logger(), "Forward dynamics solver has control over %i joints",
              m_number_joints);

  return true;
}

bool ForwardDynamicsSolver::buildGenericModel()
{
  // Set all masses and inertias to minimal (yet stable) values.
  const double link_mass = m_min.load();
  double ip_min = 0.000001;
  for (size_t i = 0; i < m_chain.segments.size(); ++i)
  {
    // Fixed joint segment
    if (m_chain.segments[i].getJoint().getType() == KDL::Joint::None)
    {
      m_chain.segments[i].setInertia(KDL::RigidBodyInertia::Zero());
    }
    else  // relatively moving segment
    {
      m_chain.segments[i].setInertia(
        KDL::RigidBodyInertia(link_mass,                      // mass
                              KDL::Vector::Zero(),            // center of gravity
                              KDL::RotationalInertia(ip_min,  // ixx
                                                     ip_min,  // iyy
                                                     ip_min   // izz
                                                     // ixy, ixy, iyz default to 0.0
                                                     )));
    }
  }

  // Only give the last segment a generic mass and inertia.
  // See https://arxiv.org/pdf/1908.06252.pdf for a motivation for this setting.
  double m = 1;
  double ip = 1;
  m_chain.segments[m_chain.segments.size() - 1].setInertia(
    KDL::RigidBodyInertia(m, KDL::Vector::Zero(), KDL::RotationalInertia(ip, ip, ip)));

  return true;
}

}  // namespace cartesian_controller_base
