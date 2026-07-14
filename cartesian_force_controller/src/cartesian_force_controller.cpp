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
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_force_controller/cartesian_force_controller.h>

#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <Eigen/Geometry>
#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

namespace cartesian_force_controller
{
CartesianForceController::CartesianForceController()
: Base::CartesianControllerBase(), m_hand_frame_control(true)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  // In urdf_from_topic mode the chain is built asynchronously (after this
  // callback returns), so defer the in-chain validation to onChainRebuilt(),
  // which the topic callback invokes once the URDF is installed.  Validate now
  // only when the chain already exists (stock parameter mode).
  if (Base::chainBuilt() && !Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_ft_sensor_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly (null-safe: in topic
  // mode this just stashes the reference name until onChainRebuilt()).
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"), 10,
    std::bind(&CartesianForceController::targetWrenchCallback, this, std::placeholders::_1));

  m_ft_sensor_wrench_subscriber =
    get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_node()->get_name() + std::string("/ft_sensor_wrench"), 10,
      std::bind(&CartesianForceController::ftSensorWrenchCallback, this, std::placeholders::_1));

  // TF buffer/listener so a target wrench's header.frame_id may name ANY frame
  // in the live TF tree (e.g. a shared base_link/world mount frame, or the
  // other arm), not only links of this controller's own kinematic chain.  The
  // buffer-only listener ctor spins its own thread, so wrench-frame resolution
  // does not depend on this controller node being added to an external executor.
  if (!m_tf_buffer)
  {
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  }

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianForceController::update(const rclcpp::Time & time,
                                                                   const rclcpp::Duration & period)
{
  // Apply any pending kinematic-chain swap before touching the IK/FK solvers.
  Base::synchronizeKinematics();

  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  auto internal_period = rclcpp::Duration::from_seconds(0.02);

  // Compute the net force
  ctrl::Vector6D error = computeForceError();

  // Turn Cartesian error into joint motion
  Base::computeJointControlCmds(error, internal_period);

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianForceController::computeForceError()
{
  ctrl::Vector6D target_wrench;

  // Honor the target wrench's ``header.frame_id`` so callers can express the
  // commanded force/torque in whichever frame they choose (e.g. base vs. tool),
  // straight from the WrenchStamped, as ROS intends.  Resolution order:
  //   1. the robot base link           -> used as-is (no rotation),
  //   2. a link of the kinematic chain -> rotated via the controller's own FK,
  //   3. ANY other frame in the TF tree (e.g. a shared base_link/world mount
  //      frame, or the other arm)      -> rotated via a live TF lookup.
  // Only the reference frame's ORIENTATION is applied (the wrench stays applied
  // at the end-effector), matching displayInBaseLink's convention.  When the
  // frame_id is empty (unspecified) or cannot be resolved by any of the above,
  // fall back to the legacy ``hand_frame_control`` flag (true => end-effector
  // frame, false => robot base frame) so existing setups are unaffected.
  //
  // NOTE: ``m_target_wrench_frame`` is written from the subscription callback
  // and read here in the update() thread.  Robot link names are short and hit
  // std::string SSO (no heap allocation), so the worst a data race can do is a
  // one-cycle garbled name, which robotChainContains() then rejects (fall
  // back) -- it never dereferences freed memory.  This matches the benign race
  // already accepted for ``m_target_wrench`` itself.
  const std::string frame = m_target_wrench_frame;

  if (!frame.empty() && frame == Base::m_robot_base_link)
  {
    // Already expressed in the base frame -- use as-is (no rotation).
    target_wrench = m_target_wrench;
  }
  else if (!frame.empty() && Base::robotChainContains(frame))
  {
    // A link of this controller's own chain: rotate via the controller's FK
    // (fast path, no TF dependency).
    target_wrench = Base::displayInBaseLink(m_target_wrench, frame);
  }
  else if (!frame.empty() && tryRotateWrenchFromTf(frame, m_target_wrench, target_wrench))
  {
    // Any other frame in the live TF tree (e.g. a shared base_link/world mount
    // frame, or the other arm): rotated into the base frame by the helper.
  }
  else
  {
    if (!frame.empty())
    {
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "target_wrench frame_id '"
          << frame << "' is neither the base link, a link of the kinematic chain from "
          << Base::m_robot_base_link << " to " << Base::m_end_effector_link
          << ", nor resolvable via TF; falling back to hand_frame_control");
    }
    m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

    if (m_hand_frame_control)  // Assume end-effector frame by convention
    {
      target_wrench = Base::displayInBaseLink(m_target_wrench, Base::m_end_effector_link);
    }
    else  // Default to robot base frame
    {
      target_wrench = m_target_wrench;
    }
  }

  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench, m_new_ft_sensor_ref) + target_wrench;
}

bool CartesianForceController::tryRotateWrenchFromTf(const std::string & frame,
                                                     const ctrl::Vector6D & wrench_in_frame,
                                                     ctrl::Vector6D & wrench_in_base)
{
  if (!m_tf_buffer)
  {
    return false;
  }

  ctrl::Matrix3D R_base_frame;
  try
  {
    // Orientation of ``frame`` expressed in the robot base link.  TimePointZero
    // = latest available transform; the wrench is a steady-state command, so
    // exact time synchronization is unnecessary.
    const geometry_msgs::msg::TransformStamped tf =
      m_tf_buffer->lookupTransform(Base::m_robot_base_link, frame, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    R_base_frame = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();

    // Cache for this frame so a transient lookup miss on a later cycle reuses
    // the last good rotation instead of snapping back to the tool frame.
    m_tf_rot_cache = R_base_frame;
    m_tf_rot_cache_frame = frame;
    m_tf_rot_cached = true;
  }
  catch (const tf2::TransformException & ex)
  {
    if (m_tf_rot_cached && m_tf_rot_cache_frame == frame)
    {
      R_base_frame = m_tf_rot_cache;  // reuse last good rotation for this frame
    }
    else
    {
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                  "target_wrench frame_id '"
                                    << frame << "': TF lookup to '" << Base::m_robot_base_link
                                    << "' failed (" << ex.what() << ")");
      return false;
    }
  }

  // Rotate force (linear, first 3) and torque (angular, last 3) into the base
  // frame.  Only the orientation is applied -- consistent with
  // displayInBaseLink's free-vector convention (the wrench stays applied at the
  // end-effector; the frame_id selects the reference orientation).
  wrench_in_base.head<3>() = R_base_frame * wrench_in_frame.head<3>();
  wrench_in_base.tail<3>() = R_base_frame * wrench_in_frame.tail<3>();
  return true;
}

void CartesianForceController::setFtSensorReferenceFrame(const std::string & new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // In urdf_from_topic mode the chain (and therefore the solvers) is not built
  // until the first URDF arrives on the topic, which happens *after*
  // on_configure() returns.  Stash the reference name now and let
  // onChainRebuilt() recompute the cached transform once the chain is
  // installed -- calling JntToCart() here would dereference null solvers.
  if (!Base::m_ik_solver || !Base::m_forward_kinematics_solver)
  {
    return;
  }

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_ik_solver->getPositions());

  KDL::Frame sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, sensor_ref, m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(jnts, new_sensor_ref, m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
}

void CartesianForceController::onChainRebuilt()
{
  // The base class has already installed the new chain and FK solver.  We
  // need to (a) sanity-check that ft_sensor_ref_link is still part of the
  // chain and (b) recompute the cached static transform from the FT sensor
  // frame to whichever frame we report wrenches in (end_effector_link for the
  // pure force controller, compliance_ref_link for the compliance controller;
  // both are tracked in m_new_ft_sensor_ref).
  if (!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(
      get_node()->get_logger(),
      "After URDF rebuild: ft_sensor_ref_link='"
        << m_ft_sensor_ref_link << "' is no longer part of the kinematic chain from "
        << Base::m_robot_base_link << " to " << Base::m_end_effector_link
        << "; FT sensor transform will be incorrect until a valid URDF is published");
    return;
  }
  if (!m_new_ft_sensor_ref.empty() && !Base::robotChainContains(m_new_ft_sensor_ref))
  {
    RCLCPP_ERROR_STREAM(
      get_node()->get_logger(),
      "After URDF rebuild: cached reference frame '"
        << m_new_ft_sensor_ref << "' is no longer part of the kinematic chain");
    return;
  }
  setFtSensorReferenceFrame(m_new_ft_sensor_ref.empty() ? Base::m_end_effector_link
                                                        : m_new_ft_sensor_ref);
}

void CartesianForceController::targetWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in target wrench. Ignoring input.");
    return;
  }

  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;

  // Remember the frame the wrench was expressed in so computeForceError() can
  // rotate it into the base frame (honoring header.frame_id).
  m_target_wrench_frame = wrench->header.frame_id;
}

void CartesianForceController::ftSensorWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;
  tmp[3] = wrench->wrench.torque.x;
  tmp[4] = wrench->wrench.torque.y;
  tmp[5] = wrench->wrench.torque.z;

  // Compute how the measured wrench appears in the frame of interest.
  tmp = m_ft_sensor_transform * tmp;

  m_ft_sensor_wrench[0] = tmp[0];
  m_ft_sensor_wrench[1] = tmp[1];
  m_ft_sensor_wrench[2] = tmp[2];
  m_ft_sensor_wrench[3] = tmp[3];
  m_ft_sensor_wrench[4] = tmp[4];
  m_ft_sensor_wrench[5] = tmp[5];
}

}  // namespace cartesian_force_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_force_controller::CartesianForceController,
                       controller_interface::ControllerInterface)
