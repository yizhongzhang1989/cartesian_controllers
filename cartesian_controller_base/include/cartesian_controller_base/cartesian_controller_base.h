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
/*!\file    cartesian_controller_base.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_CONTROLLER_BASE_H_INCLUDED
#define CARTESIAN_CONTROLLER_BASE_H_INCLUDED

#include <cartesian_controller_base/IKSolver.h>
#include <cartesian_controller_base/SpatialPDController.h>
#include <cartesian_controller_base/Utility.h>
#include <realtime_tools/realtime_publisher.h>

#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <memory>
#include <mutex>
#include <pluginlib/class_loader.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>

#include "ROS2VersionConfig.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cartesian_controller_base
{
/**
 * @brief Base class for each cartesian controller
 *
 * This class implements a common forward dynamics based solver for Cartesian
 * end effector error correction. Different child class controllers will define
 * what this error represents and should call \ref computeJointControlCmds with
 * that error.  The control commands are sent to the hardware with \ref
 * writeJointControlCmds.
 *
 */
class CartesianControllerBase : public controller_interface::ControllerInterface
{
public:
  CartesianControllerBase();
  virtual ~CartesianControllerBase();

  virtual controller_interface::InterfaceConfiguration command_interface_configuration()
    const override;

  virtual controller_interface::InterfaceConfiguration state_interface_configuration()
    const override;

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /**
     * @brief Write joint control commands to the real hardware
     *
     * Depending on the hardware interface used, this is either joint positions
     * or velocities.
     */
  void writeJointControlCmds();

  /**
     * @brief Compute one control step using forward dynamics simulation
     *
     * Check \ref ForwardDynamicsSolver for details.
     *
     * @param error The error to minimize
     * @param period The period for this control cycle
     */
  void computeJointControlCmds(const ctrl::Vector6D & error, const rclcpp::Duration & period);

  /**
     * @brief Display the given vector in the given robot base link
     *
     * @param vector The quantity to transform
     * @param from The reference frame where the quantity was formulated
     *
     * @return The quantity in the robot base frame
     */
  ctrl::Vector6D displayInBaseLink(const ctrl::Vector6D & vector, const std::string & from);

  /**
     * @brief Display the given tensor in the robot base frame
     *
     * @param tensor The quantity to transform
     * @param from The reference frame where the quantity was formulated
     *
     * @return The quantity in the robot base frame
     */
  ctrl::Matrix6D displayInBaseLink(const ctrl::Matrix6D & tensor, const std::string & from);

  /**
     * @brief Display a given vector in a new reference frame
     *
     * The vector is assumed to be given in the robot base frame.
     *
     * @param vector The quantity to transform
     * @param to The reference frame in which to formulate the quantity
     *
     * @return The quantity in the new frame
     */
  ctrl::Vector6D displayInTipLink(const ctrl::Vector6D & vector, const std::string & to);

  /**
     * @brief Check if specified links are part of the robot chain
     *
     * @param s Link to check for existence
     *
     * @return True if existent, false otherwise
     */
  bool robotChainContains(const std::string & s)
  {
    for (const auto & segment : this->m_robot_chain.segments)
    {
      if (segment.getName() == s)
      {
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Whether a kinematic chain has been built yet.
   *
   * In urdf_from_topic mode the chain build is deferred until the first URDF
   * arrives on the topic (after on_configure() returns), so this is false
   * during on_configure().  Derived controllers use it to defer chain-
   * dependent validation (e.g. checking ft_sensor_ref_link / compliance_ref_
   * link is in the chain) from on_configure() to onChainRebuilt(), which the
   * topic callback invokes once the chain is installed.
   *
   * @return True once a kinematic chain + solvers are installed.
   */
  bool chainBuilt() const { return m_chain_built.load(); }

  /**
   * @brief Helper method to check the controller's state during input callbacks
   *
   * @return True if the controller is active, false otherwise
   */
  bool isActive() const { return m_active; };

  /**
   * @brief Apply any pending kinematic-chain swap before each control cycle.
   *
   * Derived controllers MUST call this at the very top of their update()
   * method, before any access to m_ik_solver, m_forward_kinematics_solver, or
   * m_robot_chain.  It is RT-safe: when no swap is pending it is a single
   * acquire-load of an atomic flag; when a swap is pending it briefly takes a
   * mutex (uncontended) and move-assigns shared_ptrs.  See the param callback
   * registered in on_configure() for how new chains are queued.
   */
  void synchronizeKinematics();

  /**
   * @brief Hook called from the RT thread right after a successful chain swap.
   *
   * Derived classes that cache anything derived from the KDL chain (for
   * example, the cartesian_force_controller's m_ft_sensor_transform) should
   * override this to refresh that state.  The default implementation does
   * nothing.
   */
  virtual void onChainRebuilt() {}

  KDL::Chain m_robot_chain;

  std::shared_ptr<KDL::TreeFkSolverPos_recursive> m_forward_kinematics_solver;

  /**
     * @brief Allow users to choose the IK solver type on startup
     */
  std::shared_ptr<pluginlib::ClassLoader<IKSolver>> m_solver_loader;
  std::shared_ptr<IKSolver> m_ik_solver;

  // Dynamic parameters
  std::string m_end_effector_link;
  std::string m_robot_base_link;
  int m_iterations;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    m_joint_state_pos_handles;

  // Names of the actuated joints, kept for live URDF rebuilds.
  std::vector<std::string> m_joint_names;

private:
  // Forward declaration of the staging slot used by the live URDF rebuild
  // pipeline.  Defined in the .cpp file because the helper logic does not need
  // to be visible to derived controllers.
  struct PendingChainSwap;

  /**
   * @brief Build a complete KDL chain + IK/FK solvers from a URDF string.
   *
   * Pure function over the inputs: does not touch any member except the IK
   * plugin loader (which is internally thread-safe in pluginlib).  Used both
   * by on_configure() and by the parameter callback that watches
   * robot_description.
   *
   * @param[in]  robot_description  Full URDF/xacro-expanded XML string
   * @param[in]  robot_base_link    Chain root (must exist in the URDF)
   * @param[in]  end_effector_link  Chain tip (must exist in the URDF)
   * @param[in]  joint_names        Actuated joint names (must all exist in URDF)
   * @param[out] out                Pre-built chain + solvers on success
   * @param[out] error_msg          Human-readable diagnostic on failure
   * @return true on success, false otherwise (out is unspecified on failure)
   */
  bool buildKinematics(const std::string & robot_description,
                       const std::string & robot_base_link,
                       const std::string & end_effector_link,
                       const std::vector<std::string> & joint_names,
                       PendingChainSwap & out, std::string & error_msg);

  /**
   * @brief Parameter callback that watches `robot_description`.
   *
   * Runs on the executor (non-RT) thread.  On a successful URDF parse, queues
   * a chain swap that the RT thread will pick up in synchronizeKinematics().
   * Rejects (returns successful=false) on parse failure, missing chain, or
   * missing actuated joints.
   */
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Latched-topic callback that watches the canonical `robot_description`.
   *
   * Active only when `urdf_from_topic` is true: the controller reads its URDF
   * exclusively from this topic (single source of truth). Builds a new chain
   * and either installs it directly (before activation) or stages it for the
   * RT thread (while active), mirroring onParameterUpdate().
   */
  void robotDescriptionTopicCallback(const std_msgs::msg::String & msg);

  // Live-rebuild plumbing.  m_pending_chain_swap is guarded by
  // m_chain_swap_mutex; m_chain_swap_pending is the RT-side hint that there
  // is something to swap.  m_ik_solver_plugin_name caches the pluginlib name
  // so the param callback can instantiate fresh IKSolver objects.
  std::string m_ik_solver_plugin_name;
  std::atomic<bool> m_chain_swap_pending{false};
  std::mutex m_chain_swap_mutex;
  std::shared_ptr<PendingChainSwap> m_pending_chain_swap;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

  // Topic-sourced URDF (single source of truth).  When m_urdf_from_topic is
  // true the chain build is deferred until the first message on
  // m_robot_description_topic arrives; m_chain_built gates on_activate.
  bool m_urdf_from_topic = {false};
  std::string m_robot_description_topic;
  std::atomic<bool> m_chain_built{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_description_sub;

  /**
     * @brief Stop joint motion when in velocity control
     */
  void stopCurrentMotion()
  {
    for (size_t i = 0; i < m_joint_cmd_vel_handles.size(); ++i)
    {
      m_joint_cmd_vel_handles[i].get().set_value(0.0);
    }
  }

  /**
     * @brief Publish the controller's end-effector pose and twist
     *
     * The data are w.r.t. the specified robot base link.
     * If this function is called after `computeJointControlCmds()` has
     * been called, then the controller's internal state represents the state
     * right after the error computation, and corresponds to the new target
     * state that will be send to the actuators in this control cycle.
     */
  void publishStateFeedback();

  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::PoseStamped>
    m_feedback_pose_publisher;
  realtime_tools::RealtimePublisherSharedPtr<geometry_msgs::msg::TwistStamped>
    m_feedback_twist_publisher;

  std::vector<std::string> m_cmd_interface_types;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    m_joint_cmd_pos_handles;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    m_joint_cmd_vel_handles;

  trajectory_msgs::msg::JointTrajectoryPoint m_simulated_joint_motion;
  SpatialPDController m_spatial_controller;
  ctrl::Vector6D m_cartesian_input;

  // Against multi initialization in multi inheritance scenarios
  bool m_initialized = {false};
  bool m_configured = {false};
  bool m_active = {false};

  // Dynamic parameters
  double m_error_scale;
  std::string m_robot_description;
};

}  // namespace cartesian_controller_base

#endif
