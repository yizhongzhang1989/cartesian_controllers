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
/*!\file    cartesian_controller_base.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/cartesian_controller_base.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>

#include <cmath>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <mutex>
#include <std_msgs/msg/string.hpp>
#include <utility>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist_stamped__struct.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace cartesian_controller_base
{
// Internal staging slot for the live URDF rebuild pipeline.  Forward declared
// inside CartesianControllerBase; defined here so that derived controllers do
// not transitively depend on its layout.  Pre-built on the executor thread,
// move-assigned into the controller's live members from the RT update path.
struct CartesianControllerBase::PendingChainSwap
{
  std::string robot_description;
  KDL::Chain robot_chain;
  std::shared_ptr<IKSolver> ik_solver;
  std::shared_ptr<KDL::TreeFkSolverPos_recursive> forward_kinematics_solver;
};

CartesianControllerBase::CartesianControllerBase() {}

CartesianControllerBase::~CartesianControllerBase() = default;

controller_interface::InterfaceConfiguration
CartesianControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * m_cmd_interface_types.size());
  for (const auto & type : m_cmd_interface_types)
  {
    for (const auto & joint_name : m_joint_names)
    {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
CartesianControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size());  // Only position
  for (const auto & joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
  }
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_init()
{
  if (!m_initialized)
  {
    auto_declare<std::string>("ik_solver", "forward_dynamics");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_base_link", "");
    auto_declare<std::string>("end_effector_link", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<double>("solver.error_scale", 1.0);
    auto_declare<int>("solver.iterations", 1);
    auto_declare<bool>("solver.publish_state_feedback", false);
    // Topic-sourced URDF (single source of truth).  Default false keeps the
    // stock behaviour (read the robot_description parameter once).  When true,
    // the controller subscribes to robot_description_topic and uses ONLY that.
    auto_declare<bool>("urdf_from_topic", false);
    auto_declare<std::string>("robot_description_topic", "/cartesian/robot_description");
    m_initialized = true;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  if (m_configured)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Load user specified inverse kinematics solver
  m_ik_solver_plugin_name = get_node()->get_parameter("ik_solver").as_string();
  m_solver_loader.reset(new pluginlib::ClassLoader<IKSolver>(
    "cartesian_controller_base", "cartesian_controller_base::IKSolver"));

  // URDF source selection.  Default (stock): read the robot_description
  // parameter once.  When urdf_from_topic=true the controller instead reads its
  // URDF EXCLUSIVELY from a latched robot_description topic (single source of
  // truth), deferring the kinematic-chain build until the first URDF arrives.
  m_urdf_from_topic = get_node()->get_parameter("urdf_from_topic").as_bool();
  m_robot_description_topic =
    get_node()->get_parameter("robot_description_topic").as_string();

  // Static configuration needed regardless of the URDF source (these define the
  // claimed hardware interfaces and the chain endpoints; they do NOT change on a
  // live URDF update).
  m_robot_base_link = get_node()->get_parameter("robot_base_link").as_string();
  if (m_robot_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_end_effector_link = get_node()->get_parameter("end_effector_link").as_string();
  if (m_end_effector_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get names of actuated joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (m_urdf_from_topic)
  {
    // Defer the chain build: subscribe to the canonical URDF (latched) and let
    // robotDescriptionTopicCallback() build the chain on the first message.
    // on_activate is gated on m_chain_built.
    rclcpp::QoS qos(1);
    qos.transient_local().reliable().keep_last(1);
    m_robot_description_sub = get_node()->create_subscription<std_msgs::msg::String>(
      m_robot_description_topic, qos,
      std::bind(&CartesianControllerBase::robotDescriptionTopicCallback, this,
                std::placeholders::_1));
    m_chain_built.store(false);
    RCLCPP_INFO(get_node()->get_logger(),
                "urdf_from_topic=true: deferring kinematics until a URDF is "
                "received on '%s' (single source of truth)",
                m_robot_description_topic.c_str());
  }
  else
  {
    // Stock path: read the robot_description parameter once and build now.
#if defined CARTESIAN_CONTROLLERS_JAZZY
    m_robot_description = this->get_robot_description();
#else
    m_robot_description = get_node()->get_parameter("robot_description").as_string();
#endif
    if (m_robot_description.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Build the kinematic chain and instantiate/initialize the solvers.  The
    // same helper is reused by the parameter/topic callbacks that watch
    // robot_description, so that live URDF updates take effect without an
    // unload/load cycle.
    PendingChainSwap initial;
    std::string build_err;
    if (!buildKinematics(m_robot_description, m_robot_base_link, m_end_effector_link,
                         m_joint_names, initial, build_err))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", build_err.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    m_robot_chain = std::move(initial.robot_chain);
    m_ik_solver = std::move(initial.ik_solver);
    m_forward_kinematics_solver = std::move(initial.forward_kinematics_solver);
    m_chain_built.store(true);
  }

  m_iterations = get_node()->get_parameter("solver.iterations").as_int();
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();

  // Initialize Cartesian pd controllers
  m_spatial_controller.init(get_node());

  // Check command interfaces.
  // We support position, velocity, or both.
  m_cmd_interface_types = get_node()->get_parameter("command_interfaces").as_string_array();
  if (m_cmd_interface_types.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  for (const auto & type : m_cmd_interface_types)
  {
    if (type != hardware_interface::HW_IF_POSITION && type != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unsupported command interface: %s. Choose position or velocity", type.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Controller-internal state publishing
  m_feedback_pose_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string(get_node()->get_name()) + "/current_pose", 3));

  m_feedback_twist_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        std::string(get_node()->get_name()) + "/current_twist", 3));

  m_configured = true;

  // Register the parameter callback AFTER the controller has been fully
  // configured.  This ensures the initial robot_description set (which happens
  // implicitly during controller load) does not trigger a redundant rebuild,
  // and that the first time we react to a robot_description update we already
  // have a valid set of cached members to swap out.
  m_param_callback_handle = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    { return this->onParameterUpdate(params); });

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  stopCurrentMotion();

  if (m_active)
  {
    m_joint_cmd_pos_handles.clear();
    m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool CartesianControllerBase::buildKinematics(const std::string & robot_description,
                                              const std::string & robot_base_link,
                                              const std::string & end_effector_link,
                                              const std::vector<std::string> & joint_names,
                                              PendingChainSwap & out, std::string & error_msg)
{
  // Serialize chain construction across ALL controllers in this process.
  //
  // urdf::Model::initString() loads URDF parser plugins through pluginlib,
  // whose ClassLoader is NOT thread-safe.  In urdf_from_topic mode every
  // Cartesian controller subscribes to the same latched canonical URDF, so
  // their topic callbacks fire buildKinematics() concurrently on the
  // controller_manager's multi-threaded executor.  Unserialized, the parallel
  // initString() calls race and some spuriously fail ("Failed to parse urdf
  // model"), leaving those controllers with no kinematic chain.  A function-
  // local static mutex is shared by every instance, so it serializes parsing
  // process-wide.  This runs off the RT path (on_configure / occasional live
  // URDF updates), so the brief lock is negligible.
  static std::mutex s_build_mutex;
  std::lock_guard<std::mutex> build_lock(s_build_mutex);

  if (robot_description.empty())
  {
    error_msg = "robot_description is empty";
    return false;
  }

  urdf::Model robot_model;
  if (!robot_model.initString(robot_description))
  {
    error_msg = "Failed to parse urdf model from 'robot_description'";
    return false;
  }

  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    error_msg = "Failed to parse KDL tree from urdf model";
    return false;
  }

  KDL::Chain robot_chain;
  if (!robot_tree.getChain(robot_base_link, end_effector_link, robot_chain))
  {
    error_msg = "Failed to parse robot chain from urdf model. Do robot_base_link='" +
                robot_base_link + "' and end_effector_link='" + end_effector_link + "' exist?";
    return false;
  }

  // Parse joint limits.  Joint set is fixed for the controller's lifetime
  // (changing it would invalidate the hardware-interface bindings claimed at
  // on_activate), so any mismatch is a hard failure.
  KDL::JntArray upper_pos_limits(joint_names.size());
  KDL::JntArray lower_pos_limits(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    const auto joint = robot_model.getJoint(joint_names[i]);
    if (!joint)
    {
      error_msg = "Joint '" + joint_names[i] + "' does not appear in robot_description";
      return false;
    }
    if (joint->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits(i) = std::nan("0");
      lower_pos_limits(i) = std::nan("0");
    }
    else
    {
      // Non-existent urdf limits are zero initialized
      upper_pos_limits(i) = joint->limits->upper;
      lower_pos_limits(i) = joint->limits->lower;
    }
  }

  // Instantiate a *fresh* IK solver.  Re-initializing the existing instance
  // would not be safe vs. the RT update() thread, which may concurrently be
  // calling getJointControlCmds() on it.  pluginlib's createSharedInstance is
  // re-entrant for repeated use of the same loader.
  std::shared_ptr<IKSolver> ik_solver;
  try
  {
    ik_solver = m_solver_loader->createSharedInstance(m_ik_solver_plugin_name);
  }
  catch (pluginlib::PluginlibException & ex)
  {
    error_msg = std::string("Failed to instantiate IK solver plugin '") +
                m_ik_solver_plugin_name + "': " + ex.what();
    return false;
  }
  if (!ik_solver->init(get_node(), robot_chain, upper_pos_limits, lower_pos_limits))
  {
    error_msg = "IK solver init() failed for plugin '" + m_ik_solver_plugin_name + "'";
    return false;
  }

  // Build a TreeFkSolver over a one-chain tree.  This is what the base class
  // uses for displayInBaseLink() and friends.
  KDL::Tree tmp("not_relevant");
  tmp.addChain(robot_chain, "not_relevant");
  auto fk_solver = std::make_shared<KDL::TreeFkSolverPos_recursive>(tmp);

  out.robot_description = robot_description;
  out.robot_chain = std::move(robot_chain);
  out.ik_solver = std::move(ik_solver);
  out.forward_kinematics_solver = std::move(fk_solver);
  return true;
}

rcl_interfaces::msg::SetParametersResult CartesianControllerBase::onParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Validate parameters that have hard safety constraints and rebuild the
  // kinematic chain when robot_description changes. Other parameters flow
  // through unchanged and are read live by their owning components.
  for (const auto & param : parameters)
  {
    if (param.get_name() == "solver.forward_dynamics.link_mass")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
          !std::isfinite(param.as_double()) || param.as_double() <= 0.0)
      {
        result.successful = false;
        result.reason =
          "solver.forward_dynamics.link_mass must be a finite double greater than zero";
        return result;
      }
      continue;
    }
    if (param.get_name() == "solver.forward_dynamics.link_inertia")
    {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
          !std::isfinite(param.as_double()) || param.as_double() <= 0.0)
      {
        result.successful = false;
        result.reason =
          "solver.forward_dynamics.link_inertia must be a finite double greater than zero";
        return result;
      }
      continue;
    }
    if (param.get_name() != "robot_description")
    {
      continue;
    }
    if (m_urdf_from_topic)
    {
      // The latched robot_description topic is the single source of truth;
      // ignore any parameter-based URDF so the two cannot diverge.
      continue;
    }
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    {
      result.successful = false;
      result.reason = "robot_description must be a string";
      return result;
    }
    const std::string new_urdf = param.as_string();
    if (new_urdf == m_robot_description)
    {
      // No-op update relative to the *currently-installed* chain.  We
      // must, however, drop any previously-staged swap, otherwise a
      // sequence like push(U1) -> push(U0) (with U0 == current cached)
      // would leave U1 pending, and the next update() cycle would
      // resurrect U1 long after the operator intended to revert.
      if (m_chain_swap_pending.exchange(false, std::memory_order_acq_rel))
      {
        std::lock_guard<std::mutex> lock(m_chain_swap_mutex);
        m_pending_chain_swap.reset();
        RCLCPP_INFO(get_node()->get_logger(),
                    "robot_description matches current chain; dropped "
                    "previously-staged swap");
      }
      continue;
    }

    PendingChainSwap pending;
    std::string err;
    if (!buildKinematics(new_urdf, m_robot_base_link, m_end_effector_link, m_joint_names,
                         pending, err))
    {
      result.successful = false;
      result.reason =
        "Rejected robot_description update: " + err + " (existing chain kept intact)";
      RCLCPP_WARN(get_node()->get_logger(), "%s", result.reason.c_str());
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(m_chain_swap_mutex);
      m_pending_chain_swap = std::make_shared<PendingChainSwap>(std::move(pending));
    }
    m_chain_swap_pending.store(true, std::memory_order_release);
    RCLCPP_INFO(get_node()->get_logger(),
                "Queued kinematic chain rebuild from new robot_description (%zu bytes)",
                new_urdf.size());
  }

  return result;
}

void CartesianControllerBase::robotDescriptionTopicCallback(const std_msgs::msg::String & msg)
{
  if (msg.data.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring empty robot_description on '%s'",
                m_robot_description_topic.c_str());
    return;
  }
  if (msg.data == m_robot_description)
  {
    return;  // no-op relative to the currently-installed chain
  }

  PendingChainSwap pending;
  std::string err;
  if (!buildKinematics(msg.data, m_robot_base_link, m_end_effector_link, m_joint_names,
                       pending, err))
  {
    RCLCPP_WARN(get_node()->get_logger(),
                "Rejected robot_description from topic '%s': %s (existing chain kept)",
                m_robot_description_topic.c_str(), err.c_str());
    return;
  }

  // Before activation the RT update() loop is not running, so it is safe to
  // install the new chain directly -- this makes m_ik_solver valid in time for
  // on_activate().  While active, stage it for the RT thread instead (the same
  // path onParameterUpdate uses), so synchronizeKinematics() swaps it in.
  if (!m_active)
  {
    m_robot_description = std::move(pending.robot_description);
    m_robot_chain = std::move(pending.robot_chain);
    m_ik_solver = std::move(pending.ik_solver);
    m_forward_kinematics_solver = std::move(pending.forward_kinematics_solver);
    const bool first = !m_chain_built.exchange(true);
    onChainRebuilt();
    RCLCPP_INFO(get_node()->get_logger(),
                "Installed kinematic chain from topic URDF (%zu bytes)%s",
                m_robot_description.size(), first ? " [first]" : "");
  }
  else
  {
    {
      std::lock_guard<std::mutex> lock(m_chain_swap_mutex);
      m_pending_chain_swap = std::make_shared<PendingChainSwap>(std::move(pending));
    }
    m_chain_swap_pending.store(true, std::memory_order_release);
    m_chain_built.store(true);
    RCLCPP_INFO(get_node()->get_logger(),
                "Queued kinematic chain rebuild from topic URDF (%zu bytes)",
                msg.data.size());
  }
}

void CartesianControllerBase::synchronizeKinematics()
{
  // Fast path: no pending swap.  Acquire-load pairs with the release-store in
  // onParameterUpdate(), so once we see the flag set we are guaranteed to see
  // the m_pending_chain_swap write that preceded it.
  if (!m_chain_swap_pending.load(std::memory_order_acquire))
  {
    return;
  }

  std::shared_ptr<PendingChainSwap> pending;
  {
    std::lock_guard<std::mutex> lock(m_chain_swap_mutex);
    pending = std::move(m_pending_chain_swap);
  }
  // Clear the flag *after* we have taken ownership of the staged swap so a
  // concurrent param update cannot see a stale flag and skip its own publish.
  m_chain_swap_pending.store(false, std::memory_order_release);

  if (!pending)
  {
    return;
  }

  // Install the new chain + solvers atomically from the RT thread's point of
  // view: each assignment is a single pointer/value replacement.  The old
  // shared_ptrs are released here, which means their destructors run on the
  // RT thread.  KDL solvers and chains have small destructors (deleting a
  // handful of JntArrays and KDL::Solver objects), so this is acceptable for
  // a typical 100-500 Hz controller cycle.  If a hard-RT use case ever needs
  // it, the dropped shared_ptrs can be pushed to a disposal queue drained by
  // a non-RT thread instead.
  m_robot_description = std::move(pending->robot_description);
  m_robot_chain = std::move(pending->robot_chain);
  m_ik_solver = std::move(pending->ik_solver);
  m_forward_kinematics_solver = std::move(pending->forward_kinematics_solver);

  // Give derived controllers a chance to refresh anything that depends on the
  // chain (e.g. cartesian_force_controller's cached FT-sensor transform).
  onChainRebuilt();

  RCLCPP_INFO(get_node()->get_logger(),
              "Kinematic chain swapped in from updated robot_description");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  if (m_active)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // With a topic-sourced URDF the first message may have arrived as a staged
  // swap; install it now.  Then require a valid chain before activating so we
  // never dereference a null solver (gated activation, single source of truth).
  synchronizeKinematics();
  if (!m_chain_built.load() || !m_ik_solver)
  {
    const std::string where =
      m_urdf_from_topic ? (" on '" + m_robot_description_topic + "'") : std::string();
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot activate: no robot_description received yet%s. Is the "
                 "canonical URDF being published (aux_frame_manager)?",
                 where.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get command handles.
  for (const auto & type : m_cmd_interface_types)
  {
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, m_joint_names, type,
                                                      (type == hardware_interface::HW_IF_POSITION)
                                                        ? m_joint_cmd_pos_handles
                                                        : m_joint_cmd_vel_handles))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.",
                   m_joint_names.size(), type.c_str(),
                   (type == hardware_interface::HW_IF_POSITION) ? m_joint_cmd_pos_handles.size()
                                                                : m_joint_cmd_vel_handles.size());
      return CallbackReturn::ERROR;
    }
  }

  // Get state handles.
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, m_joint_names,
                                                    hardware_interface::HW_IF_POSITION,
                                                    m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(), hardware_interface::HW_IF_POSITION,
                 m_joint_state_pos_handles.size());
    return CallbackReturn::ERROR;
  }

  // Copy joint state to internal simulation
  if (!m_ik_solver->setStartState(m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set start state");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  };
  m_ik_solver->updateKinematics();

  // Clear any cached per-axis last-error in the spatial PD so the first
  // real control cycle after (re)activation cannot synthesize a spurious
  // D-term kick from a stale error sample (e.g. an FT bias measured
  // during the previous engagement, or zero on the very first activation
  // when the error vector is suddenly non-zero).  Without this, with
  // D>0 the discrete (error - m_last_p_error) / dt produces a one-shot
  // command pulse of magnitude  D * |error_now| / dt  on cycle #1.
  m_spatial_controller.reset();

  // Provide safe command buffers with starting where we are
  computeJointControlCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  writeJointControlCmds();

  m_active = true;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  stopCurrentMotion();

  if (m_active)
  {
    m_joint_cmd_pos_handles.clear();
    m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void CartesianControllerBase::writeJointControlCmds()
{
  if (get_node()->get_parameter("solver.publish_state_feedback").as_bool())
  {
    publishStateFeedback();
  }

  auto nan_in = [](const auto & values) -> bool
  {
    for (const auto & value : values)
    {
      if (std::isnan(value))
      {
        return true;
      }
    }
    return false;
  };

  if (nan_in(m_simulated_joint_motion.positions) || nan_in(m_simulated_joint_motion.velocities))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "NaN detected in internal model. It's unlikely to recover from this. Shutting down.");
    get_node()->shutdown();
    return;
  }

  // Write all available types.
  for (const auto & type : m_cmd_interface_types)
  {
    if (type == hardware_interface::HW_IF_POSITION)
    {
      for (size_t i = 0; i < m_joint_names.size(); ++i)
      {
        m_joint_cmd_pos_handles[i].get().set_value(m_simulated_joint_motion.positions[i]);
      }
    }
    if (type == hardware_interface::HW_IF_VELOCITY)
    {
      for (size_t i = 0; i < m_joint_names.size(); ++i)
      {
        m_joint_cmd_vel_handles[i].get().set_value(m_simulated_joint_motion.velocities[i]);
      }
    }
  }
}

void CartesianControllerBase::computeJointControlCmds(const ctrl::Vector6D & error,
                                                      const rclcpp::Duration & period)
{
  // PD controlled system input
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();
  m_cartesian_input = m_error_scale * m_spatial_controller(error, period);

  // Simulate one step forward
  m_simulated_joint_motion = m_ik_solver->getJointControlCmds(period, m_cartesian_input);

  m_ik_solver->updateKinematics();
}

ctrl::Vector6D CartesianControllerBase::displayInBaseLink(const ctrl::Vector6D & vector,
                                                          const std::string & from)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, from);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

ctrl::Matrix6D CartesianControllerBase::displayInBaseLink(const ctrl::Matrix6D & tensor,
                                                          const std::string & from)
{
  // Get rotation to base
  KDL::Frame R_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), R_kdl, from);

  // Adjust format
  ctrl::Matrix3D R;
  R << R_kdl.M.data[0], R_kdl.M.data[1], R_kdl.M.data[2], R_kdl.M.data[3], R_kdl.M.data[4],
    R_kdl.M.data[5], R_kdl.M.data[6], R_kdl.M.data[7], R_kdl.M.data[8];

  // Treat diagonal blocks as individual 2nd rank tensors.
  // Display in base frame.
  ctrl::Matrix6D tmp = ctrl::Matrix6D::Zero();
  tmp.topLeftCorner<3, 3>() = R * tensor.topLeftCorner<3, 3>() * R.transpose();
  tmp.bottomRightCorner<3, 3>() = R * tensor.bottomRightCorner<3, 3>() * R.transpose();

  return tmp;
}

ctrl::Vector6D CartesianControllerBase::displayInTipLink(const ctrl::Vector6D & vector,
                                                         const std::string & to)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, to);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M.Inverse() * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

void CartesianControllerBase::publishStateFeedback()
{
  // End-effector pose
  auto pose = m_ik_solver->getEndEffectorPose();
  if (m_feedback_pose_publisher->trylock())
  {
    m_feedback_pose_publisher->msg_.header.stamp = get_node()->now();
    m_feedback_pose_publisher->msg_.header.frame_id = m_robot_base_link;
    m_feedback_pose_publisher->msg_.pose.position.x = pose.p.x();
    m_feedback_pose_publisher->msg_.pose.position.y = pose.p.y();
    m_feedback_pose_publisher->msg_.pose.position.z = pose.p.z();

    pose.M.GetQuaternion(m_feedback_pose_publisher->msg_.pose.orientation.x,
                         m_feedback_pose_publisher->msg_.pose.orientation.y,
                         m_feedback_pose_publisher->msg_.pose.orientation.z,
                         m_feedback_pose_publisher->msg_.pose.orientation.w);

    m_feedback_pose_publisher->unlockAndPublish();
  }

  // End-effector twist
  auto twist = m_ik_solver->getEndEffectorVel();
  if (m_feedback_twist_publisher->trylock())
  {
    m_feedback_twist_publisher->msg_.header.stamp = get_node()->now();
    m_feedback_twist_publisher->msg_.header.frame_id = m_robot_base_link;
    m_feedback_twist_publisher->msg_.twist.linear.x = twist[0];
    m_feedback_twist_publisher->msg_.twist.linear.y = twist[1];
    m_feedback_twist_publisher->msg_.twist.linear.z = twist[2];
    m_feedback_twist_publisher->msg_.twist.angular.x = twist[3];
    m_feedback_twist_publisher->msg_.twist.angular.y = twist[4];
    m_feedback_twist_publisher->msg_.twist.angular.z = twist[5];

    m_feedback_twist_publisher->unlockAndPublish();
  }
}

}  // namespace cartesian_controller_base
