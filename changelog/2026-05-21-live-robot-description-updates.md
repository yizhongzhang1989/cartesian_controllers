# 2026-05-21 — Live `robot_description` updates

### Motivation

Use cases such as **online tool-frame calibration**, **TCP / FT-sensor frame
tuning** and **swapping end-effectors at runtime** require the controller's
internal kinematic chain (the KDL tree, IK solver and FK solver built in
`on_configure`) to reflect the latest URDF without taking the controller
offline. Upstream only reads `robot_description` once during `on_configure`,
so any change requires a full deactivate / cleanup / configure / activate
cycle. Doing that on a real robot is disruptive and, in some launch
arrangements, requires re-spawning the controller from scratch.

This patch makes the base controller react to live parameter updates of
`robot_description` and swap in the new chain on the next real-time
control cycle.

### Concurrency model

The KDL chain, the `IKSolver` instance and the `KDL::TreeFkSolverPos_recursive`
are read from the RT thread (`update()`) every cycle. Rebuilding them is
**not** RT-safe (URDF parsing, plugin loader, allocations), so the build runs
on the executor thread and is published to the RT thread with a tiny critical
section:

1. Parameter callback fires on the executor thread (driven by the
   `SetParameters` service that ROS 2 Humble dispatches from the spinner).
2. It calls `buildKinematics(...)` to produce a `PendingChainSwap` (fresh
   `robot_description`, `KDL::Chain`, fresh `IKSolver` shared pointer, fresh
   `TreeFkSolverPos_recursive`). All allocation, parsing and plugin loading
   happen here, off the RT thread.
3. Under `m_chain_swap_mutex` the staged `PendingChainSwap` is stored, then
   `m_chain_swap_pending.store(true, std::memory_order_release)` signals the
   RT thread.
4. At the very top of each `update()`, the controller calls
   `Base::synchronizeKinematics()`, which does a lock-free
   `m_chain_swap_pending.load(std::memory_order_acquire)` fast path. When
   set, it briefly takes the mutex, move-assigns the four members
   (`m_robot_description`, `m_robot_chain`, `m_ik_solver`,
   `m_forward_kinematics_solver`), clears the atomic flag with release
   ordering, and invokes the `onChainRebuilt()` hook.

The mutex is only ever taken outside the RT loop **except** for the swap
itself, which is a handful of move-assignments. The fast path on every cycle
is one relaxed-cost atomic load.

### Pending-swap hygiene (`dropped previously-staged swap`)

If a controller is **inactive** when the URDF changes, no `update()` cycle
runs, so the swap stays staged. If a subsequent push then sets
`robot_description` back to whatever is currently installed in the controller
(for example, the operator pushes an offset then reverts it before the
controller is ever activated), the second callback would otherwise leave the
first push staged. The next activation would then resurrect a chain the
operator had already reverted.

The callback therefore explicitly clears `m_pending_chain_swap` when the
incoming URDF matches `m_robot_description`, logging
`robot_description matches current chain; dropped previously-staged swap`.

### `onChainRebuilt()` hook

`cartesian_force_controller` keeps a cached `FT_sensor_ref` frame derived from
the chain. After a swap, that cached frame is no longer guaranteed to refer
to a link that still exists. `onChainRebuilt()` lets each derived controller
revalidate and refresh chain-dependent state:

- `cartesian_force_controller::onChainRebuilt()` checks that
  `m_ft_sensor_ref_link` (and `m_new_ft_sensor_ref` if non-empty) are still
  in the new chain via `Base::robotChainContains(...)`, then re-applies
  `setFtSensorReferenceFrame(...)`.
- Motion and compliance controllers inherit the default no-op.

### Files changed

#### `cartesian_controller_base/include/cartesian_controller_base/cartesian_controller_base.h`

- Added includes: `<atomic>`, `<mutex>`,
  `<rcl_interfaces/msg/set_parameters_result.hpp>`.
- Declared the destructor out-of-line (was an inline `{}` — conflicts with a
  forward-declared `PendingChainSwap` member).
- Added `void synchronizeKinematics();` and
  `virtual void onChainRebuilt() {}` to `protected`.
- Moved `m_joint_names` from `private` to `protected` (needed by
  `buildKinematics` and by derived controllers' hooks).
- Added in `private`:
  - forward declaration `struct PendingChainSwap;`
  - `bool buildKinematics(const std::string &, const std::string &, const std::string &, const std::vector<std::string> &, PendingChainSwap &, std::string &);`
  - `rcl_interfaces::msg::SetParametersResult onParameterUpdate(const std::vector<rclcpp::Parameter> &);`
  - members: `std::string m_ik_solver_plugin_name;`,
    `std::atomic<bool> m_chain_swap_pending{false};`,
    `std::mutex m_chain_swap_mutex;`,
    `std::shared_ptr<PendingChainSwap> m_pending_chain_swap;`,
    `rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;`.

#### `cartesian_controller_base/src/cartesian_controller_base.cpp`

- Added `#include <utility>`.
- Defined `struct CartesianControllerBase::PendingChainSwap { std::string
  robot_description; KDL::Chain robot_chain; std::shared_ptr<IKSolver>
  ik_solver; std::shared_ptr<KDL::TreeFkSolverPos_recursive>
  forward_kinematics_solver; };` inside the namespace.
- Defined the out-of-line destructor as `= default;`.
- Refactored `on_configure` to delegate URDF parsing, KDL build and IK init
  to `buildKinematics`. The first build happens with the URDF the controller
  was configured with.
- After `m_configured = true`, registered the parameter callback with
  `m_param_callback_handle = get_node()->add_on_set_parameters_callback(...)`.
- Added `buildKinematics(...)` body: parses `urdf::Model`, builds
  `KDL::Tree` via `kdl_parser`, extracts the chain from `robot_base_link` to
  `end_effector_link`, parses joint limits, instantiates a **fresh**
  `IKSolver` via
  `m_solver_loader->createSharedInstance(m_ik_solver_plugin_name)` (so the
  RT thread's old solver pointer is never mutated), initializes it, and
  builds a single-chain `TreeFkSolverPos_recursive`.
- Added `onParameterUpdate(...)` body: filters for `robot_description`;
  rejects empty values; early-outs **and clears any staged swap** when
  `new_urdf == m_robot_description`; otherwise calls `buildKinematics`,
  stages the result under `m_chain_swap_mutex`, sets
  `m_chain_swap_pending` with release ordering, and logs
  `Queued kinematic chain rebuild from new robot_description (N bytes)`.
- Added `synchronizeKinematics()` body: acquire-load fast path,
  mutex-protected move-assign of the four chain members, release-clear of
  the atomic flag, invocation of `onChainRebuilt()`, and log
  `Kinematic chain swapped in from updated robot_description`.

#### `cartesian_force_controller/include/cartesian_force_controller/cartesian_force_controller.h`

- Added `void onChainRebuilt() override;` to `protected`.

#### `cartesian_force_controller/src/cartesian_force_controller.cpp`

- Inserted `Base::synchronizeKinematics();` at the very top of `update()`,
  before any chain access.
- Implemented `onChainRebuilt()`: validates `m_ft_sensor_ref_link` is in the
  new chain; validates `m_new_ft_sensor_ref` if non-empty; calls
  `setFtSensorReferenceFrame(m_new_ft_sensor_ref.empty()
      ? Base::m_end_effector_link
      : m_new_ft_sensor_ref);`
  so the FT sensor reference transform is recomputed against the new chain.

#### `cartesian_motion_controller/src/cartesian_motion_controller.cpp`

- Inserted `Base::synchronizeKinematics();` at the top of `update()`, before
  `synchronizeJointPositions(...)`.

#### `cartesian_compliance_controller/src/cartesian_compliance_controller.cpp`

- Inserted `Base::synchronizeKinematics();` at the top of `update()`. The
  compliance controller diamond-inherits from motion + force, but both
  branches share a single base subobject, so one call is enough to drive
  the swap.

### Validation

Tested end-to-end against the DUCO GCR5_910 in fake-hardware mode with all
three controllers (`cartesian_force_controller`,
`cartesian_motion_controller`, `cartesian_compliance_controller`). Each
controller was made active in turn and a new `robot_description` was pushed
via its per-controller `SetParameters` service. Measured
`current_pose.position.y` (TCP, world frame) versus the pushed
`compliance_link` Z offset:

| Active controller | baseline `y` | after push | revert |
|---|---|---|---|
| `cartesian_force_controller`      | 0.249500000022 | 0.299500000019 (Δ=+0.0500) | 0.249500000022 (byte-identical) |
| `cartesian_compliance_controller` | 0.249500000022 | 0.329499999537 (Δ=+0.0800) | 0.249499999914 (~3e-10 IK noise) |
| `cartesian_motion_controller`     | 0.249499999914 | 0.279499999820 (Δ=+0.0300) | 0.249500000190 (~3e-10 IK noise) |

Inactive controllers logged
`Queued kinematic chain rebuild from new robot_description (N bytes)` on each
push. The active controller additionally logged
`Kinematic chain swapped in from updated robot_description` on its next
`update()` cycle. The hygiene path was exercised: when a no-op revert came in
while a swap was staged on an inactive controller, the controller logged
`robot_description matches current chain; dropped previously-staged swap`,
and the next activation showed no leftover offset.

### Known limitations / TODOs

- The patch assumes `robot_base_link`, `end_effector_link` and the joint
  ordering remain stable across pushes; changing those still requires a
  reconfigure. `buildKinematics` validates each new URDF is consistent with
  the configured `joints`.
- The `cartesian_controller_handles` package was not patched; its rebuild
  semantics are independent and were not exercised by the test.
- Upstream packages `cartesian_controller_simulation` and
  `cartesian_controller_tests` were not modified.
