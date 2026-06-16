# 2026-06-16 — Topic-sourced `robot_description` (single source of truth)

## What

Added an optional **topic source** for the controller's URDF, gated by a new
boolean parameter `urdf_from_topic` (default **false** = stock behaviour). When
true, the controller reads its URDF **exclusively** from a latched
`robot_description` topic (name in `robot_description_topic`, default
`/cartesian/robot_description`) instead of the `robot_description` parameter
forwarded by the controller_manager.

New parameters:

| param | default | meaning |
|---|---|---|
| `urdf_from_topic` | `false` | read the URDF from a latched topic instead of the parameter |
| `robot_description_topic` | `/cartesian/robot_description` | the topic to read when `urdf_from_topic` is true |

## Why

In ros2_control (Humble 2.53.1) the controller_manager's URDF is **immutable
after first load** (`ResourceManager ... Ignoring attempt to reload a robot
description file`). Auxiliary frames (FT-sensor / compliance / operation origin)
therefore could only reach the FZI controllers by being baked into the URDF at
bringup — forcing a custom bringup and leaving three divergent runtime URDF
copies (controller_manager, robot_state_publisher, each controller's private
param). Reading the URDF from a single latched topic (published by the
`aux_frame_manager`, the sole writer) makes that one URDF the consistent source
for every controller, decoupled from the immutable controller_manager URDF, with
no change to the basic manufacturer bringup.

## How

`cartesian_controller_base`:

- `on_configure` reads the static config (`robot_base_link`,
  `end_effector_link`, `joints`, `command_interfaces`) regardless of source. In
  topic mode it then **defers** the kinematic-chain build and creates a
  TRANSIENT_LOCAL subscription to `robot_description_topic`; in parameter mode it
  builds immediately as before.
- `robotDescriptionTopicCallback` routes each received URDF through the existing
  `buildKinematics` -> `PendingChainSwap` machinery. Before activation it
  installs the chain directly (no RT thread running); while active it stages the
  swap for `synchronizeKinematics()` (the RT path), exactly like the parameter
  callback. `m_chain_built` tracks first success.
- `on_activate` is **gated**: it first drains any staged swap, then refuses to
  activate (clear error message) until a valid chain exists — so a missing URDF
  fails cleanly instead of dereferencing a null solver.
- `onParameterUpdate` ignores `robot_description` parameter updates while
  `urdf_from_topic` is true, so the two sources cannot diverge.

The hardware-interface binding is unaffected: `command_interface_configuration`
/ `state_interface_configuration` build from the `joints` + `command_interfaces`
parameters, not the URDF, so a topic-sourced URDF (which only adds **fixed**
aux frames) never changes what the controller claims from hardware. The actuated
joint set remains fixed at activation (ros2_control invariant).

## Compatibility

Default `urdf_from_topic=false` preserves stock behaviour exactly (the parameter
path, including the live parameter-update rebuild, is unchanged). The feature is
opt-in per controller via YAML.

## Validation

Mock Duco GCR5-910: a `cartesian_motion_controller` with
`urdf_from_topic:=true` and `end_effector_link: op_tip` (a frame present ONLY in
the `aux_frame_manager`'s canonical topic URDF, NOT in the controller_manager's
URDF) deferred configure, built its chain from the first topic message
("Installed kinematic chain from topic URDF [first]"), and reached `active`. A
live offset edit (op_tip z +0.15 m) propagated to the active controller's
`current_pose` by exactly 0.15 m (RT staged-swap path). See the
`aux_frame_manager` package.
