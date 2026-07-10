# 2026-07-10 - Live forward-dynamics link mass

## What

Made `solver.forward_dynamics.link_mass` live-tunable. The forward-dynamics
solver now reads the parameter before each virtual dynamics step and rebuilds
the generic chain inertia from that value before calculating the joint-space
mass matrix.

The parameter must be a finite double greater than zero. Invalid startup values
prevent solver initialization, and invalid runtime updates are rejected by the
controller's parameter callback.

## Why

The Cartesian controller dashboard exposes virtual mass alongside the existing
live gains and solver settings. Previously, a parameter update changed the ROS
parameter value but the solver continued using the value cached during
configuration, which made a dashboard control misleading.

## Scope

`link_mass` is the artificial mass assigned to each moving non-tip segment of
the forward-dynamics model. The final chain segment retains its hard-coded unit
mass and inertia. This is not the physical robot mass or a scalar Cartesian
end-effector mass.

The parameter only affects controllers using `ik_solver: forward_dynamics`.

## Validation

- `cartesian_controller_base` builds successfully in Release mode.
- KDL's `ChainDynParam` retains the chain by reference, so rebuilding segment
  inertias before `JntToMass()` affects the next solver step.
- Dashboard API accepted a no-op `0.1` write, rejected `-0.1`, and left the ROS
  parameter at `0.1`.

A controller-manager restart is required once after installing this patch so the
process loads the rebuilt `libik_solvers.so`. Subsequent mass changes are live.
