# 2026-07-13 - Live forward-dynamics link inertia

## What

Added `solver.forward_dynamics.link_inertia`, the rotational counterpart of
`solver.forward_dynamics.link_mass`. It is the isotropic rotational inertia
(`ixx = iyy = izz`, kg·m²) assigned to the final (end-effector) segment of the
forward-dynamics model. The solver reads it before each virtual dynamics step
and rebuilds the generic chain inertia from it before calculating the
joint-space mass matrix, so it is live-tunable.

The parameter must be a finite double greater than zero. Invalid startup values
prevent solver initialization, and invalid runtime updates are rejected by the
controller's parameter callback. The default is `1.0`, which reproduces the
value that was previously hard-coded.

## Why

`link_mass` gave operators a live knob for the virtual translational inertia,
but the end-effector's rotational inertia stayed fixed at 1 kg·m², so rotational
admittance/force response could not be softened or stiffened the same way. The
Cartesian controller dashboard now exposes both, next to each other.

## Scope

`link_inertia` sets the rotational inertia of the final chain segment only (the
end-effector). Intermediate segments keep their near-zero rotational inertia and
`link_mass` for translational mass. It is not the physical robot inertia.

The parameter only affects controllers using `ik_solver: forward_dynamics`.

## Validation

- `cartesian_controller_base` builds successfully; the three FZI controllers and
  the compliance controller (which inherits the solver) build against it.
- `ros2 param get` returns the `1.0` default on the force, motion, and
  compliance controllers; a `5.0` write is accepted; `-1.0` and `0.0` are
  rejected by the parameter callback.
- Effect confirmed on fake hardware: with a fixed base-frame torque pulse the
  end-effector rotated 24.3° at `link_inertia = 1.0` and 0.9° at
  `link_inertia = 30.0` (higher inertia rotates less), while `link_mass`
  continued to behave as before.

A controller-manager restart is required once after installing this patch so the
process loads the rebuilt solver library. Subsequent inertia changes are live.
