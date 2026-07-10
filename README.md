![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_humble_action.yml/badge.svg)
![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_iron_action.yml/badge.svg)
![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_jazzy_action.yml/badge.svg)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Cartesian Controllers
This package provides a set of Cartesian `motion`, `force` and `compliance controllers` for the `ros2_control` framework.
The controllers are meant for `joint position` and `joint velocity` interfaces on the manipulators.
As a unique selling point, they use fast forward dynamics simulations of
virtually conditioned twins of the real setup as a solver for the inverse kinematics problem.
Integrating from joint accelerations to joint velocities and joint positions
gives them a delay-free, noise suppressing, and an inherently more stable contact behavior than conventional
*Admittance* controllers.
The controllers from this package are designed to trade smooth and stable behavior for accuracy where
appropriate, and behave physically plausible for targets outside the robots reach.
The package is for users who require interfaces to direct task space control
without the need for collision checking.

The **ROS1** version is [here](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros1). Also see [this talk at ROSCon'19](https://vimeo.com/378682968) and [these
slides](https://roscon.ros.org/2019/talks/roscon2019_cartesiancontrollers.pdf)
to get a brief overview.

## Why this package?
Users may refer to `MoveIt` for end-effector motion planning, but
integrating a full planning stack is often unnecessary for simple applications.
Additionally, there are a lot of use cases where direct control in task space is mandatory:
dynamic following of target poses, such as **visual servoing**, **teleoperation**, **Cartesian teaching,** or
any form of **closed loop control with external sensors** for physical interactions with environments, such as **Machine Learning** applications.
This package provides such a controller suite for the [ros2_control](https://control.ros.org/master/index.html) framework.

## Installation
Switch into the `src` folder of your current ROS2 workspace and
```bash
git clone -b ros2 https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```
This builds the `cartesian_controllers` without its simulation environment.
The simulation is mostly relevant if you are just getting to know the `cartesian_controllers` and want to inspect how things work.
You can install it according to this [readme](cartesian_controller_simulation/README.md).

Now source your workspace again and you are ready to go.

## Getting started
This assumes you have the `cartesian_controller_simulation` package installed.
In a sourced terminal, call
```bash
ros2 launch cartesian_controller_simulation simulation.launch.py
```

This will start a simulated world in which you can inspect
and try things. Here are some quick tutorials with further details:
- [Solver details](resources/doc/Solver_details.md)
- [Cartesian motion controller](cartesian_motion_controller/README.md)
- [Cartesian force controller](cartesian_force_controller/README.md)
- [Cartesian compliance controller](cartesian_compliance_controller/README.md)
- [Cartesian controller handles](cartesian_controller_handles/README.md)
- [Teleoperation](cartesian_controller_utilities/README.md)
- [Example on Universal Robots](https://github.com/stefanscherzinger/cartesian_controllers_universal_robots/tree/ros2)

## Citation and further reading
If you use the *cartesian_controllers* in your research projects, please
consider citing our initial idea of the forward dynamics-based control
approach ([Paper](https://ieeexplore.ieee.org/document/8206325)):
```bibtex
@InProceedings{FDCC,
  Title                    = {Forward Dynamics Compliance Control (FDCC): A new approach to cartesian compliance for robotic manipulators},
  Author                   = {S. Scherzinger and A. Roennau and R. Dillmann},
  Booktitle                = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  Year                     = {2017},
  Pages                    = {4568-4575},
  Doi                      = {10.1109/IROS.2017.8206325}
}

```

If you are interested in more details, have a look at
- *Inverse Kinematics with Forward Dynamics Solvers for Sampled Motion Tracking* ([Paper](https://arxiv.org/pdf/1908.06252.pdf))
- *Virtual Forward Dynamics Models for Cartesian Robot Control* ([Paper](https://arxiv.org/pdf/2009.11888.pdf))
- *Contact Skill Imitation Learning for Robot-Independent Assembly Programming* ([Paper](https://arxiv.org/pdf/1908.06272.pdf))
- *Human-Inspired Compliant Controllers for Robotic Assembly* ([PhD Thesis](https://publikationen.bibliothek.kit.edu/1000139834), especially Chapter 4)

Here's an application of imitation learning for force-controlled assembly on a UR10e
- *Learning Human-Inspired Force Strategies for Robotic Assembly* ([Paper](https://arxiv.org/abs/2303.12440))

## Change log (fork)

This repository is a fork of
[fzi-forschungszentrum-informatik/cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).
Per-change records live under [`changelog/`](changelog/README.md), one file
per change, named `YYYY-MM-DD-<slug>.md`. The high-level summary of what
this fork adds on top of upstream is:

- **Live `robot_description` updates**
  ([2026-05-21](changelog/2026-05-21-live-robot-description-updates.md)) —
  the base controller now subscribes to parameter updates of
  `robot_description` and rebuilds its KDL chain, IK solver and FK solver
  at runtime, with no controller unload/reload required. The expensive
  build runs on the executor thread and is swapped into the RT thread via
  an `std::atomic<bool>` flag and a brief mutex hand-off. A virtual
  `onChainRebuilt()` hook lets derived controllers refresh chain-dependent
  state (e.g. the FT-sensor reference frame in
  `cartesian_force_controller`).

- **Topic-sourced `robot_description`**
  ([2026-06-16](changelog/2026-06-16-topic-sourced-robot-description.md)) —
  opt-in (`urdf_from_topic:=true`) single source of truth: the controller
  reads its URDF exclusively from a latched `robot_description` topic
  (default `/cartesian/robot_description`), deferring the chain build until
  the URDF arrives and gating activation on it. This decouples the FZI
  controllers from the controller_manager's immutable URDF, so auxiliary
  frames can be added by the `aux_frame_manager` (sole writer) without a
  custom bringup. Default false preserves stock behaviour.

- **Live forward-dynamics virtual mass**
  ([2026-07-10](changelog/2026-07-10-live-forward-dynamics-link-mass.md)) —
  `solver.forward_dynamics.link_mass` is refreshed before every virtual
  dynamics step and must remain finite and greater than zero. This makes the
  dashboard control affect the running solver without reloading a controller.
