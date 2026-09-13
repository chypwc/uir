# Project Intelligence Checklist

This is the live progress tracker for the main robotics frontier. [PLAN.md](PLAN.md) defines what the programme teaches and why, and [INTELLIGENCE_CHECKLIST.md](INTELLIGENCE_CHECKLIST.md) independently tracks the parallel intelligence-foundations frontier. This checklist records the current main phase, exactly one main-track active task, ordered artifact-level work, and links to evidence owned elsewhere.

When a phase closes, preserve its concise acceptance conclusion and evidence links in the programme table, remove its detailed working tasks, and expand the next phase selected by the plan's dependencies. [CHECKLIST.legacy.md](archive/CHECKLIST.legacy.md) and [CAPABILITY_TRACK.legacy.md](archive/CAPABILITY_TRACK.legacy.md) are historical and non-authoritative.

## Working rules

1. Keep exactly one main-track implementation-sized capability cycle, one stage, and one task active here; do not duplicate the independent intelligence track's tasks.
2. Complete each cycle in dependency order: learn and write, review and quiz, create or extend the specification, implement with verifying tests, write and review the retrospective companion section, then close.
3. Make every task name a concrete artifact or verifiable action; name an exact boundary or failure case instead of adding a generic limitations task.
4. Link evidence instead of copying results into this file.
5. Preserve accepted implementation and useful learning material when closing an earlier process gap.
6. Put non-blocking work in the backlog; do not expand the active capability around it.
7. Keep queued capabilities as concise artifact-level outlines and decompose them into implementation-sized cycles only when they approach activation; do not rewrite closed or in-progress history merely to adopt a newer tracking format.
8. Record actual focused hours in the shared [effort log](docs/effort_log.md), following the daily target and accounting rules in `PLAN.md`; review effort alongside accepted artifacts at capability closure.

## Programme status

Phase numbers identify curriculum ownership. Follow the dependency spine in [PLAN.md](PLAN.md): the main route to the first manipulation research release is Phase 1, Phases 2–5, Phase 8, Phase 9A, and Phase 11A, consuming accepted intelligence cycles when needed. Phase 7 is a separate capstone; Phase 10 and the road study follow Phase 11A. All required programme outcomes remain tracked below.

| Milestone | Owner | Outcome | Status | Evidence |
|---|---|---|---|---|
| `M0` — Environment bootstrap | Phase 0 | Reproducible development environment | Complete | [Setup](docs/environment/environment_setup.md), [verification](docs/environment/environment_verification.md) |
| `M1` — Classical intelligence foundation | Phases 1–5 | Motion/control release, then classical mobile inspection release | Active | — |
| `M2` — Reusable learning foundation | Phase 6 | Educational C++/LibTorch cores; one selected learner's mobile and non-planar reuse | Queued by dependency | — |
| `M3` — Synthetic Intelligence Engine Capstone | Phase 7 | Independent controlled-text integration and ablations | Queued by dependency | — |
| `M4` — Classical manipulation platform | Phase 8 | Frozen classical arm and contact benchmark | Queued after Phase 5 | — |
| `M5` — Manipulation learning | Phase 9A, then 9B | Frozen data/cloning reference; later one advanced learner | Queued after Phase 8 | — |
| First manipulation research release | Phase 11A | Three model types, one task, one planner, primary conclusion | Queued after Phase 9A and required cores | — |
| `M6` — Generalization and Adaptation Layer | Phase 10 | Two-source/one-held-out-arm benchmark | Queued after Phase 11A and Phase 9B | — |
| `M7` — Core Intelligence Release | Phase 11B | Broader learning, synthetic, manipulation, adaptation, and road evidence | Queued after its named outcomes | — |
| `M8` — Classical UAV platform | Phase 12 | Classical inspection-and-return baseline | Queued after Core release | — |
| `M9` — UAV intelligence transfer | Phase 13 | Selected learned flight-model and transfer evidence | Queued after Phase 12 | — |
| `M10` — Multi-embodiment Intelligence Transfer Release | Phase 14 | Full programme and transfer conclusion | Queued after required outcomes | — |

### Early release boundaries

| Release | Acceptance owner | Packaging and evidence |
|---|---|---|
| Motion and Control Foundations Release | Phase 1 / `P1.13` | Accepted laboratories, mobile motion/control and faults, reviewed notes, source/environment/scenario manifests, reproduction commands, demonstration, and linked acceptance results |
| Classical Mobile Inspection Release | Phase 5 | Accepted classical inspection mission and educational references, with the same release manifest fields and linked acceptance results |
| Manipulation World-Model Release | Phase 11A | Frozen analytical–neural–hybrid comparison, data/checkpoint manifests, tests, reviewed notes, and linked acceptance results |

Release rows name future acceptance gates; no release is complete until its concise acceptance conclusion and reproducible evidence are linked. Optional epistemic autonomy does not block any row.

## Parallel intelligence track

The parallel track is active at `IF.4` — Finite MDPs, Bellman equations, and exact dynamic programming. Former `IF.1`–`IF.3` surveys are retired; their mathematical topics are written only as just-in-time prerequisite blocks when an active consumer first needs them. The active task, ordered capability cycles, phase destinations, and evidence belong only in [INTELLIGENCE_CHECKLIST.md](INTELLIGENCE_CHECKLIST.md). Consume each accepted intelligence cycle when its robot application needs it. Give a shared unfinished prerequisite one owner; retire the parallel checklist only after its remaining independent work has transferred. Reuse the same notes and installed cores.

## Current work

- **Milestone:** `M1` — Classical intelligence foundation.
- **Phase:** Phase 1 — Geometry, mechanics, and control.
- **Capability:** `P1.3` — General robot kinematics and Jacobians.
- **Cycle:** Cycle 3 — Space Jacobian.
- **Active stage:** Review and quiz.
- **Active task:** Review [Space Jacobians](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-space-jacobians) and complete its retrieval questions, including the pose derivative, preceding-joint column transformation, and distinction between the linear space-twist block and tool-origin velocity. The preceding pose-rate and task-Jacobian blocks supply the prerequisites. The requested body-Jacobian draft is non-blocking learning preparation for Cycle 4, not activation of that cycle.
- **Next engineering checkpoint:** Finish the required space-Jacobian review and quiz, then review the drafted [Jacobian specification](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md#space-and-body-jacobians) before implementation. The learner requested a minimal mathematical acceptance batch for both Jacobians and their relation; drafting both contracts does not activate Cycle 4.
- **Next mathematical capability:** `P1.4` — General inverse-kinematics foundations and planar manipulator laboratory.
- **Blockers:** None. `P1.3` Cycles 1 and 2 are closed with evidence linked below.

## Phase 1 — Geometry, mechanics, and control

Phase 1 closes when its shared mathematical references and two bounded learning systems pass. The shared references must verify continuous- and discrete-time state-space models, linearisation, stability, controllability, observability, introductory Lyapunov reasoning, and LQR before later phases reuse them. First, a deterministic 2R/3R planar manipulator laboratory must solve declared inverse-kinematics cases and track joint trajectories with classical controllers, including one admitted LQR application, under declared gravity, friction, saturation, disturbance, and model mismatch. Second, the headless simulated differential-drive robot must estimate its planar motion, follow bounded straight and curved trajectories under declared disturbances, and reach a stopped-safe state after every declared command failure. The two manipulator specifications own the laboratory scenarios and thresholds; the final acceptance specification owns the mobile-system scenarios and thresholds. The phase acceptance summary and linked evidence must keep the educational state-space reference, manipulator evidence, and mobile-robot integration and safety evidence distinct while explaining their dependency chain from the shared geometry, mechanics, and control foundations.

### Evidence-based gap assessment

| Area | Current evidence | Gap to the Phase 1 goal | Disposition |
| ----------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Linear algebra and coordinate frames | [Chapter 1](notes/part_01_motion_mechanics_control/01_linear_algebra_foundations.qmd) covers vectors, matrices, rank, null spaces, determinants, and orthogonal maps; [Chapter 2](notes/part_01_motion_mechanics_control/02_geometry_and_coordinate_frames.qmd) covers frames, coordinate changes, and a cumulative test; Chapter 7 introduces a constraint Jacobian, rank, and tangent null space; the opening [Chapter 10](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd) block defines the total derivative and multivariable chain rule | Task, space, and body robot Jacobians and robot singularities remain untaught | Complete the queued Chapter 10 Jacobian blocks in `P1.3` |
| Planar rigid-body and differential-drive motion | [Chapter 3](notes/part_01_motion_mechanics_control/03_kinematics_and_numerical_integration.qmd), [Chapter 6](notes/part_01_motion_mechanics_control/06_planar_rigid_body_motion_se2_and_twists.qmd), the [specification](docs/01_motion_models/differential_drive_motion_model.md), and the [package](ros_ws/src/differential_drive_motion_model) cover ideal planar kinematics, exact constant-input integration, and Forward Euler | Encoder odometry, timestamps, drift, and broader ODE treatment remain outside the closed ideal-motion capability | Preserve the accepted `P1.1` evidence and add the deferred topics only in their owning later capabilities |
| Spatial rigid-body motion | Reviewed [Chapter 7](notes/part_01_motion_mechanics_control/07_degrees_of_freedom_and_spatial_motion_so3_se3.qmd) and [Chapter 8](notes/part_01_motion_mechanics_control/08_screw_motion_se3_exponential_and_spatial_integration.qmd), the approved [specification](docs/02_spatial_kinematics/spatial_geometry_kernel.md), the retrospective [implementation companion](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd), and the deterministic C++20 [package and tests](ros_ws/src/rigid_body_kinematics) cover the bounded spatial geometry kernel | No remaining gap for `P1.2`; general forward and velocity kinematics belong to `P1.3` | Preserve the closed kernel and extend it only through approved later capability contracts |
| General robot kinematics | [Chapter 10](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd) space- and body-form PoE blocks are reviewed; the [Cycle 1](#cycle-1--space-form-product-of-exponentials-forward-kinematics) and [Cycle 2](#cycle-2--body-form-product-of-exponentials-forward-kinematics) closure records link the accepted core, companion, tests, and existing RViz consumer | Jacobian and singularity cycles remain open | Continue `P1.3` Cycle 3 without reopening the accepted forward-kinematics cycles |
| Inverse kinematics and planar manipulation | No current note, specification, or package solves a manipulator task | Analytic and numerical inverse kinematics, convergence, multiple solutions, reachability, damped least squares, joint limits, redundancy, null-space objectives, manipulability, and manipulator-specific acceptance evidence are absent | Teach general open-serial-chain inverse-kinematics foundations, verify them in a bounded 2R/3R planar laboratory, and reserve the spatial production arm and manipulation mission for Phase 8 |
| Wheeled constraints and odometry | Chapter 3 derives the ideal no-sideways differential-drive constraint and wheel/body mapping; Chapters 3 and 7 distinguish holonomic and nonholonomic constraints | The general Pfaffian-to-parametric framework, rolling-constraint matrices, encoder odometry, Ackermann steering, and the bicycle comparison are absent | Add a wheeled-constraints and odometry chapter, then implement one differential-drive odometry pipeline |
| Dynamic systems and mechanics | The preserved [dynamics draft](notes/part_01_motion_mechanics_control/16_dynamic_systems_mechanics_and_actuator_limits.qmd) covers one-dimensional Newtonian motion, wheel traction balance, and ideal gearing | General state ODEs, planar yaw dynamics, manipulator mass, Coriolis/centrifugal and gravity terms, inertia, friction and traction bounds, resistance, actuator saturation, braking, and a reference numerical method are absent | Preserve and complete the shared dynamic-systems foundation, then derive separate bounded differential-drive and 2R manipulator plants |
| Trajectory generation | No current learning or implementation artifact | Path versus trajectory, time scaling, curvature, velocity, acceleration, jerk, wheel feasibility, and safe rejection are absent | Add one bounded straight-and-constant-curvature trajectory chapter and generator |
| State-space, feedback, and control | No current approved learning or implementation artifact | Continuous- and discrete-time state-space models, linearisation about equilibria and trajectories, eigenvalue and Lyapunov stability, controllability, observability, stabilisability, detectability, Riccati equations, LQR, error dynamics, open-loop and feedback distinctions, P/PI/PD/PID, sampling, delay, saturation, anti-windup, derivative filtering, disturbance rejection, gravity compensation, computed-torque control, and metrics are absent | Build one bounded state-space and LQR reference laboratory in `P1.8`, establish the practical feedback benchmark in `P1.9`, and apply the admitted LQR and manipulator controllers in `P1.10` without duplicating the Riccati solver |
| ROS 2 integration and safety | [Chapter 5](notes/part_01_motion_mechanics_control/05_ros2_packages_and_development_workflow.qmd) covers package and build workflow; the accepted [Chapter 11 RViz example](notes/part_01_motion_mechanics_control/11_general_robot_kinematics_and_jacobians_implementation.qmd#sec-general-kinematics-impl-rviz) covers a marker publisher, timer, static/animated launch, two headless tests, and recorded visual evidence | `tf2`, simulation-time integration, Gazebo acceptance, command supervision, watchdog, emergency stop, stopped-safe behaviour, telemetry, replay, and fault injection are not established | Preserve the bounded RViz consumer, learn `tf2` at the `P1.5` adapter boundary, retain Gazebo as the integration and physics simulator, and make no certified functional-safety claim |

### Scope boundary

- Phase 1 includes reusable forward and velocity kinematics plus one educational manipulator laboratory: closed-form 2R and numerical inverse kinematics, a 3R planar redundancy example, bounded manipulability and null-space reasoning, a deterministic 2R dynamic plant, joint trajectories, and classical joint control.
- The Phase 1 manipulator is a numerical teaching instrument, not a manipulation product. Its mathematical packages use modern C++, Eigen, and deterministic tests and have no gripper, contact, grasping, collision scene, object mission, perception dependency, ROS runtime, or Gazebo arm. A separate downstream RViz package may consume their public interfaces for explanatory visual inspection without owning kinematics, dynamics, physics, control, or correctness evidence.
- Phase 8 owns the spatial six- or seven-degree-of-freedom product: Pinocchio validation, production URDF/SRDF, `ros2_control`, MoveIt 2, collision-aware planning, gripper and contact models, task-space force-aware control, mission execution, recovery, and safety evidence. It reuses rather than reteaches Phase 1 foundations.
- Ackermann steering and the kinematic bicycle model remain a mathematical comparison. Do not create a second vehicle implementation.
- Probabilistic odometry, covariance estimation, sensor fusion, and slip estimation belong to Phase 2. Obstacle-aware and kinodynamic planning belong to Phase 5.
- Phase 1 owns continuous- and discrete-time state-space models, local linearisation, stability, controllability, observability, introductory Lyapunov reasoning, LQR, and one LQR tracking application. Finite-horizon optimal-control formulation, indirect methods, shooting, direct collocation, differential flatness, iLQR, DDP, nonlinear MPC, dynamic programming, and POMDP planning belong to Phase 5.
- OpenCV and PCL enter with perception in Phase 3. Pinocchio, `ros2_control`, and MoveIt 2 enter with the Phase 8 manipulation product. Phase 1 uses modern C++, CMake, Eigen, GTest, ROS 2, `tf2`, Gazebo, and Python analysis only where an artifact requires them.
- RViz displays declared mathematical, estimated, planned, or simulated outputs but does not establish numerical correctness or physical behaviour; deterministic tests remain the mathematical evidence, and Gazebo remains the authoritative Phase 1 integration and physics simulator.
- Safety evidence is limited to verified behaviour in the declared simulation and fault model; it is not a hardware or functional-safety certification.
- Treat each concept checkbox in a note-writing block as a separate draft-and-review gate. Do not batch several unchecked learning blocks into one review.

### Interface, limit, and evidence ownership

| Boundary | Owner |
|---|---|
| Reference geometry and kinematic feasibility | `differential_drive_trajectory`; it reports infeasible references but does not simulate actuator saturation |
| Tracking, regulation, output limiting, and anti-windup | `differential_drive_control`; it consumes the declared measurement interface and emits bounded left/right wheel-effort requests |
| Operational command envelope, freshness, emergency-stop priority, reset, and stopped-safe state | `motion_safety_supervisor`; it does not reproduce controller or plant dynamics |
| Realised wheel-effort, effort slew, traction, resistance, and disturbance constraints | `differential_drive_planar_plant` offline and the selected Gazebo actuator/physics boundary in integration |
| Offline controller performance claims | `docs/04_motion_control/differential_drive_control_benchmark.md`, backed by tested metrics and a durable configuration/results manifest |
| State-space analysis, Riccati solution, and LQR reference correctness | `docs/04_motion_control/linear_state_space_control_lab.md`, `linear_systems_control_lab`, and its deterministic reference tests; the selected 2R application reuses frozen gains in `planar_manipulator_lab` without duplicating the Riccati solver |
| Educational manipulator kinematics interfaces and deterministic correctness | `docs/02_spatial_kinematics/planar_manipulator_kinematics_lab.md` and the focused `planar_manipulator_lab` tests |
| Educational manipulator dynamics and controller-comparison claims | `docs/04_motion_control/planar_manipulator_dynamics_and_control.md`, backed by focused tests and a frozen scenario/results manifest |
| Simulation safety claims | `docs/05_phase_01_integration/motion_safety_supervisor.md`, backed by fault-injection results and explicit simulation-only claim boundaries |
| Final Phase 1 conclusion | The concise acceptance conclusion in this checklist, linking `docs/05_phase_01_integration/phase_01_acceptance.md` and existing component evidence without duplicating it |

### Theory-to-project map

| Capability | Learning artifact | Side project and principal tools | Status |
|---|---|---|---|
| `P1.1` — Ideal planar motion kernel | Close Chapters 2–3 and preserve Chapters 4 and 6 | Existing `differential_drive_motion_model`, Python and `pytest` | Complete |
| `P1.2` — Spatial geometry kernel | Complete Chapter 8 and its Chapter 9 implementation companion | `rigid_body_kinematics`, modern C++, CMake, Eigen, and GTest | Complete |
| `P1.3` — General kinematics and Jacobians | Complete Chapter 10 and its Chapter 11 implementation companion | Extend `rigid_body_kinematics` with bounded forward and velocity kinematics; add a scenario-specific downstream `rigid_body_kinematics_visualization` consumer with ROS 2, `visualization_msgs`, and RViz | Cycles 1 and 2 complete; Cycle 3 active at learning; later cycles remain open |
| `P1.CI` — Package and documentation CI | No additional artifact required | Retain local builds, tests, and document renders | Skipped by user decision on 2026-09-09; not a Phase 1 gate |
| `P1.4` — General inverse-kinematics foundations and planar manipulator laboratory | Chapter 12 and its Chapter 13 implementation companion | Begin the ROS-free `planar_manipulator_lab` with 2R/3R inverse kinematics, modern C++, Eigen, and GTest; extend the downstream RViz consumer for selected spatial results and retain Python plots for numerical histories | Queued |
| `P1.5` — Wheel-odometry pipeline | Chapter 14 and its Chapter 15 implementation companion | `differential_drive_odometry`, modern C++ core and thin ROS 2 adapter, with the accepted Python kernel as an independent reference | Queued |
| `P1.6` — Differential-drive dynamics laboratory | Chapter 16 and its Chapter 17 implementation companion | `differential_drive_planar_plant`, modern C++ and deterministic numerical tests | Queued |
| `P1.7` — Bounded trajectory generator | Chapter 18 and its Chapter 19 implementation companion | `differential_drive_trajectory`, modern C++ and independent constraint checks | Queued |
| `P1.8` — State-space systems and LQR reference laboratory | Chapter 20 and its Chapter 21 implementation companion | `linear_systems_control_lab`, modern C++, Eigen, GTest, and independent Python/SciPy references | Queued |
| `P1.9` — Offline controller benchmark | Chapter 22 and its Chapter 23 implementation companion | `differential_drive_control`, modern C++ core and Python analysis | Queued |
| `P1.10` — Planar manipulator dynamics and control laboratory | Chapter 24 and its Chapter 25 implementation companion | Extend `planar_manipulator_lab` with a 2R plant, joint trajectories, classical controllers including one frozen-gain LQR application, and matched comparisons | Queued |
| `P1.11` — Nominal Gazebo integration | Chapter 26 implementation companion | `differential_drive_gazebo`, thin ROS 2 adapters, RViz inspection beside Gazebo, and evaluator-only ground truth | Queued |
| `P1.12` — Motion safety and fault acceptance | Extend Chapter 26 | `motion_safety_supervisor`, diagnostics, fault injection, and `rosbag2` MCAP | Queued |
| `P1.13` — Phase 1 evaluation and learning closure | Chapter 27 cumulative review | Final acceptance campaign, linked results and conclusion, and book render | Queued |

### P1.1 — Close the preserved ideal planar motion kernel

Do not rewrite or port the accepted implementation during this capability.

#### Learn and review

- [x] Complete and self-check the retrieval and cumulative tests in `notes/part_01_motion_mechanics_control/03_kinematics_and_numerical_integration.qmd`; correct any blocking misconception.
- [x] Add a cumulative test with answers or hints to `notes/part_01_motion_mechanics_control/02_geometry_and_coordinate_frames.qmd`, review it, correct any blocking misconception, and render and inspect the updated chapter.

#### Verify and close

- [x] Check that the equations, conventions, units, validation behaviour, and exclusions agree across the theory, specification, implementation companion, public API, and deterministic tests.
- [x] Re-run the package test suite and link the result if the accepted source has changed; otherwise preserve the existing evidence.
- [x] Close `P1.1` without reimplementing accepted work.

Evidence:

- [Theory](notes/part_01_motion_mechanics_control/03_kinematics_and_numerical_integration.qmd)
- [Implementation companion](notes/part_01_motion_mechanics_control/04_kinematics_and_numerical_integration_implementation.qmd)
- [Planar rigid-body theory](notes/part_01_motion_mechanics_control/06_planar_rigid_body_motion_se2_and_twists.qmd)
- [Specification](docs/01_motion_models/differential_drive_motion_model.md)
- [Package and deterministic tests](ros_ws/src/differential_drive_motion_model)

### P1.2 — Spatial geometry kernel

The closed capability supplies the geometry layer of the spatial and general kinematics workbench. Its accepted learning and verification history is retained below.

#### Learn and review

- [x] Finish `notes/part_01_motion_mechanics_control/07_degrees_of_freedom_and_spatial_motion_so3_se3.qmd` with a spatial-twist retrieval test, cumulative chapter test and answers, representation limitations, numerical checks, and implementation implications.
- [x] Review and approve Chapter 7 one small learning block at a time.
- [x] After approval, reconcile its already-started notation entries and add only its newly approved glossary terms.
- [x] Reconcile `notes/_quarto.yml` with the reviewed source set, render and inspect Chapter 7, and remove stale generated pages that are not requested deliverables.
- [x] Preserve the dynamics draft and its useful content through the earlier chapter-number migration.
- [x] Before drafting Chapter 8, route its prerequisite blocks through *Modern Robotics*, Chapter 3, and compare the formulation with Craig, Chapter 2, and Corke, Chapters 2–3.
- [x] Split the former Chapter 8 source without rewriting reviewed content: retain the spatial-geometry material through the motion-workflow summary and the Chasles--Mozzi appendix in `notes/part_01_motion_mechanics_control/08_screw_motion_se3_exponential_and_spatial_integration.qmd`; move the material beginning at “Configuration coordinates and forward kinematics” into `notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd`; update chapter introductions, cross-references, `notes/_quarto.yml`, and local links in the same batch.
- [x] In the same chapter-renumbering batch, rename the preserved dynamics draft to `notes/part_01_motion_mechanics_control/16_dynamic_systems_mechanics_and_actuator_limits.qmd` without rewriting or losing its content, and update every repository reference.
- [x] Draft and review the screw-axis, pitch, and revolute-versus-prismatic motion block in `notes/part_01_motion_mechanics_control/08_screw_motion_se3_exponential_and_spatial_integration.qmd`.
- [x] Draft and review the $SE(3)$ exponential and logarithm block, including how project linear-first twists map to each textbook's convention.
- [x] Draft and review the constant-twist body-versus-space integration block.
- [x] Draft and review the small-angle, near-$\pi$, branch-ambiguity, and worked-spatial-example block.
- [x] Obtain user approval of these exact prerequisite blocks before specifying or implementing the spatial geometry kernel.
- [x] Add only Chapter 8's newly introduced notation and reviewed technical terms to the chapter registries, then render and inspect the approved chapter.

#### Specify

- [x] Create `docs/02_spatial_kinematics/spatial_geometry_kernel.md` with the linear-first twist convention, frames, types, tolerances, valid domains, invalid-input behaviour, exclusions, and library-independent acceptance cases.
- [x] Review and approve the [minimum spatial geometry kernel specification](docs/02_spatial_kinematics/spatial_geometry_kernel.md) before implementation begins.

#### Implement and document

- [x] Implement, test, understand, and retrospectively document the validated $SO(3)$ rotation and free-vector-rotation mathematical cycle; complete the minimum `ament_cmake` library target, Eigen and GTest linkage, C++17 requirement, explicit compiler warnings, and public header/source/test layout needed by this cycle without adding a ROS runtime dependency to the mathematical core.
- [x] Implement, test, understand, and retrospectively document the validated $SE(3)$ representation and point-transformation mathematical cycle.
- [x] Implement, test, understand, and retrospectively document the $SO(3)$ and $SE(3)$ composition-and-inversion mathematical cycle, including matrix reconstruction needed to inspect the results; preserve the durable explanation in the [composition-and-inversion companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-composition-inversion) and the deterministic evidence in the [`rigid_body_kinematics` tests](ros_ws/src/rigid_body_kinematics/test).
- [x] Run the checklist-declared intermediate C++ quality gate after the representations, vector and point operations, composition, inversion, public API, and package structure are stable; verify the clean package build, all registered functional and lint tests, installed downstream CMake target, public interface, mathematical equations, frames, units, validation, and numerical failure behaviour.
- [x] Implement, test, understand, and retrospectively document the $SO(3)$ and linear-first $SE(3)$ hat-and-vee mathematical cycle; preserve the reviewed explanation in the [hat-and-vee companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-hat-vee) and its 16 deterministic cases in [`test_lie_algebra.cpp`](ros_ws/src/rigid_body_kinematics/test/test_lie_algebra.cpp).
- [x] Implement, test, understand, and retrospectively document the $SO(3)$ exponential mathematical cycle, including its exact-zero, stable small-angle, and nominal Rodrigues branches; preserve the reviewed explanation in the [$SO(3)$ exponential companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-so3-exponential) and its nine deterministic cases in [`test_so3_exponential.cpp`](ros_ws/src/rigid_body_kinematics/test/test_so3_exponential.cpp).
- [x] Implement, test, understand, and retrospectively document the principal $SO(3)$ logarithm mathematical cycle, including its identity, small-angle, nominal-angle, and deterministic near-$\pi$ branches; preserve the reviewed explanation in the [principal $SO(3)$ logarithm companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-so3-logarithm) and its deterministic evidence in [`test_so3_logarithm.cpp`](ros_ws/src/rigid_body_kinematics/test/test_so3_logarithm.cpp).
- [x] Implement, test, understand, and retrospectively document the linear-first $SE(3)$ exponential mathematical cycle, keeping translational coordinates in metres and rotational coordinates in radians distinct; preserve the reviewed explanation in the [$SE(3)$ exponential companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-se3-exponential) and its five deterministic cases in [`test_se3_exponential.cpp`](ros_ws/src/rigid_body_kinematics/test/test_se3_exponential.cpp).
- [x] Implement, test, understand, and retrospectively document the principal $SE(3)$ logarithm mathematical cycle using the completed principal $SO(3)$ logarithm; preserve the reviewed explanation in the [principal $SE(3)$ logarithm companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-se3-logarithm) and its 11 deterministic cases in [`test_se3_logarithm.cpp`](ros_ws/src/rigid_body_kinematics/test/test_se3_logarithm.cpp).
- [x] Implement, test, understand, and retrospectively document the constant-twist-integration mathematical cycle using the completed $SE(3)$ exponential and explicit body-versus-space multiplication order; preserve the reviewed explanation in the [constant-twist-integration companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-constant-twist-integration) and its 10 deterministic cases in [`test_constant_twist_integration.cpp`](ros_ws/src/rigid_body_kinematics/test/test_constant_twist_integration.cpp).
- [x] Implement, test, understand, and retrospectively document the $SE(3)$ adjoint mathematical cycle using the project's linear-first twist order; preserve the reviewed explanation in the [adjoint companion section](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-se3-adjoint) and its five deterministic cases in [`test_se3_adjoint.cpp`](ros_ws/src/rigid_body_kinematics/test/test_se3_adjoint.cpp).
- [x] Implement, test, understand, and retrospectively document planar-pose embedding and extraction plus normalised planar-yaw quaternion conversion without attempting to replace `tf2`; preserve the reviewed explanations in the [planar interoperability](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-planar-pose-interoperability) and [normalized planar-yaw quaternion](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-planar-yaw-quaternion) companion sections and their 15 deterministic cases in [`test_planar_geometry.cpp`](ros_ws/src/rigid_body_kinematics/test/test_planar_geometry.cpp).

#### Verify and close

- [x] Test identity, inverse round trips, noncommuting composition order, known rotations about two axes, point-versus-vector behaviour, planar embedding, and quaternion sign equivalence.
- [x] Test exponential/logarithm round trips away from declared branch ambiguities and the identity $\widehat{\operatorname{Ad}_{T}\boldsymbol\xi}=T\widehat{\boldsymbol\xi}T^{-1}$.
- [x] Test the identity and small-angle series path, deterministic near-$\pi$ axis handling, and documented rejection or canonicalisation at ambiguous logarithm branches.
- [x] Reject non-finite, malformed, non-orthogonal, reflective, and inapplicable non-planar inputs without returning a valid-looking result.
- [x] Run the checklist-declared pre-closure C++ quality gate across the stable package, public API, Eigen representations, CMake export, and focused GTest evidence.
- [x] Migrate the `rigid_body_kinematics` public build contract from C++17 to ISO C++20 using `PUBLIC cxx_std_20` and disabled compiler extensions; reconcile the declared minimum CMake version with that compile feature, and avoid unrelated production-code or API rewrites.
- [x] Preserve the hash-locked `cpplint==2.0.2` development dependency, replace this package's vendored `ament_cpplint` test registration with the pinned C++20-aware executable, retain `colcon test-result` reporting, and reconcile the six public header guards with the modern lint policy without changing the public API.
- [x] Audit every existing public header, implementation source, and test in `rigid_body_kinematics` under the C++20 contract; replace justified compatibility workarounds with clearer C++20 standard-library facilities, including replacing handwritten or runtime-derived values of $\pi$ with `std::numbers::pi`, while preserving the approved API, equations, numerical branches, and failure behaviour.
- [x] Update the retrospective implementation companion's CMake and code snippets from C++17 compatibility forms to C++20, including why the requirement propagates to installed downstream consumers and how `<numbers>` supplies the standard double-precision value of $\pi$.
- [x] Configure, build, and run all registered functional and lint tests with the supported Phase 1 toolchain, then install the package and compile and run a downstream consumer through `rigid_body_kinematics::rigid_body_kinematics` to verify propagation of C++20, Eigen, headers, and the library.
- [x] Link the approved [Chapter 7](notes/part_01_motion_mechanics_control/07_degrees_of_freedom_and_spatial_motion_so3_se3.qmd), [Chapter 8](notes/part_01_motion_mechanics_control/08_screw_motion_se3_exponential_and_spatial_integration.qmd), [specification](docs/02_spatial_kinematics/spatial_geometry_kernel.md), [implementation companion](notes/part_01_motion_mechanics_control/09_spatial_geometry_kernel_implementation.qmd), [package](ros_ws/src/rigid_body_kinematics), and [deterministic tests](ros_ws/src/rigid_body_kinematics/test) and close `P1.2`.

### P1.3 — General robot kinematics and Jacobians

The outcome is a bounded reusable implementation of general forward and velocity kinematics that the next educational manipulator capability and later robot products can share.

#### Cycle 1 — Space-form product-of-exponentials forward kinematics

Closed on 2026-09-08 using the working tree based on `309db59`, including the learner's uncommitted launch/package changes. Evidence is retained in the existing specifications, tests, README, companion, and media linked below.

- **Build and tests:** Both packages built; all 19 core CTest registrations and the visualization registration passed. The cycle-specific core targets contain 30 GoogleTest cases and the visualization target contains two. Final `colcon test-result` totals were 219 core and three visualization entries, with zero errors, failures, or skips; those totals include wrappers and lint entries, not just mathematical tests. The initial cppcheck 2.13.0 skip was resolved by the explicit rerun below; no persistent configuration changed. Visualization formatting passed clang-format 18.
- **Factor order:** The [GK-ACC-008 verifier](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md#gk-acc-008-executable-negative-control) returned `PASS`, with correct/reversed positions `(1, 2, 0)` / `(5, 2, 0)` metres. This is manual executable evidence, not a registered unit test.
- **Live integration:** Both README launch modes ran for ten seconds on local-only ROS domain 191 and each delivered 193 updates. RViz subscribed; arrays had nine markers with IDs 0–5 and 7–9, frame `world`, and matching timestamps. Static end position agreed with `(0, 2, 1)` metres within `1e-12`; animated height varied by more than 0.5 metres. One SIGINT to launch stopped both children cleanly with exit zero. These normal-permission checks superseded sandbox socket/display failures and a duplicate-interrupt harness attempt; counts are observations, not a real-time guarantee.
- **Documentation and scope:** Closure-time project HTML and a 43-page standalone companion PDF layout preview passed inspection; temporary-copy cross-references used project HTML labels. This was not a full-book PDF release. The existing user-inspected PNG and accepted eight-second, 200-frame, looping GIF supply visual evidence, not numerical correctness. The GIF's approximately 10.46 MiB size and no-trail scope remain accepted. Temporary checks were not production edits or retained deliverables; the dedicated repeat/default-policy and additional inter-joint-overflow cases remain omissions, not verified cases.

Reproduce the package checks from `ros_ws` after the ROS/environment setup in the [README](ros_ws/src/rigid_body_kinematics_visualization/README.md):

```zsh
colcon build --packages-select \
  rigid_body_kinematics rigid_body_kinematics_visualization
colcon test --packages-select \
  rigid_body_kinematics rigid_body_kinematics_visualization \
  --ctest-args --output-on-failure
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 \
  ctest --test-dir build/rigid_body_kinematics \
  -R '^cppcheck$' --output-on-failure
colcon test-result --test-result-base build/rigid_body_kinematics --verbose
colcon test-result \
  --test-result-base build/rigid_body_kinematics_visualization --verbose
```

- [x] **Learn and write:** Develop the configuration/task-map prerequisites and space-form product, home pose, normalized revolute/prismatic screws, and multiplication order in [Chapter 10](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-serial-product-of-exponentials).
- [x] **Review and quiz:** Complete the prerequisite and space-form review and retrieval questions, including frames, units, product order, and the supported serial-chain assumptions.
- [x] **Specify:** Review the [core contract](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md) and reconcile the [visualization contract](docs/02_spatial_kinematics/spatial_2r_forward_kinematics_visualization.md) with nine markers, axis labels, the accepted GIF size, and user-approved exclusion of the trail.
- [x] **Implement and verify:** Complete the learner-authored core and its 30 cycle-specific tests, two marker tests, static/animated RViz consumer, installed launch/configuration, and [README](ros_ws/src/rigid_body_kinematics_visualization/README.md). Both package builds and all registered checks pass. GK-ACC-008 passes as a manual executable acceptance check, not another production test.
- [x] **Write and review the companion:** Complete the [Chapter 11 cycle companion](notes/part_01_motion_mechanics_control/11_general_robot_kinematics_and_jacobians_implementation.qmd), equation-to-code explanations, cycle-wide diagrams, launch commands, and attached [PNG](notes/figures/spatial_2r_space_poe_rviz.png) and [GIF](notes/figures/spatial_2r_space_pose_rivz.gif). Project HTML and a complete-chapter standalone PDF layout preview were checked; this does not claim a full-book PDF release.
- [x] **Close:** The closure audit confirms ordered-product mathematics, explicit rejection, pure-core/downstream-ROS separation, static/animated launch subscription and clean shutdown, and linked deterministic versus visual evidence. The selected repeat-call/default-policy and additional joint-product-overflow tests remain explicit omissions; no trail test is required. This closes only Cycle 1, not P1.3 or Phase 1.

#### Cycle 2 — Body-form product-of-exponentials forward kinematics

- [x] The existing Chapter 10 draft contains body screw axes, the body-form product, its multiplication order, and equivalence with the space form and the declared transform chain. Reuse this draft rather than writing a duplicate; this records draft presence, not review acceptance.
- [x] Review the body-form derivation, home-frame axis conversion, lever-arm example, and transform-chain equivalence. The learner confirmed the same space-referenced output pose, fixed home axes, and unchanged joint order with the home pose on the left; local clarification edits are in Chapter 10. This clears the Cycle 2 learning gate, not chapter-wide acceptance.
- [x] Review and refine the [body-form specification](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md#body-form-forward-kinematics). The learner selected one RRP fixture at zero and one nonzero configuration, then approved retaining detailed validation tests on the shared helper's space-form path. Coordinate-count, scaling-overflow, and composition-overflow checks exercise both entry points; body conversion overflow remains distinct. Independent NumPy/SciPy checks establish the fixture values separately from production evaluation.
- [x] Implement and verify the [body evaluator](ros_ws/src/rigid_body_kinematics/src/forward_kinematics.cpp) with the existing model and shared query validator. On 2026-09-09, the local package build and tests passed after sourcing Jazzy and activating the repository environment: 222 reported tests, 0 errors, 0 failures, 33 skipped. The [forward-kinematics test executable](ros_ws/src/rigid_body_kinematics/test/test_forward_kinematics.cpp) reports 18 passing tests, including three body-specific tests and the three reused dual-entry checks. Reproduce from `ros_ws` with `colcon build --packages-select rigid_body_kinematics`, `colcon test --packages-select rigid_body_kinematics --ctest-args --output-on-failure`, and `colcon test-result --test-result-base build/rigid_body_kinematics --verbose`. This verifies implementation, not companion review or cycle closure.
- [x] Write and review the [body-form companion](notes/part_01_motion_mechanics_control/11_general_robot_kinematics_and_jacobians_implementation.qmd#sec-general-kinematics-impl-body-forward-kinematics): home-frame conversion, pseudocode and production excerpt, rotated-home RRP usage, verification meaning, and compact connection diagram. The learner requested closure after review, including a direct reference from the RRP position to the earlier 2R equation. The [chapter HTML](output/book/part_01_motion_mechanics_control/11_general_robot_kinematics_and_jacobians_implementation.html#sec-general-kinematics-impl-body-forward-kinematics) was rendered again and all eight references in the body section resolve. Independent SciPy checks confirm both RRP poses. This accepts the cycle's section, not the unfinished chapter or its registries.
- [x] **Close (2026-09-09):** The bounded capability, C++, and numerical audits found no blocking contract or implementation issue. Fixed home-axis conversion, increasing joint order with the home pose on the left, owned results, input non-mutation, shared validation, and separate conversion/scaling/composition rejection match the specification. The local build and package test rerun reports 222 tests, 0 errors, 0 failures, and 33 skipped; all 18 forward-kinematics tests pass. The 33 skips are the ROS wrapper's cppcheck 2.13 performance workaround, not skipped functional acceptance cases. A manual cppcheck run using `build/rigid_body_kinematics/compile_commands.json`, filtered to `*forward_kinematics.cpp`, passed for the evaluator and test source; package-wide static-analysis coverage is not claimed. A temporary standalone CMake consumer linked the installed `rigid_body_kinematics::rigid_body_kinematics` target, inherited C++20, and ran the companion's nonzero RRP example: body translation/rotation residuals were approximately `3.33e-16 m` and `1.57e-16 rad`. The HTML render, reference checks, and `git diff --check` pass. Retain the approved non-duplication of detailed shared-validation tests; no new visualization, Jacobian, or chapter-wide completion is implied.

#### Cycle 3 — Space Jacobian

- [x] Draft the [pose-rate and space-twist block](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-end-effector-pose-rate-space-twist), [Task Jacobians](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-task-jacobians), and [Space Jacobians](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-space-jacobians), with product-rule derivation, column interpretation, planar 2R examples, and retrieval questions. The space-column derivation and interpretation follow *Modern Robotics*, Sections 5.1.1 and 5.1.3; Craig, Section 5.7, supplies the complementary point-velocity interpretation. Independent NumPy/SciPy checks confirm the worked values and every space/body column against pose differences for planar 2R and non-planar RRP fixtures. The 2026-09-12 space-section coherence review connects the home hinge to its current column, separates hinge lines from attached frames, and consolidates repeated explanations; additional home-hinge and single-joint checks pass. The [chapter HTML](output/book/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.html#sec-space-jacobians) renders successfully; all 14 cross-references in the space/body sections resolve, and the scoped `git diff --check` passes. Draft presence and numerical checks do not establish learner review acceptance.
- [ ] **Active:** Review the space-Jacobian block and complete its retrieval questions on column meaning, frames, units, differentiation, and application; establish understanding of the finite-difference verification method before its specification is accepted.
- [ ] Review and accept the drafted [space-Jacobian contract](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md#space-jacobian-calculation), interfaces, inherited validation, and [minimal mathematical acceptance](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md#minimal-mathematical-acceptance). At the learner's request, the specification now covers both Jacobians and their relation using one existing RRP fixture, home and nonzero reference configurations, and one local derivative check; the additional planar fixture and duplicated validation suite are not required. Independent NumPy/SciPy checks confirm the stated matrices, adjoint relation, and finite-difference tolerances; the specification parses as Markdown and the scoped diff check passes. This records a draft, not learner acceptance or implementation evidence.
- [ ] Implement the space Jacobian for a validated $n$-joint open chain with the space-side checks in GK-JACC-001–003, using the single RRP fixture selected in the specification.
- [ ] Write and review the retrospective space-Jacobian companion section after its implementation and tests work.
- [ ] Run the focused package build and acceptance cases, inspect the affected artifacts, link the evidence, and close the space-Jacobian cycle.

#### Cycle 4 — Body Jacobian and frame conversion

- [ ] Learn and write body end-effector velocity, the body Jacobian, and the body–space adjoint relation. At the learner's request, [Body Jacobians](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-body-jacobians) and [The body–space adjoint relation](notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-body-space-jacobian-adjoint) are drafted, including the inverse-product derivation, later-joint dependence, current-pose frame conversion, planar 2R examples, and retrieval questions. On 2026-09-13, independent numerical checks confirm both adjoint directions, the inverse blocks, and the new worked and quiz values; analytic planar columns agree with pose differences at four configurations. The [chapter HTML](output/book/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.html#sec-body-space-jacobian-adjoint) renders successfully, all ten equation references and the example links in the new section resolve, and the scoped diff check passes. This is non-blocking learning preparation, not learner review acceptance; Cycle 3 remains active.
- [ ] Review the body-Jacobian block and complete its retrieval questions on frame conversion, multiplication direction, equivalence, application, and adjoint verification.
- [ ] Review and accept the drafted [body-Jacobian contract](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md#body-jacobian-calculation) and [current-pose adjoint relation](docs/02_spatial_kinematics/general_kinematics_and_jacobians.md#bodyspace-relationship). These were drafted together with the space contract at the learner's request; reuse GK-JACC-001–003 without another fixture or public conversion function.
- [ ] Implement the body Jacobian independently from its inverse later-joint formula; extend the shared RRP checks with body columns, local pose differences, and the current-pose adjoint comparison.
- [ ] Write and review the retrospective body-Jacobian companion section after its implementation and tests work.
- [ ] Run the focused package build and acceptance cases, inspect the affected artifacts, link the evidence, and close the body-Jacobian cycle.

#### Cycle 5 — Task Jacobians, rank, and singularity evidence

- [ ] Learn and write task Jacobians, full-twist versus reduced-task Jacobians, rank, range, null space, singular values, conditioning, and task-specific singularities.
- [ ] Review the analysis block and complete its retrieval questions, including the straightened planar 2R position task and the contrast with the full twist Jacobian.
- [ ] Extend the specification with the bounded task-projection and rank-analysis evidence required for the planar 2R and small non-planar fixtures, without adding inverse kinematics or a speculative generic task-map API.
- [ ] Implement deterministic analysis tests that compare the planar 2R task Jacobian with finite differences, demonstrate its $2\times2$ position-task rank loss at a straightened configuration, contrast it with the full twist Jacobian's retained angular row, and exercise the declared non-planar fixture.
- [ ] Write and review the retrospective finite-difference and rank-analysis companion section after the evidence works.
- [ ] Run the focused package build and acceptance cases, inspect the affected artifacts, link the evidence, and close the task-Jacobian and singularity cycle.

#### Chapter synthesis and capability closure

- [ ] Add a cumulative test with answers or hints and an explicit dependency map to the inverse-kinematics laboratory in `P1.4` and the full manipulation product in Phase 8.
- [ ] Review and approve Chapter 10, then update notation, glossary, `notes/_quarto.yml`, and the rendered book in that order.
- [ ] Run the complete affected-package build and deterministic test set, confirm that the general-purpose package excludes inverse kinematics, pseudoinverse control, URDF parsing, collision, and dynamics, and inspect the final P1.3 artifacts.
- [ ] Link the approved theory, specification, companion, package, and deterministic tests and close `P1.3`.
- [ ] Review the available [effort log](docs/effort_log.md) entries for `P1.1`–`P1.3` alongside the accepted artifacts; identify unrecorded historical effort without inventing retrospective totals.

### Queued Phase 1 capabilities

The following outlines retain required artifacts and decisive acceptance boundaries. Expand only the next capability when it approaches activation, deriving its small cycles from the approved prerequisites. For every implementation, write the companion only after the code and focused tests work and the learner understands them. Chapter paths below are prospective within `notes/part_01_motion_mechanics_control/`; create no placeholder chapters or specifications.

#### P1.4 — General inverse kinematics and planar laboratory

- [ ] Review Chapter 12 `12_inverse_kinematics_redundancy_and_planar_manipulators.qmd`: general task/pose inverse kinematics, analytic 2R branches and reachable annulus, numerical pseudoinverse and damped least squares, singularity/rank, joint limits, 3R redundancy, exact null-space projection, damped leakage, and manipulability. Route through Modern Robotics Chapters 4–6 with Craig and Corke support.
- [ ] Specify `docs/02_spatial_kinematics/planar_manipulator_kinematics_lab.md`; implement the bounded ROS-free C++20/Eigen `planar_manipulator_lab` using the accepted general kinematics core and a versioned offline result schema. Extend the existing downstream RViz consumer only for selected explanatory cases.
- [ ] Verify analytic/iterative agreement, both valid branches, unreachable and boundary targets, non-convergence, joint-limit rejection, near-singular damping, finite-difference Jacobians, exact secondary-task preservation, and damped leakage. Local solver failure must not claim global infeasibility.
- [ ] Write and review Chapter 13 `13_planar_manipulator_kinematics_implementation.qmd` retrospectively, link evidence, close the capability, and review recorded effort and accepted outcomes from `P1.1`–`P1.4`.

#### P1.5 — Wheeled constraints and odometry

- [ ] Review Chapter 14 `14_wheeled_robot_constraints_and_odometry.qmd`: rolling constraints, differential-drive wheel/body mapping, cumulative encoders and exact planar integration; Ackermann/bicycle remains a mathematical comparison. Explain calibration, slip, drift, and uncalibrated covariance.
- [ ] Specify `docs/03_wheel_odometry/differential_drive_odometry.md`; implement a pure C++ odometer and thin JointState/Odometry/`tf2` adapters in `differential_drive_odometry`, using the accepted Python motion kernel as an independent reference.
- [ ] Verify initialisation without false displacement, straight/spin/curved motion, wheel order, frame/unit/message agreement, missing or non-finite samples, duplicate/out-of-order timestamps, and invalid-update non-mutation.
- [ ] Write and review Chapter 15 `15_wheeled_robot_constraints_and_odometry_implementation.qmd` after focused tests work, then close with linked evidence.

#### P1.6 — Mechanics and differential-drive plant

- [ ] Complete preserved Chapter 16 `16_dynamic_systems_mechanics_and_actuator_limits.qmd`: vector ODEs, local linearisation, Euler and one higher-order reference integrator, longitudinal/yaw mechanics, reflected wheel inertia without double counting, resistance, friction, traction, torque/slew limits, disturbances, and stopping.
- [ ] Specify `docs/04_motion_control/motion_stack_interfaces.md`, `differential_drive_planar_plant.md`, and `docs/decisions/0001_phase_01_gazebo_wheel_effort_interface.md`. Select a supported effort boundary that exercises the regulator; preserve the declared trajectory/tracker/regulator/supervisor/actuator ownership chain.
- [ ] Implement the deterministic C++ `differential_drive_planar_plant`; verify analytic force/yaw cases, signs and units, convergence, applicable energy properties, actuator/traction bounds, and explicit limit status.
- [ ] Write and review Chapter 17 `17_dynamic_systems_mechanics_and_actuator_limits_implementation.qmd` retrospectively and close with the focused evidence.

#### P1.7 — Bounded trajectory generation

- [ ] Review Chapter 18 `18_bounded_trajectory_generation.qmd`: path/trajectory distinction, straight and constant-curvature paths, one quintic time scaling, chain-rule velocity/acceleration/jerk, wheel feasibility, and geometric infeasibility.
- [ ] Specify `docs/04_motion_control/bounded_trajectory_generation.md`; implement the pure C++ `differential_drive_trajectory` with independent constraint checking and exact final-sample inclusion.
- [ ] Verify analytic start/end geometry, monotonic time, endpoint conditions, wheel and body bounds, sampling, invalid inputs, and requests that cannot be repaired by increasing duration.
- [ ] Write and review Chapter 19 `19_bounded_trajectory_generation_implementation.qmd` after tests pass and close with linked evidence.

#### P1.8 — State-space and LQR reference

- [ ] Review Chapter 20 `20_state_space_stability_and_lqr.qmd`: continuous/discrete models, equilibrium/trajectory linearisation, eigenvalue and Lyapunov stability, controllability/observability, stabilisability/detectability, and finite/infinite-horizon LQR assumptions and Riccati equations.
- [ ] Specify `docs/04_motion_control/linear_state_space_control_lab.md`; implement `linear_systems_control_lab` in C++/Eigen with an established Riccati solver boundary, independent SciPy checks, and a versioned model/gain export. Do not implement a general control toolbox or precompute the unapproved arm model.
- [ ] Verify analytic/finite-difference linearisation, small rank/stability fixtures, Lyapunov and Riccati residuals, gain sign/units, closed-loop behaviour, and explicit rejection of invalid or unsupported models and costs.
- [ ] Write and review Chapter 21 `21_state_space_stability_and_lqr_implementation.qmd` retrospectively, then close the reference; `P1.10` owns its arm application.

#### P1.9 — Feedback and offline controller benchmark

- [ ] Review Chapter 22 `22_feedback_stability_and_practical_pid_control.qmd`: plant/measurement/reference distinctions, error dynamics, P/PI/PD/PID, sampling, delay, noise, saturation, filtered derivative, anti-windup, and nonholonomic tracking.
- [ ] Specify `docs/04_motion_control/differential_drive_control_benchmark.md`; implement `differential_drive_control` and the frozen feedforward, proportional, and PID-based comparisons on the accepted plant and trajectories. Controllers receive declared simulated measurements; raw plant state is evaluator-only.
- [ ] Verify known responses, applicable constant-load steady-state error, anti-windup recovery, filtering, deterministic disturbances/noise/latency, frozen gains, independently checked metrics, and a durable configuration/result manifest.
- [ ] Write and review Chapter 23 `23_feedback_stability_and_practical_pid_control_implementation.qmd` after the benchmark works; retain the benchmark's results, including negative results, alongside its configuration manifest, link them here and close the offline capability.

#### P1.10 — Planar manipulator dynamics and control

- [ ] Review Chapter 24 `24_planar_manipulator_dynamics_and_classical_control.qmd`: vertical-plane 2R Euler–Lagrange dynamics, mass/bias/gravity/friction, forward/inverse dynamics, joint references, PD, gravity compensation, computed torque, and a local frozen-gain LQR tracking application.
- [ ] Specify `docs/04_motion_control/planar_manipulator_dynamics_and_control.md`; extend `planar_manipulator_lab` with pure model/controller/runner components. Import verified gains from `P1.8`; keep controller parameters distinct from evaluator plant parameters and terminate declared joint-limit violations without inventing hard-stop contact.
- [ ] Verify independent model terms, mass-matrix properties, equilibrium, energy/refinement behaviour, linearisation and gain provenance, saturation, invalid inputs, and matched controller scenarios under disturbances and parameter mismatch. Report the LQR validity region explicitly.
- [ ] Write and review Chapter 25 `25_planar_manipulator_dynamics_and_classical_control_implementation.qmd` retrospectively; retain the arm laboratory's scenario/results manifests, link them here and close the educational arm laboratory.

#### P1.11 — Nominal ROS 2/Gazebo integration

- [ ] Review current official simulation-clock, bridge, effort-interface, launch, `tf2`, RViz, and headless-execution guidance; specify `docs/05_phase_01_integration/gazebo_closed_loop_integration.md` for no-motion, straight, spin, and curved scenarios.
- [ ] Implement `differential_drive_gazebo` and thin adapters for the accepted trajectory, odometry, and regulator. The Gazebo effort boundary must receive the regulator output; simulator truth is evaluator-only and the offline plant does not run as another physics engine in the loop.
- [ ] Verify the complete reference-to-effort-to-motion path, timestamps/frames, odometry comparison, nominal headless cases, and optional Gazebo/RViz inspection. Numerical and simulation tests own correctness; RViz remains explanatory.
- [ ] Write and review the first substantive retrospective section of Chapter 26 `26_ros2_gazebo_integration_and_motion_safety_implementation.qmd`, retaining the agreed PNG-first media convention, then close nominal integration.

#### P1.12 — Motion safety and fault acceptance

- [ ] Review the operational-safety boundary and specify `docs/05_phase_01_integration/motion_safety_supervisor.md`: envelope, command/feedback freshness, sequence identity, clock/reset semantics, emergency-stop latch, safe reset, diagnostics, and stopped-safe effort with conservative stale-feedback handling.
- [ ] Implement the pure C++ supervisor and thin ROS adapter; keep plant limits, controller limits, and operational safety ownership distinct. Invalid data must not refresh watchdogs or mutate accepted state.
- [ ] Verify pure state transitions and inject command/feedback dropout, invalid commands, duplicate/out-of-order/reset timestamps, and emergency stop. Record reaction/stop time, stopping distance, interventions, diagnostics, scenario manifests, and retained MCAP hashes or locations.
- [ ] Extend Chapter 26 retrospectively; retain the simulation-only claim boundary in the safety specification, link the fault-injection results here, and close the capability.

#### P1.13 — Phase 1 acceptance and release

- [ ] Review Chapter 27 `27_geometry_mechanics_and_control_cumulative_review.qmd`, correct blocking misconceptions, and reconcile the reviewed book and registries without adding duplicate concepts.
- [ ] Freeze `docs/05_phase_01_integration/phase_01_acceptance.md` as the owner of final mobile scenarios, thresholds, environment, held-out cases, commands, and result manifests. Reference the state-space and arm specifications and result manifests for their separate educational evidence.
- [ ] Run the complete clean-environment mobile acceptance campaign across nominal motion, geometry mismatch, bounded disturbance, encoder noise, command latency, effort/traction limits, dropout, invalid data, timestamps, and emergency stop. Use tested metrics and frozen controller comparisons, preserving failures and negative results.
- [ ] Confirm the accepted `P1.8` and `P1.10` evidence and passing local package builds and tests; render the complete reviewed Phase 1 book and record a concise phase acceptance conclusion here linking the existing component results.
- [ ] Package the Motion and Control Foundations Release with source revision, environment/scenario manifests, build/test/reproduction commands, reviewed notes and companions, and representative demonstration. Link the release and acceptance evidence here, mark Phase 1 complete, condense its working detail, and expand Phase 2.

## Backlog

- [ ] Remove the two accidental duplicate `body_to_wheel` code blocks appended to `notes/README.md` when that file is next in scope.
