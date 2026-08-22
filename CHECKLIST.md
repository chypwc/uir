# Project Intelligence Checklist

This is the sole live progress tracker. [PLAN.md](PLAN.md) defines what the programme teaches and why. This checklist records the current phase, exactly one active task, ordered artifact-level work, and links to evidence owned elsewhere.

When a phase closes, link its verification report in the programme table, remove its detailed working tasks, and expand the next phase. [CHECKLIST.legacy.md](archive/CHECKLIST.legacy.md) and [CAPABILITY_TRACK.legacy.md](archive/CAPABILITY_TRACK.legacy.md) are historical and non-authoritative.

## Working rules

1. Keep exactly one implementation-sized capability cycle, one stage, and one task active.
2. Complete each cycle in dependency order: learn and write, review and quiz, create or extend the specification, implement with verifying tests, write and review the retrospective companion section, then close.
3. Make every task name a concrete artifact or verifiable action; name an exact boundary or failure case instead of adding a generic limitations task.
4. Link evidence instead of copying results into this file.
5. Preserve accepted implementation and useful learning material when closing an earlier process gap.
6. Put non-blocking work in the backlog; do not expand the active capability around it.
7. Keep queued capabilities as concise artifact-level outlines and decompose them into implementation-sized cycles only when they approach activation; do not rewrite closed or in-progress history merely to adopt a newer tracking format.

## Programme status

All phases belong to the learning programme. Phase 7 is the intermediate epistemic-autonomy capstone; Phase 10 and Phase 13 are distinct required release-grade capstone products.

| Milestone                                       | Phases | Outcome                                                                                                                                                                                                            | Status   | Evidence                                                                                                     |
| ----------------------------------------------- | -----: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------- | ------------------------------------------------------------------------------------------------------------ |
| `M0` — Environment bootstrap                    |      0 | Reproducible development environment                                                                                                                                                                               | Complete | [Setup](docs/environment/environment_setup.md), [verification](docs/environment/environment_verification.md) |
| `M1` — Classical intelligence foundation        |    1–5 | Bounded planar-manipulator laboratory, classical mobile-robot inspection mission, and reusable state-space, estimation, perception, world-state, optimisation, decision, planning, telemetry, and safety contracts | Active   | —                                                                                                            |
| `M2` — Reusable learning foundation             |      6 | Reproducible reinforcement-learning, data, safety-cost, and evaluation workflows                                                                                                                                   | Queued   | —                                                                                                            |
| `M3` — Intermediate Epistemic Autonomy Capstone |      7 | Decision-gated mobile-robot model-adequacy, active-evidence, governance, fallback, and abstention study with reusable contracts                                                                                    | Queued   | —                                                                                                            |
| `M4` — Classical manipulation platform          |      8 | Frozen classical manipulation and safety baseline                                                                                                                                                                  | Queued   | —                                                                                                            |
| `M5` — Manipulation learning                    |      9 | Frozen manipulation data and selected model-free learned baseline                                                                                                                                                  | Queued   | —                                                                                                            |
| `M6` — Capstone Product A                       |     10 | Core Intelligence Release                                                                                                                                                                                          | Queued   | —                                                                                                            |
| `M7` — Classical UAV platform                   |     11 | Frozen classical UAV inspection-and-return baseline                                                                                                                                                                | Queued   | —                                                                                                            |
| `M8` — UAV intelligence transfer                |     12 | Learned prediction, planning, epistemic-governance, uncertainty, and safety transfer to flight                                                                                                                     | Queued   | —                                                                                                            |
| `M9` — Capstone Product B                       |     13 | Multi-embodiment Intelligence Transfer Release                                                                                                                                                                     | Queued   | —                                                                                                            |

## Current work

- **Milestone:** `M1` — Classical intelligence foundation.
- **Phase:** Phase 1 — Geometry, mechanics, and control.
- **Capability:** `P1.2` — Spatial geometry kernel.
- **Cycle:** Planar-pose embedding and extraction with planar-yaw quaternion conversion.
- **Active stage:** Implement with verifying tests.
- **Active task:** Implement and test planar-pose embedding and extraction plus normalised planar-yaw quaternion conversion.
- **Next capability:** `P1.3` — General robot kinematics and Jacobians.
- **Blockers:** None.

## Phase 1 — Geometry, mechanics, and control

Phase 1 closes when its shared mathematical references and two bounded learning systems pass. The shared references must verify continuous- and discrete-time state-space models, linearisation, stability, controllability, observability, introductory Lyapunov reasoning, and LQR before later phases reuse them. First, a deterministic 2R/3R planar manipulator laboratory must solve declared inverse-kinematics cases and track joint trajectories with classical controllers, including one admitted LQR application, under declared gravity, friction, saturation, disturbance, and model mismatch. Second, the headless simulated differential-drive robot must estimate its planar motion, follow bounded straight and curved trajectories under declared disturbances, and reach a stopped-safe state after every declared command failure. The two manipulator specifications own the laboratory scenarios and thresholds; the final acceptance specification owns the mobile-system scenarios and thresholds. The phase report must keep the educational state-space reference, manipulator evidence, and mobile-robot integration and safety evidence distinct while explaining their dependency chain from the shared geometry, mechanics, and control foundations.

### Evidence-based gap assessment

| Area                                            | Current evidence                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Gap to the Phase 1 goal                                                                                                                                                                                                                   | Disposition                                                                                                                                              |
| ----------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Linear algebra and coordinate frames            | [Chapter 1](notes/01_linear_algebra_foundations.qmd) covers vectors, matrices, rank, null spaces, determinants, and orthogonal maps; [Chapter 2](notes/02_geometry_and_coordinate_frames.qmd) covers frames, coordinate changes, and a cumulative test; Chapter 7 introduces a constraint Jacobian, rank, and tangent null space; the opening [Chapter 10](notes/10_general_robot_kinematics_and_jacobians.qmd) block defines the total derivative and multivariable chain rule | Task, space, and body robot Jacobians and robot singularities remain untaught | Complete the queued Chapter 10 Jacobian blocks in `P1.3` |
| Planar rigid-body and differential-drive motion | [Chapter 3](notes/03_kinematics_and_numerical_integration.qmd), [Chapter 6](notes/06_planar_rigid_body_motion_se2_and_twists.qmd), the [specification](docs/01_motion_models/differential_drive_motion_model.md), and the [package](ros_ws/src/differential_drive_motion_model) cover ideal planar kinematics, exact constant-input integration, and Forward Euler                                                                                                              | Encoder odometry, timestamps, drift, and broader ODE treatment remain outside the closed ideal-motion capability                                                                                                                          | Preserve the accepted `P1.1` evidence and add the deferred topics only in their owning later capabilities                                                |
| Spatial rigid-body motion                       | The reviewed [Chapter 7](notes/07_degrees_of_freedom_and_spatial_motion_so3_se3.qmd) covers degrees of freedom, $SO(3)$, $SE(3)$, quaternions, body and spatial twists, and the adjoint; the approved and rendered [Chapter 8](notes/08_screw_motion_se3_exponential_and_spatial_integration.qmd) covers screw axes, the $SE(3)$ exponential and logarithm, constant-twist integration, numerical branches, worked cases, retrieval tests, and the Chasles--Mozzi appendix; the [minimum spatial geometry kernel specification](docs/02_spatial_kinematics/spatial_geometry_kernel.md) freezes the bounded interface and analytic acceptance cases | The substantive implementation companion, C++ package, and deterministic tests do not yet exist                                                                                                                        | Review the specification, then implement and verify `P1.2`                                                                                               |
| General robot kinematics                        | The opening [Chapter 10](notes/10_general_robot_kinematics_and_jacobians.qmd) block defines configuration coordinates, task variables, forward maps, the total derivative, the multivariable chain rule, a planar 2R example, and a retrieval test                                                                                                                                                                                                                              | The transform-chain and product-of-exponentials models, task and body/space Jacobians, rank and singularity analysis, full review, specification, implementation, and tests remain absent                                                 | Review the existing opening block when `P1.3` becomes active, then complete the reusable general-kinematics chapter and workbench                        |
| Inverse kinematics and planar manipulation      | No current note, specification, or package solves a manipulator task                                                                                                                                                                                                                                                                                                                                                                                                            | Analytic and numerical inverse kinematics, convergence, multiple solutions, reachability, damped least squares, joint limits, redundancy, null-space objectives, manipulability, and manipulator-specific acceptance evidence are absent  | Add a bounded 2R/3R planar manipulator kinematics stage in Phase 1; reserve the spatial production arm and manipulation mission for Phase 8              |
| Wheeled constraints and odometry                | Chapter 3 derives the ideal no-sideways differential-drive constraint and wheel/body mapping; Chapters 3 and 7 distinguish holonomic and nonholonomic constraints                                                                                                                                                                                                                                                                                                               | The general Pfaffian-to-parametric framework, rolling-constraint matrices, encoder odometry, Ackermann steering, and the bicycle comparison are absent                                                                                    | Add a wheeled-constraints and odometry chapter, then implement one differential-drive odometry pipeline                                                  |
| Dynamic systems and mechanics                   | The preserved [dynamics draft](notes/16_dynamic_systems_mechanics_and_actuator_limits.qmd) covers one-dimensional Newtonian motion, wheel traction balance, and ideal gearing                                                                                                                                                                                                                                                                                                   | General state ODEs, planar yaw dynamics, manipulator mass, Coriolis/centrifugal and gravity terms, inertia, friction and traction bounds, resistance, actuator saturation, braking, and a reference numerical method are absent           | Preserve and complete the shared dynamic-systems foundation, then derive separate bounded differential-drive and 2R manipulator plants                   |
| Trajectory generation                           | No current learning or implementation artifact                                                                                                                                                                                                                                                                                                                                                                                                                                  | Path versus trajectory, time scaling, curvature, velocity, acceleration, jerk, wheel feasibility, and safe rejection are absent                                                                                                           | Add one bounded straight-and-constant-curvature trajectory chapter and generator                                                                         |
| State-space, feedback, and control | No current approved learning or implementation artifact | Continuous- and discrete-time state-space models, linearisation about equilibria and trajectories, eigenvalue and Lyapunov stability, controllability, observability, stabilisability, detectability, Riccati equations, LQR, error dynamics, open-loop and feedback distinctions, P/PI/PD/PID, sampling, delay, saturation, anti-windup, derivative filtering, disturbance rejection, gravity compensation, computed-torque control, and metrics are absent | Build one bounded state-space and LQR reference laboratory in `P1.8`, establish the practical feedback benchmark in `P1.9`, and apply the admitted LQR and manipulator controllers in `P1.10` without duplicating the Riccati solver |
| ROS 2 integration and safety                    | [Chapter 5](notes/05_ros2_packages_and_development_workflow.qmd) covers package and build workflow; existing functions validate numeric inputs                                                                                                                                                                                                                                                                                                                                  | ROS messages, `tf2`, simulation time, Gazebo acceptance, command supervision, watchdog, emergency stop, stopped-safe behaviour, telemetry, replay, and fault injection are absent                                                         | Learn these at their adapter boundary, verify simulation safety behaviour, and make no certified functional-safety claim                                 |

### Scope boundary

- Phase 1 includes reusable forward and velocity kinematics plus one educational manipulator laboratory: closed-form 2R and numerical inverse kinematics, a 3R planar redundancy example, bounded manipulability and null-space reasoning, a deterministic 2R dynamic plant, joint trajectories, and classical joint control.
- The Phase 1 manipulator is a numerical teaching instrument, not a manipulation product. It uses modern C++, Eigen, deterministic tests, and offline visualisation; it has no gripper, contact, grasping, collision scene, object mission, perception dependency, ROS runtime, or Gazebo arm.
- Phase 8 owns the spatial six- or seven-degree-of-freedom product: Pinocchio validation, production URDF/SRDF, `ros2_control`, MoveIt 2, collision-aware planning, gripper and contact models, task-space force-aware control, mission execution, recovery, and safety evidence. It reuses rather than reteaches Phase 1 foundations.
- Ackermann steering and the kinematic bicycle model remain a mathematical comparison. Do not create a second vehicle implementation.
- Probabilistic odometry, covariance estimation, sensor fusion, and slip estimation belong to Phase 2. Obstacle-aware and kinodynamic planning belong to Phase 5.
- Phase 1 owns continuous- and discrete-time state-space models, local linearisation, stability, controllability, observability, introductory Lyapunov reasoning, LQR, and one LQR tracking application. Finite-horizon optimal-control formulation, indirect methods, shooting, direct collocation, differential flatness, iLQR, DDP, nonlinear MPC, dynamic programming, and POMDP planning belong to Phase 5.
- OpenCV and PCL enter with perception in Phase 3. Pinocchio, `ros2_control`, and MoveIt 2 enter with the Phase 8 manipulation product. Phase 1 uses modern C++, CMake, Eigen, GTest, ROS 2, `tf2`, Gazebo, and Python analysis only where an artifact requires them.
- Safety evidence is limited to verified behaviour in the declared simulation and fault model; it is not a hardware or functional-safety certification.
- Treat each concept checkbox in a note-writing block as a separate draft-and-review gate. Do not batch several unchecked learning blocks into one review.

### Interface, limit, and evidence ownership

| Boundary | Owner |
|---|---|
| Reference geometry and kinematic feasibility | `differential_drive_trajectory`; it reports infeasible references but does not simulate actuator saturation |
| Tracking, regulation, output limiting, and anti-windup | `differential_drive_control`; it consumes the declared measurement interface and emits bounded left/right wheel-effort requests |
| Operational command envelope, freshness, emergency-stop priority, reset, and stopped-safe state | `motion_safety_supervisor`; it does not reproduce controller or plant dynamics |
| Realised wheel-effort, effort slew, traction, resistance, and disturbance constraints | `differential_drive_planar_plant` offline and the selected Gazebo actuator/physics boundary in integration |
| Offline controller performance claims | `docs/reports/phase_01_controller_benchmark.md`, backed by tested metrics and a durable configuration/results manifest |
| State-space analysis, Riccati solution, and LQR reference correctness | `docs/04_motion_control/linear_state_space_control_lab.md`, `linear_systems_control_lab`, and its deterministic reference tests; the selected 2R application reuses frozen gains in `planar_manipulator_lab` without duplicating the Riccati solver |
| Educational manipulator kinematics interfaces and deterministic correctness | `docs/02_spatial_kinematics/planar_manipulator_kinematics_lab.md` and the focused `planar_manipulator_lab` tests |
| Educational manipulator dynamics and controller-comparison claims | `docs/reports/phase_01_planar_manipulator_lab_verification.md`, backed by the dynamics/control specification, focused tests, and frozen scenario manifest |
| Simulation safety claims | `docs/reports/phase_01_motion_safety_verification.md`, backed by fault-injection evidence |
| Final Phase 1 conclusion | `docs/reports/phase_01_geometry_mechanics_control_verification.md`, which references rather than duplicates the component reports |

### Theory-to-project map

| Capability | Learning artifact | Side project and principal tools | Status |
|---|---|---|---|
| `P1.1` — Ideal planar motion kernel | Close Chapters 2–3 and preserve Chapters 4 and 6 | Existing `differential_drive_motion_model`, Python and `pytest` | Complete |
| `P1.2` — Spatial geometry kernel | Complete Chapter 8 and its Chapter 9 implementation companion | `rigid_body_kinematics`, modern C++, CMake, Eigen, and GTest | Active: planar pose and yaw-quaternion conversion cycle |
| `P1.3` — General kinematics and Jacobians | Complete Chapter 10 and its Chapter 11 implementation companion | Extend `rigid_body_kinematics` with bounded forward and velocity kinematics | Queued: opening theory block drafted; review and remaining blocks pending |
| `P1.4` — Planar manipulator kinematics laboratory | Chapter 12 and its Chapter 13 implementation companion | Begin `planar_manipulator_lab` with 2R/3R inverse kinematics, modern C++, Eigen, GTest, and offline visualisation | Queued |
| `P1.5` — Wheel-odometry pipeline | Chapter 14 and its Chapter 15 implementation companion | `differential_drive_odometry`, modern C++ core and thin ROS 2 adapter, with the accepted Python kernel as an independent reference | Queued |
| `P1.6` — Differential-drive dynamics laboratory | Chapter 16 and its Chapter 17 implementation companion | `differential_drive_planar_plant`, modern C++ and deterministic numerical tests | Queued |
| `P1.7` — Bounded trajectory generator | Chapter 18 and its Chapter 19 implementation companion | `differential_drive_trajectory`, modern C++ and independent constraint checks | Queued |
| `P1.8` — State-space systems and LQR reference laboratory | Chapter 20 and its Chapter 21 implementation companion | `linear_systems_control_lab`, modern C++, Eigen, GTest, and independent Python/SciPy references | Queued |
| `P1.9` — Offline controller benchmark | Chapter 22 and its Chapter 23 implementation companion | `differential_drive_control`, modern C++ core and Python analysis | Queued |
| `P1.10` — Planar manipulator dynamics and control laboratory | Chapter 24 and its Chapter 25 implementation companion | Extend `planar_manipulator_lab` with a 2R plant, joint trajectories, classical controllers including one frozen-gain LQR application, and matched comparisons | Queued |
| `P1.11` — Nominal Gazebo integration | Chapter 26 implementation companion | `differential_drive_gazebo`, thin ROS 2 adapters, and evaluator-only ground truth | Queued |
| `P1.12` — Motion safety and fault acceptance | Extend Chapter 26 | `motion_safety_supervisor`, diagnostics, fault injection, and `rosbag2` MCAP | Queued |
| `P1.13` — Phase 1 evaluation and learning closure | Chapter 27 cumulative review | Final acceptance campaign, phase verification report, and book render | Queued |

### P1.1 — Close the preserved ideal planar motion kernel

Do not rewrite or port the accepted implementation during this capability.

#### Learn and review

- [x] Complete and self-check the retrieval and cumulative tests in `notes/03_kinematics_and_numerical_integration.qmd`; correct any blocking misconception.
- [x] Add a cumulative test with answers or hints to `notes/02_geometry_and_coordinate_frames.qmd`, review it, correct any blocking misconception, and render and inspect the updated chapter.

#### Verify and close

- [x] Check that the equations, conventions, units, validation behaviour, and exclusions agree across the theory, specification, implementation companion, public API, and deterministic tests.
- [x] Re-run the package test suite and link the result if the accepted source has changed; otherwise preserve the existing evidence.
- [x] Close `P1.1` without reimplementing accepted work.

Evidence:

- [Theory](notes/03_kinematics_and_numerical_integration.qmd)
- [Implementation companion](notes/04_kinematics_and_numerical_integration_implementation.qmd)
- [Planar rigid-body theory](notes/06_planar_rigid_body_motion_se2_and_twists.qmd)
- [Specification](docs/01_motion_models/differential_drive_motion_model.md)
- [Package and deterministic tests](ros_ws/src/differential_drive_motion_model)

### P1.2 — Spatial geometry kernel

The outcome is the geometry layer of the spatial and general kinematics workbench. Preserve the current uncommitted Chapter 7 work.

#### Learn and review

- [x] Finish `notes/07_degrees_of_freedom_and_spatial_motion_so3_se3.qmd` with a spatial-twist retrieval test, cumulative chapter test and answers, representation limitations, numerical checks, and implementation implications.
- [x] Review and approve Chapter 7 one small learning block at a time.
- [x] After approval, reconcile its already-started notation entries and add only its newly approved glossary terms.
- [x] Reconcile `notes/_quarto.yml` with the reviewed source set, render and inspect Chapter 7, and remove stale generated pages that are not requested deliverables.
- [x] Preserve the dynamics draft and its useful content through the earlier chapter-number migration.
- [x] Before drafting Chapter 8, route its prerequisite blocks through *Modern Robotics*, Chapter 3, and compare the formulation with Craig, Chapter 2, and Corke, Chapters 2–3.
- [x] Split the former Chapter 8 source without rewriting reviewed content: retain the spatial-geometry material through the motion-workflow summary and the Chasles--Mozzi appendix in `notes/08_screw_motion_se3_exponential_and_spatial_integration.qmd`; move the material beginning at “Configuration coordinates and forward kinematics” into `notes/10_general_robot_kinematics_and_jacobians.qmd`; update chapter introductions, cross-references, `notes/_quarto.yml`, and local links in the same batch.
- [x] In the same chapter-renumbering batch, rename the preserved dynamics draft to `notes/16_dynamic_systems_mechanics_and_actuator_limits.qmd` without rewriting or losing its content, and update every repository reference.
- [x] Draft and review the screw-axis, pitch, and revolute-versus-prismatic motion block in `notes/08_screw_motion_se3_exponential_and_spatial_integration.qmd`.
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
- [x] Implement, test, understand, and retrospectively document the $SO(3)$ and $SE(3)$ composition-and-inversion mathematical cycle, including matrix reconstruction needed to inspect the results; preserve the durable explanation in the [composition-and-inversion companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-composition-inversion) and the deterministic evidence in the [`rigid_body_kinematics` tests](ros_ws/src/rigid_body_kinematics/test).
- [x] Run the checklist-declared intermediate C++ quality gate after the representations, vector and point operations, composition, inversion, public API, and package structure are stable; verify the clean package build, all registered functional and lint tests, installed downstream CMake target, public interface, mathematical equations, frames, units, validation, and numerical failure behaviour.
- [x] Implement, test, understand, and retrospectively document the $SO(3)$ and linear-first $SE(3)$ hat-and-vee mathematical cycle; preserve the reviewed explanation in the [hat-and-vee companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-hat-vee) and its 16 deterministic cases in [`test_lie_algebra.cpp`](ros_ws/src/rigid_body_kinematics/test/test_lie_algebra.cpp).
- [x] Implement, test, understand, and retrospectively document the $SO(3)$ exponential mathematical cycle, including its exact-zero, stable small-angle, and nominal Rodrigues branches; preserve the reviewed explanation in the [$SO(3)$ exponential companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-so3-exponential) and its nine deterministic cases in [`test_so3_exponential.cpp`](ros_ws/src/rigid_body_kinematics/test/test_so3_exponential.cpp).
- [x] Implement, test, understand, and retrospectively document the principal $SO(3)$ logarithm mathematical cycle, including its identity, small-angle, nominal-angle, and deterministic near-$\pi$ branches; preserve the reviewed explanation in the [principal $SO(3)$ logarithm companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-so3-logarithm) and its deterministic evidence in [`test_so3_logarithm.cpp`](ros_ws/src/rigid_body_kinematics/test/test_so3_logarithm.cpp).
- [x] Implement, test, understand, and retrospectively document the linear-first $SE(3)$ exponential mathematical cycle, keeping translational coordinates in metres and rotational coordinates in radians distinct; preserve the reviewed explanation in the [$SE(3)$ exponential companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-se3-exponential) and its five deterministic cases in [`test_se3_exponential.cpp`](ros_ws/src/rigid_body_kinematics/test/test_se3_exponential.cpp).
- [x] Implement, test, understand, and retrospectively document the principal $SE(3)$ logarithm mathematical cycle using the completed principal $SO(3)$ logarithm; preserve the reviewed explanation in the [principal $SE(3)$ logarithm companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-se3-logarithm) and its 11 deterministic cases in [`test_se3_logarithm.cpp`](ros_ws/src/rigid_body_kinematics/test/test_se3_logarithm.cpp).
- [x] Implement, test, understand, and retrospectively document the constant-twist-integration mathematical cycle using the completed $SE(3)$ exponential and explicit body-versus-space multiplication order; preserve the reviewed explanation in the [constant-twist-integration companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-constant-twist-integration) and its 10 deterministic cases in [`test_constant_twist_integration.cpp`](ros_ws/src/rigid_body_kinematics/test/test_constant_twist_integration.cpp).
- [x] Implement, test, understand, and retrospectively document the $SE(3)$ adjoint mathematical cycle using the project's linear-first twist order; preserve the reviewed explanation in the [adjoint companion section](notes/09_spatial_geometry_kernel_implementation.qmd#sec-spatial-kernel-se3-adjoint) and its five deterministic cases in [`test_se3_adjoint.cpp`](ros_ws/src/rigid_body_kinematics/test/test_se3_adjoint.cpp).
- [ ] **Active:** Implement, test, understand, and retrospectively document planar-pose embedding and extraction plus normalised planar-yaw quaternion conversion without attempting to replace `tf2`.

#### Verify and close

- [ ] Test identity, inverse round trips, noncommuting composition order, known rotations about two axes, point-versus-vector behaviour, planar embedding, and quaternion sign equivalence.
- [ ] Test exponential/logarithm round trips away from declared branch ambiguities and the identity $\widehat{\operatorname{Ad}_{T}\boldsymbol\xi}=T\widehat{\boldsymbol\xi}T^{-1}$.
- [ ] Test the identity and small-angle series path, deterministic near-$\pi$ axis handling, and documented rejection or canonicalisation at ambiguous logarithm branches.
- [ ] Reject non-finite, malformed, non-orthogonal, reflective, and inapplicable non-planar inputs without returning a valid-looking result.
- [ ] Run the checklist-declared pre-closure C++ quality gate across the stable package, public API, Eigen representations, CMake export, and focused GTest evidence.
- [ ] Link the approved note blocks, specification, companion, package, and tests and close `P1.2`.

### P1.3 — General robot kinematics and Jacobians

The outcome is a bounded reusable implementation of general forward and velocity kinematics that the next educational manipulator capability and later robot products can share.

#### Cycle 1 — Space-form product-of-exponentials forward kinematics

- [x] Draft the prerequisite opening block of `notes/10_general_robot_kinematics_and_jacobians.qmd` on configuration space versus coordinate vector, task variables, the forward map $T(\mathbf q)$, the total derivative, and the multivariable chain rule.
- [ ] Review the prerequisite opening block and complete its retrieval test before continuing the Cycle 1 learning stage.
- [ ] Learn and write the home configuration, validated revolute and prismatic space screw axes, transform-chain equivalence, multiplication order, and space-form product of exponentials; route the block through *Modern Robotics*, Chapter 4, compare it with Craig, Chapter 3, and use Corke, Chapter 7, as an independent numerical viewpoint.
- [ ] Review the space-form block and complete its retrieval questions on notation, frames, units, product order, application, and supported serial-chain assumptions.
- [ ] Create `docs/02_spatial_kinematics/general_kinematics_and_jacobians.md` with the shared serial-chain contract and the space-form operation's inputs, outputs, validation, exclusions, and independent analytic acceptance cases.
- [ ] Implement space-form forward kinematics in `rigid_body_kinematics` with focused tests for zero joint displacement, single revolute and prismatic joints, invalid models, and a planar 2R chain checked against independent trigonometric kinematics.
- [ ] After the implementation and focused tests work, create the first substantive section of `notes/11_general_robot_kinematics_and_jacobians_implementation.qmd` and review its serial-chain representation, equation-to-code mapping, multiplication order, public interface, validation, and test meaning.
- [ ] Run the focused package build and acceptance cases, inspect the affected artifacts, link the evidence, and close the space-form cycle.

#### Cycle 2 — Body-form product-of-exponentials forward kinematics

- [ ] Learn and write body screw axes, the body-form product, its multiplication order, and its equivalence to the space form and the declared transform chain using *Modern Robotics*, Chapter 4, and the selected supporting sources.
- [ ] Review the body-form block and complete its retrieval questions on frame meaning, axis conversion, product order, equivalence, and supported inputs.
- [ ] Extend the general-kinematics specification with body-form inputs, validation, exclusions, and fixed space/body pose-equivalence cases.
- [ ] Implement body-form forward kinematics with focused tests for independent single-joint cases and agreement with the space form on the planar 2R and one small non-planar chain.
- [ ] Write and review the retrospective body-form companion section after its implementation and tests work.
- [ ] Run the focused package build and acceptance cases, inspect the affected artifacts, link the evidence, and close the body-form cycle.

#### Cycle 3 — Space Jacobian

- [ ] Learn and write spatial end-effector velocity, the derivation of the space Jacobian from the pose map, and the configuration-dependent transformation of its columns using *Modern Robotics*, Chapter 5, Craig, Chapter 5, and Corke, Chapter 8.
- [ ] Review the space-Jacobian block and complete its retrieval questions on column meaning, frames, units, differentiation, application, and finite-difference verification.
- [ ] Extend the specification with the space-Jacobian contract, validation, exclusions, analytic fixtures, and finite-difference acceptance method.
- [ ] Implement the space Jacobian for a validated $n$-joint open chain with independent zero-configuration checks and finite-difference velocity checks on the planar 2R and one small non-planar chain.
- [ ] Write and review the retrospective space-Jacobian companion section after its implementation and tests work.
- [ ] Run the focused package build and acceptance cases, inspect the affected artifacts, link the evidence, and close the space-Jacobian cycle.

#### Cycle 4 — Body Jacobian and frame conversion

- [ ] Learn and write body end-effector velocity, the body Jacobian, and the body–space adjoint relation.
- [ ] Review the body-Jacobian block and complete its retrieval questions on frame conversion, multiplication direction, equivalence, application, and adjoint verification.
- [ ] Extend the specification with the body-Jacobian contract and fixed body–space adjoint acceptance cases.
- [ ] Implement the body Jacobian with independent finite-difference checks and deterministic verification of the body–space adjoint relation.
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
- [ ] Record the actual effort and elapsed time for `P1.1`–`P1.3` so the scope checkpoint after the first planar-manipulator stage has a reliable baseline.

### P1.4 — Planar manipulator inverse-kinematics laboratory

The outcome is the first stage of one educational planar-manipulator side project. It turns the reusable kinematics from `P1.3` into inspectable 2R/3R inverse-kinematics experiments without starting the Phase 8 manipulation product.

#### Learn and review

- [ ] Draft `notes/12_inverse_kinematics_redundancy_and_planar_manipulators.qmd` from *Modern Robotics*, Chapter 6, while reusing Chapters 4–5, supported by Craig, Chapters 3–5, and Corke, Chapters 7–8.
- [ ] Define an inverse-kinematics problem from the forward map and a declared task error; distinguish existence, reachability, uniqueness, branch choice, exact solutions, approximate solutions, convergence, and solver failure.
- [ ] Derive closed-form position inverse kinematics for a planar 2R arm, including its reachable annulus, elbow-up and elbow-down branches, boundary cases, and the effect of joint limits.
- [ ] Analyse the 2R position-task Jacobian through rank and singular values and connect task-specific rank loss to singular configurations and numerical conditioning.
- [ ] Derive Newton–Raphson inverse kinematics from first-order task linearisation and then derive the Moore–Penrose-pseudoinverse update for square, overdetermined, underdetermined, and rank-deficient local models.
- [ ] Derive damped least squares as a regularised local least-squares problem with explicit task units, frames, stopping tolerances, step bounds, damping policy, and near-singular behaviour; state that damping guarantees neither reachability nor convergence.
- [ ] For a 3R planar arm with a two-dimensional position task, derive redundancy, the exact Moore–Penrose null-space projector, one projected joint-centering objective, and a bounded position-task manipulability measure.
- [ ] Show that a damped inverse produces only an approximate projector that can disturb the primary task, and that null-space bias does not enforce hard joint limits or solve a general constrained inverse-kinematics problem.
- [ ] Relate repeated resolved-rate updates to numerical inverse kinematics while deferring dynamic task-space control, collision constraints, grasping, contact, and global motion planning.
- [ ] Add worked reachable, unreachable, multiple-solution, singular, near-singular, joint-limit, and redundant cases; add retrieval tests, a cumulative test with answers, limitations, review, notation, glossary, `notes/_quarto.yml`, and render evidence.

#### Specify

- [ ] Create `docs/02_spatial_kinematics/planar_manipulator_kinematics_lab.md` with fixed 2R and 3R fixtures, link lengths, joint conventions and limits, task definitions, solver inputs and outputs, SI units, frames, tolerances, iteration and step limits, damping and seed policies, status values, invalid-input behaviour, exclusions, and frozen acceptance cases.
- [ ] Define the independent evidence paths before implementation: planar trigonometric closed forms for 2R pose and branches, finite differences for Jacobians, forward evaluation for task residual, and null-space residual for the 3R secondary objective.

#### Implement and document

- [ ] When implementation begins, create `notes/13_planar_manipulator_kinematics_implementation.qmd` with equation-to-code mappings, angle normalisation, branch enumeration, solver state, stopping order, damping and step policy, joint-limit handling, status semantics, and verification oracles.
- [ ] Create `ros_ws/src/planar_manipulator_lab` as a pure C++ `ament_cmake` package that depends on `rigid_body_kinematics`, Eigen, and GTest but has no ROS runtime, URDF, Pinocchio, MoveIt 2, `ros2_control`, Gazebo, collision, or contact dependency.
- [ ] Implement the independent closed-form 2R position solver, a bounded iterative pseudoinverse/damped-least-squares solver that consumes the reusable forward map and Jacobian, and the declared 3R exact-pseudoinverse null-space joint-centering experiment without duplicating the general kinematics core.
- [ ] Add a deterministic offline scenario runner that writes a versioned machine-readable result table; use a small Python plotting script only to visualise arm configurations, residual histories, branch choices, and joint-limit behaviour from those results.

#### Verify and close

- [ ] Verify 2R forward/inverse round trips throughout the reachable annulus, both elbow branches where they exist, inner and outer boundary cases, and the declared angle-equivalence convention.
- [ ] Verify explicit unreachable, singular, near-singular, non-converged, invalid-dimension, invalid-model, and non-finite outcomes without returning a valid-looking success result; verify deterministic filtering of analytic branches that violate joint limits.
- [ ] Compare analytic and numerical solutions on frozen targets; verify residual and iteration thresholds, deterministic results for fixed seeds, and bounded damped-least-squares behaviour near singularities. When a numerical candidate violates limits or no valid candidate is found, require non-success without claiming that the globally constrained problem is infeasible.
- [ ] Verify the 3R position-task Jacobian and exact null-space projection independently, show that the secondary joint-centering velocity preserves the primary task to tolerance, demonstrate the declared leakage from the damped approximate projector, and verify the declared manipulability behaviour at regular and singular configurations.
- [ ] Link the approved theory, specification, companion, package, deterministic tests, scenario manifest, and plots and close the kinematics stage of `planar_manipulator_lab`.
- [ ] Compare actual effort and elapsed time for `P1.1`–`P1.4` with the revised 14–18 week Phase 1 duration; ask the user to revise duration or deliberately narrow remaining scope if the estimate is no longer credible rather than silently skipping required learning or evidence.

### P1.5 — Wheeled constraints and wheel-odometry pipeline

The outcome is one deterministic differential-drive odometry pipeline. The bicycle model remains a learning comparison.

#### Learn and review

- [ ] Draft `notes/14_wheeled_robot_constraints_and_odometry.qmd` from *Modern Robotics*, Chapter 13, supported by Corke, Chapter 4, *Probabilistic Robotics*, Chapter 5, and LaValle, Chapter 13.
- [ ] Derive rolling and no-slip constraints in Pfaffian form, distinguish holonomic from nonholonomic constraints, and relate $A(\mathbf q)\dot{\mathbf q}=0$ to $\dot{\mathbf q}=G(\mathbf q)\mathbf u$.
- [ ] Derive the full and reduced differential-drive models, wheel/body Jacobians, feasible wheel-speed set, cumulative encoder increments, timestamped pose and twist update, and exact $SE(2)$ odometry integration.
- [ ] Derive Ackermann steering geometry, the bounded kinematic bicycle model, $\kappa=\tan\delta/L$, and minimum turning radius; compare its feasible motions with differential drive without implementing a car.
- [ ] Explain calibration error, unequal effective radii, quantisation, slip, skid, drift, stale data, and why Phase 1 odometry cannot claim a calibrated covariance.
- [ ] Add retrieval tests, cumulative test and answers, limitations, review, notation, glossary, `notes/_quarto.yml`, and render evidence.

#### Specify

- [ ] Create `docs/03_wheel_odometry/differential_drive_odometry.md` with cumulative-wheel-angle and timestamp inputs, pose and body-twist outputs, frames, SI units, wheel identifiers, initialisation, state-mutation rules, covariance-field policy, ROS adapter contract, exclusions, and acceptance cases.

#### Implement and document

- [ ] When implementation begins, create `notes/15_wheeled_robot_constraints_and_odometry_implementation.qmd` covering encoder differencing, timestamp validation, state ownership, ROS message semantics, `tf2`, parameters, and the pure-core/adapter boundary.
- [ ] Create `ros_ws/src/differential_drive_odometry` as an `ament_cmake` package with a pure modern C++ odometry core and a thin C++ ROS 2 adapter; preserve the accepted Python `differential_drive_motion_model` as an independent equation-level reference rather than a runtime dependency or a target for retroactive rewriting.
- [ ] Implement a pure odometer state machine that accepts cumulative wheel angles and a timestamp and returns planar pose, body twist, status, and timestamp.
- [ ] Add a thin ROS 2 node that consumes `sensor_msgs/JointState` and publishes consistent `nav_msgs/Odometry` and `odom` to `base_link` transforms through `tf2`.

#### Verify and close

- [ ] Test initialisation without false displacement, no motion, straight motion, rotation in place, curved motion, arbitrary initial heading, and equivalent constant motion under different valid sample intervals.
- [ ] Test wheel ordering, SI units, duplicate and out-of-order timestamps, missing joints, non-finite samples, invalid geometry, and the rule that invalid updates do not mutate state.
- [ ] Verify agreement among message pose, quaternion, body twist, timestamp, and transform; state explicitly that covariance is not yet calibrated.
- [ ] Link the approved theory, specification, companion, pure tests, and ROS adapter tests and close `P1.5`.

### P1.6 — Dynamic systems, mechanics, and differential-drive planar plant

The outcome is a deterministic plant that makes actuator limits and controller comparisons physically meaningful without pretending to be a high-fidelity tyre or motor model.

#### Learn and review

- [ ] Before drafting, route dynamic-system concepts through *Feedback Systems*, Chapters 3, 5, and 6; route mechanics through *Modern Robotics*, Chapter 8, supported by Craig, Chapters 6 and 8, Corke, Chapter 9, and LaValle, Section 13.2.4, for the smooth differential-drive state model.
- [ ] Continue `notes/16_dynamic_systems_mechanics_and_actuator_limits.qmd`, preserving all useful material from the renamed draft.
- [ ] Define state, input, output, the vector ODE $\dot{\mathbf x}=f(\mathbf x,\mathbf u,t)$, conversion of higher-order mechanics to first-order state equations, equilibrium, and the local linearisation needed to construct later reference models; defer full continuous/discrete stability, controllability, observability, Lyapunov, Riccati, and LQR treatment to `P1.8`.
- [ ] Generalise numerical integration from scalar Forward Euler to vector state, compare it with one declared higher-order reference method, and explain accuracy, convergence, step size, and numerical stability.
- [ ] Derive planar Newton–Euler longitudinal and yaw dynamics, mass moment of inertia, unequal wheel-force yaw moment, wheel torque to traction, gearing, and simple resistance; for two declared identical driven wheels, reflect wheel inertia into effective longitudinal mass and yaw inertia through the ideal no-slip relation, and define bare chassis inertias so the wheel contribution is not counted twice.
- [ ] Derive static traction bounds, demanded-versus-realised force, one bounded friction model, wheel-torque magnitude and slew-rate limits, braking, and stopping-distance reasoning; explain torque-speed, current, power, and detailed electrical models only as explicit deferrals.
- [ ] Add worked analytic and numerical cases, retrieval tests, cumulative test and answers, and model limitations, then review and approve the chapter.
- [ ] After approval, add only the newly approved dynamics notation and glossary terms, update `notes/_quarto.yml`, and render and inspect the chapter.

#### Specify

- [ ] Create `docs/04_motion_control/motion_stack_interfaces.md` and freeze the semantic chain `trajectory sample → nonholonomic tracker → body/wheel velocity target → wheel-effort regulator → safety supervisor → simulator wheel-effort adapter`, including types, SI units, timestamps, feedback sources, and the distinct limit owners named above.
- [ ] Create `docs/decisions/0001_phase_01_gazebo_wheel_effort_interface.md` from current official Gazebo and ROS 2 documentation; select a supported wheel-effort boundary that exercises the Phase 1 regulator without `cmd_vel` bypass or early `ros2_control` adoption, and stop for a scope decision if no minimal supported boundary exists.
- [ ] Create `docs/04_motion_control/differential_drive_planar_plant.md` with state $(x,y,\theta,v,\omega)$, wheel-torque inputs, bare chassis mass and yaw inertia, identical-wheel and ideal-no-slip assumptions, reflected wheel inertia without double counting, geometry, resistance and friction parameters, wheel-torque magnitude and slew limits, traction limits, disturbance inputs, integration policy, outputs, validity ranges, exclusions, and analytic acceptance cases.

#### Implement and document

- [ ] When implementation begins, create `notes/17_dynamic_systems_mechanics_and_actuator_limits_implementation.qmd` with state layout, force and moment balance, reflected wheel inertia, saturation order, integrator choice, configuration, and diagnostics.
- [ ] Create `ros_ws/src/differential_drive_planar_plant` as a deterministic C++ `ament_cmake` package with a pure library and GTest suite.
- [ ] Implement symmetric and differential wheel-torque dynamics, declared resistance, actuator saturation, traction limiting, external longitudinal force and yaw-moment disturbances, and explicit actuator-limited and traction-limited status.
- [ ] Implement Forward Euler for comparison and the reviewed higher-order method as the reference simulation path.

#### Verify and close

- [ ] Verify rest equilibrium, symmetric-torque straight acceleration, antisymmetric-torque yaw acceleration, zero-net-yaw symmetry, known constant-force motion, and declared stopping cases.
- [ ] Verify numerical convergence against an analytic one-dimensional case and check energy, sign, unit, and bound invariants where the model supports them.
- [ ] Verify that torque and realised traction never exceed declared limits and that saturation and disturbance status is observable.
- [ ] Link the approved theory, specification, companion, package, and deterministic tests and close `P1.6`.

### P1.7 — Bounded straight-and-arc trajectory generator

The outcome is the smallest trajectory set needed for the Phase 1 controller comparison.

#### Learn and review

- [ ] Draft `notes/18_bounded_trajectory_generation.qmd` from *Modern Robotics*, Chapters 9 and 13, supported by Craig, Chapter 7, and Corke, Chapters 3–4.
- [ ] Distinguish path, trajectory, and sampled command sequence; define path coordinate $s$, time scaling $s(t)$, and derive pose, velocity, acceleration, curvature, angular rate, and jerk through the chain rule.
- [ ] Derive straight and constant-curvature differential-drive paths, $\omega=v\kappa$, wheel-speed and wheel-acceleration feasibility, and endpoint continuity.
- [ ] Derive one bounded quintic time scaling with duration selected from analytic derivative limits; use a trapezoidal profile only as a comparison that exposes acceleration or jerk discontinuities.
- [ ] Distinguish invalid input from geometric or wheel-demand infeasibility that increasing duration cannot resolve, derive the sampling effects, and require an exact final sample when duration is not an integer multiple of the sample period; defer arbitrary splines, obstacle avoidance, and time-optimal planning.
- [ ] Add worked cases, retrieval tests, cumulative test and answers, limitations, review, notation, glossary, `notes/_quarto.yml`, and render evidence.

#### Specify

- [ ] Create `docs/04_motion_control/bounded_trajectory_generation.md` with start pose, signed path length, constant curvature, sample period, declared speed, acceleration, angular, jerk, curvature, and wheel limits, output sample schema, invalid-input behaviour, exclusions, and fixed acceptance cases.

#### Implement and document

- [ ] When implementation begins, create `notes/19_bounded_trajectory_generation_implementation.qmd` with duration selection, analytic derivatives, sample construction, constraint checker, and floating-point tolerances.
- [ ] Create `ros_ws/src/differential_drive_trajectory` as a pure C++ `ament_cmake` package with no ROS runtime dependency.
- [ ] Implement deterministic straight and constant-curvature trajectories that start and finish at rest and expose time, pose, $v_d$, $\omega_d$, acceleration, angular acceleration, jerk, curvature, and wheel demands.
- [ ] Implement the constraint checker as a test/evaluation oracle that does not share the generator's duration-selection path.

#### Verify and close

- [ ] Verify exact declared start and end poses, monotonic time and path progress, zero endpoint speed and acceleration, $\omega_d=\kappa v_d$, and independent circle geometry.
- [ ] Verify every sample against declared speed, acceleration, angular, jerk, curvature, wheel-speed, and wheel-acceleration limits.
- [ ] Verify deterministic sampling, inclusion of the exact final sample, and explicit rejection of invalid limits, time steps, and requests that are infeasible under the declared geometry or wheel constraints.
- [ ] Link the approved theory, specification, companion, package, and deterministic tests and close `P1.7`.

### P1.8 — State-space systems and LQR reference laboratory

The outcome is a bounded mathematical reference laboratory for the state-space and linear-control results that later estimation, optimal-control, manipulator, and UAV capabilities reuse. It is not a general control toolbox and does not move Phase 5 optimal-control methods into Phase 1.

#### Learn and review

- [ ] Draft `notes/20_state_space_stability_and_lqr.qmd` primarily from *Feedback Systems*, Chapters 3 and 5–7, supported by *Principles of Robot Autonomy*, Chapter 3, and *Modern Robotics*, Chapter 11.
- [ ] Define continuous- and discrete-time nonlinear state-space models, state, input, output, equilibrium, nominal trajectory, and the units and physical meaning of every state-space component.
- [ ] Derive local linearisation about an equilibrium and along a nominal trajectory using the dynamics and output Jacobians; distinguish continuous-time and discrete-time models and verify the approximation domain rather than treating a local model as globally exact.
- [ ] Classify continuous- and discrete-time linear stability from eigenvalues; define controllability and observability, derive their finite-dimensional matrices and rank tests, and distinguish controllability from stabilisability and observability from detectability.
- [ ] Introduce Lyapunov stability through positive-definite quadratic functions and the continuous- and discrete-time Lyapunov equations; state what the bounded analysis proves and what it does not prove for a nonlinear system.
- [ ] Derive finite- and infinite-horizon continuous- and discrete-time LQR, the quadratic state and control cost, Riccati recursion or equation, feedback sign, gain dimensions and units, and closed-loop dynamics.
- [ ] Explain that LQR assumes a declared linear model, full state or an independently supplied state estimate, quadratic cost, and no hard constraints; defer finite-horizon nonlinear optimal control, shooting, collocation, differential flatness, iLQR, DDP, nonlinear MPC, dynamic programming, and POMDPs to Phase 5.
- [ ] Add worked stable, unstable, uncontrollable, unobservable, stabilisable, detectable, Lyapunov, continuous-LQR, and discrete-LQR cases; add retrieval tests, a cumulative test with answers, limitations, review, notation, glossary, `notes/_quarto.yml`, and render evidence.

#### Specify

- [ ] Create `docs/04_motion_control/linear_state_space_control_lab.md` with fixed continuous and discrete fixtures, state and input definitions and SI units, equilibrium and nominal-trajectory data, linearisation policy, rank and stability tolerances, cost matrices, result schemas, invalid-input behaviour, exclusions, and analytic acceptance cases.
- [ ] Freeze independent evidence paths before implementation: analytic Jacobians and finite differences for linearisation, hand-checkable eigenvalue and rank cases, Riccati-equation residuals, and independently calculated closed-loop eigenvalues and costs.

#### Implement and document

- [ ] When implementation begins, create `notes/21_state_space_stability_and_lqr_implementation.qmd` with model shapes, continuous/discrete conventions, derivative evaluation, rank tolerance, eigenvalue classification, cost-matrix validation, Riccati-solver boundary, gain sign, result schema, and failure behaviour.
- [ ] Create `ros_ws/src/linear_systems_control_lab` as a pure modern C++ `ament_cmake` package using Eigen and deterministic GTest checks without a ROS runtime, simulator, or general nonlinear-optimisation dependency; retain Python and SciPy only as an independent numerical reference path.
- [ ] Implement the bounded model evaluation, equilibrium and dimension checks, analytic or finite-difference linearisation path, controllability and observability matrices, stability classification, Lyapunov residual checks, and continuous/discrete LQR adapter in C++; place the Riccati calculation behind a narrow established-solver boundary selected by the specification, verify it independently with SciPy, and do not implement a general Riccati solver.
- [ ] Add one deterministic scenario runner for the frozen reference cases and a versioned model-and-gain export schema that `P1.10` can use after its 2R specification is frozen; do not precompute manipulator gains before the owning model and scenario exist.

#### Verify and close

- [ ] Verify analytic linearisation against finite differences and known scalar or low-dimensional models at equilibria and along a nominal trajectory; expose the approximation error outside the declared local region.
- [ ] Verify stable and unstable continuous/discrete fixtures, controllable and uncontrollable fixtures, observable and unobservable fixtures, and the declared stabilisable and detectable cases without relying only on the implementation under test.
- [ ] Verify continuous and discrete Riccati residuals, gain dimensions and sign, closed-loop eigenvalues, quadratic cost behaviour, and agreement with hand-checkable or independent reference cases.
- [ ] Reject dimension mismatches, non-finite values, invalid sample periods, invalid rank tolerances, nonsymmetric or inadmissible cost matrices, and unsupported Riccati cases without returning a valid-looking result.
- [ ] Link the approved theory, specification, companion, package, scenario fixtures, and deterministic tests and close `P1.8`; the selected robot application remains owned by `P1.10`.

### P1.9 — Feedback, PID, and offline controller benchmark

The outcome is a reproducible controller comparison on the same plant, trajectories, scenarios, metrics, and compute path.

#### Learn and review

- [ ] Draft `notes/22_feedback_stability_and_practical_pid_control.qmd` primarily from *Feedback Systems*, Chapters 3, 5–6, and 11, supported by *Modern Robotics*, Chapter 11 and Section 13.3.4, Corke, Chapter 9, and the approved `P1.8` state-space chapter without repeating its linear-systems derivations.
- [ ] Define plant, reference, output, error, sensor, controller, actuator, feedforward, open loop, closed loop, and disturbance paths before deriving a controller.
- [ ] Derive first- and second-order error dynamics, equilibria, poles or eigenvalues, local stability, damping ratio, natural frequency, steady-state error, overshoot, settling time, and control effort.
- [ ] Derive the roles and limitations of P, PI, PD, and PID; include constant-disturbance rejection, sampled implementation, derivative filtering, saturation, integrator windup, and one explicit anti-windup method.
- [ ] Explain why delay reduces stability margin, how noise reaches derivative action, how model mismatch changes the response, and where local linear reasoning ceases to justify a claim; consult *Feedback Systems*, Sections 9.2 and 10.3 or its Chapter 14 delay discussion for the delay claim and Sections 2.3–2.5 for disturbance, tracking, and robustness principles without expanding into a full frequency-domain design chapter.
- [ ] Define a nonholonomic-compatible control architecture; do not apply three independent PID loops directly to world-frame $x$, $y$, and heading.
- [ ] Add worked cases, retrieval tests, cumulative test and answers, limitations, review, notation, glossary, `notes/_quarto.yml`, and render evidence.

#### Specify

- [ ] Create `docs/04_motion_control/differential_drive_control_benchmark.md` before controller implementation; depend on `differential_drive_trajectory`, `differential_drive_planar_plant`, and the shared motion-stack interfaces rather than copying them, then freeze controller layers and names, component-level scenarios, sample time, deterministic noise and latency schedules, disturbances, model mismatches, seeds, metrics, thresholds, and exclusions.
- [ ] Define the compared stacks explicitly: feedforward/open-loop, proportional feedback, and a PID-based stack with derivative filtering, controller-output limiting to the shared wheel-effort contract, and anti-windup; do not make “PID always wins” an acceptance criterion.
- [ ] Define the offline feedback contract: controllers receive only the declared simulated measurement with injected measurement effects, while raw plant state is evaluator-only; the later Gazebo controller receives odometry and joint-state feedback, never simulator ground truth.
- [ ] Define a durable results manifest containing configuration, seeds, software revision, metric schema, commands, summary results, and hashes or retention locations for generated traces; do not make bulky transient logs an unnamed evidence store.

#### Implement and document

- [ ] When implementation begins, create `notes/23_feedback_stability_and_practical_pid_control_implementation.qmd` with error coordinates, discrete state, gain units, update order, saturation and anti-windup order, latency buffer, telemetry schema, and reproducibility controls.
- [ ] Create `ros_ws/src/differential_drive_control` as a C++ `ament_cmake` package with pure controller and experiment-runner libraries independent of ROS and Gazebo.
- [ ] Implement the frozen feedforward, proportional, and PID-based stacks without changing their contracts between scenarios.
- [ ] Implement deterministic injection of bounded sensor noise and command latency, and configure the depended-on plant to realise wheel-effort saturation, traction limits, external force and yaw-moment disturbance, and declared parameter mismatch; do not reimplement plant constraints in the benchmark runner.
- [ ] Compute tracking error, overshoot, settling time, control effort, trajectory smoothness, and every declared constraint violation from the same logged series.

#### Verify and close

- [ ] Verify equilibrium and simple known controller responses before running trajectory comparisons.
- [ ] On the precise inner velocity plant for which the error dynamics were derived, demonstrate the predicted proportional steady-state error under a constant load and its bounded correction by integral action; do not claim this behaviour for every proportional trajectory controller.
- [ ] Verify that anti-windup bounds the integrator and recovers after saturation and that the filtered derivative has the declared response to measurement noise.
- [ ] Run every frozen straight and curved scenario for every controller stack; preserve negative results and do not retune per scenario unless the benchmark explicitly allows it.
- [ ] Use Python only to generate reproducible tables and plots from the recorded series; keep acceptance calculations in the tested benchmark path.
- [ ] Create `docs/reports/phase_01_controller_benchmark.md` with interpreted offline results, negative results, limitations, and links to the durable manifest and deterministic tests.
- [ ] Link the approved theory, specification, companion, package, benchmark report, and deterministic tests and close `P1.9`; final Gazebo acceptance remains owned by `P1.13`.

### P1.10 — Planar manipulator dynamics and classical-control laboratory

The outcome is the second and final stage of `planar_manipulator_lab`: a deterministic vertical-plane 2R plant and a frozen comparison of classical joint controllers. It applies the shared mechanics, trajectory, and feedback learning without importing the Phase 8 manipulation stack.

#### Learn and review

- [ ] Draft `notes/24_planar_manipulator_dynamics_and_classical_control.qmd` primarily from *Modern Robotics*, Chapters 8 and 11, supported by Craig, Chapters 6 and 9–10, Corke, Chapter 9, and the already approved state-space, LQR, and feedback chapters.
- [ ] Define the fixed-base vertical-plane 2R model with link length, mass, centre-of-mass distance, planar mass moment of inertia, joint position and velocity, gravity, viscous joint friction, joint limits, and torque limits, including the frames and SI units of every quantity.
- [ ] Derive kinetic and potential energy and then the Euler–Lagrange equations in the form $M(\mathbf q)\ddot{\mathbf q}+\mathbf c(\mathbf q,\dot{\mathbf q})+\mathbf g(\mathbf q)+D\dot{\mathbf q}=\boldsymbol\tau$; distinguish the Coriolis/centrifugal bias vector from one possible matrix representation.
- [ ] Derive forward dynamics, inverse dynamics, static gravity torque, and the symmetry and positive-definiteness properties of the mass matrix over the declared model domain.
- [ ] Reuse the approved quintic time-scaling ideas to construct one fixed joint-space reference with position, velocity, and acceleration; distinguish joint-space tracking from end-effector task error.
- [ ] Derive joint-space PD, gravity-compensated PD, inverse-dynamics feedforward, and computed-torque PD with gain units, nominal error dynamics, torque saturation, and explicit limitations under model mismatch and unmodelled contact.
- [ ] Linearise the 2R state dynamics about one declared static equilibrium, define the state and input deviations, reuse `P1.8` to compute and freeze one LQR gain, and derive a local inverse-dynamics-feedforward plus LQR feedback trajectory-tracking application without implementing another Riccati solver.
- [ ] Explain why integral control, task-space force control, impedance/admittance control, collision avoidance, payload contact, and adaptive or robust control are not required by this laboratory.
- [ ] Add worked equilibrium, gravity, forward/inverse-dynamics, energy, setpoint, and trajectory-tracking cases; add retrieval tests, a cumulative test with answers, limitations, review, notation, glossary, `notes/_quarto.yml`, and render evidence.

#### Specify

- [ ] Create `docs/04_motion_control/planar_manipulator_dynamics_and_control.md` with the fixed 2R model, state, parameters, force and sign conventions, valid joint envelope, torque bounds, integration policy, controller interfaces, fixed references, declared LQR equilibrium, frozen gain provenance, local-validity region, disturbances, model-mismatch cases, metrics, deterministic seeds, status values, invalid-input behaviour, exclusions, and frozen acceptance thresholds. Treat a joint-limit crossing as a detected experiment violation and termination condition rather than inventing unmodelled hard-stop contact dynamics.
- [ ] Define a durable scenario and results manifest shared by every admitted controller; freeze the same initial states, references, sample times, disturbance schedules, nominal parameters, mismatched controller parameters, metrics, and retuning policy before implementation comparisons begin.

#### Implement and document

- [ ] When implementation begins, create `notes/25_planar_manipulator_dynamics_and_classical_control_implementation.qmd` with state layout, model-term evaluation, forward/inverse-dynamics order, integrator, saturation, controller update order, frozen LQR gain import and validation, gain units, telemetry, scenario configuration, and reproducibility controls.
- [ ] Extend `ros_ws/src/planar_manipulator_lab` with separate pure model, controller, simulation, metric, and experiment-runner components; do not couple it to the differential-drive plant or controller packages and do not add a ROS runtime or simulator adapter.
- [ ] Implement the fixed 2R mass matrix, Coriolis/centrifugal bias, gravity, viscous friction, forward and inverse dynamics, declared reference, deterministic integrator, torque saturation, joint-envelope monitoring, external joint-disturbance input, and model-mismatch injection.
- [ ] After the 2R specification is frozen, use the accepted `linear_systems_control_lab` path to independently calculate and export the declared local linear model and LQR gain; version the model, equilibrium, cost matrices, solver configuration, residuals, and gain consumed by the C++ laboratory.
- [ ] Implement the frozen joint-space PD, gravity-compensated PD, computed-torque PD, and admitted frozen-gain LQR tracking stacks without per-scenario retuning; keep the controller's nominal model distinct from the evaluator-owned plant parameters and do not duplicate the `P1.8` Riccati solver.
- [ ] Extend the offline runner and plotting path to record and display joint and end-effector errors, torque, effort, energy where applicable, saturation, disturbances, and constraint violations from one versioned result schema.

#### Verify and close

- [ ] Verify independent closed-form mass, bias, and gravity cases; mass-matrix symmetry and positive definiteness; static gravity compensation; and forward/inverse-dynamics round trips over the frozen valid domain.
- [ ] Verify the declared 2R equilibrium, analytic or independently checked local linearisation, exported Riccati residual, gain dimensions and units, and local closed-loop eigenvalues before applying the LQR controller.
- [ ] Verify rest and equilibrium cases, bounded zero-gravity and zero-friction energy error, numerical refinement behaviour, torque bounds, joint-envelope violation detection, saturation status, invalid-input rejection, and deterministic repeatability.
- [ ] Run every applicable controller on the same setpoint and quintic-reference scenarios under nominal parameters, declared disturbances, torque saturation, and frozen mass, inertia, and friction mismatch without per-scenario retuning; identify any scenario outside the LQR model's declared local-validity region rather than presenting it as an equivalent global controller comparison.
- [ ] Compare joint RMSE, end-effector position RMSE, maximum error, settling time where applicable, peak torque, integrated squared effort, saturation time, and constraint violations; preserve negative results and do not require computed torque to win every case.
- [ ] Create `docs/reports/phase_01_planar_manipulator_lab_verification.md` with interpreted kinematics, dynamics, and control results, the scenario-manifest link, limitations, negative results, and the explicit boundary to the Phase 8 product.
- [ ] Link both approved manipulator theory chapters, the state-space and feedback prerequisites, both manipulator specifications and companions, the package, deterministic tests, scenario manifest, plots, and verification report and close `P1.10`.

### P1.11 — Nominal ROS 2 and Gazebo closed-loop integration

The outcome is one headless nominal integration path that demonstrably exercises the accepted trajectory, odometry, controller, and wheel-effort interfaces. Simulator ground truth is evaluator-only.

#### Learn and review

- [ ] Consult current official ROS 2 and Gazebo documentation for simulation time, joint state, the selected wheel-effort boundary, launch, bridging, and headless execution; do not rely on stale textbook software guidance.
- [ ] Review the shared motion-stack interface specification and wheel-effort architecture decision against the installed environment before writing the integration specification.

#### Specify

- [ ] Create `docs/05_phase_01_integration/gazebo_closed_loop_integration.md` with the robot and world, launch graph, selected wheel-effort adapter, topics, frames, timestamps, parameters, nominal scenarios, evaluator-only ground truth, telemetry proving that the regulator output reaches the effort boundary, tolerances, exclusions, and acceptance commands.
- [ ] Limit this capability to no-motion, straight, rotation-in-place, and constant-curvature nominal cases; leave injected failures and final mobile-system thresholds to `P1.12` and `P1.13`.

#### Implement and document

- [ ] When implementation begins, create `notes/26_ros2_gazebo_integration_and_motion_safety_implementation.qmd` with the durable simulation-clock, launch, bridge, effort-adapter, frame, timestamp, and evaluator boundary.
- [ ] Create `ros_ws/src/differential_drive_gazebo` with the minimum robot, world, launch, nominal scenario runner, and evaluator required by the specification.
- [ ] Add thin ROS 2 adapters to the trajectory and controller packages; do not duplicate their pure algorithms in the Gazebo package.
- [ ] Integrate trajectory reference, odometry and joint-state feedback, controller wheel-effort request, the selected Gazebo effort adapter, `tf2`, and evaluator-only ground truth with explicit units, frames, and timestamps.
- [ ] Keep `differential_drive_planar_plant` as the offline benchmark rather than a second physics engine in the Gazebo loop, and do not use `cmd_vel` or simulator ground truth to bypass the accepted controller stack.

#### Verify and close

- [ ] Run the nominal headless scenarios and verify that controller inputs contain only odometry and declared joint-state measurements while ground truth reaches only the evaluator.
- [ ] Compare odometry with simulator ground truth on no-motion, straight, spin, and curved segments using the integration specification's tolerances.
- [ ] Trace one complete sample from trajectory reference through regulator wheel effort to Gazebo joint response and recorded odometry.
- [ ] Link the integration specification, implementation companion, packages, launch tests, and nominal results and close `P1.11`.

### P1.12 — Motion safety supervisor and fault acceptance

The outcome is separately owned, reusable command-safety logic verified through unit, integration, and declared simulation fault cases.

#### Learn and review

- [ ] Consult current official ROS 2 time, diagnostics, lifecycle or state-machine, and `rosbag2` MCAP documentation required by the safety boundary.
- [ ] Review why operational safety state, control stability, and physical actuator limits are different claims, and retain the explicit non-certified simulation-safety boundary for the specification.

#### Specify

- [ ] Create `docs/05_phase_01_integration/motion_safety_supervisor.md` with the raw and validated wheel-effort command schemas, safe envelope, source and sequence identity, clock choice, freshness semantics, state machine, priority rules, diagnostics, reset preconditions, stopped-safe output, exclusions, and deterministic acceptance cases.
- [ ] In the specification, distinguish operational safety state from control stability and physical actuator limits and bound every claim to the declared simulation and fault model.
- [ ] Specify command dropout separately from joint-state or odometry dropout, and specify duplicate, out-of-order, paused, and reset simulation timestamps without allowing invalid data to refresh a watchdog.
- [ ] Specify that emergency stop latches until an explicit safe reset, invalid or unsupported commands are rejected, controller output limits remain controller-owned, and the plant or Gazebo remains the owner of realised physical constraints.
- [ ] Specify the stopped-safe effort policy—zero effort, bounded braking, or a state-dependent combination—from the reviewed stopping model, including the conservative fallback when velocity feedback is stale or missing.
- [ ] Define the durable safety-evidence manifest and MCAP retention policy: commit scenario configuration, software revision, summary metrics, event manifest, and hashes or declared locations; do not commit bulky bags by default.

#### Implement and document

- [ ] Extend `notes/26_ros2_gazebo_integration_and_motion_safety_implementation.qmd` with command validation, watchdog clock, state transitions, emergency-stop priority, reset, diagnostics, recording, and shutdown behaviour only when implementation begins.
- [ ] Create `ros_ws/src/motion_safety_supervisor` with a pure deterministic state machine, focused unit tests, and a thin ROS 2 node; keep Gazebo assets and scenario orchestration in `differential_drive_gazebo`.
- [ ] Implement safe-envelope validation, sequence and timestamp checks, command and feedback watchdogs, latched emergency stop, explicit reset policy, stopped-safe output, and structured diagnostic status.
- [ ] Add fault scenarios and record reference, raw and validated efforts, odometry, joint states, ground truth, diagnostics, limit events, and safety interventions through `rosbag2` with MCAP storage.

#### Verify and close

- [ ] Verify pure state transitions, priority, timeout boundaries, invalid-input non-mutation, reset preconditions, and stopped-safe output deterministically.
- [ ] Inject invalid command, command-publisher dropout, joint-state dropout, odometry dropout, duplicate and out-of-order timestamps, simulation-time reset, and emergency stop in headless integration tests.
- [ ] Measure watchdog reaction time, stop time, stopping distance, intervention sequence, and diagnostic status for every applicable fault.
- [ ] Create `docs/reports/phase_01_motion_safety_verification.md` with interpreted fault results, failures, limitations, evidence-manifest link, and the simulation-only safety claim.
- [ ] Link the specification, companion, supervisor package, integration tests, safety report, and evidence manifest and close `P1.12`.

### P1.13 — Phase 1 system evaluation and learning closure

The outcome is the final Phase 1 conclusion. This capability evaluates accepted components; any defect returns to its owning package and specification rather than being patched only in the evaluator.

#### Learn and review

- [ ] Create `notes/27_geometry_mechanics_and_control_cumulative_review.qmd` with separate retrieval, derivation, application, and limitation blocks spanning frames, planar and spatial kinematics, Jacobians, inverse kinematics, redundancy, odometry, mobile and manipulator dynamics, trajectories, continuous- and discrete-time state-space models, linearisation, stability, controllability, observability, Lyapunov reasoning, LQR, practical feedback, integration, and safety.
- [ ] Complete and self-check every cumulative block, correct each blocking misconception, and obtain user approval.
- [ ] Add the approved cumulative review to `notes/_quarto.yml`; because it introduces no new concepts or notation, verify rather than duplicate glossary and notation entries.

#### Specify

- [ ] Create `docs/05_phase_01_integration/phase_01_acceptance.md` as the sole owner of final mobile-system scenarios, parameterisations, metrics, thresholds, commands, environment, held-out seeds, log manifest, and pass/fail rules; reference the accepted component specifications instead of copying their requirements, and freeze the document before running the final campaign. The state-space reference specification, two planar-manipulator specifications, and `P1.10` verification report remain authoritative for their separate educational laboratories.
- [ ] Freeze nominal straight and curved cases plus wheel-geometry mismatch, bounded external disturbance, encoder noise, command latency, physical effort and traction saturation, command dropout, feedback dropout, invalid command, timestamp fault, and emergency-stop cases.
- [ ] Require identical system scenarios for each admitted controller stack where comparison is meaningful, and state when a safety-only scenario has no controller ranking.

#### Verify and close

- [ ] Run the complete headless acceptance campaign from a clean documented environment and preserve its configuration, seeds, event manifest, machine-readable metrics, and retained-log hashes or locations.
- [ ] Compare the frozen controller stacks without per-scenario retuning and report tracking error, overshoot, settling time, effort, smoothness, constraint violations, watchdog reaction time, stop time, and stopping distance.
- [ ] Verify every final pass/fail rule against the tested metric path; preserve failed cases and negative results.
- [ ] Confirm that the frozen `P1.8` state-space reference and `P1.10` planar-manipulator report pass their own mathematical, kinematics, dynamics, and control gates; reference them without rerunning or copying their cases into the mobile-system campaign.
- [ ] Create `docs/reports/phase_01_geometry_mechanics_control_verification.md` with the final interpretation, distinct links to the planar-manipulator, mobile-controller, and safety reports, limitations, safety-claim boundary, and deferred work.
- [ ] Render and inspect the complete Phase 1 book, link the final report in the programme table, mark Phase 1 complete, reduce this detailed section to its milestone row, and expand Phase 2.

## Backlog

- [ ] Remove the two accidental duplicate `body_to_wheel` code blocks appended to `notes/README.md` when that file is next in scope.
