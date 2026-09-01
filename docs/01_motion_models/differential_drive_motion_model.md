# Differential-Drive Motion-Model Capability Specification

## Capability and question

The active capability is a deterministic, ideal differential-drive motion model. It answers the Phase 1 question: given the robot geometry, wheel angular speeds, current planar pose, and elapsed time, what body velocity and next pose does the reviewed kinematic model predict?

The prerequisite theory is documented and reviewed in [Linear Algebra Foundations](../../notes/part_01_motion_mechanics_control/01_linear_algebra_foundations.qmd), [Geometry and Coordinate Frames](../../notes/part_01_motion_mechanics_control/02_geometry_and_coordinate_frames.qmd), and [Differential-Drive Kinematics and Numerical Integration: Model and Derivation](../../notes/part_01_motion_mechanics_control/03_kinematics_and_numerical_integration.qmd).

## Intended behaviour

The capability shall:

1. convert signed left and right wheel angular speeds into signed body-forward speed and yaw rate;
2. invert that mapping to convert a desired body velocity into ideal wheel angular speeds;
3. advance a planar pose with the exact constant-input straight-or-arc update;
4. advance the same pose with Forward Euler for comparison and convergence tests; and
5. reject invalid geometry, timing, state, and velocity inputs explicitly.

## Mathematical model

The planar pose is

$$
\mathbf q
=
\begin{bmatrix}
x&y&\theta
\end{bmatrix}^{\mathsf T},
$$

where $x$ and $y$ are the world-frame coordinates of the axle midpoint in metres and $\theta$ is the counter-clockwise body heading in radians. The body velocity is

$$
\mathbf u
=
\begin{bmatrix}
v&\omega
\end{bmatrix}^{\mathsf T},
$$

where $v$ is signed body-forward speed in $\mathrm{m\,s^{-1}}$ and $\omega$ is signed yaw rate in $\mathrm{rad\,s^{-1}}$.

For wheel radius $r>0$ in metres, track width $L>0$ in metres, and signed wheel angular speeds $\dot\phi_L$ and $\dot\phi_R$ in $\mathrm{rad\,s^{-1}}$, the wheel-to-body mapping is

$$
v
=
\frac{r}{2}
\left(\dot\phi_R+\dot\phi_L\right),
\qquad
\omega
=
\frac{r}{L}
\left(\dot\phi_R-\dot\phi_L\right).
$$

The inverse body-to-wheel mapping is

$$
\dot\phi_L
=
\frac{v}{r}
-
\frac{L\omega}{2r},
\qquad
\dot\phi_R
=
\frac{v}{r}
+
\frac{L\omega}{2r}.
$$

For a constant input over $\Delta t>0$ seconds, define the signed heading change

$$
\Delta\theta
\mathrel{:=}
\omega\Delta t
$$

and the unnormalised sinc function

$$
\operatorname{sinc}(z)
\mathrel{:=}
\begin{cases}
\dfrac{\sin z}{z},&z\neq0,\\[4pt]
1,&z=0.
\end{cases}
$$

The exact pose update is

$$
\begin{aligned}
x_{k+1}
&=
x_k+
v\Delta t\,
\operatorname{sinc}\left(\frac{\Delta\theta}{2}\right)
\cos\left(\theta_k+\frac{\Delta\theta}{2}\right),\\
y_{k+1}
&=
y_k+
v\Delta t\,
\operatorname{sinc}\left(\frac{\Delta\theta}{2}\right)
\sin\left(\theta_k+\frac{\Delta\theta}{2}\right),\\
\theta_{k+1}
&=
\theta_k+\Delta\theta.
\end{aligned}
$$

The Forward Euler update is

$$
\begin{aligned}
x_{k+1}^{\mathrm E}
&=
x_k^{\mathrm E}
+
\Delta t\,v\cos\theta_k^{\mathrm E},\\
y_{k+1}^{\mathrm E}
&=
y_k^{\mathrm E}
+
\Delta t\,v\sin\theta_k^{\mathrm E},\\
\theta_{k+1}^{\mathrm E}
&=
\theta_k^{\mathrm E}
+
\Delta t\,\omega.
\end{aligned}
$$

## Interfaces

The initial implementation shall be a Python 3.12 library containing deterministic, stateless functions with no ROS runtime dependency.

| Operation | Inputs | Output |
|---|---|---|
| Wheel to body | $\dot\phi_L$, $\dot\phi_R$, $r$, $L$ | $v$, $\omega$ |
| Body to wheel | $v$, $\omega$, $r$, $L$ | $\dot\phi_L$, $\dot\phi_R$ |
| Exact integration | $\mathbf q_k$, $v$, $\omega$, $\Delta t$ | $\mathbf q_{k+1}$ |
| Forward Euler integration | $\mathbf q_k$, $v$, $\omega$, $\Delta t$ | $\mathbf q_{k+1}^{\mathrm E}$ |

All numeric inputs and outputs shall use SI units shown in the mathematical model. Angles shall remain unwrapped; the capability shall not silently normalise $\theta$.

## Requirements

| ID | Requirement |
|---|---|
| MOT-KIN-001 | The wheel-to-body operation shall implement the reviewed differential-drive equations exactly for finite valid inputs. |
| MOT-KIN-002 | The body-to-wheel operation shall implement the algebraic inverse for finite valid inputs. |
| MOT-INT-001 | The exact integrator shall implement the continuous straight-or-arc update for every finite $\omega$, including $\omega=0$, without directly evaluating $v/\omega$ at zero. |
| MOT-INT-002 | The Forward Euler integrator shall evaluate the pose derivative at the beginning of each step. |
| MOT-VAL-001 | Every operation shall reject non-finite numeric inputs. |
| MOT-VAL-002 | Operations using geometry shall reject $r\leq0$ and $L\leq0$. |
| MOT-VAL-003 | Integration operations shall reject $\Delta t\leq0$. |
| MOT-API-001 | Operations shall be deterministic and shall not mutate their inputs or depend on hidden state. |
| MOT-API-002 | Public interfaces shall identify frames, units, sign conventions, valid inputs, outputs, and failure behaviour. |

Invalid inputs shall produce an explicit exception. The implementation shall not silently clamp inputs, substitute geometry, wrap angles, or replace invalid values with zero.

## Acceptance cases

Use $r=0.10\,\mathrm m$ and $L=0.50\,\mathrm m$ unless a case states otherwise.

| ID | Case | Expected result |
|---|---|---|
| MOT-ACC-001 | $\dot\phi_L=\dot\phi_R=2\,\mathrm{rad\,s^{-1}}$ | $v=0.20\,\mathrm{m\,s^{-1}}$ and $\omega=0\,\mathrm{rad\,s^{-1}}$. |
| MOT-ACC-002 | $\dot\phi_L=2\,\mathrm{rad\,s^{-1}}$, $\dot\phi_R=-2\,\mathrm{rad\,s^{-1}}$ | $v=0$ and $\omega=-0.80\,\mathrm{rad\,s^{-1}}$. Exact integration changes only $\theta$. |
| MOT-ACC-003 | $\mathbf q_0=[0\;0\;0]^\mathsf T$, $\dot\phi_L=1\,\mathrm{rad\,s^{-1}}$, $\dot\phi_R=3\,\mathrm{rad\,s^{-1}}$, and $\Delta t=1\,\mathrm s$ | $v=0.20\,\mathrm{m\,s^{-1}}$, $\omega=0.40\,\mathrm{rad\,s^{-1}}$, and $\mathbf q_1=[0.5\sin(0.4)\;\;0.5(1-\cos(0.4))\;\;0.4]^\mathsf T$. |
| MOT-ACC-004 | Apply body-to-wheel and then wheel-to-body to a finite valid $(v,\omega)$ pair. | The recovered pair agrees with the original pair to absolute tolerance $10^{-12}$ for the declared test values. |
| MOT-ACC-005 | Integrate $\mathbf q_0=[0\;0\;0]^\mathsf T$, $v=1\,\mathrm{m\,s^{-1}}$, and $\omega=1\,\mathrm{rad\,s^{-1}}$ over $1\,\mathrm s$ with Forward Euler step sizes $0.10$, $0.05$, and $0.025\,\mathrm s$. | Position error decreases each time, and each error ratio $E(\Delta t)/E(\Delta t/2)$ lies between $1.8$ and $2.2$. |
| MOT-ACC-006 | Supply a non-finite input, non-positive $r$ or $L$, or non-positive $\Delta t$ to an operation that requires it. | The operation raises its documented exception and returns no motion result. |

Floating-point comparisons shall use declared absolute or relative tolerances rather than exact equality, except when checking deliberately unchanged stored values.

## Exclusions

This capability does not include:

- ROS nodes, topics, messages, parameters, or launch files;
- Gazebo integration or a visual simulation;
- encoder quantisation, sensor timestamps, covariance, or full odometry;
- wheel slip, unequal effective radii, tyre deformation, backlash, terrain, or parameter estimation;
- force, torque, mass, inertia, motor, actuator, or battery dynamics;
- trajectory generation, feedback control, PID, limits, watchdogs, or emergency stops; or
- angle wrapping, global localisation, mapping, or sensor fusion.

These concerns require later capability cycles and must not expand the first implementation.

## Traceability

| Requirement | Theory | Planned verification |
|---|---|---|
| MOT-KIN-001, MOT-KIN-002 | Wheel and body velocity mappings in Chapter 3, Section 2.1 | MOT-ACC-001 through MOT-ACC-004 |
| MOT-INT-001 | Exact constant-curvature motion and sinc limit in Chapter 3, Section 3.2 | MOT-ACC-002 and MOT-ACC-003 |
| MOT-INT-002 | Forward Euler derivation in Chapter 3, Section 4.1 | MOT-ACC-005 |
| MOT-VAL-001 through MOT-VAL-003 | Model assumptions and engineering failure modes in Chapter 3 | MOT-ACC-006 |
| MOT-API-001, MOT-API-002 | Project coding and reproducibility rules | Unit tests and interface review |

## Gate to implementation

Implementation may begin after the user reviews this minimum specification and confirms completion of the applicable retrieval and cumulative test in Chapter 3. When implementation begins, create `notes/part_01_motion_mechanics_control/04_kinematics_and_numerical_integration_implementation.qmd` alongside the first manageable code block; do not create it as an empty placeholder.
