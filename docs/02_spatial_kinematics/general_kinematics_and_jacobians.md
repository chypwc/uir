# General Kinematics and Jacobians Capability Specification

## Capability and active cycle

The complete capability will provide reusable forward and velocity kinematics for fixed-base open serial chains. This first specification increment owns only Cycle 1: validated space-form product-of-exponentials forward kinematics. Given one fixed serial-chain model and one allowed joint-coordinate column, the operation shall return the pose of the selected end-effector frame relative to the fixed space frame or fail explicitly without returning a plausible-looking pose.

The reviewed theory is documented in [General Robot Kinematics and Jacobians](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd), especially the section on serial-chain pose through products of exponentials. The existing [Spatial Geometry Kernel Capability Specification](spatial_geometry_kernel.md) owns validated $SO(3)$ and $SE(3)$ representations, the linear-first hat map, the $SE(3)$ exponential, transformation composition, numerical policy, and unit-aware pose comparison. This specification reuses those operations rather than redefining them.

## Intended behaviour

The Cycle 1 capability shall:

1. represent a fixed-base open serial chain by one validated home end-effector pose and an ordered list of validated revolute or prismatic joint definitions;
2. construct every fixed space screw axis from joint-axis geometry measured at the same declared zero arrangement and expressed in the same space frame;
3. keep immutable model data separate from the current joint-coordinate input;
4. evaluate the space-form product of exponentials in physical base-to-end-effector joint order;
5. support arbitrary spatial axis directions, mixed revolute--prismatic chains, and planar chains as a restricted case rather than as a separate representation;
6. validate model invariants, joint-coordinate shape, finiteness, declared domains, and supported numerical magnitude before returning a pose; and
7. return a validated `Transform3` on success or an explicit typed failure containing no transform on failure.

## Conventions, frames, types, and units

### Transformation and multiplication convention

$\mathbf T_{se}$ is the pose of end-effector frame $\{e\}$ relative to fixed space frame $\{s\}$. It maps homogeneous point-coordinate columns from $\{e\}$ to $\{s\}$ by left multiplication:

$$
\bar{\mathbf p}_s
=
\mathbf T_{se}\bar{\mathbf p}_e.
$$

Transformations are active, coordinate columns are used, positive rotations follow the right-hand rule, and products act on columns rightmost first. Every position and translation is measured in metres. Rotation matrices are dimensionless, while angular coordinates are labelled in radians even though radians are dimensionless in algebra.

The mathematical core may use documented frame preconditions rather than compile-time frame tags in this cycle. It shall not infer, relabel, or silently mix frames. Every joint axis used by one model is measured when $\mathbf q=\mathbf 0_n$ and expressed using the same fixed frame $\{s\}$ as the home pose.

### Joint order and coordinate column

Let $n\geq1$ be the number of independent one-degree-of-freedom joints. Joints are stored in physical order from the fixed base toward the end effector: joint $1$ is most proximal and joint $n$ is most distal. The current joint-coordinate input is

$$
\mathbf q
=
\begin{bmatrix}
q_1&\cdots&q_n
\end{bmatrix}^{\mathsf T}
\in
\mathcal Q,
\qquad
\mathcal Q
=
\mathcal Q_1\times\cdots\times\mathcal Q_n.
$$

For a revolute joint, $q_j$ is a signed angle in radians and positive motion follows the right-hand rule about the declared positive axis. For a prismatic joint, $q_j$ is a signed displacement in metres and positive motion follows the declared axis direction. Because the entries can have different units, the complete column $\mathbf q$ shall not be assigned one physical unit or validated with one Euclidean norm.

### Abstract data types

The contract uses the following abstract types. The implementation stage shall map them to concrete modern C++20 and Eigen types without changing their semantics.

| Type | Mathematical content | Required invariant |
|---|---|---|
| `Transform3` | A validated element of $SE(3)$ | Semantics and validation are owned by the spatial geometry kernel |
| `RevoluteLimits` | Optional inclusive lower and upper angular bounds | Every present bound is finite and measured in radians; when both are present, the lower bound does not exceed the upper bound |
| `PrismaticLimits` | Optional inclusive lower and upper displacement bounds | Every present bound is finite and measured in metres; when both are present, the lower bound does not exceed the upper bound |
| `RevoluteJoint` | Positive axis direction, one point on the axis, `RevoluteLimits`, and the derived ${}^{s}\mathbf S_j$ | Finite valid geometry and unit positive direction after normalization |
| `PrismaticJoint` | Positive translation direction, `PrismaticLimits`, and the derived ${}^{s}\mathbf S_j$ | Finite valid geometry and unit positive direction after normalization |
| `JointDefinition` | A type-safe alternative containing either a `RevoluteJoint` or a `PrismaticJoint` | Exactly one supported joint kind with its correct geometry, units, and limits |
| `SerialChainModel` | $\mathbf M$ and the ordered joint definitions, including their derived $\{{}^{s}\mathbf S_j\}_{j=1}^{n}$ | Nonempty, immutable, internally consistent model shared by the space-form and body-form operations |
| `Eigen::Ref<const Eigen::VectorXd>` | Read-only dynamic $n$-entry column $\mathbf q$ | Exactly one finite entry per joint; entry $q_j$ has the unit and domain declared by joint $j$ |
| `NumericalPolicy` | Validated thresholds used to evaluate spatial exponentials robustly | Evaluation-only input inherited from the spatial geometry kernel; not physical robot data and not stored in `SerialChainModel` |

`RevoluteLimits` and `PrismaticLimits` have the same optional-bound structure but remain distinct public types because their values have different units. Every present bound is inclusive and finite; an absent bound represents an unbounded side and shall not be encoded by an infinity. Both bounds absent from `RevoluteLimits` declare a continuous revolute joint whose numerical coordinate is an unwrapped finite angle. The evaluator shall test each supplied value against the corresponding typed limits and shall never wrap, canonicalize, or clamp it on the caller's behalf.

## Shared serial-chain model

### Zero arrangement and home pose

The model's reference coordinate column is

$$
\mathbf 0_n
=
\begin{bmatrix}
0&\cdots&0
\end{bmatrix}^{\mathsf T}
\in\mathbb R^n.
$$

The complete robot placement assigned by the unconstrained kinematic geometry to this reference column is the zero reference arrangement. Physical joint stops or a deliberately tighter operating interval may make $\mathbf 0_n\notin\mathcal Q$; in that case the zero reference arrangement parameterises the kinematic model but is not an admissible command. The notation $\mathbf T_{se}(\mathbf 0_n)$ below denotes this reference value, while the public evaluator still accepts only supplied columns $\mathbf q\in\mathcal Q$. The model stores only the selected end-effector pose from the reference arrangement as

$$
\boxed{
\mathbf M
\mathrel{:=}
\mathbf T_{se}(\mathbf 0_n)
=
\begin{bmatrix}
\mathbf R_{se,0}&{}^{s}\mathbf p_{e,0}\\
\mathbf 0_3^{\mathsf T}&1
\end{bmatrix}
\in SE(3)
}.
$$

Here $\mathbf R_{se,0}$ is the orientation of $\{e\}$ relative to $\{s\}$ at zero, and ${}^{s}\mathbf p_{e,0}$ is the position of $O_e$ relative to $O_s$, expressed in $\{s\}$ and measured in metres. The home pose $\mathbf M$ is fixed model data; it is not the current pose and does not contain the placement of every link.

### Validated joint-axis geometry

For each joint index $j$, the model constructor accepts a finite nonzero positive axis-direction column expressed in $\{s\}$ at the zero arrangement and normalizes that direction with an overflow-safe norm. Denote the resulting unit column by ${}^{s}\mathbf s_j$, so

$$
\|{}^{s}\mathbf s_j\|_2=1.
$$

For a revolute joint, the definition also contains a finite point ${}^{s}\mathbf p_{\ell,j}$ on the axis line, measured from $O_s$, expressed in $\{s\}$, and measured in metres. The model constructor derives the normalized linear-first space screw axis

$$
\boxed{
{}^{s}\mathbf S_j
=
\begin{bmatrix}
-{}^{s}\mathbf s_j\times{}^{s}\mathbf p_{\ell,j}\\
{}^{s}\mathbf s_j
\end{bmatrix}
}.
$$

Its upper block has units of metres per radian of joint motion and its lower block is the dimensionless positive rotation direction. Choosing another point on the same oriented axis line shall produce the same screw axis.

For a prismatic joint, no axis point is required. The constructor derives

$$
\boxed{
{}^{s}\mathbf S_j
=
\begin{bmatrix}
{}^{s}\mathbf s_j\\
\mathbf 0_3
\end{bmatrix}
}.
$$

Its upper block is a dimensionless positive translation direction and its lower block is zero. The complete mixed-unit screw column shall not be normalized with one six-dimensional Euclidean norm.

The constructor shall retain the joint-specific limits together with the derived screw axis. It shall not accept a current-axis measurement, infer a joint type from approximate numeric patterns, or repair non-finite or zero axis geometry.

### Model invariants

A valid `SerialChainModel` satisfies all of the following conditions:

1. the ordered joint list contains $n\geq1$ entries;
2. $\mathbf M$ is a validated `Transform3` representing $\mathbf T_{se}(\mathbf 0_n)$;
3. all joint directions and revolute axis points were supplied in $\{s\}$ at the same zero arrangement;
4. every derived ${}^{s}\mathbf S_j$ is finite and has the joint-type structure defined above;
5. every revolute or prismatic limit object defines a nonempty valid interval $\mathcal Q_j$;
6. joint definitions, screw axes, and indices have equal length and matching order; and
7. the positive axis direction agrees with the sign and unit assigned to $q_j$.

The model shall be validated once at construction and remain immutable so that many joint-coordinate columns can reuse the same geometry. Runtime frame labels are not required in Cycle 1, so agreement of documented frame names remains a construction precondition that numeric data alone cannot verify.

## Space-form forward-pose operation

For joint $j$, multiply its fixed space screw axis by its current coordinate and apply the linear-first hat map:

$$
{}^{s}\boldsymbol\eta_j
\mathrel{:=}
{}^{s}\mathbf S_j q_j,
\qquad
\widehat{{}^{s}\boldsymbol\eta_j}
=
\widehat{{}^{s}\mathbf S_j}\,q_j.
$$

For a revolute joint, the two blocks of ${}^{s}\boldsymbol\eta_j$ are measured in metres and radians. For a prismatic joint, its linear block is measured in metres and its angular block remains zero. The spatial geometry kernel's $SE(3)$ exponential produces the active joint displacement

$$
\mathbf E_j(q_j)
\mathrel{:=}
\exp\!\left(
\widehat{{}^{s}\mathbf S_j}\,q_j
\right)
\in SE(3).
$$

The required output is the space-form product of exponentials

$$
\boxed{
\mathbf T_{se}(\mathbf q)
=
\mathbf E_1(q_1)
\mathbf E_2(q_2)
\cdots
\mathbf E_n(q_n)
\mathbf M
=
\left(
\prod_{j=1}^{n}
\exp\!\left(
\widehat{{}^{s}\mathbf S_j}\,q_j
\right)
\right)
\mathbf M
}.
$$

The product notation is defined as $\prod_{j=1}^{n}\mathbf E_j=\mathbf E_1\mathbf E_2\cdots\mathbf E_n$. Because matrices act rightmost first, $\mathbf M$ places the home end effector, joint $n$ moves it first in the derivation, and the remaining joint displacements are applied toward the base until joint $1$ carries the complete outward assembly. This is a calculation of the final pose, not a prescribed time order for moving the physical joints.

The implementation shall preserve the written joint order. It may evaluate the equation by initializing the result with $\mathbf M$ and premultiplying by $\mathbf E_j$ for $j=n,n-1,\ldots,1$, or by an algebraically equivalent method that produces the same ordered product. It shall not reorder, sort, parallel-reduce, or otherwise regroup factors in a way that changes the declared result. Rigid displacements generally do not commute.

For every valid input, the operation returns a validated `Transform3` with the semantics $\mathbf T_{se}(\mathbf q)$. It satisfies

$$
\mathbf T_{se}(\mathbf 0_n)=\mathbf M
$$

and, when only $q_k$ is nonzero,

$$
\mathbf T_{se}(\mathbf q)=\mathbf E_k(q_k)\mathbf M.
$$

## Interfaces and result contract

The first implementation shall be a C++20 library operation in `rigid_body_kinematics`, using Eigen for numeric storage and the existing spatial geometry kernel for validated transformations and exponentials. The mathematical core shall have no ROS runtime, URDF parser, simulator, global state, clock, or parameter-server dependency.

| Operation | Inputs | Success output | Failure output |
|---|---|---|---|
| `RevoluteLimits::from_bounds` | Optional lower and upper bounds in radians | Validated immutable `RevoluteLimits` | `SerialChainException` and no limits object |
| `PrismaticLimits::from_bounds` | Optional lower and upper bounds in metres | Validated immutable `PrismaticLimits` | `SerialChainException` and no limits object |
| `RevoluteJoint::from_axis` | Positive space-frame direction, space-frame axis point in metres, and validated `RevoluteLimits` | Validated immutable revolute joint containing its derived ${}^{s}\mathbf S_j$ | `SerialChainException` and no joint |
| `PrismaticJoint::from_axis` | Positive space-frame direction and validated `PrismaticLimits` | Validated immutable prismatic joint containing its derived ${}^{s}\mathbf S_j$ | `SerialChainException` and no joint |
| `SerialChainModel::from_home_and_joints` | Validated home pose $\mathbf M$ and ordered `JointDefinition` values | Immutable validated model containing the ordered derived ${}^{s}\mathbf S_j$ columns | `SerialChainException` and no model |
| `space_form_forward_kinematics` | Validated `SerialChainModel`, current $\mathbf q$ as `Eigen::Ref<const Eigen::VectorXd>`, and numerical policy | Validated `Transform3` representing $\mathbf T_{se}(\mathbf q)$ | `SerialChainException` and no transform |

The shared model is named `SerialChainModel`, and the Cycle 1 evaluator is named `space_form_forward_kinematics`. Cycle 2 will add the symmetric `body_form_forward_kinematics` operation using the same model. Public operations shall be deterministic, non-mutating, and safe to call repeatedly with one model. Failure messages shall identify the failing joint index when one joint caused the error.

The Cycle 1 evaluator shall have the following public shape:

```cpp
[[nodiscard]] Transform3 space_form_forward_kinematics(
  const SerialChainModel& model,
  Eigen::Ref<const Eigen::VectorXd> joint_coordinates,
  const NumericalPolicy& policy = NumericalPolicy{});
```

The Eigen reference is a read-only non-owning view. It permits a dynamic Eigen column or a compatible mapped Eigen view without copying and does not transfer ownership to the evaluator.

`NumericalPolicy` belongs to an evaluation, not to the robot model. The model stores the physical zero-arrangement data $\mathbf M$, the ordered joint geometry, and the joint domains; it does not store formula-selection thresholds such as the small-angle threshold or `maximum_exponential_angle`. A caller may therefore evaluate the same immutable model with the default policy or supply an explicit validated policy for a controlled numerical test. Cycle 2 shall use the same final policy parameter for `body_form_forward_kinematics`.

The limits, joint, and model types shall use the named static factories above with non-public unchecked constructors. Validation proceeds once through typed limits, then joint geometry, then the complete ordered model. After `SerialChainModel::from_home_and_joints` succeeds, neither forward-kinematics operation shall repeat validation of immutable home or joint geometry on each evaluation.

Serial-chain construction and evaluation shall use the C++20 typed-exception pattern already established by the spatial geometry kernel. `SerialChainException` derives from `std::runtime_error` and carries one machine-readable `SerialChainError` code. A successful operation returns its validated object directly; a failed operation throws `SerialChainException` and returns no object. If an inherited geometry-kernel operation fails while evaluating a serial-chain request, the public serial-chain boundary shall translate the applicable category and preserve useful diagnostic context rather than exposing a partial result or an undocumented mixture of exception contracts.

The `SerialChainError` enumeration shall distinguish at least:

| Category | Meaning |
|---|---|
| `non_finite` | A supplied coordinate, direction, or axis point is NaN or infinity |
| `invalid_model` | The joint list is empty or a joint axis direction is zero |
| `invalid_joint_limits` | Revolute or prismatic limits have a non-finite present bound or have a lower bound greater than the upper bound |
| `dimension_mismatch` | The supplied $\mathbf q$ does not contain exactly one entry per joint |
| `joint_out_of_domain` | A finite $q_j$ is outside the interval declared by its `RevoluteLimits` or `PrismaticLimits` |
| `invalid_policy` | The inherited spatial numerical policy is invalid |
| `unsupported_magnitude` | Finite accepted inputs require an exponential magnitude outside the policy or cause a required intermediate or output to become non-finite |

An error shall carry no partial pose. The evaluator shall not substitute $\mathbf M$, the identity, a partially accumulated product, or a clamped coordinate after a failure.

## Validation and numerical domain

### Model construction

`RevoluteLimits::from_bounds` and `PrismaticLimits::from_bounds` shall validate every present bound and, when both bounds are present, their order. The admissible interval need not contain the kinematic reference value zero. `RevoluteJoint::from_axis` and `PrismaticJoint::from_axis` shall reject non-finite geometry before normalization, reject a zero axis direction, normalize a valid nonzero direction with an overflow-safe norm, require a finite axis point for a revolute joint, and derive the screw axis from the selected joint type rather than accepting an inconsistent mixed six-column. `SerialChainModel::from_home_and_joints` shall accept an already validated `Transform3`, reject an empty joint list, and preserve the supplied joint order.

Normalizing a declared direction is part of constructing its coordinate representation; it does not alter the physical axis line or positive sense. No other invalid model data shall be projected, clamped, inferred, or repaired.

### Pose evaluation

Each pose evaluation uses the supplied `NumericalPolicy`, or a fresh default policy when the caller omits that argument. Changing this policy may select a different stable numerical formula or reject an unsupported magnitude, but it shall not change the stored model, the declared joint domains, or the intended rigid motion.

Before forming any joint exponential, evaluation shall perform these checks in deterministic order:

1. validate the inherited `NumericalPolicy`;
2. verify that $\mathbf q$ contains exactly $n$ entries;
3. verify that every $q_j$ is finite;
4. verify exact membership $q_j\in\mathcal Q_j$ using the applicable inclusive typed limits;
5. for every revolute joint, verify $|q_j|\leq\texttt{maximum\_exponential\_angle}$; and
6. reject any required intermediate or output that becomes non-finite.

A failure at step 3 reports `non_finite` because the caller supplied NaN or infinity. A failure at step 5 or 6 reports `unsupported_magnitude`: the supplied model and coordinate values were finite and passed their declared domains, but the requested exponential magnitude is outside the numerical policy or the finite calculation cannot be represented by the implementation. These categories distinguish invalid supplied data from arithmetic overflow or an explicitly unsupported finite magnitude.

Joint-domain tolerances shall not enlarge a physical interval. A coordinate just outside a declared bound fails rather than being clamped to the bound. The evaluator shall not reduce a revolute coordinate modulo $2\pi$, even when the final rotation would agree with another representative, because the caller's declared numerical input and domain must remain observable. The existing spatial-kernel small-angle threshold selects a stable formula only; it does not change the represented joint motion.

For a normalized revolute screw axis, the rotation-vector norm passed to the $SE(3)$ exponential is $|q_j|$, so the inherited `maximum_exponential_angle` applies directly. A prismatic coordinate has no angular block and is not compared with this angular limit. It remains subject to its declared domain and to finite-intermediate and finite-result checks.

The evaluator cannot infer whether an untyped scalar was physically supplied in radians rather than degrees, or in metres rather than another length unit. Supplying the declared SI unit is therefore a caller precondition documented by the joint type and domain.

### Verification residuals

When comparing the result with an independent calculation, write

$$
\mathbf T_a
=
\begin{bmatrix}
\mathbf R_a&\mathbf p_a\\
\mathbf 0_3^{\mathsf T}&1
\end{bmatrix},
\qquad
\mathbf T_b
=
\begin{bmatrix}
\mathbf R_b&\mathbf p_b\\
\mathbf 0_3^{\mathsf T}&1
\end{bmatrix}.
$$

Translation and rotation shall be compared separately:

$$
e_p
=
\|\mathbf p_a-\mathbf p_b\|_2,
\qquad
\mathbf R_\Delta
=
\mathbf R_a^{\mathsf T}\mathbf R_b,
\qquad
e_R
=
\|\boldsymbol\phi_\Delta\|_2,
$$

where $\boldsymbol\phi_\Delta$ is the principal rotation vector of $\mathbf R_\Delta$. The length residual $e_p$ is measured in metres and the angular residual $e_R$ in radians; they shall use separate tolerances and shall not be combined in one unscaled norm. The moderate analytic fixtures below use $e_p\leq10^{-12}\,\mathrm m$ and $e_R\leq10^{-12}\,\mathrm{rad}$.

## Requirements

| ID | Requirement |
|---|---|
| GK-REP-001 | Public operations shall use the declared $\mathbf T_{se}$ frame meaning, active transformations, column vectors, rightmost-first multiplication, SI units, right-handed positive rotation, and linear-first six-columns. |
| GK-MOD-001 | Named static factories with non-public unchecked constructors shall create immutable validated limit, joint, and serial-chain objects; a serial-chain model shall contain one validated home pose and a nonempty ordered list of one-degree-of-freedom joint definitions constructed from one zero arrangement. |
| GK-MOD-002 | Model construction shall derive a normalized space screw axis from each joint's declared type, positive direction, and required axis point without inferring the type from numeric values or accepting inconsistent geometry. |
| GK-DOM-001 | Every revolute joint shall use `RevoluteLimits` in radians and every prismatic joint shall use `PrismaticLimits` in metres; present bounds shall be finite and inclusive, absent bounds shall represent unbounded sides, and a valid interval may exclude the zero reference coordinate. |
| GK-FK-001 | Space-form forward kinematics shall implement $\mathbf T_{se}(\mathbf q)=\exp(\widehat{{}^{s}\mathbf S_1}q_1)\cdots\exp(\widehat{{}^{s}\mathbf S_n}q_n)\mathbf M$. |
| GK-FK-002 | Evaluation shall preserve base-to-end-effector joint-factor order and shall not rely on commutativity or silently substitute current-axis screw coordinates. |
| GK-VAL-001 | Construction shall reject an empty joint list, non-finite joint geometry, zero axis directions, and invalid typed limits without returning a joint or model. |
| GK-VAL-002 | Evaluation shall reject an invalid policy, wrong coordinate count, non-finite coordinate, out-of-domain coordinate, unsupported angular magnitude, or non-finite calculation without returning a transform. |
| GK-NUM-001 | Formula thresholds shall only select numerically stable evaluation; the operation shall not wrap, clamp, canonicalize, project, or otherwise change a valid declared joint coordinate. |
| GK-NUM-002 | `NumericalPolicy` shall be an evaluation input with a default value, shall not be stored in `SerialChainModel`, and shall be applied by `space_form_forward_kinematics`. |
| GK-API-001 | The core shall be deterministic, non-mutating, reusable across calls, and independent of ROS runtime and simulator state. |
| GK-API-002 | Success shall return the validated model or `Transform3` directly; failure shall throw `SerialChainException` carrying a `SerialChainError`, useful context including a joint index when applicable, and no geometric value. |

## Library-independent acceptance cases

The following expected values come from direct rigid-body geometry, elementary trigonometry, or the transform-chain model rather than from another product-of-exponentials library.

| ID | Case | Independent expected result |
|---|---|---|
| GK-ACC-001 | Use a valid model whose every joint domain contains zero and set $\mathbf q=\mathbf 0_n$. | Every exponential is $\mathbf I_4$ and the result equals the stored $\mathbf M$ within separate pose tolerances. |
| GK-ACC-002 | One revolute joint is supplied the dimensionless positive direction $[0\;0\;5]^{\mathsf T}$, defining the axis $+z_s$ through $O_s$. Let $\mathbf M=[\mathbf I_3,[2\;0\;0]^{\mathsf T};\mathbf0_3^{\mathsf T},1]$ and $q_1=\pi/2\,\mathrm{rad}$. | Normalization gives $[0\;0\;1]^{\mathsf T}$, after which $\mathbf R_{se}=\mathbf R_z(\pi/2)$ and ${}^{s}\mathbf p_e=[0\;2\;0]^{\mathsf T}\,\mathrm m$. |
| GK-ACC-003 | One revolute joint has positive axis $+z_s$ through $[2\;0\;0]^{\mathsf T}\,\mathrm m$, $\mathbf M=[\mathbf I_3,[3\;0\;0]^{\mathsf T};\mathbf0_3^{\mathsf T},1]$, and $q_1=\pi/2\,\mathrm{rad}$. | Rotating the home point about the displaced axis gives $\mathbf R_{se}=\mathbf R_z(\pi/2)$ and ${}^{s}\mathbf p_e=[2\;1\;0]^{\mathsf T}\,\mathrm m$. The derived screw is $[0\;-2\;0\;0\;0\;1]^{\mathsf T}$. |
| GK-ACC-004 | One prismatic joint is supplied the dimensionless positive direction $[0\;-4\;0]^{\mathsf T}$, defining translation along $-y_s$. Let $\mathbf M=[\mathbf I_3,[1\;2\;3]^{\mathsf T};\mathbf0_3^{\mathsf T},1]$ and $q_1=0.4\,\mathrm m$. | Normalization gives $[0\;-1\;0]^{\mathsf T}$, after which $\mathbf R_{se}=\mathbf I_3$ and ${}^{s}\mathbf p_e=[1\;1.6\;3]^{\mathsf T}\,\mathrm m$. |
| GK-ACC-005 | One spatial revolute joint has positive axis $+x_s$ through $O_s$, $\mathbf M=[\mathbf I_3,[0\;1\;0]^{\mathsf T};\mathbf0_3^{\mathsf T},1]$, and $q_1=\pi/2\,\mathrm{rad}$. | $\mathbf R_{se}=\mathbf R_x(\pi/2)$ and ${}^{s}\mathbf p_e=[0\;0\;1]^{\mathsf T}\,\mathrm m$, demonstrating that the core is not restricted to the $x_s$--$y_s$ plane. |
| GK-ACC-006 | A revolute--prismatic chain has joint 1 rotating about $+z_s$ through $O_s$, joint 2 translating along $+x_s$, $\mathbf M=[\mathbf I_3,[1\;0\;0]^{\mathsf T};\mathbf0_3^{\mathsf T},1]$, and $\mathbf q=[\pi/2\;0.5]^{\mathsf T}$ with joint-dependent units. | The direct transform chain gives $\mathbf R_{se}=\mathbf R_z(\pi/2)$ and ${}^{s}\mathbf p_e=[0\;1.5\;0]^{\mathsf T}\,\mathrm m$. |
| GK-ACC-007 | A planar 2R chain has $L_1=2\,\mathrm m$, $L_2=1\,\mathrm m$, $\mathbf M=[\mathbf I_3,[3\;0\;0]^{\mathsf T};\mathbf0_3^{\mathsf T},1]$, space screws $[0\;0\;0\;0\;0\;1]^{\mathsf T}$ and $[0\;-2\;0\;0\;0\;1]^{\mathsf T}$, and $\boldsymbol\theta=[\pi/2\;-\pi/2]^{\mathsf T}\,\mathrm{rad}$. | Independent link trigonometry gives $\mathbf R_{se}=\mathbf I_3$ and ${}^{s}\mathbf p_e=[1\;2\;0]^{\mathsf T}\,\mathrm m$. |
| GK-ACC-008 | Deliberately swap the two factors in GK-ACC-007 while retaining the same axes, angles, and home pose. | The incorrect product gives ${}^{s}\mathbf p_e=[5\;2\;0]^{\mathsf T}\,\mathrm m$ rather than $[1\;2\;0]^{\mathsf T}\,\mathrm m`; the negative control proves that the test detects a factor-order defect. |
| GK-ACC-009 | Attempt construction with, separately, an empty joint list, a zero axis direction, a non-finite direction, a non-finite revolute axis point, a non-finite limit bound, or a lower limit greater than its upper limit. | An empty list or zero direction fails with `invalid_model`; non-finite joint geometry fails with `non_finite`; and every invalid limit case fails with `invalid_joint_limits`. No failed operation returns a joint or model. |
| GK-ACC-010 | For a valid two-joint model, supply, separately, an invalid `NumericalPolicy`, a one-entry $\mathbf q$, a NaN or infinite coordinate, a coordinate outside its declared interval, and a revolute coordinate larger than `maximum_exponential_angle` while still allowed by its model interval. | Evaluation fails respectively with `invalid_policy`, `dimension_mismatch`, `non_finite`, `joint_out_of_domain`, and `unsupported_magnitude`; no case returns $\mathbf M$, a partial product, or another transform. |
| GK-ACC-011 | Let $D_{\max}$ be the largest finite IEEE 754 binary64 value. Use a one-joint prismatic model translating along $+x_s$, an unbounded upper joint domain, home translation $[D_{\max}\;0\;0]^{\mathsf T}\,\mathrm m$, and $q_1=D_{\max}\,\mathrm m$. | The supplied values and individual prismatic displacement are finite, but composition requires the unrepresentable translation $[2D_{\max}\;0\;0]^{\mathsf T}\,\mathrm m$. Evaluation fails with `unsupported_magnitude` and returns no transform. |
| GK-ACC-012 | Construct a valid finite interval that excludes zero, then evaluate its included endpoints, zero, and values immediately outside each endpoint using fixed binary64 values. Evaluate one moderate valid input repeatedly, both with the policy argument omitted and with an explicit `NumericalPolicy{}`. | Construction succeeds; included endpoints succeed; zero and the outside values fail with `joint_out_of_domain` without tolerance-based clamping; repeated evaluations return equivalent poses; and the omitted and explicit default policies produce equivalent poses and the same success behaviour. |

Threshold tests shall use fixed inputs and a documented IEEE 754 binary64 environment. Optional randomized stress tests may use a fixed seed and bounded domains, but they shall supplement rather than replace the analytic fixtures.

## Exclusions for Cycle 1

This specification increment does not include:

- body screw-axis storage or `body_form_forward_kinematics`, which belongs to Cycle 2;
- task, space, or body Jacobians, joint-rate mappings, singularity analysis, or finite-difference Jacobian checks, which belong to later `P1.3` cycles;
- inverse kinematics, pseudoinverses, redundancy resolution, manipulability optimisation, or joint-limit avoidance objectives, which belong to `P1.4`;
- intermediate link-pose output, branching kinematic trees, closed chains, parallel mechanisms, coupled or mimic joints, helical joints, floating bases, or mobile-base composition;
- URDF or SRDF parsing, ROS messages or nodes, `tf2`, MoveIt, simulator integration, frame-tree lookup, timestamps, or interpolation;
- collision geometry, dynamics, forces, torques, control, trajectory generation, uncertainty propagation, automatic differentiation, or optimization;
- recomputing space screw axes from the robot's current pose, silently changing factor order, or interpreting the product as a physical joint-motion schedule; or
- projection or repair of invalid model geometry, automatic angle wrapping, degree-to-radian conversion, unit inference, or support beyond the spatial kernel's declared numerical domain.

A relative-transform chain may be used as an independent acceptance calculation, but Cycle 1 does not add a second production forward-kinematics representation merely to duplicate the space-form operation.

## Traceability

| Requirement | Reviewed source | Planned verification |
|---|---|---|
| GK-REP-001, GK-MOD-001, GK-DOM-001 | Chapter 10 sections on configuration coordinates, zero arrangement, home pose, and joint screw axes | Public-model review and GK-ACC-001, GK-ACC-009, and GK-ACC-012 |
| GK-MOD-002 | Chapter 10 normalized revolute and prismatic space screw-axis definitions; Chapter 8 screw-axis geometry | GK-ACC-002 through GK-ACC-006 and invalid-axis cases in GK-ACC-009 |
| GK-FK-001, GK-FK-002 | Chapter 10 space-form product and transform-chain equivalence | GK-ACC-001 through GK-ACC-008 |
| GK-VAL-001, GK-VAL-002, GK-NUM-001, GK-NUM-002 | Chapter 10 supported-input boundaries and the spatial geometry kernel's numerical policy | GK-ACC-009 through GK-ACC-012 |
| GK-API-001, GK-API-002 | Project reusable-core rules and the spatial geometry kernel result contract | Public-interface review, focused deterministic tests, and package-level test reporting |

## Gate to implementation

Implementation may begin only after the user reviews and approves this Cycle 1 specification. The first implementation block shall establish the smallest public model representation and its focused construction tests before adding the space-form evaluator. Production code, public headers, tests, and build files remain learner-authored under the project workflow.
