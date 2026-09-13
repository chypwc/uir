# General Kinematics and Jacobians Capability Specification

## Capability and scope

This specification defines space-form and [body-form forward kinematics](#body-form-forward-kinematics) for fixed-base open serial chains. Given one fixed serial-chain model and one allowed joint-coordinate column within the selected evaluator's numerical domain, either operation shall return the pose of the selected end-effector frame relative to the fixed space frame or fail explicitly without returning a plausible-looking pose.

It also defines the [space Jacobian, body Jacobian, and their adjoint relation](#space-and-body-jacobians) for the same model. These operations return the instantaneous joint-rate-to-twist maps, not a new pose or a ROS message. The existing forward-kinematics contracts remain unchanged.

The governing theory is documented in [General Robot Kinematics and Jacobians](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd). The existing [Spatial Geometry Kernel Capability Specification](spatial_geometry_kernel.md) owns validated $SO(3)$ and $SE(3)$ representations, the linear-first hat map, the $SE(3)$ exponential, transformation composition, adjoints, numerical policy, and unit-aware pose comparison. This specification reuses those operations rather than redefining them.

## Intended behaviour

The shared model and space-form operation shall:

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

The mathematical core may use documented frame preconditions rather than compile-time frame tags. It shall not infer, relabel, or silently mix frames. Every joint axis used by one model is measured when $\mathbf q=\mathbf 0_n$ and expressed using the same fixed frame $\{s\}$ as the home pose.

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

The model shall be validated once at construction and remain immutable so that many joint-coordinate columns can reuse the same geometry. Runtime frame labels are not required, so agreement of documented frame names remains a construction precondition that numeric data alone cannot verify.

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

The evaluators shall be C++20 library operations in `rigid_body_kinematics`, using Eigen for numeric storage and the existing spatial geometry kernel for validated transformations and exponentials. The mathematical core shall have no ROS runtime, URDF parser, simulator, global state, clock, or parameter-server dependency.

| Operation | Inputs | Success output | Failure output |
|---|---|---|---|
| `RevoluteLimits::from_bounds` | Optional lower and upper bounds in radians | Validated immutable `RevoluteLimits` | `SerialChainException` and no limits object |
| `PrismaticLimits::from_bounds` | Optional lower and upper bounds in metres | Validated immutable `PrismaticLimits` | `SerialChainException` and no limits object |
| `RevoluteJoint::from_axis` | Positive space-frame direction, space-frame axis point in metres, and validated `RevoluteLimits` | Validated immutable revolute joint containing its derived ${}^{s}\mathbf S_j$ | `SerialChainException` and no joint |
| `PrismaticJoint::from_axis` | Positive space-frame direction and validated `PrismaticLimits` | Validated immutable prismatic joint containing its derived ${}^{s}\mathbf S_j$ | `SerialChainException` and no joint |
| `SerialChainModel::from_home_and_joints` | Validated home pose $\mathbf M$ and ordered `JointDefinition` values | Immutable validated model containing the ordered derived ${}^{s}\mathbf S_j$ columns | `SerialChainException` and no model |
| `space_form_forward_kinematics` | Validated `SerialChainModel`, current $\mathbf q$ as `Eigen::Ref<const Eigen::VectorXd>`, and numerical policy | Validated `Transform3` representing $\mathbf T_{se}(\mathbf q)$ | `SerialChainException` and no transform |

The shared model is named `SerialChainModel`. The `space_form_forward_kinematics` and `body_form_forward_kinematics` operations shall use that same model. Public operations shall be deterministic, non-mutating, and safe to call repeatedly with one model. Failure messages shall identify the failing joint index when one joint caused the error.

The space-form evaluator shall have the following public shape:

```cpp
[[nodiscard]] Transform3 space_form_forward_kinematics(
  const SerialChainModel& model,
  Eigen::Ref<const Eigen::VectorXd> joint_coordinates,
  const NumericalPolicy& policy = NumericalPolicy{});
```

The Eigen reference is a read-only non-owning view. It permits a dynamic Eigen column or a compatible mapped Eigen view without copying and does not transfer ownership to the evaluator.

`NumericalPolicy` belongs to an evaluation, not to the robot model. The model stores the physical zero-arrangement data $\mathbf M$, the ordered joint geometry, and the joint domains; it does not store formula-selection thresholds such as the small-angle threshold or `maximum_exponential_angle`. A caller may therefore evaluate the same immutable model with the default policy or supply an explicit validated policy for a controlled numerical test. The `body_form_forward_kinematics` operation shall use the same final policy parameter.

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
| GK-API-002 | Success shall return the validated model, `Transform3`, or owning Jacobian matrix required by the operation directly; failure shall throw `SerialChainException` carrying a `SerialChainError`, useful context including a joint index when applicable, and no geometric value. |

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

### Selected essential-test scope

The learner-selected test batch omits a dedicated repeated-evaluation/default-policy-equivalence test from GK-ACC-012. Its interval-boundary checks remain required. Determinism, non-mutation, model reuse, and equivalent default-policy behaviour remain API requirements; omitting a dedicated test does not weaken those behaviours or establish executable evidence for them.

A separate overflow test for composition between two joint displacements is not required in this batch. Retain the home-pose-composition overflow case GK-ACC-011 and the implemented screw-scaling overflow check. The evaluator must still reject unsupported intermediate results at every composition.

The explicit reversed-factor negative control GK-ACC-008 may be verified by the [executable acceptance check below](#gk-acc-008-executable-negative-control) rather than an additional registered unit test; this case is not waived. The [owning checklist](../../CHECKLIST.md) records verification outcomes.

### GK-ACC-008: executable negative control

This manual acceptance verifier retains the axes, angles, and home pose of GK-ACC-007 and reverses only the two exponential factors. The independent analytic positions are `(1, 2, 0)` metres for the correct product and `(5, 2, 0)` metres for the reversed product. It requires translation error below `1e-12` metres and a reversed result more than one metre from the correct expected position. It is not an additional registered production test.

To repeat it, save the following verifier as `/tmp/order_check.cpp`:

```cpp
#include <Eigen/Core>
#include <iostream>
#include <numbers>
#include "rigid_body_kinematics/forward_kinematics.hpp"
#include "rigid_body_kinematics/joint_definition.hpp"
#include "rigid_body_kinematics/joint_limits.hpp"
#include "rigid_body_kinematics/serial_chain_model.hpp"
#include "rigid_body_kinematics/transform3.hpp"

int main()
{
  namespace rbk = rigid_body_kinematics;
  const auto limits = rbk::RevoluteLimits::from_bounds(
    -std::numbers::pi, std::numbers::pi);
  const auto j1 = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), limits);
  const auto j2 = rbk::RevoluteJoint::from_axis(
    Eigen::Vector3d::UnitZ(), Eigen::Vector3d(2, 0, 0), limits);
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 3) = 3;
  const auto home = rbk::Transform3::from_matrix(m);
  const auto model = rbk::SerialChainModel::from_home_and_joints(home, {j1, j2});
  Eigen::VectorXd q(2);
  q << std::numbers::pi / 2, -std::numbers::pi / 2;
  const rbk::Vector6LinearFirst eta1 = j1.space_screw_axis() * q(0);
  const rbk::Vector6LinearFirst eta2 = j2.space_screw_axis() * q(1);
  const auto e1 = rbk::Transform3::from_exponential_coordinates(eta1);
  const auto e2 = rbk::Transform3::from_exponential_coordinates(eta2);
  const auto correct = rbk::space_form_forward_kinematics(model, q);
  const auto reversed = e2.compose(e1).compose(home);
  const auto pc = correct.transform_point(Eigen::Vector3d::Zero());
  const auto pr = reversed.transform_point(Eigen::Vector3d::Zero());
  const bool pass = (pc - Eigen::Vector3d(1, 2, 0)).norm() < 1e-12 &&
    (pr - Eigen::Vector3d(5, 2, 0)).norm() < 1e-12 &&
    (pr - Eigen::Vector3d(1, 2, 0)).norm() > 1;
  std::cout << "Correct position [m]: " << pc.transpose()
            << "\nReversed position [m]: " << pr.transpose()
            << "\nGK-ACC-008: " << (pass ? "PASS" : "FAIL") << '\n';
  return pass ? 0 : 1;
}

```

Compile and execute from `ros_ws`:

```zsh
c++ -std=c++20 /tmp/order_check.cpp \
  -I install/rigid_body_kinematics/include -I /usr/include/eigen3 \
  -L install/rigid_body_kinematics/lib \
  -Wl,-rpath,"$PWD/install/rigid_body_kinematics/lib" \
  -lrigid_body_kinematics -o /tmp/order_check
/tmp/order_check
```

The expected output ends with `GK-ACC-008: PASS`; a failed comparison returns a nonzero exit status.

## Traceability

| Requirement | Reviewed source | Planned verification |
|---|---|---|
| GK-REP-001, GK-MOD-001, GK-DOM-001 | Chapter 10 sections on configuration coordinates, zero arrangement, home pose, and joint screw axes | Public-model review and GK-ACC-001, GK-ACC-009, and GK-ACC-012 |
| GK-MOD-002 | Chapter 10 normalized revolute and prismatic space screw-axis definitions; Chapter 8 screw-axis geometry | GK-ACC-002 through GK-ACC-006 and invalid-axis cases in GK-ACC-009 |
| GK-FK-001, GK-FK-002 | Chapter 10 space-form product and transform-chain equivalence | GK-ACC-001 through GK-ACC-008 |
| GK-VAL-001, GK-VAL-002, GK-NUM-001, GK-NUM-002 | Chapter 10 supported-input boundaries and the spatial geometry kernel's numerical policy | GK-ACC-009 through GK-ACC-012 |
| GK-API-001, GK-API-002 | Project reusable-core rules and the spatial geometry kernel result contract | Public-interface review, focused deterministic tests, and package-level test reporting |

## Body-form forward kinematics

### Scope and public interface

The body-form evaluator shall be declared in `rigid_body_kinematics/forward_kinematics.hpp`:

```cpp
[[nodiscard]] Transform3 body_form_forward_kinematics(
  const SerialChainModel & model,
  Eigen::Ref<const Eigen::VectorXd> joint_coordinates,
  const NumericalPolicy & policy = NumericalPolicy{});
```

Reuse the existing `SerialChainModel`, joint definitions, typed limits, and error types. The input model continues to own the home pose and ordered space-frame joint geometry. Do not add a second model, public body-axis factory or accessor, persistent body-axis cache, or new numerical-policy field. Do not change which models the existing factories accept.

Body axes shall be derived as local values from the fixed home pose during evaluation. Repeating this calculation for another query is an implementation choice, not a change of reference configuration: neither the current pose nor a partially accumulated pose may replace the home pose in the conversion. No current joint coordinates or mutable cache are stored in the model.

### Governing equations and calculation

The owning derivation is Chapter 10's “Body-form product of exponentials” and its planar 2R example; Chapter 8's normalisation section owns the screw-axis coordinate transformation. For the same joint index $j$, define the home end-effector-frame screw column

$$
\boxed{
{}^{e}\mathbf B_j
:=
\operatorname{Ad}_{\mathbf M^{-1}}{}^{s}\mathbf S_j
\in\mathbb R^6
}.
$$

Here $\mathbf M=\mathbf T_{se}(\mathbf0_n)\in SE(3)$ is the existing home pose, and $\operatorname{Ad}_{\mathbf M^{-1}}\in\mathbb R^{6\times6}$ changes screw coordinates from $\{s\}$ to the home end-effector frame $\{e\}$. The superscript $e$ does not refer to a newly calculated current pose. Both screw columns use linear-first order and describe the same joint geometry, positive direction, and coordinate $q_j$.

Multiplication by $q_j$ gives ${}^{e}\boldsymbol\eta_j={}^e\mathbf B_jq_j$. Its upper block has units of metres and its lower block has units of radians; the latter is zero for a prismatic joint. The hat map turns this six-column into its $4\times4$ Lie-algebra matrix. Define the joint displacement and required pose by

$$
\mathbf G_j(q_j)
:=
\exp\!\left(\widehat{{}^{e}\mathbf B_j}\,q_j\right),
\qquad
\boxed{
\mathbf T_{se}(\mathbf q)
=
\mathbf M\mathbf G_1(q_1)\cdots\mathbf G_n(q_n)
}.
$$

The output remains the pose of $\{e\}$ relative to $\{s\}$, not its inverse. Initialise the accumulated pose with $\mathbf M$ and append the body displacement factors on the right in increasing joint order $1,\ldots,n$. Use the kernel's inverse, adjoint, exponential, and composition operations; do not implement another matrix exponential. The implementation shall evaluate the body factors, not delegate the result to `space_form_forward_kinematics`.

The identity

$$
\mathbf G_j(q_j)=\mathbf M^{-1}\mathbf E_j(q_j)\mathbf M
$$

implies equality with the space-form pose in exact arithmetic. For supported moderate inputs, both numerical results shall agree within the separate translation and rotation tolerances below. When zero is inside every joint domain and the required conversion is numerically supported, $\mathbf q=\mathbf0_n$ returns $\mathbf M$. A zero query outside a declared joint domain must still fail.

### Validation and numerical range

Preserve the existing query contract: validate the exponential policy, coordinate count, all coordinate finiteness, typed joint limits, and revolute angular magnitude before body-axis conversion or exponentiation. Match the existing evaluator's deterministic order: policy, count, a complete finiteness pass, then increasing joint index with each interval check preceding that joint's revolute-angle check. Check $|q_j|$ against `maximum_exponential_angle` only for revolute joints. Do not wrap, clamp, renormalise body screws, or rescale coordinates; the kernel still checks the actual exponential-coordinate magnitude after floating-point conversion.

Compute the home inverse and its adjoint once per call. For each joint, require a finite converted ${}^{e}\mathbf B_j$ and finite scaled ${}^{e}\boldsymbol\eta_j$ before using the exponential. A finite model does not guarantee that the inverse, adjoint, converted screw, scaled screw, or composed pose is representable. Reject any such unsupported calculation with `SerialChainError::unsupported_magnitude`.

Keep `invalid_policy`, `dimension_mismatch`, `non_finite`, and `joint_out_of_domain` for their existing supplied-input errors. Translate geometry-kernel exceptions at the body evaluator boundary: preserve an invalid-policy category; other kernel evaluation failures become `unsupported_magnitude`. Identify home-frame conversion failures separately from joint failures, and include the one-based joint index and useful underlying context when applicable. No failure returns a partial transform or mutates the model or coordinate input.

Body-frame conversion introduces intermediates that the space evaluator need not form. Consequently, equality of the mathematical formulas is not a promise of identical success domains or bitwise-identical results at extreme magnitudes. Do not narrow shared model construction or space-form acceptance merely to make both numerical domains coincide. Do not silently fall back to the space evaluator after a body-form failure.

### Body-form requirements

The body-form operation shall satisfy the shared frame/unit, joint-domain, non-mutation, and typed-failure requirements, together with the following:

| ID          | Requirement                                                                                                                                                                                    | Acceptance                                     |
| ----------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| GK-BODY-001 | Derive body screws from the same model's fixed home inverse using the linear-first adjoint; preserve joint type, sign, coordinate units, and order.                                            | GK-BACC-001 and GK-BACC-002                |
| GK-BODY-002 | Evaluate $\mathbf M\exp(\widehat{{}^{e}\mathbf B_1}q_1)\cdots\exp(\widehat{{}^{e}\mathbf B_n}q_n)$ and return $\mathbf T_{se}$, with home/space/body agreement in the shared supported domain. | GK-BACC-001 and GK-BACC-002                |
| GK-BODY-003 | Reuse the unchanged shared model and query signature, with the same default evaluation policy and no new model ownership or public axis-storage interface.                                     | Interface review and all body acceptance calls |
| GK-BODY-004 | Apply the existing query validation and numerical bounds without changing physical limits or silently repairing inputs.                                                                        | GK-BACC-005                                    |
| GK-BODY-005 | Reject unsupported home conversion, body-screw conversion/scaling, exponential, or composition with the declared serial-chain failure and no partial output.                                   | GK-BACC-005 and GK-BACC-006                    |

### Essential and distinct acceptance cases

Use one spatial revolute–revolute–prismatic (RRP) chain for geometric acceptance. All joint geometry is specified in the space frame at home. Let $L_1=2\,\mathrm m$ and $L_2=1\,\mathrm m$. The first two links initially point along $+x_s$; the third joint extends the second link along its length.

| Joint | Type and home geometry | Coordinate and limits |
|---|---|---|
| 1 | Revolute about $+z_s$ through the base origin | $q_1$ in radians, $[-\pi,\pi]$ |
| 2 | Revolute about $+y_s$ through $[L_1,0,0]^{\mathsf T}$ | $q_2$ in radians, $[-\pi,\pi]$ |
| 3 | Prismatic along $+x_s$ at home | $q_3$ in metres, $[0,1]$ |

Choose the home end-effector orientation $\mathbf R_{se,0}=\mathbf R_z(\pi/2)$ and home position $[L_1+L_2,0,0]^{\mathsf T}$, so

$$
\mathbf M=
\begin{bmatrix}
0&-1&0&3\\
1&0&0&0\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}.
$$

The translation column is measured in metres. This rotated tool-frame choice changes the end-effector coordinate axes, not the physical home link directions or prismatic direction. It exercises both rotation and origin shift in the body-axis conversion without requiring another robot fixture.

Direct link geometry gives the independent pose for this fixture:

$$
{}^{s}\mathbf p_e(\mathbf q)=
\begin{bmatrix}
\bigl(L_1+(L_2+q_3)\cos q_2\bigr)\cos q_1\\
\bigl(L_1+(L_2+q_3)\cos q_2\bigr)\sin q_1\\
-(L_2+q_3)\sin q_2
\end{bmatrix},
\qquad
\mathbf R_{se}(\mathbf q)=
\mathbf R_z(q_1)\mathbf R_y(q_2)\mathbf R_{se,0}.
$$

The prismatic coordinate changes the second link's effective length, not its orientation. Use these geometric relations or the explicit expected matrices below as the oracle, not another invocation of the production exponential calculation.

Use deterministic binary64 inputs. For both geometric cases, require translation residual $e_p\leq10^{-12}\,\mathrm m$ and rotation residual $e_R\leq10^{-12}\,\mathrm{rad}$ as defined above. Compare each body result with the independent expected pose and with the space-form evaluator. Matching the two evaluators alone does not establish correctness.

| ID | Fixture and query | Independent expected result and distinct purpose |
|---|---|---|
| GK-BACC-001 | The RRP fixture at $q_1=q_2=0\,\mathrm{rad}$ and $q_3=0\,\mathrm m$. | Return the complete $\mathbf M$ above, including its non-identity rotation and nonzero translation. Establish the home-pose boundary using the same fixture. |
| GK-BACC-002 | The same RRP fixture at $q_1=\pi/2\,\mathrm{rad}$, $q_2=-\pi/2\,\mathrm{rad}$, and $q_3=0.5\,\mathrm m$. | Return position $[0,2,1.5]^{\mathsf T}\,\mathrm m$ and rotation $\begin{bmatrix}-1&0&0\\0&0&-1\\0&-1&0\end{bmatrix}$. Establish mixed joint types, nonparallel rotation axes, factor order, tool-frame rotation, origin shift, and agreement with the space form in one nonzero configuration. |
| GK-BACC-005 | Run the existing coordinate-count, screw-scaling-overflow, and composition-overflow (GK-ACC-011) checks through both evaluators. Retain the detailed shared-validation checks on the space evaluator. | The coordinate-count case checks that both entry points invoke query validation. Source review shall confirm that both call the same unchanged helper before evaluation; existing space-form cases retain coverage of NaN/infinity, inclusive and rejected limits, invalid policy, revolute angular bounds, and the prismatic-unit distinction. Scaling and composition checks run through both entry points because their arithmetic and error handling are implemented separately. |
| GK-BACC-006 | Let $D_{\max}$ be the largest finite binary64 value. Use one revolute joint about $+z_s$ through $[-D_{\max},0,0]^{\mathsf T}$, home rotation $\mathbf I_3$, home position $[D_{\max},0,0]^{\mathsf T}$, and $q_1=0.25\,\mathrm{rad}$.                                                                                                                                                | Model construction succeeds with finite space screw $[0,D_{\max},0,0,0,1]^{\mathsf T}$. Its body screw would require $[0,2D_{\max},0,0,0,1]^{\mathsf T}$, which is unrepresentable. The body evaluator throws `SerialChainException` with `unsupported_magnitude` and joint 1 context, without returning a pose. This is a conversion overflow, distinct from scaling or composition overflow. |

The two RRP queries are the only required geometric scenarios for body-form acceptance; separate single-joint, planar 2R, and spatial 2R fixtures are not required for this evaluator. Preserve existing space-form tests. Detailed shared-validation cases need not be duplicated for the body evaluator while both entry points use the same helper in the specified order; revisit this evidence strategy if either path adds or bypasses validation. Reuse the three dual-entry checks above without copying entire tests. The conversion-overflow case remains separate because it tests a new numerical failure, not nominal geometry. No public body-axis accessor, exhaustive input permutations, or dedicated repeated-call/default-policy test is required. Test count is not an acceptance target.

## Space and body Jacobians

### Scope and public interfaces

Add two C++20 library operations to the existing `rigid_body_kinematics` package, declared in `rigid_body_kinematics/jacobian.hpp` in namespace `rigid_body_kinematics`. Follow the [C++ style guide](../conventions/cpp_style.md). Use the existing model, joint definitions, numerical policy, and error types; do not add a ROS node, topic, message, or runtime dependency.

```cpp
using JacobianLinearFirst = Eigen::Matrix<double, 6, Eigen::Dynamic>;

[[nodiscard]] JacobianLinearFirst space_jacobian(
  const SerialChainModel & model,
  Eigen::Ref<const Eigen::VectorXd> joint_coordinates,
  const NumericalPolicy & policy = NumericalPolicy{});

[[nodiscard]] JacobianLinearFirst body_jacobian(
  const SerialChainModel & model,
  Eigen::Ref<const Eigen::VectorXd> joint_coordinates,
  const NumericalPolicy & policy = NumericalPolicy{});
```

For $n\geq1$ model joints, each call receives the current $\mathbf q\in\mathcal Q$ and returns an owning, finite $6\times n$ matrix by value. Column $j$ corresponds to joint $j$ in base-to-end-effector order. Neither call receives joint rates: the caller may multiply the returned matrix by $\dot{\mathbf q}$ when a twist is required. Both operations shall be deterministic and shall not mutate or retain references to their inputs.

Their meanings are

$$
\boldsymbol\xi_s=\mathbf J_s(\mathbf q)\dot{\mathbf q},
\qquad
\boldsymbol\xi_e=\mathbf J_e(\mathbf q)\dot{\mathbf q}.
$$

Both twists describe end-effector motion relative to fixed $\{s\}$. The space twist uses coordinates in $\{s\}$; the body twist uses coordinates in the attached $\{e\}$ at the supplied configuration. Rows 1–3 are linear and rows 4–6 are angular. Their output units are $\mathrm{m\,s^{-1}}$ and $\mathrm{rad\,s^{-1}}$, respectively. Each Jacobian column carries those output units divided by its joint-rate unit: a revolute column's blocks have units $\mathrm{m\,rad^{-1}}$ and $\mathrm{rad\,rad^{-1}}$; a prismatic column's upper block is dimensionless and its angular block is zero, with units $\mathrm{rad\,m^{-1}}$.

The upper block of $\mathbf J_s\dot{\mathbf q}$ is the linear space-twist component $\boldsymbol\nu_s$, not the tool-origin velocity. For current pose $\mathbf T_{se}=[\mathbf R_{se},{}^{s}\mathbf p_e;\mathbf0_3^{\mathsf T},1]$, the latter is

$$
{}^{s}\dot{\mathbf p}_e
=\boldsymbol\nu_s+\boldsymbol\omega_s\times{}^{s}\mathbf p_e
=\mathbf R_{se}\boldsymbol\nu_e.
$$

Do not introduce another model, persistent body-axis cache, public per-column evaluator, twist-evaluation wrapper, or new numerical-policy field. Task-coordinate Jacobians, rank analysis, pseudoinverses, inverse kinematics, Jacobian time derivatives, dynamics, URDF parsing, and visualization changes are outside this addition. A singular configuration is not itself an error: these operations evaluate a Jacobian, not its inverse.

### Space-Jacobian calculation

The owning formula is [the space-Jacobian column and rate map](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#eq-space-jacobian-columns-rate-map). Reuse the stored home space screws ${}^{s}\mathbf S_j$ and $\mathbf E_j(q_j)=\exp(\widehat{{}^{s}\mathbf S_j}q_j)$:

$$
\begin{aligned}
\mathbf J_{s,1}(\mathbf q)&={}^{s}\mathbf S_1,\\
\mathbf J_{s,j}(\mathbf q)
&=\operatorname{Ad}_{\mathbf E_1(q_1)\cdots\mathbf E_{j-1}(q_{j-1})}
{}^{s}\mathbf S_j,\qquad j=2,\ldots,n,\\
\mathbf J_s(\mathbf q)
&=\begin{bmatrix}\mathbf J_{s,1}(\mathbf q)&\cdots&\mathbf J_{s,n}(\mathbf q)\end{bmatrix}.
\end{aligned}
$$

Evaluate the columns with one forward pass. Start the preceding-joint displacement at $\mathbf I_4$, calculate column $j$ using its current adjoint, and append $\mathbf E_j$ on the right only when another column remains. Thus column $j$ uses only joints $1,\ldots,j-1$. The space Jacobian requires neither $\mathbf E_n$ nor multiplication by the home pose; do not calculate a complete forward pose merely to obtain it. For $n=1$, return the single stored space column after query validation.

### Body-Jacobian calculation

The owning formula is [the body-Jacobian column and rate map](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#eq-body-jacobian-columns-rate-map). Derive finite local home body screws once per call using

$$
{}^{e}\mathbf B_j=\operatorname{Ad}_{\mathbf M^{-1}}{}^{s}\mathbf S_j,
\qquad
\mathbf G_j(q_j)=\exp(\widehat{{}^{e}\mathbf B_j}q_j).
$$

The superscript $e$ on $\mathbf B_j$ refers to the home frame. These columns remain fixed with respect to the query coordinates; the output columns use $\{e\}$ at the current configuration:

$$
\begin{aligned}
\mathbf J_{e,n}(\mathbf q)&={}^{e}\mathbf B_n,\\
\mathbf J_{e,j}(\mathbf q)
&=\operatorname{Ad}_{\mathbf G_n(q_n)^{-1}\cdots\mathbf G_{j+1}(q_{j+1})^{-1}}
{}^{e}\mathbf B_j,\qquad j=1,\ldots,n-1,\\
\mathbf J_e(\mathbf q)
&=\begin{bmatrix}\mathbf J_{e,1}(\mathbf q)&\cdots&\mathbf J_{e,n}(\mathbf q)\end{bmatrix}.
\end{aligned}
$$

Evaluate in descending joint order, while writing each result to its original column. Start the inverse later-joint displacement at $\mathbf I_4$. After calculating column $j$, append $\mathbf G_j^{-1}=\exp(-\widehat{{}^{e}\mathbf B_j}q_j)$ on the right only when an earlier column remains. For three joints, the displacements used for columns 3, 2, and 1 are respectively $\mathbf I_4$, $\mathbf G_3^{-1}$, and $\mathbf G_3^{-1}\mathbf G_2^{-1}$. Do not reverse the returned column order. For $n=1$, return the single converted home body column after query validation.

Use the kernel's exponential, composition, inverse, and adjoint operations. Implement this body-column calculation independently of `space_jacobian`; do not implement it by calling `space_jacobian` and converting its output. This allows their relationship to provide a useful consistency check. Neither evaluator shall use finite differences as its production algorithm.

### Body–space relationship

For the same model, configuration, joint ordering, and component units, the required relation is [the current-pose adjoint identity](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#eq-body-space-jacobian-adjoint):

$$
\begin{aligned}
\mathbf J_e(\mathbf q)
&=\operatorname{Ad}_{\mathbf T_{se}(\mathbf q)^{-1}}\mathbf J_s(\mathbf q),\\
\mathbf J_s(\mathbf q)
&=\operatorname{Ad}_{\mathbf T_{se}(\mathbf q)}\mathbf J_e(\mathbf q).
\end{aligned}
$$

The $6\times6$ inverse adjoint is

$$
\operatorname{Ad}_{\mathbf T_{se}^{-1}}
=\begin{bmatrix}
\mathbf R_{se}^{\mathsf T}&
-\mathbf R_{se}^{\mathsf T}\left[{}^{s}\mathbf p_e\right]_\times\\
\mathbf0_{3\times3}&\mathbf R_{se}^{\mathsf T}
\end{bmatrix}.
$$

Use the complete current pose, not $\mathbf M^{-1}$ and not rotation alone. At home the relation reduces to $\mathbf J_e(\mathbf0_n)=\operatorname{Ad}_{\mathbf M^{-1}}\mathbf J_s(\mathbf0_n)$. The relation needs no third public function: tests shall obtain the current pose from the existing forward-kinematics operation and use `Transform3::inverse()`, `Transform3::adjoint()`, and matrix multiplication. Do not add another adjoint implementation.

These equalities are exact mathematical statements. Numerical comparisons use the tolerances below and require that both Jacobians, the current pose, and the conversion are supported. Different required intermediates can produce different success domains at extreme magnitudes; do not require bitwise equality or silently fall back to the other algorithm after failure.

### Minimal validation and failure contract

Reuse the existing private serial-chain query validation: policy, coordinate count, a complete coordinate-finiteness pass, then each joint's inclusive interval and revolute angular bound in increasing joint order. Share that implementation internally if a separate source file requires it; do not duplicate the checks or expose a new public validation API. Validate all supplied coordinates even when a particular Jacobian does not depend on one of them. Keep existing error codes and never wrap, clamp, or repair a query. Do not revalidate immutable model geometry.

Require finite body-axis conversions, scaled exponential coordinates, required adjoints and displacements, and returned columns. A non-finite supplied coordinate remains `non_finite`; an unrepresentable required calculation is `unsupported_magnitude`. At each Jacobian boundary, preserve a kernel `invalid_policy` category and translate other kernel calculation failures to `unsupported_magnitude`, with the affected joint index or home-conversion context. Failure throws `SerialChainException`, returns no partial or substitute Jacobian, and leaves inputs unchanged. Checks concern the factors actually required by the selected Jacobian; no unused full-pose calculation is required just to reproduce a forward-kinematics failure.

No new exhaustive invalid-input, extreme-magnitude, or policy-permutation suite is required for this addition. Preserve the existing shared-validation tests and review that both new entry points call the unchanged validator and check their own finite intermediates. Reusing those tests is not a claim that every new arithmetic failure path has been tested. Revisit focused error tests if validation logic changes or a defect is observed; the reduced test scope does not remove the failure contract.

### Minimal mathematical acceptance

Use only the existing [RRP fixture](#essential-and-distinct-acceptance-cases): $L_1=2\,\mathrm m$, $L_2=1\,\mathrm m$, home rotation $\mathbf R_z(\pi/2)$, and the same joint geometry and limits. There are two reference configurations: home and the existing nonzero query. The finite-difference check uses small perturbations around that same nonzero query, not another robot scenario.

All expected matrices below use the declared joint units. Obtain expected values from geometry, not by calling the production Jacobian, exponential, or adjoint to manufacture the oracle. For a revolute hinge through $\mathbf p\in\mathbb R^3$ in metres with dimensionless unit direction $\mathbf a\in\mathbb R^3$, both expressed in $\{s\}$ at the query, the geometric space column is $[-(\mathbf a\times\mathbf p)^{\mathsf T}\;\mathbf a^{\mathsf T}]^{\mathsf T}$. The body's linear column is $\mathbf R_{se}^{\mathsf T}[\mathbf a\times({}^{s}\mathbf p_e-\mathbf p)]$ and its angular column is $\mathbf R_{se}^{\mathsf T}\mathbf a$. For a prismatic joint, rotate its current unit sliding direction into the selected frame and use a zero angular block.

**Home columns.** At $\mathbf q=\mathbf0_3$, the expected matrices are

$$
\mathbf J_s(\mathbf0_3)
=\begin{bmatrix}
0&0&1\\
0&0&0\\
0&2&0\\
0&0&0\\
0&1&0\\
1&0&0
\end{bmatrix},
\qquad
\mathbf J_e(\mathbf0_3)
=\begin{bmatrix}
3&0&0\\
0&0&-1\\
0&-1&0\\
0&1&0\\
0&0&0\\
1&0&0
\end{bmatrix}.
$$

They are respectively $[{}^{s}\mathbf S_1\;{}^{s}\mathbf S_2\;{}^{s}\mathbf S_3]$ and $[{}^{e}\mathbf B_1\;{}^{e}\mathbf B_2\;{}^{e}\mathbf B_3]$. The prismatic home coordinate is an included lower endpoint; do not apply central differences at that endpoint.

**Nonzero columns and current pose.** At $q_1=\pi/2\,\mathrm{rad}$, $q_2=-\pi/2\,\mathrm{rad}$, and $q_3=0.5\,\mathrm m$, write the query as $\mathbf q_\star$. The expected matrices are

$$
\mathbf J_s(\mathbf q_\star)
=\begin{bmatrix}
0&0&0\\
0&0&0\\
0&2&1\\
0&-1&0\\
0&0&0\\
1&0&0
\end{bmatrix},
\qquad
\mathbf J_e(\mathbf q_\star)
=\begin{bmatrix}
2&0&0\\
0&0&-1\\
0&-1.5&0\\
0&1&0\\
-1&0&0\\
0&0&0
\end{bmatrix}.
$$

The current pose is the independently established GK-BACC-002 value,

$$
\mathbf T_{se}(\mathbf q_\star)
=\begin{bmatrix}
-1&0&0&0\\
0&0&-1&2\\
0&-1&0&1.5\\
0&0&0&1
\end{bmatrix}.
$$

This single nonzero configuration exercises both preceding-joint displacements, mixed joint types, the body inverse-factor order, and rotation and translation in the current adjoint. Compare both computed Jacobians with the literal geometric matrices before checking their mutual relation; agreement with each other alone is insufficient.

**One local derivative check.** Construct $\mathbf T_{\mathrm{geo}}(\mathbf q)$ from the RRP position and orientation formulas in the fixture definition, using ordinary trigonometry and matrix assembly, not either production forward-kinematics or Jacobian operation. For $j=1,2,3$, let $\mathbf d_j\in\mathbb R^3$ be the joint-coordinate basis column with entry $j$ equal to one and the other entries zero. Use $h_1=h_2=10^{-6}\,\mathrm{rad}$ and $h_3=10^{-6}\,\mathrm m$:

$$
\begin{aligned}
\mathbf D_j
&=\frac{\mathbf T_{\mathrm{geo}}(\mathbf q_\star+h_j\mathbf d_j)
-\mathbf T_{\mathrm{geo}}(\mathbf q_\star-h_j\mathbf d_j)}{2h_j},\\
\mathbf X_{s,j}
&=\mathbf D_j\mathbf T_{\mathrm{geo}}(\mathbf q_\star)^{-1},\\
\mathbf X_{e,j}
&=\mathbf T_{\mathrm{geo}}(\mathbf q_\star)^{-1}\mathbf D_j.
\end{aligned}
$$

Each $\mathbf D_j$ and $\mathbf X_{s,j},\mathbf X_{e,j}$ is $4\times4$. The latter two approximate the hat matrices of the corresponding Jacobian columns, not finite transforms. Extract the linear estimate from their top three entries in column 4. For either matrix $\mathbf X$, extract the angular estimate from the skew part of its upper-left block:

$$
\begin{bmatrix}
(X_{32}-X_{23})/2\\
(X_{13}-X_{31})/2\\
(X_{21}-X_{12})/2
\end{bmatrix}.
$$

Indices here are one-based. Taking the skew part handles finite-difference roundoff; it does not project a production pose or repair a query. All perturbed configurations lie inside the fixture's limits. Compare all three columns of each computed Jacobian with these estimates; no extra random configurations, step-size sweep, or separate joint-rate fixture is required.

**Tolerances and required checks.** Use binary64 arithmetic and absolute per-column tolerances at these moderate scales. If $u_j$ denotes the coordinate unit of joint $j$ ($\mathrm{rad}$ or $\mathrm m$), compare the three-entry linear error norm in $\mathrm m/u_j$ and the three-entry angular error norm in $\mathrm{rad}/u_j$ separately. Require each norm to be at most $10^{-12}$ in its stated unit for analytic matrices and adjoint comparisons, and at most $10^{-8}$ in its stated unit for finite differences. Do not use one mixed-unit six-vector norm or whole-matrix relative comparison. Every result must also have six rows, three columns, and finite entries.

| ID | Required mathematical check | Distinct purpose |
|---|---|---|
| GK-JACC-001 | Evaluate both Jacobians at home and compare with the two explicit home matrices. | Home screws, linear-first row order, rotated home-frame conversion, and included zero-coordinate boundary. |
| GK-JACC-002 | At $\mathbf q_\star$, compare both Jacobians with the explicit nonzero matrices. Obtain the current pose using `space_form_forward_kinematics`, retain its established GK-BACC-002 pose check, and verify $\mathbf J_e=\operatorname{Ad}_{\mathbf T_{se}^{-1}}\mathbf J_s$. | Ordered space/body calculations, mixed joints, and agreement using the current frame rather than the fixed home frame. The adjoint assertion reuses this case; it is not another fixture. |
| GK-JACC-003 | At the same $\mathbf q_\star$, compare every column with the central-difference estimates from $\mathbf T_{\mathrm{geo}}$. | Independent verification that the matrices are the space/body pose-derivative maps, with the correct side of inverse-pose multiplication. |

These three mathematical checks are the complete new acceptance batch. They may share fixture setup and comparison helpers; test-function count is not a target. Do not add separate single-joint or planar fixtures, duplicated input-validation cases, repeated-call tests, a second test of the inverse adjoint direction, or exhaustive boundary/overflow cases for this addition. Existing package regressions remain required, especially if the private query validator is moved for reuse.

### Jacobian requirements and traceability

| ID | Requirement | Mathematical source or existing contract | Acceptance |
|---|---|---|---|
| GK-JAC-001 | `space_jacobian` shall return the ordered preceding-joint adjoint columns, with first column ${}^{s}\mathbf S_1$. | [Space-Jacobian formula](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#eq-space-jacobian-columns-rate-map) | GK-JACC-001–003 |
| GK-JAC-002 | `body_jacobian` shall independently return the inverse later-joint adjoint columns derived from fixed home body screws, with last column ${}^{e}\mathbf B_n$. | [Body-Jacobian formula](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#eq-body-jacobian-columns-rate-map) | GK-JACC-001–003 |
| GK-JAC-003 | Both matrices shall satisfy the current-pose adjoint relation within the declared shared numerical domain and tolerances. | [Body–space relation](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#eq-body-space-jacobian-adjoint) | GK-JACC-002 |
| GK-JAC-004 | Return an owning finite $6\times n$ matrix, preserve joint and linear-first order and units, and keep the core deterministic, non-mutating, and ROS-runtime independent. | Shared model and the public interfaces above | GK-JACC-001–003 and interface review |
| GK-JAC-005 | Reuse shared query validation and typed errors, reject unsupported required intermediates, and return no partial Jacobian. | Existing query contract and the minimal validation section above | Existing validator regressions and focused source review; no claim of exhaustive new-path failure testing |
