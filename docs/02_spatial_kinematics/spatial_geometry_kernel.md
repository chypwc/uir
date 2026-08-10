# Spatial Geometry Kernel Capability Specification

## Capability and question

The active capability is a deterministic, library-independent spatial geometry kernel. It answers the Phase 1 question: given frame-labelled rotations, rigid transformations, twists, or planar poses, what geometrically valid result follows from the reviewed $SO(3)$ and $SE(3)$ model, and when must the calculation fail rather than return a plausible-looking value?

The prerequisite theory is documented and reviewed in [Degrees of Freedom and Spatial Motion: $SO(3)$ and $SE(3)$](../../notes/07_degrees_of_freedom_and_spatial_motion_so3_se3.qmd) and [Screw Motion, the $SE(3)$ Exponential, and Spatial Integration](../../notes/08_screw_motion_se3_exponential_and_spatial_integration.qmd). Those chapters own the derivations; this specification freezes the minimum implementation contract and its independent acceptance evidence.

## Intended behaviour

The capability shall:

1. represent and validate rotations and rigid transformations without silently projecting invalid arrays onto $SO(3)$ or $SE(3)$;
2. keep free-vector rotation distinct from point transformation;
3. compose and invert frame-labelled transformations in the declared multiplication order;
4. provide $SO(3)$ and linear-first $SE(3)$ hat and vee maps, exponential and principal-logarithm maps, and the linear-first adjoint;
5. integrate constant space and body twists with the correct left or right group action;
6. embed planar poses in $SE(3)$, extract only validated planar transformations, and convert planar yaw to a normalized ROS-order quaternion; and
7. return explicit deterministic success classifications or failures for numerical branch boundaries, invalid representations, and unsupported magnitudes.

## Conventions, frames, types, and units

### Geometric convention

$\mathbf T_{ab}$ is the pose of frame $\{b\}$ relative to frame $\{a\}$ and maps coordinates expressed in $\{b\}$ into coordinates expressed in $\{a\}$ by left multiplication:

$$
\mathbf T_{ab}
=
\begin{bmatrix}
\mathbf R_{ab}&{}^{a}\mathbf p_b\\
\mathbf 0_3^{\mathsf T}&1
\end{bmatrix},
\qquad
{}^{a}\mathbf p_P
=
\mathbf R_{ab}{}^{b}\mathbf p_P
+{}^{a}\mathbf p_b.
$$

Here $\mathbf R_{ab}\in SO(3)$ is dimensionless, ${}^{a}\mathbf p_b\in\mathbb R^3$ is measured in metres, and column vectors are used throughout. Products act rightmost first, so compatible transformations compose as $\mathbf T_{ac}=\mathbf T_{ab}\mathbf T_{bc}$. Positive rotations follow the right-hand rule.

A free vector has no point of application and therefore receives no translation:

$$
{}^{a}\mathbf v
=
\mathbf R_{ab}{}^{b}\mathbf v.
$$

Public interfaces shall name the source and destination frames. The first implementation may use fixed-size numeric storage without compile-time frame tags, but it shall document frame preconditions and shall not infer, relabel, or mix frames silently.

### Linear-first order

Every finite exponential-coordinate column and actual twist uses the project linear-first order:

$$
{}^{a}\boldsymbol\eta
=
\begin{bmatrix}
{}^{a}\boldsymbol\rho\\
{}^{a}\boldsymbol\phi
\end{bmatrix},
\qquad
{}^{a}\boldsymbol\xi
=
\begin{bmatrix}
{}^{a}\boldsymbol\nu\\
{}^{a}\boldsymbol\omega
\end{bmatrix}.
$$

The finite blocks ${}^{a}\boldsymbol\rho$ and ${}^{a}\boldsymbol\phi$ are measured in metres and radians. The rate blocks ${}^{a}\boldsymbol\nu$ and ${}^{a}\boldsymbol\omega$ are measured in $\mathrm{m\,s^{-1}}$ and $\mathrm{rad\,s^{-1}}$. Radians are dimensionless in algebra but retain their label to distinguish angular from linear quantities. The kernel shall never normalize a mixed-unit six-vector with one Euclidean norm.

### Abstract data types

The contract uses the following abstract fixed-size types; the implementation companion shall map them to concrete Eigen and C++ types without changing their semantics.

| Type | Mathematical content | Units and invariant |
|---|---|---|
| `Vector3` | $\mathbb R^3$ column | Declared by each operation |
| `Matrix3` | $3\times3$ real array | Dimensionless unless stated otherwise |
| `Rotation3` | validated $\mathbf R\in SO(3)$ | Dimensionless; no reflection or silent projection |
| `Transform3` | validated $(\mathbf R,\mathbf p)\in SE(3)$ | Rotation dimensionless; translation in metres |
| `TangentSO3` | skew matrix $\boldsymbol\Phi\in\mathfrak{so}(3)$ | Radians for finite coordinates or $\mathrm{rad\,s^{-1}}$ for rates |
| `TangentSE3` | structured $4\times4$ matrix in $\mathfrak{se}(3)$ | Linear and angular blocks retain separate units |
| `Vector6LinearFirst` | $[\text{linear};\text{angular}]$ | Block units declared by each operation |
| `PlanarPose` | $[x\;y\;\theta]^{\mathsf T}$ | metres, metres, radians |
| `QuaternionXYZW` | $[q_x\;q_y\;q_z\;q_w]^{\mathsf T}$ | Dimensionless ROS 2 field order; unit norm |

Raw dynamically shaped arrays, when accepted by an adapter, shall be shape-checked before conversion to these fixed-size core types. A shape error shall not reach a geometric formula.

## Mathematical model

### Representation checks and group operations

A rotation is valid only when

$$
\mathbf R^{\mathsf T}\mathbf R=\mathbf I_3,
\qquad
\det(\mathbf R)=1.
$$

A homogeneous transformation additionally has the bottom row $[0\;0\;0\;1]$. Numerical validation uses separate residuals

$$
e_{\mathrm{orth}}
=
\|\mathbf R^{\mathsf T}\mathbf R-\mathbf I_3\|_{\mathrm F},
\qquad
e_{\det}
=
|\det(\mathbf R)-1|,
\qquad
e_{\mathrm{row}}
=
\left\|
\begin{bmatrix}
\mathbf r\\t_{44}-1
\end{bmatrix}
\right\|_\infty.
$$

For validated inputs,

$$
\mathbf T_{ab}^{-1}
=
\begin{bmatrix}
\mathbf R_{ab}^{\mathsf T}&-\mathbf R_{ab}^{\mathsf T}{}^{a}\mathbf p_b\\
\mathbf 0_3^{\mathsf T}&1
\end{bmatrix}.
$$

Every constructor and group operation shall reject non-finite inputs or results. A tolerated bottom-row residual may be classified as valid by an array adapter, after which the typed value stores $(\mathbf R,\mathbf p)$ and serializes the structural row exactly. The rotation block itself shall not be projected, orthogonalized, or repaired.

### Hat, vee, and adjoint maps

For $\mathbf x=[x_1\;x_2\;x_3]^{\mathsf T}$, define

$$
[\mathbf x]_\times
=
\begin{bmatrix}
0&-x_3&x_2\\
x_3&0&-x_1\\
-x_2&x_1&0
\end{bmatrix},
\qquad
[\mathbf x]_\times\mathbf y=\mathbf x\times\mathbf y.
$$

The $SO(3)$ hat and vee maps are inverse on their declared domains. In linear-first order, the $SE(3)$ maps are

$$
\widehat{\boldsymbol\eta}
=
\begin{bmatrix}
[\boldsymbol\phi]_\times&\boldsymbol\rho\\
\mathbf 0_3^{\mathsf T}&0
\end{bmatrix},
\qquad
\widehat{\boldsymbol\xi}
=
\begin{bmatrix}
[\boldsymbol\omega]_\times&\boldsymbol\nu\\
\mathbf 0_3^{\mathsf T}&0
\end{bmatrix}.
$$

Vee operations shall reject matrices outside the applicable skew or structured tangent tolerance; they shall not silently discard symmetric or bottom-row components.

For $\mathbf T_{ab}=[\mathbf R_{ab},{}^{a}\mathbf p_b;\mathbf 0_3^{\mathsf T},1]$, the linear-first adjoint is

$$
\operatorname{Ad}_{\mathbf T_{ab}}
=
\begin{bmatrix}
\mathbf R_{ab}&[{}^{a}\mathbf p_b]_\times\mathbf R_{ab}\\
\mathbf 0_{3\times3}&\mathbf R_{ab}
\end{bmatrix},
\qquad
{}^{a}\boldsymbol\xi
=
\operatorname{Ad}_{\mathbf T_{ab}}{}^{b}\boldsymbol\xi.
$$

It shall satisfy the convention-checking identity

$$
\widehat{\operatorname{Ad}_{\mathbf T}\boldsymbol\xi}
=
\mathbf T\widehat{\boldsymbol\xi}\mathbf T^{-1}.
$$

### Exponential and principal logarithm

Following Chapter 8, this specification uses the lowercase symbol $\exp$ for the matrix exponential and does not introduce separate $\operatorname{Exp}_{SO(3)}$ or $\operatorname{Exp}_{SE(3)}$ operators. For any square matrix $\mathbf A\in\mathbb R^{m\times m}$,

$$
\exp(\mathbf A)
\mathrel{:=}
\sum_{k=0}^{\infty}\frac{\mathbf A^k}{k!}.
$$

The **$SO(3)$ exponential map** first uses the hat map to write a finite rotation-vector column $\boldsymbol\phi\in\mathbb R^3$ as the skew matrix $\boldsymbol\Phi=[\boldsymbol\phi]_\times\in\mathfrak{so}(3)$ and then applies the matrix exponential:

$$
\boldsymbol\phi
\longmapsto
\boldsymbol\Phi
\longmapsto
\mathbf R(\boldsymbol\phi)
\mathrel{:=}
\exp(\boldsymbol\Phi)
\in SO(3).
$$

Thus the direction of a nonzero $\boldsymbol\phi$ is the oriented rotation axis and $\theta=\|\boldsymbol\phi\|_2$ is the rotation angle in radians. Rodrigues' formula evaluates this map as

$$
\mathbf R(\boldsymbol\phi)
=
\mathbf I_3
+A(\theta)\boldsymbol\Phi
+B(\theta)\boldsymbol\Phi^2,
\qquad
A(\theta)=\frac{\sin\theta}{\theta},
\qquad
B(\theta)=\frac{1-\cos\theta}{\theta^2},
$$

with continuous series values at zero. Define

$$
C(\theta)=\frac{\theta-\sin\theta}{\theta^3},
\qquad
\mathbf J(\boldsymbol\phi)
=
\mathbf I_3+B(\theta)\boldsymbol\Phi+C(\theta)\boldsymbol\Phi^2.
$$

For $0\leq\theta\leq\theta_{\mathrm{series}}$, the implementation shall evaluate

$$
\begin{aligned}
A(\theta)
&=
1-\frac{\theta^2}{6}
+\frac{\theta^4}{120}
-\frac{\theta^6}{5040},\\
B(\theta)
&=
\frac12-\frac{\theta^2}{24}
+\frac{\theta^4}{720}
-\frac{\theta^6}{40320},\\
C(\theta)
&=
\frac16-\frac{\theta^2}{120}
+\frac{\theta^4}{5040}
-\frac{\theta^6}{362880}.
\end{aligned}
$$

The **$SE(3)$ exponential map** acts on the complete finite linear-first exponential-coordinate column

$$
{}^{a}\boldsymbol\eta
=
\begin{bmatrix}
{}^{a}\boldsymbol\rho\\
{}^{a}\boldsymbol\phi
\end{bmatrix}
\in\mathbb R^6,
\qquad
\widehat{{}^{a}\boldsymbol\eta}
=
\begin{bmatrix}
[{}^{a}\boldsymbol\phi]_\times&{}^{a}\boldsymbol\rho\\
\mathbf 0_3^{\mathsf T}&0
\end{bmatrix}
\in\mathfrak{se}(3).
$$

Both blocks are expressed in frame $\{a\}$; ${}^{a}\boldsymbol\rho$ is measured in metres and ${}^{a}\boldsymbol\phi$ in radians. Applying the same matrix exponential produces the finite rigid-displacement transformation

$$
\mathbf T_\Delta
\mathrel{:=}
\exp\!\left(
\widehat{{}^{a}\boldsymbol\eta}
\right)
=
\begin{bmatrix}
\mathbf R({}^{a}\boldsymbol\phi)&\mathbf J({}^{a}\boldsymbol\phi){}^{a}\boldsymbol\rho\\
\mathbf 0_3^{\mathsf T}&1
\end{bmatrix}.
$$

Therefore $\mathbf R({}^{a}\boldsymbol\phi)=\exp([{}^{a}\boldsymbol\phi]_\times)$ is the $SO(3)$ exponential result and $\mathbf T_\Delta=\exp(\widehat{{}^{a}\boldsymbol\eta})$ is the $SE(3)$ exponential result. These names describe two uses of the same matrix exponential, not two unrelated scalar exponential functions. In general the final translation is ${}^{a}\mathbf p=\mathbf J({}^{a}\boldsymbol\phi){}^{a}\boldsymbol\rho$, so ${}^{a}\boldsymbol\rho$ equals the translation column only when the rotation is zero or in another special case where the mapping leaves it unchanged.

The $SO(3)$ and $SE(3)$ logarithms shall select the principal rotation magnitude $\theta\in[0,\pi]$. For a validated rotation, define

$$
c_\theta=\frac{\operatorname{tr}(\mathbf R)-1}{2},
\qquad
\mathbf u=
\begin{bmatrix}
R_{32}-R_{23}\\R_{13}-R_{31}\\R_{21}-R_{12}
\end{bmatrix},
\qquad
s_\theta=\frac12\|\mathbf u\|_2,
\qquad
\theta=\operatorname{atan2}(s_\theta,c_\theta).
$$

The deterministic order is identity, near-$\pi$, then nominal or small-angle evaluation. When the computed angle satisfies $\theta\leq\varepsilon_{\mathrm{zero}}$, return $\boldsymbol\phi=\mathbf 0_3$ before normalizing an axis. With the fixed zero threshold, this case is selected only when the computed angle is zero. Away from $\pi$,

$$
\boldsymbol\phi
=
\frac{\theta}{2s_\theta}\mathbf u,
$$

For $0<\theta\leq\theta_{\mathrm{series}}$, evaluate its factor as

$$
\frac{\theta}{2\sin\theta}
=
\frac12
+\frac{\theta^2}{12}
+\frac{7\theta^4}{720};
$$

above the threshold, use the trigonometric ratio. When $\pi-\theta\leq\varepsilon_\pi$, recover the axis from

$$
\mathbf Q
=
\frac{\tfrac12(\mathbf R+\mathbf R^{\mathsf T})-c_\theta\mathbf I_3}{1-c_\theta}
=
\mathbf s_\omega\mathbf s_\omega^{\mathsf T}.
$$

Choose the lowest index $k$ attaining $\max_i Q_{ii}$, take the positive square root for component $k$, recover the remaining components from column $k$, and normalize the candidate. For a resolvable rotation below $\pi$, choose its sign so that $\mathbf s_\omega^{\mathsf T}\mathbf u>0$. If the dot product is zero at the exact branch boundary, keep the selected largest-magnitude component positive. Thus the lowest-index rule resolves both an exact diagonal tie and the canonical sign at $\theta=\pi$.

After selecting $\boldsymbol\phi$, recover the finite linear block from

$$
\boldsymbol\rho
=
\mathbf J(\boldsymbol\phi)^{-1}\mathbf p,
\qquad
\mathbf J^{-1}
=
\mathbf I_3-\frac12\boldsymbol\Phi+D(\theta)\boldsymbol\Phi^2,
$$

where

$$
D(\theta)
=
\begin{cases}
\dfrac{1}{12}
+\dfrac{\theta^2}{720}
+\dfrac{\theta^4}{30240}
+\dfrac{\theta^6}{1209600},
&0\leq\theta\leq\theta_{\mathrm{series}},\\[0.8em]
\dfrac{1}{\theta^2}
-\dfrac{1}{2\theta}\cot\!\left(\dfrac{\theta}{2}\right),
&\theta>\theta_{\mathrm{series}}.
\end{cases}
$$

Identity rotation gives $\boldsymbol\rho=\mathbf p$. The logarithm shall return the finite coordinate column as its primary result and shall not factor a normalized screw implicitly.

### Constant-twist integration

For a finite interval $\Delta t\geq0$ seconds with a constant space twist expressed in frame $\{a\}$,

$$
\mathbf T_{ab}(t+\Delta t)
=
\exp\!\left(
\widehat{{}^{a}\boldsymbol\xi}_s\Delta t
\right)
\mathbf T_{ab}(t).
$$

For a constant body twist expressed in frame $\{b\}$,

$$
\mathbf T_{ab}(t+\Delta t)
=
\mathbf T_{ab}(t)
\exp\!\left(
\widehat{{}^{b}\boldsymbol\xi}_b\Delta t
\right).
$$

The two updates describe the same instantaneous motion when ${}^{a}\boldsymbol\xi_s=\operatorname{Ad}_{\mathbf T_{ab}(t)}{}^{b}\boldsymbol\xi_b$. A zero interval shall return the input pose unchanged. A negative interval is outside this integration interface; signed finite exponential coordinates remain available through the exponential map.

### Planar interoperability

The embedding of $[x\;y\;\theta]^{\mathsf T}$ is

$$
\mathbf T^{(3)}(x,y,\theta)
=
\begin{bmatrix}
\cos\theta&-\sin\theta&0&x\\
\sin\theta&\cos\theta&0&y\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}.
$$

Extraction first validates the complete $SE(3)$ input and then requires

$$
\delta_R=\|\mathbf R\mathbf e_z-\mathbf e_z\|_2\leq\varepsilon_{R,\mathrm{planar}},
\qquad
\delta_p=|\mathbf e_z^{\mathsf T}\mathbf p|\leq\varepsilon_{p,\mathrm{planar}}.
$$

The residuals are respectively dimensionless and measured in metres. A valid planar transform returns

$$
x=p_1,
\qquad
y=p_2,
\qquad
\theta=\operatorname{atan2}(R_{21},R_{11})\in(-\pi,\pi].
$$

This operation is an extraction, not a projection. It shall reject non-planar inputs and shall not discard vertical translation, roll, or pitch.

Planar yaw converts to the normalized ROS 2 field-order quaternion

$$
\mathbf q_{xyzw}(\theta)
=
\begin{bmatrix}
0&0&\sin(\theta/2)&\cos(\theta/2)
\end{bmatrix}^{\mathsf T}.
$$

The output shall be normalized before return to remove finite-precision norm drift. The quaternions $\mathbf q$ and $-\mathbf q$ represent the same orientation; tests shall compare their rotation action or use a sign-invariant metric rather than require componentwise equality.

## Interfaces and result contract

The initial implementation shall be a C++ library with Eigen storage and no ROS runtime dependency in the mathematical core. Public functions shall be deterministic, stateless, and non-mutating.

| Operation family | Inputs | Output |
|---|---|---|
| Validation/construction | Raw $3\times3$ rotation or raw $4\times4$ transform, numerical policy | Validated typed value or explicit failure |
| Vector and point actions | $\mathbf R_{ab}$ or $\mathbf T_{ab}$, frame-compatible column | Rotated vector or transformed point |
| Group operations | Frame-compatible validated rotations or transforms | Composition or inverse |
| Hat and vee | Three- or six-column, or tangent matrix | Corresponding tangent matrix or column |
| Adjoint | $\mathbf T_{ab}$ and optional ${}^{b}\boldsymbol\xi$ | $6\times6$ linear-first adjoint or ${}^{a}\boldsymbol\xi$ |
| Exponential | $\boldsymbol\phi$ or $[\boldsymbol\rho;\boldsymbol\phi]$ | Validated rotation or transform |
| Principal logarithm | Validated rotation or transform | Finite coordinates plus deterministic case classification |
| Constant-twist integration | Initial transform, constant space or body twist, $\Delta t$ | Updated transform |
| Planar conversion | Planar pose or validated transform | Embedded transform or extracted planar pose |
| Planar yaw quaternion | Finite yaw angle | Normalized `QuaternionXYZW` |

Failure shall be represented by a typed result or documented exception that contains no geometric value. The exact C++ carrier is deferred to the implementation companion, but the observable error categories shall distinguish at least `invalid_shape`, `non_finite`, `invalid_rotation`, `invalid_transform`, `invalid_tangent`, `frame_mismatch` when frames are runtime-labelled, `non_planar`, `invalid_time`, `invalid_policy`, and `unsupported_magnitude`.

The principal logarithm success classification shall distinguish `identity`, `pure_translation`, `small_angle`, `nominal`, and `near_pi`. `identity` requires the selected zero rotation and translation satisfying the fixed translation-zero criterion; `pure_translation` has selected zero rotation and a nonzero preserved translation. The classification shall not replace a nonzero translation with zero.

## Numerical policy and valid domain

The first implementation shall use IEEE 754 binary64 arithmetic and the following public defaults:

| Quantity | Default | Role |
|---|---:|---|
| $\varepsilon_{\mathrm{orth}}$ | $10^{-12}$ | Dimensionless $SO(3)$ orthogonality acceptance |
| $\varepsilon_{\det}$ | $10^{-12}$ | Dimensionless proper-determinant acceptance |
| $\varepsilon_{\mathrm{row}}$ | $10^{-12}$ | Dimensionless homogeneous-row acceptance |
| $\varepsilon_{\mathrm{tangent}}$ | $10^{-12}$ | Dimensionless structural/skew acceptance |
| $\theta_{\mathrm{series}}$ | $10^{-4}\,\mathrm{rad}$ | Inclusive upper bound for stable small-angle series |
| $\varepsilon_\pi$ | $10^{-6}\,\mathrm{rad}$ | Inclusive near-$\pi$ formula-selection band |
| $\varepsilon_{\mathrm{zero}}$ | $0\,\mathrm{rad}$ | Preserve every representable nonzero rotation |
| $\varepsilon_{p,0}$ | $0\,\mathrm m$ | Preserve every representable nonzero translation |
| $\varepsilon_{R,\mathrm{planar}}$ | $10^{-12}$ | Dimensionless planar-orientation acceptance |
| $\varepsilon_{p,\mathrm{planar}}$ | $10^{-12}\,\mathrm m$ | Planar vertical-position acceptance |
| $\theta_{\max}$ | $10^6\,\mathrm{rad}$ | Maximum exponential angular displacement after twist-time multiplication |

Every policy value shall be finite and nonnegative, with $0<\theta_{\mathrm{series}}<\pi-\varepsilon_\pi$, $0\leq\varepsilon_\pi<\pi$, and $\theta_{\max}\geq\pi$. Invalid policy is rejected before geometric data is processed. Formula thresholds select stable evaluations and do not authorize a change in represented motion.

The values $\varepsilon_{\mathrm{zero}}=0$ and $\varepsilon_{p,0}=0$ are fixed for this minimum capability rather than configurable defaults; the implementation shall reject a policy that changes either value. This preserves every resolvable nonzero rotation and translation and keeps `identity` reserved for an input selected as the complete identity.

All operation inputs shall have the declared shape, contain finite components, respect their type invariants, and use compatible frames and units. Norms shall be calculated with scaling or an equivalently overflow-safe routine. Every input angle evaluated by a trigonometric function, including planar yaw, shall satisfy $|\theta|\leq\theta_{\max}$; exponential and integration inputs require $\|\boldsymbol\phi\|_2\leq\theta_{\max}$ after any twist-time product. A result is outside the supported domain if a required intermediate or output is non-finite; it shall fail as `unsupported_magnitude` rather than clamp, wrap, or return a partial result. In particular, a helical angular displacement shall not be reduced modulo $2\pi$ because its axial translation does not repeat.

After a rotation has passed validation, $c_\theta$ may be clamped to $[-1,1]$ only to correct floating-point overshoot no larger than the declared validation tolerance. A material overshoot is a failure. Likewise, $\max(0,Q_{kk})$ may remove only a post-validation negative round-off value within tolerance; it shall not conceal an invalid symmetric axis matrix.

Moderate analytic acceptance fixtures below use absolute tolerances $10^{-12}$ for dimensionless components, $10^{-12}\,\mathrm m$ for positions, and $10^{-12}\,\mathrm{rad}$ for angular coordinates. Principal exponential/logarithm round trips use separate residual limits $e_{R,\mathrm{rt}}\leq10^{-10}$ and $e_{p,\mathrm{rt}}\leq10^{-10}\,\mathrm m$. Relative tolerance may supplement but shall not replace these absolute limits near zero.

## Requirements

| ID | Requirement |
|---|---|
| SGK-REP-001 | Public operations shall use column vectors, rightmost-first products, the declared $\mathbf T_{ab}$ frame meaning, SI units, right-handed positive rotation, and linear-first six-columns. |
| SGK-REP-002 | Vector rotation and point transformation shall be distinct operations; translation shall never be applied to a free vector. |
| SGK-VAL-001 | Constructors and operations shall reject non-finite, malformed, non-orthogonal, reflective, structurally invalid, frame-incompatible, or unsupported-magnitude inputs without returning a valid-looking result. |
| SGK-VAL-002 | Validation shall not silently project or orthogonalize an invalid rotation or flatten an inapplicable transform to a plane. |
| SGK-GRP-001 | Composition and inversion shall implement the reviewed $SO(3)$ and $SE(3)$ group equations in the declared frame order. |
| SGK-LIE-001 | Hat, vee, and adjoint operations shall implement the reviewed linear-first conventions and validate the domain of every inverse map. |
| SGK-EXP-001 | $SO(3)$ and $SE(3)$ exponentials shall implement Rodrigues' and left-Jacobian formulas with stable zero and small-angle evaluation. |
| SGK-LOG-001 | Logarithms shall return the declared principal branch, preserve finite linear coordinates, and apply the deterministic identity, near-$\pi$, tie, and axis-sign rules. |
| SGK-LOG-002 | Logarithm formula thresholds shall not erase a representable nonzero motion; normalized screw factorization shall not occur implicitly. |
| SGK-INT-001 | Constant space twists shall update by left multiplication and constant body twists by right multiplication; zero time shall preserve the pose and negative time shall fail. |
| SGK-PLN-001 | Planar embedding shall preserve the $SE(2)$ group action, while extraction shall accept only validated transforms satisfying separate orientation and position residual limits. |
| SGK-QUA-001 | Planar yaw conversion shall return a finite normalized quaternion in ROS 2 `xyzw` field order and tests shall recognize sign-equivalent quaternions. |
| SGK-API-001 | Operations shall be deterministic, stateless, non-mutating, and independent of ROS runtime state. |
| SGK-API-002 | Failure categories, numerical policy, branch classifications, frames, units, and valid domains shall be public and testable. |

## Library-independent acceptance cases

The following expected values come from analytic geometry or algebra, not from Eigen, `tf2`, or another implementation of the same API.

| ID | Case | Independent expected result |
|---|---|---|
| SGK-ACC-001 | Exponentiate $\boldsymbol\phi_x=[\pi/2\;0\;0]^{\mathsf T}$ and $\boldsymbol\phi_z=[0\;0\;\pi/2]^{\mathsf T}$. | $\mathbf R_x=\begin{bmatrix}1&0&0\\0&0&-1\\0&1&0\end{bmatrix}$ and $\mathbf R_z=\begin{bmatrix}0&-1&0\\1&0&0\\0&0&1\end{bmatrix}$. |
| SGK-ACC-002 | Compose the two rotations in both orders. | $\mathbf R_z\mathbf R_x=\begin{bmatrix}0&0&1\\1&0&0\\0&1&0\end{bmatrix}$, while $\mathbf R_x\mathbf R_z=\begin{bmatrix}0&-1&0\\0&0&-1\\1&0&0\end{bmatrix}$; the results are unequal. |
| SGK-ACC-003 | Use $\mathbf T=[\mathbf R_z,[1\;2\;3]^{\mathsf T};\mathbf 0^{\mathsf T},1]$ on input column $[1\;0\;0]^{\mathsf T}$. | Free-vector rotation returns $[0\;1\;0]^{\mathsf T}$; point transformation returns $[1\;3\;3]^{\mathsf T}\,\mathrm m$. |
| SGK-ACC-004 | Compose a finite transform with its inverse on both sides. | Both products agree with $\mathbf I_4$ within the declared dimensionless and metre-valued component tolerances. |
| SGK-ACC-005 | Apply hat then vee to $\boldsymbol\eta=[1\;-2\;3\;0.1\;-0.2\;0.3]^{\mathsf T}$, with the first block in metres and second in radians. | The original linear-first column is recovered; the hatted top-right block is $[1\;-2\;3]^{\mathsf T}$ and the rotational block is skew-symmetric. |
| SGK-ACC-006 | Let $\mathbf T=[\mathbf I_3,[1\;2\;3]^{\mathsf T};\mathbf 0^{\mathsf T},1]$ and $\boldsymbol\xi=[4\;5\;6\;1\;0\;-1]^{\mathsf T}$ in linear-first order. | $\operatorname{Ad}_{\mathbf T}\boldsymbol\xi=[2\;9\;4\;1\;0\;-1]^{\mathsf T}$ and its hat equals $\mathbf T\widehat{\boldsymbol\xi}\mathbf T^{-1}$. |
| SGK-ACC-007 | Exponentiate and then take the principal logarithm of $\boldsymbol\rho=[0.3\;-0.2\;0.1]^{\mathsf T}\,\mathrm m$ and $\boldsymbol\phi=[0\;0\;10^{-8}]^{\mathsf T}\,\mathrm{rad}$. | The case is `small_angle`, the rotation is not erased, and the translation is $[0.300000001\;-0.1999999985\;0.1]^{\mathsf T}\,\mathrm m$ to the declared tolerance; the coordinates round-trip within the branch-qualified limits. |
| SGK-ACC-008 | Take the logarithm of $\mathbf R=\begin{bmatrix}0&1&0\\1&0&0\\0&0&-1\end{bmatrix}$. | The case is `near_pi`; the lowest-index tie rule returns $\boldsymbol\phi=\frac{\pi}{\sqrt2}[1\;1\;0]^{\mathsf T}$ and repeated calls return the same sign. |
| SGK-ACC-009 | Use normalized screw $\mathbf s_v=[0\;-2\;0.05]^{\mathsf T}\,\mathrm{m\,rad^{-1}}$, $\mathbf s_\omega=[0\;0\;1]^{\mathsf T}$, and $\theta=\pi/2$. | Exponentiating $\boldsymbol\eta=[\mathbf s_v\theta;\mathbf s_\omega\theta]$ returns $\mathbf R_z(\pi/2)$ and $\mathbf p=[2\;-2\;\pi/40]^{\mathsf T}\,\mathrm m$; the principal logarithm reconstructs the same finite coordinates. |
| SGK-ACC-010 | Exponentiate pure translation $\boldsymbol\rho=[0.4\;-0.2\;0.1]^{\mathsf T}\,\mathrm m$, $\boldsymbol\phi=\mathbf0_3$. | Rotation is exactly $\mathbf I_3$, translation equals $\boldsymbol\rho$, and the logarithm classification is `pure_translation`. The complete identity instead returns the zero coordinate and `identity`. |
| SGK-ACC-011 | Start with $\mathbf T_0=[\mathbf I_3,[1\;0\;0]^{\mathsf T};\mathbf0^{\mathsf T},1]$, body twist $\boldsymbol\xi_b=[0\;1\;0\;0\;0\;1]^{\mathsf T}$, space twist $\boldsymbol\xi_s=\operatorname{Ad}_{\mathbf T_0}\boldsymbol\xi_b=[0\;0\;0\;0\;0\;1]^{\mathsf T}$, and $\Delta t=\pi/2\,\mathrm s$. | Left space integration and right body integration both return $[\mathbf R_z(\pi/2),[0\;1\;0]^{\mathsf T};\mathbf0^{\mathsf T},1]$. With $\Delta t=0$, both return $\mathbf T_0$ unchanged. |
| SGK-ACC-012 | Embed $[2\;-1\;\pi/2]^{\mathsf T}$ and extract it. | The transform is $\begin{bmatrix}0&-1&0&2\\1&0&0&-1\\0&0&1&0\\0&0&0&1\end{bmatrix}$ and extraction returns $[2\;-1\;\pi/2]^{\mathsf T}$ within separate unit-aware tolerances. |
| SGK-ACC-013 | Convert yaw $\pi/2$ and yaw $\pi/2+2\pi$ to ROS-order quaternions. | The first is $[0\;0\;\sqrt2/2\;\sqrt2/2]^{\mathsf T}$, the second is its negative, both have unit norm, and both induce the same rotation. |
| SGK-ACC-014 | Round-trip representative small, nominal, and near-$\pi$ transforms through $\exp(\log_{\mathrm{selected}}(\mathbf T))$. | Each reconstructed transform satisfies $e_{R,\mathrm{rt}}\leq10^{-10}$ and $e_{p,\mathrm{rt}}\leq10^{-10}\,\mathrm m$. The reverse $\operatorname{vee}(\log_{\mathrm{selected}}(\exp(\widehat{\boldsymbol\eta})))=\boldsymbol\eta$ is required only for principal inputs with $\|\boldsymbol\phi\|_2<\pi$; at $\pi$, compare the canonical result or reconstructed rotation. |
| SGK-ACC-015 | Test $\theta_{\mathrm{series}}$ and $\pi-\varepsilon_\pi$ immediately below, exactly on, and immediately above each boundary using fixed `nextafter` values. | Inclusive comparisons select the declared branch deterministically, adjacent formula branches reconstruct the same transform within round-trip limits, and no value is silently canonicalized to zero. |
| SGK-ACC-016 | Supply a NaN or infinity; a wrong shape; $\operatorname{diag}(1,1,-1)$; a materially non-orthogonal matrix; a malformed homogeneous row; a non-skew vee input; negative $\Delta t$; or an angular increment greater than $\theta_{\max}$. | The documented failure category is returned and no geometric value is present. The reflection fails even though its orthogonality residual is zero. |
| SGK-ACC-017 | Attempt planar extraction after adding either $10^{-6}\,\mathrm m$ of vertical translation or a $10^{-6}\,\mathrm{rad}$ roll to an otherwise planar pose. | With default tolerances, both inputs fail as `non_planar`; neither is flattened or projected. |

Threshold tests shall use fixed inputs and a documented binary64 environment. Optional randomized stress tests shall use a fixed seed and bounded domain and shall supplement, not replace, these analytic cases.

## Exclusions

This capability does not include:

- ROS nodes, messages, publishers, subscribers, parameters, launch files, or a replacement for `tf2`;
- dynamic transform trees, timestamps, interpolation, extrapolation, buffering, or frame-graph lookup;
- arbitrary quaternion algebra, interpolation, Euler-angle conversion, or a general invalid-quaternion repair API;
- projection of nearby matrices onto $SO(3)$, $SE(3)$, or the planar subgroup;
- normalized screw extraction from a zero logarithm or implicit factorization of finite logarithm coordinates;
- time-varying twist integration, additive matrix integration, numerical ODE solvers, uncertainty, covariance, or automatic differentiation;
- forward kinematics, Jacobians, inverse kinematics, URDF parsing, collision geometry, dynamics, control, or trajectory generation; or
- single-precision guarantees, bitwise agreement across different platforms, or support for angular increments beyond the declared bound.

These concerns belong to later capability cycles or explicit interoperability layers and shall not expand the first spatial kernel.

## Traceability

| Requirement | Reviewed source | Planned verification |
|---|---|---|
| SGK-REP-001, SGK-REP-002, SGK-GRP-001 | Chapter 7 sections on frame-labelled $SO(3)$/$SE(3)$ actions, composition, and inversion | SGK-ACC-001 through SGK-ACC-004 |
| SGK-LIE-001 | Chapter 7 spatial tangent, twist, and adjoint derivations | SGK-ACC-005 and SGK-ACC-006 |
| SGK-EXP-001, SGK-LOG-001, SGK-LOG-002 | Chapter 8 sections on exponential coordinates, principal logarithms, small angles, and near-$\pi$ branches | SGK-ACC-007 through SGK-ACC-010 and SGK-ACC-014 through SGK-ACC-015 |
| SGK-INT-001 | Chapter 8 constant space- and body-twist integration | SGK-ACC-011 |
| SGK-PLN-001 | Chapter 7 planar embedding and extraction | SGK-ACC-012 and SGK-ACC-017 |
| SGK-QUA-001 | Chapter 7 planar yaw and ROS 2 component order | SGK-ACC-013 |
| SGK-VAL-001, SGK-VAL-002, SGK-API-001, SGK-API-002 | Chapter 7 representation checks, Chapter 8 numerical validation, and project reproducibility rules | SGK-ACC-014 through SGK-ACC-017 plus public-interface review |

## Gate to implementation

Implementation may begin after the user reviews and approves this minimum specification. At that point, create and review the first substantive block of `notes/09_spatial_geometry_kernel_implementation.qmd` alongside the first manageable C++ package block; do not treat an empty placeholder as implementation evidence.
