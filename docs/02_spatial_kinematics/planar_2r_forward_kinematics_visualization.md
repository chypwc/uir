# Planar 2R Forward-Kinematics Visualization Specification

Draft for review. This is the small RViz consumer of the [space-form kinematics contract](general_kinematics_and_jacobians.md), not an extension of the mathematical core. The [Chapter 10 model](../../notes/part_01_motion_mechanics_control/10_general_robot_kinematics_and_jacobians.qmd#sec-serial-product-of-exponentials) and [Chapter 11 implementation](../../notes/part_01_motion_mechanics_control/11_general_robot_kinematics_and_jacobians_implementation.qmd#sec-general-kinematics-impl-space-forward-kinematics) own the pose calculation.

## Fixed scenario and inputs

Use two revolute joints with link lengths $L_1=2\,\mathrm m$ and $L_2=1\,\mathrm m$. At zero, both links point along positive $x_s$, both positive rotation axes point along positive $z_s$, joint 1 passes through $(0,0,0)$, and joint 2 passes through $(2,0,0)\,\mathrm m$. Both joint intervals are $[-\pi,\pi]$ radians.

For this scenario only, the space frame $\{s\}$ is identified with RViz's fixed frame `world`; every displayed position is measured from its origin and expressed along its axes. All lengths are in metres and rotations follow the right-hand rule. No frame transform is required.

Construct two immutable `SerialChainModel` values through the installed public API:

| Model | Ordered joints | Home orientation | Home position in `world` | Query |
|---|---|---|---|---|
| Elbow endpoint | Joint 1 only | Identity | $(2,0,0)\,\mathrm m$ | $(q_1)$ |
| End effector | Joint 1, joint 2 | Identity | $(3,0,0)\,\mathrm m$ | $(q_1,q_2)$ |

Call `space_form_forward_kinematics` on both models with the default numerical policy. The first model returns the elbow position, which is the origin of joint 2; joint 1's origin remains the fixed base. The second returns the complete end-effector pose. Do not duplicate planar trigonometric forward kinematics or add intermediate-link output to the core API.

The input is a dimensionless scalar $u\in[0,1]$, defining

$$
\mathbf q(u)=u\begin{bmatrix}\pi/2\\-\pi/2\end{bmatrix}\ \mathrm{rad}.
$$

First implement the static case $u=1$. The final demo also supports a startup Boolean parameter `animate`: false holds $u=1$; true moves $u$ linearly from 0 to 1 over four seconds and back over four seconds, repeating. Publish at a target rate of 20 Hz. Use elapsed monotonic time for animation rather than callback count; use the ROS clock for message stamps and keep `use_sim_time=false`. These rates are display settings, not physical dynamics or real-time guarantees.

## Marker output

The C++20 `ament_cmake` package is `rigid_body_kinematics_visualization`; its executable and default node name are `planar_2r_forward_kinematics_demo`. It consumes `rclcpp`, `visualization_msgs`, and the installed `rigid_body_kinematics` target, with any directly included message dependencies declared. It exports no reusable visualization library.

Publish one `visualization_msgs::msg::MarkerArray` on `~/markers`, resolving under the default node name to `/planar_2r_forward_kinematics_demo/markers`. Use reliable, volatile, keep-last-1 publication; repeated static publication also permits RViz to start after the node.

Use one marker namespace, `planar_2r_fk`, and stable identifiers:

| ID | Marker type | Content |
|---|---|---|
| 0 | `SPHERE_LIST` | Base, elbow, end-effector origin |
| 1 | `LINE_STRIP` | Base → elbow → end effector |
| 2–4 | `ARROW` | End-effector $x$, $y$, $z$ axes, coloured red, green, blue |
| 5 | `TEXT_VIEW_FACING` | Current joint angles in radians and end-effector position in metres |
| 6 | `LINE_STRIP` | Animated end-effector trail, at most 160 points; omit until two samples exist and omit in static mode |

For the returned end-effector pose $\mathbf T_{se}=[\mathbf R,\mathbf p;\mathbf0^{\mathsf T},1]$, $\mathbf R$ is its orientation and $\mathbf p$ its origin position in `world`. Draw each axis from $\mathbf p$ to $\mathbf p+\ell\mathbf R\mathbf e_i$, where $\ell=0.3\,\mathrm m$ and $\mathbf e_i$ is the corresponding three-entry Cartesian unit column. Axis endpoints use the returned rotation matrix, not an independently reconstructed orientation.

All markers in an update shall have frame `world` and one shared timestamp. Point-based markers use identity marker poses so already transformed points are not transformed twice; place the text near the returned end-effector position with an identity orientation. Use visible positive scales and nonzero opacity. Replace existing markers through the same namespace/ID pairs; do not grow the number of markers. Set marker lifetimes to 0.5 seconds and keep at most seven markers and 160 trail points per update. These fields follow the [ROS 2 Jazzy Marker definition](https://github.com/ros2/common_interfaces/blob/jazzy/visualization_msgs/msg/Marker.msg).

## Failure behaviour and exclusions

Reject non-finite $u$ or values outside $[0,1]$ before evaluation. Construct both poses and the complete marker update before publishing; if construction, evaluation, or conversion fails, log the reason and terminate with a nonzero exit status. Do not publish a partial arm, substitute an identity/home pose, or append a failed sample to the trail. Previously displayed markers expire by their finite lifetimes while the viewer clock advances.

No ROS dependencies enter `rigid_body_kinematics`. This demo adds no URDF, `tf2`, `robot_state_publisher`, Gazebo, dynamics, control, collision checking, or general robot visualization API. RViz is explanatory evidence; deterministic tests remain the evidence for numerical correctness.

## Essential acceptance checks

Check the same marker-building path used by the node without starting RViz. Compare marker endpoints both with the core outputs and these independent geometric values, using a position tolerance of $10^{-12}\,\mathrm m$:

| Input | Base | Elbow | End effector | End-effector orientation |
|---|---|---|---|---|
| $u=0$ | $(0,0,0)$ | $(2,0,0)$ | $(3,0,0)$ | Identity |
| $u=1$ | $(0,0,0)$ | $(0,2,0)$ | $(1,2,0)$ | Identity |

Positions in the table are in metres and expressed in `world`. Verify that the line endpoints match the displayed joint centres and that axis-arrow endpoints follow the returned pose. A bounded-history check shall establish that repeated updates keep the same IDs and at most 160 trail points. One rejected-input case shall establish that no successful marker update is produced for invalid $u$.

Provide a launch file and saved RViz configuration with fixed frame `world` and the marker topic above. Inspect the static arm first, then the repeating animation, readable pose text, axes, and bounded trail. Record a PNG for the companion; apply the existing checklist's optional GIF conditions. Do not call visual agreement a substitute for the headless checks or use this demo to waive outstanding core acceptance exceptions.

## Implementation order

After review: package and installed-core linkage → static model evaluation and base/joint/link markers → headless endpoint checks → animation, axes, text, and bounded trail → launch/RViz configuration and visual inspection. The learner writes each implementation block; no package source is created by this specification draft.
