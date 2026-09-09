# C++ style and patterns

Use this guide for project-owned C++20 code and teaching examples. Apply it to new or meaningfully changed code; do not rewrite working code solely for style. Production excerpts in notes must remain faithful to their source. These are agreed conventions, not a claim that every feature has already been learned.

## Initialization

Prefer braces for direct initialization. Use parentheses when needed to select the intended constructor; keep `=` initialization for function results.

```cpp
const Eigen::Vector3d position_metres{2.0, 0.0, 0.0};
const double joint_angle_radians{0.25};
std::vector<double> coordinates(3, 0.0);  // Three zeros, not {3.0, 0.0}.

const auto model =
  SerialChainModel::from_home_and_joints(home_pose, joints);
```

Braces reject narrowing conversions but can select an initializer-list constructor. Do not mechanically replace parentheses with braces. See [C++ Core Guidelines ES.23](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es23-prefer-the--initializer-syntax).

## Constness, types, and Eigen

Make local values `const` when they do not change. Use `auto` when the result type is clear, such as a named factory returning a model. Prefer explicit types when dimensions, ownership, or numerical meaning matter.

```cpp
const Vector6LinearFirst body_screw_axis =
  home_inverse_adjoint * space_screw_axis(joint);

const Eigen::Vector3d origin_metres = Eigen::Vector3d::Zero();
```

For a stored Eigen calculation result, use an owning matrix or vector type rather than accidentally retaining a lazy expression through `auto`. Such expressions can borrow their operands. Do not assume `{}` zero-initializes an Eigen object; request `Zero()` or `Identity()` when that is the intended value. See [Eigen's common pitfalls](https://libeigen.gitlab.io/eigen/docs-nightly/TopicPitfalls.html).

## Names and mathematical meaning

| Item | Convention | Example |
| --- | --- | --- |
| Types | `PascalCase` | `SerialChainModel` |
| Functions and variables | `snake_case` | `body_screw_axis` |
| Private members | Trailing `_` | `home_pose_` |
| Named compile-time constants | `kPascalCase` | `kRotationToleranceRadians` |
| Physical quantities | Include units where useful | `elapsed_seconds` |
| Frame-dependent quantities | Identify frames where needed | `space_axis_direction` |

Document dimensions, units, frame direction, component order, and multiplication conventions alongside public interfaces. A name alone does not explain the meaning of a six-entry screw column.

## Ownership and borrowing

- Return newly calculated results by value; pass small scalars by value.
- Borrow larger read-only inputs through `const T &` or an appropriate Eigen view. State lifetime requirements when returning borrowed data.
- Prefer owned values. Use smart pointers when needed, with shared ownership only for a clear reason.
- Never return references to local objects or temporary results.
- Use `std::move` for an actual ownership purpose, not as a general performance decoration; do not move from a local return value merely to try to accelerate its return.

## Interfaces, failures, and design

- Use validated domain types such as `Transform3`, typed joint limits, and scoped error enums. Keep functions focused on one operation.
- Preserve the specified validation order and typed failures, with useful joint or operation context. Do not silently clamp, repair, or substitute inputs unless the contract requires it.
- Add `[[nodiscard]]` when ignoring a result is likely a mistake; add `noexcept` only when the operation truly cannot throw.
- Keep mathematical cores independent of ROS; use narrow adapters for messages and visualization.

Choose design patterns only to solve a stated problem. Prefer a function, value type, or small class when sufficient. Validated factories, owned model values, variant-based joint handling, and ROS adapters already serve concrete needs; do not add inheritance, caching, or interchangeable interfaces solely to use a named pattern.

## Numerical code, comments, and tests

- Prefer direct loops and named intermediate values when they clarify the governing equations. Avoid abstractions introduced only to shorten a few lines.
- Preserve essential numerical checks, including finiteness after operations that can overflow. Keep numerical thresholds separate from physical limits.
- Test independent expected results, governing properties, boundaries, and distinct failure modes. Reuse checks across equivalent interfaces without duplicating whole tests.
- Keep tolerances for different physical quantities separate, such as translation in metres and rotation in radians.
- Comments should explain mathematical purpose, frame conventions, or a non-obvious reason, not narrate syntax. Explain unfamiliar syntax in teaching material when needed.
