# Finite-MDP implementation review

Date: 2026-09-05.

## Outcome and scope

The implemented positive-support distribution, controlled marginal and expected-reward equations, automatic terminal completion, canonical row lookup, and declared-terminal membership agree with the finite-MDP model at ordinary numerical scales. The package build and existing tests pass. Full-domain numerical acceptance is not established: an accepted distribution with extreme finite rewards can return infinity without reporting a failure.

This review supports the retrospective [implementation companion](../../notes/part_05_optimisation_planning_decisions/finite_mdp_implementation.qmd); it does not close `IF.4`, Cycle 1. Policy application, dense compute conversion, and the remaining fixture work are not certified here. No production source, test, or build file was changed by this review.

## Contract and evidence

The owning [specification](../05_optimisation_planning_decisions/finite_mdp_model.md) defines positive-support records with finite rewards and positive finite probabilities, sum tolerance at most `1.0e-12`, controlled marginalisation (IF-MDP-CAL-001), expected reward (IF-MDP-CAL-002), generated terminal bookkeeping (IF-MDP-MOD-003), and declared-terminal membership (IF-MDP-ACC-016).

- State and action values are distinct zero-based index types. Returned next-state vectors follow the declared state order; their row orientation is mathematical, not encoded by `std::vector`.
- Probabilities and their tolerance are dimensionless. Reward is a generic scalar task evaluation with no imposed physical unit or coordinate frame.
- For each feasible pair, marginalisation sums probabilities with the same destination; expected reward sums each reward times its joint probability. The independent three-outcome example gives `[0.2, 0.8, 0]` and `3.5`.
- One terminal-only action is generated at the task-action count. Incoming rewards remain unchanged; each generated terminal row has probability one, self-transition, and zero reward. Sorted row keys support binary lookup followed by an exact-match check. Terminal membership uses declarations, not self-loop inference.

Reproduction commands, run from the repository's `ros_ws` directory:

```zsh
source /opt/ros/jazzy/setup.zsh
source ../.venv/bin/activate
colcon build --packages-select intelligence_foundations --symlink-install
colcon test --packages-select intelligence_foundations
colcon test-result --test-result-base build/intelligence_foundations --verbose
```

Observed result: **80 reported checks, zero errors, zero failures, 11 skipped**. The three GoogleTest executables contain 45 unit cases: 2 index cases, 12 distribution cases, and 31 finite-MDP cases. Formatting passed. Ament skipped cppcheck checks because the installed cppcheck 2.13.0 version triggered its compatibility warning; those checks were not executed evidence.

## Numerical-domain defect: expected reward can overflow

The [distribution factory](../../ros_ws/src/intelligence_foundations/src/joint_outcome_distribution.cpp) accepts probabilities within the sum tolerance without renormalising them. Its `expected_reward()` query uses compensated `double` accumulation but neither checks its result for finiteness nor reports numeric failure. The public declaration is `noexcept`.

The following complete program reproduces the defect with the installed implementation on the review host:

```cpp
#include <iomanip>
#include <iostream>
#include <limits>

#include "intelligence_foundations/joint_outcome_distribution.hpp"

int main()
{
  using intelligence_foundations::JointOutcomeDistribution;
  using intelligence_foundations::StateIndex;

  const double maximum = std::numeric_limits<double>::max();
  const auto distribution = JointOutcomeDistribution::from_outcomes(
    2U,
    {{StateIndex{0}, maximum, 0.5},
     {StateIndex{1}, maximum, 0.5000000000005}},
    1.0e-12);

  std::cout << std::setprecision(17)
            << "sum=" << 0.5 + 0.5000000000005 << '\n'
            << "expected_reward=" << distribution.expected_reward() << '\n';
}
```

After saving the reproducer as `/tmp/finite_mdp_reward_probe.cpp`, compile from the repository root against the built library:

```zsh
g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic \
  -I ros_ws/src/intelligence_foundations/include \
  /tmp/finite_mdp_reward_probe.cpp \
  ros_ws/build/intelligence_foundations/libintelligence_foundations.a \
  -o /tmp/finite_mdp_reward_probe
/tmp/finite_mdp_reward_probe
```

Observed values:

```text
sum=1.0000000000005
expected_reward=inf
```

Both records have valid distinct destinations, finite rewards, and individually valid probabilities. The sum excess is below the accepted tolerance. Their exact weighted total nevertheless exceeds the largest finite `double`. Kahan compensation does not increase the floating-point range.

This is a numerical-domain defect relative to the expected-reward calculation and explicit-failure intent, and a gap in the specification's numeric-range policy. Existing normal-case tests do not establish safety over all accepted finite inputs. Before claiming that domain, choose and specify a bounded reward domain or explicit overflow failure behaviour, then implement and verify the chosen contract. Silent renormalisation or clamping would conflict with the current specification and must not be introduced as an incidental fix.

## Independent compensation diagnostic

Under binary64 rounding to nearest, ties to even, the sequence `1, 2^-53, 2^-53` has exactly representable total `1 + 2^-52`. A separate compiled diagnostic produced ordinary sum `1` and compensated sum `1.0000000000000002`, with successive corrections `0`, `-2^-53`, and `0`. This verifies the sign and update order illustrated in the companion; it is not a claim that Kahan makes every sum exact.

A production-distribution probe using probabilities `0.5, 2^-54, 2^-54, 0.5 - 2^-53`, distinct rewards at one destination, and sum tolerance `1.0e-18` was accepted and returned marginal `1`. These were read-only diagnostic experiments, not new learner-owned tests.

## Companion verification

The companion is registered after the finite-MDP theory chapter in Part V. Its prerequisite and implementation cross-reference targets were checked against their owning chapters. Whitespace checks passed for the edited tracked files and the new note and report.

Quarto 1.9.38 rendered the chapter to HTML successfully. The book-format PDF render also completed successfully with `--no-execute`, so this verifies document assembly rather than rerunning other chapters' numerical experiments. Both renders used the activated repository virtual environment and a temporary output directory. The Kahan update and trace pages were visually inspected: equations, the trace table, and code fit the page width. Temporary render and diagnostic products were removed; the source note and this report are the retained artifacts.

Learner review of the companion remains the active task. No glossary or notation-registry completion, numerical-domain fix, or capability closure is implied.
