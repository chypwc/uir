---
name: python-quality-gate
description: Run a focused Python package and interface audit only at a CHECKLIST-declared stability checkpoint, before closing a Python capability, when the user explicitly requests the gate, or when a difficult defect requires systematic diagnosis. Do not use for teaching, terminology explanations, implementation-note drafting, routine learner-authored blocks, or ordinary focused test runs.
---

# Python Quality Gate

Apply a focused quality gate using the repository's configured environment and contracts. Keep scientific and robotics semantics visible rather than relying only on lint success.

## Invocation boundary

Invoke this gate only at a stability checkpoint named in `CHECKLIST.md`, before closing a Python capability, when the user explicitly requests a Python quality gate, or when a difficult defect requires a systematic audit. Routine implementation cycles use the focused testing cadence in `AGENTS.md` without invoking this skill. Do not invoke the gate merely because a conversation explains Python, reviews one learner-authored block, drafts an implementation companion, or runs an ordinary focused test.

## Establish the contract

1. Read the applicable `AGENTS.md`, then `PLAN.md` and `CHECKLIST.md` when present.
2. Identify the active capability, owning specification, implementation companion, package metadata, public API, modules, and tests.
3. Determine whether the user requested review, diagnosis, or implementation. A review does not authorise fixes.
4. Activate the repository virtual environment and inspect existing `pyproject.toml`, setup metadata, lint, type-check, and test configuration before selecting commands.

## Review the design

- Trace algorithms to documented equations or requirements.
- Check single-purpose modules, clear names, type annotations, frames, units, dimensions, assumptions, valid domains, and failure behaviour.
- Keep configuration separate from logic and avoid hidden mutable state.
- Prefer immutable result values where the contract promises value semantics. Make validation ownership explicit when public containers may represent invalid data before an operation consumes them.
- Check public exports, import direction, dependency boundaries, exception types, input non-mutation, and deterministic behaviour.
- Keep pure research or mathematical cores independent of ROS and file I/O where practical; use narrow adapters for messages, storage, and simulator integration.
- Reject speculative dependencies or abstractions that do not satisfy a stated requirement.

## Review numerical and data behaviour

- Check non-finite inputs, intermediate overflow, non-finite outputs, empty inputs, shape and dtype mismatches, tolerance policy, randomness, seeds, and reproducible configuration.
- Check units, frames, sign conventions, timestamps, ordering, and identity fields at interfaces.
- Keep training, evaluation, and held-out data separate. Prevent accidental state leakage between tests or experiments.
- Invoke `$scientific-numerics-review` as well when mathematical or floating-point correctness is material.

## Run configured checks

1. Use the repository virtual environment and the smallest affected package or test path.
2. Run configured formatting and lint checks, such as Ruff, only when the repository defines or already uses them. Do not introduce a formatter as an incidental change.
3. Run the configured type checker when present. Treat suppressions as scoped decisions, not a substitute for understanding the type boundary.
4. Run focused `pytest` cases, then the affected package suite. Build or install the package when its packaging contract changed.
5. Use property-based tests only for valuable algebraic, state-machine, parser, or numerical invariants; do not turn every example into generated testing.
6. If files changed, run `git diff --check`, inspect the affected diff and source regions, and repeat the relevant checks.

## Judge the tests

Prioritise public behaviour, requirements, equations, invariants, edge cases, invalid inputs, state non-mutation, integration boundaries, and regressions. Require deterministic tests unless randomness is the behaviour under test, in which case control and report the seed. Avoid testing trivial storage or unchanged Python/library behaviour.

## Report the gate

Lead with pass or fail. List findings by severity with exact file references and consequences. Separate implementation defects, contract ambiguity, missing evidence, and optional improvements. Record commands and results, including skipped configured checks. Do not call the gate complete when a required test or configured checker failed.
