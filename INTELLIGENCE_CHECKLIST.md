# Intelligence Foundations Track Checklist

This is the live progress tracker for the parallel intelligence-foundations track. [PLAN.md](PLAN.md) defines why the track exists, how its capabilities map to the main phases, and how its artifacts later converge with the robotics track. [CHECKLIST.md](CHECKLIST.md) remains the live tracker for the main robotics track.

## Working rules

- Follow the project-wide capability cycle: learn and write → review and quiz → specify → implement and verify → write the retrospective companion → close.
- Record actual focused hours in the shared [effort log](docs/effort_log.md), following the daily target and accounting rules in `PLAN.md`; review effort alongside accepted artifacts at capability closure.
- Write and review one small learning block at a time. Do not batch several unchecked mathematical blocks into one review.
- Write intelligence-foundation chapters directly in the final phase-aligned parts of the main book. The intelligence track initially owns those chapters; the owning main phase later reviews and extends the same files in place.
- `IF.1`–`IF.3` are retired as standalone prerequisite gates. Probability, dynamical-systems, system-identification, and optimisation material is written and reviewed only when the first active `IF.4+` capability needs it, in the material's final phase-aligned book part.
- From `IF.4` onward, implement one mathematical operation or algorithmic cycle at a time in modern C++20, use LibTorch for neural components, and keep Python limited to independent numerical references, plotting, dataset generation, and experiment orchestration.
- Keep the reusable cores independent of ROS, Gazebo, robot messages, any one fixture, and any one tensor encoding. Semantic adapters own state and action meaning; tensor adapters own feature order, scaling, batching, and device placement.
- Keep validated semantic models separate from numerical compute views. Scalar loops are acceptable for validation, sparse assembly, and independent small-case references; repeated numerical kernels shall use declared dense, sparse, or tensor layouts and vectorised or batched operations when their mathematical dependencies permit. Preserve genuinely sequential update order, and require deterministic numerical equivalence plus measured benefit before adding threaded or GPU execution.
- Use only versioned project-generated training data for mandatory learned artifacts. Do not use pretrained language models, downloaded policies, external foundation-model APIs, or opaque end-to-end learner or world-model libraries.
- Treat the finite MDP, continuous point-mass system, and synthetic text world as bounded verification fixtures, not robotics embodiments or claims of general language intelligence.
- Apply the Understand / Implement / Integrate commitments in `PLAN.md`. A verified educational learner does not acquire a separate robotics benchmark by default. Keep queued capabilities as artifact-level outlines; decompose and review only the cycle approaching activation.
- **Mathematically essential tests:** For each mathematical learning operation, use the smallest deterministic test set that verifies its governing equations, invariants, and essential validity conditions. Include representative normal cases, mathematically significant boundaries, and invalid inputs whose acceptance could invalidate the model or result. Prefer independent analytic expectations. Test shapes, masks, ordering, schedules, and numerical safeguards only where they affect mathematical meaning or correct computation.
- **Distinct evidence before more tests:** Before proposing a test, state the distinct mathematical property or failure mode it establishes and why existing tests do not already cover it. Reuse or extend an existing fixture when sufficient. Do not add input variations merely to exercise the same validation rule again, pursue exhaustive combinations or coverage counts, or test unchanged language/library behaviour and trivial accessors. A shared code path alone does not make tests redundant if they establish different mathematical properties. Keep separately required integration, checkpoint/reproducibility, and regression checks tied to explicit capability requirements rather than expanding each mathematical lesson into a general engineering test campaign.
- Before statistical experiments, freeze the question, contenders, data, independent training seeds, held-out evaluation, training and tuning budgets, compute limits, and stop conditions under the `PLAN.md` experiment contract.
- A well-supported negative result can close its experiment or capstone when the declared correctness and evidence criteria pass. It does not close unrelated capabilities or excuse broken implementation, insufficient evaluation, or an exhausted budget with no defensible conclusion.
- Track exactly one active task here. Make completed cycles available to consumers without waiting for the rest of their capability or milestone. If both frontiers need the same unfinished cycle, give that cycle one checklist owner and link to it from the other. Retire this checklist as merged only after all remaining independent work has been completed or transferred to its owning main phase.

## Track status

Milestone and capability identifiers group learning outcomes; they do not impose a total execution order. Select the next cycle from its actual prerequisites and the next consumer. Only the cycle in Current work is active.

| Milestone | Capabilities | Outcome | Status | Evidence |
|---|---|---|---|---|
| `IM0` — Exact sequential-decision foundation | `IF.4`, with `IF.1`–`IF.3` absorbed just in time | Reviewed finite-MDP, Bellman, exact-decision, fixture, and reference-implementation foundations | Active | — |
| `IM1` — Reinforcement-learning foundations | `IF.5`–`IF.8` | Verified tabular, deep-value, on-policy actor–critic, and continuous off-policy learner cores | Queued | — |
| `IM2` — Sequence and predictive models | `IF.9`–`IF.10` | Tiny autoregressive Transformer, learned transition models, imagined rollout, and explicit-model planning cores | Queued | — |
| `IM3` — Data-driven and generative decisions | `IF.11`–`IF.12` | Verified behavioural cloning, IQL, Decision Transformer, DDPM, score-model, and conditional sequence foundations | Queued | — |
| `IM4` — Synthetic intelligence engine | `IF.13`–`IF.14` | Grounded latent state, bounded memory, world-model imagination, learned reasoning control, and reproducible language-engine capstone | Queued | — |
| `IM5` — Track closure and convergence | `IF.15` | Candidate reusable cores, permanent synthetic regression consumer, and explicit main-track acceptance manifest | Queued | — |

## Current work

- **Milestone:** `IM0` — Exact sequential-decision foundation.
- **Capability:** `IF.4` — Finite MDPs, Bellman equations, and exact dynamic programming.
- **Cycle:** Cycle 1 — Finite Markov chains and the tabular MDP model.
- **Active stage:** Implement and verify.
- **Active task:** Add and verify focused tests for `FiniteMdp::is_terminal(StateIndex)` against IF-MDP-ACC-016 in the [finite-MDP specification](docs/05_optimisation_planning_decisions/finite_mdp_model.md), then review its syntax and declared-membership semantics. The query implementation is present; its focused tests remain unfinished.
- **Prerequisite policy:** Do not complete separate `IF.1`–`IF.3` surveys. When `IF.4+` first uses an undeclared mathematical result, pause that consumer, write and review only the required prerequisite block in its final book part, then resume the consumer.
- **Main-phase destination:** Phase 5 — Optimisation, planning, and decisions; the Markov-chain prerequisite block belongs in Phase 2's probability part.
- **Blockers:** None.

## Relationship to the main programme

The intelligence track advances just-in-time mathematical notes and simulator-neutral algorithm cores before the robotics track needs them. Completion here does not close a main robotics phase. The owning phase later supplies robot-specific semantic adapters, data, baselines, safety boundaries, integration, and acceptance evidence.

| Intelligence capability | Main `PLAN.md` destination | Early durable artifact | Later main-track responsibility |
|---|---|---|---|
| `IF.1`–`IF.3` — Retired standalone gates | First consuming phase, with content filed under Phases 1, 2, 5, 6, or 11 by subject | Small reviewed prerequisite blocks created only at first use | Extend the same blocks for robotics applications; never recreate the abandoned surveys as blockers |
| `IF.4` — MDPs and exact dynamic programming | Phase 5 | Exact finite-decision reference and environment contract | Bind mission and bounded belief-decision fixtures |
| `IF.5`–`IF.8` — RL foundations | Phase 6 | Installed learner, replay, checkpoint, and evaluation targets | Select one verified learner for the full mobile comparison and one non-planar conformance fixture; retain the others as bounded educational references |
| `IF.9` — Attention and Transformers | Phase 6 foundations; Phases 9B or 11B application | Installed attention and Transformer targets plus tiny language model | Add one selected structured robotics Transformer consumer |
| `IF.10` — World models and model-based RL | Phase 11A basics; Phase 11B selected extensions | Independently accepted transition-model, uncertainty, rollout, and CEM cycles | Bind the three-model manipulation comparison; add the geometry-aware physical residual in its first owning cycle; assess selected later models |
| `IF.11` — Imitation and offline RL | Phase 9A cloning; Phase 9B selected advanced learner | Independently accepted BC, IQL, and Decision Transformer cycles | Consume BC for the frozen manipulation dataset/reference; integrate IQL or Decision Transformer only if selected |
| `IF.12` — Diffusion and score models | Phase 6 foundations; Phases 9B or 11B application | Installed DDPM, score, conditioning, and sequence-generation targets | Establish one selected robotics application: Diffusion Policy or Diffuser |
| `IF.13`–`IF.14` — Synthetic intelligence engine | Phase 7, with later Phase 11 reuse | Permanent synthetic reference consumer and integration report | Accept the cores into the main programme; retain robot-specific models and adapters downstream |
| `IF.15` — Closure and convergence | Phases 6, 7, 9, and 11 | Versioned handoff and incompatibility manifest, extended at each actual handoff | Record unchanged reuse, extension, retraining, adapter work, or replacement; do not delay an earlier consumer until track closure |

### Cycle dependencies and early consumption

The rows below define the minimum routes, not extra active tasks. A first consumer may bring forward a small prerequisite cycle within a later-numbered capability. Record that cycle's owner, reviewed theory, specification, installed target, and acceptance evidence once; reuse them at every later consumer.

| Consumer | Required foundation | Work that does not block it |
|---|---|---|
| `IF.5` sampled tabular learning | Relevant `IF.4` model and exact-value cycles | Neural models and continuous fixtures |
| `IF.6` DQN | Accepted finite-MDP and sampled-value foundations; the shared LibTorch/toolchain and neural-update cycle | PPO, Transformers, diffusion |
| `IF.7` actor–critic/PPO or `IF.8` TD3/SAC | Reviewed value/gradient/probability results, accepted neural-update and required rollout/replay operations, and the first required fixture | Completion of the DQN learning campaign or other learner families; the three-learner comparison waits for all three |
| `IF.9` Transformer | Accepted neural/tensor operations, probability and attention prerequisites, tiny generated sequence data | Completion of the RL learner catalogue |
| `IF.10` basic transition/uncertainty/rollout/CEM cycles | Accepted neural regression and local dynamics/probability prerequisites; the point-mass oracle | Transformer, diffusion, Dyna integration, the synthetic capstone |
| `IF.11` BC cycle → Phase 9A | Accepted supervised neural update, trajectory/checkpoint/evaluator operations, and a frozen dataset | IQL, Decision Transformer, diffusion |
| `IF.11` IQL or Decision Transformer cycles | Accepted BC/data contracts; the required value-learning or attention cycles respectively | Completion of the other advanced learner |
| `IF.12` diffusion cycle | Accepted neural updates, noising/score prerequisites, and the fixture needed for the selected cycle | RL learners, IQL, Decision Transformer |
| `IF.13`–`IF.14` / Phase 7 | Accepted PPO, Transformer, basic predictive-model and memory cycles, and controlled text-world contracts | IQL, Decision Transformer, diffusion, robotics integration, Phase 11A |
| Phase 11A | Phase 8, Phase 9A, and accepted basic transition/uncertainty/rollout/CEM cycles; its own physical-residual specification | Unrelated Phase 6 algorithms, Phase 7, Phase 9B, Phase 10, advanced sequence models, `IF.15` closure |

The first neural consumer owns toolchain adoption and the small neural-update cycle; later consumers reuse the evidence. The first continuous consumer owns the point-mass fixture cycle, whether it arrives through PPO, TD3/SAC, cloning, or learned dynamics. Neither shared prerequisite is locked behind completion of its default capability number.

## Book and artifact ownership

- Organise the main book into phase-aligned thematic part folders such as `notes/part_02_probability_estimation`, `notes/part_05_optimisation_planning_decisions`, `notes/part_06_learning_foundations`, `notes/part_09_imitation_offline_rl`, and `notes/part_11_world_models`. A phase may own more than one part when its learning subjects require it, and an integration-only phase need not own a separate theoretical part.
- File every just-in-time prerequisite block in its final conceptual part, even when a later capability triggers it. The consuming chapter cites that block and briefly recalls the exact result it needs.
- Place the synthetic language-engine implementation and retrospective learning in one cross-phase `notes/intelligence_engine_laboratory` part of the same book.
- Keep one shared `notes/notation.qmd` and `notes/glossary.qmd`. Update them only after each intelligence chapter reaches the existing review gate.
- Keep theory and retrieval tests in the main book, requirements under `docs/`, project-owned production cores and deterministic tests under `ros_ws/src`, and interpreted capstone or merge conclusions under `docs/reports/`.
- Begin with two pure `ament_cmake` packages: `intelligence_foundations` for reusable C++20 and LibTorch cores, and `synthetic_intelligence_engine` for the three fixtures and language-engine consumer. The foundations package must not depend on the synthetic package. Later robot packages depend on installed foundation targets through their own adapters and never depend on the synthetic engine.
- Retain `synthetic_intelligence_engine` after convergence as a permanent reproducible regression consumer of the shared cores.

## Fixed fixture scope

### Finite MDP fixture

Use one tiny fully specified finite MDP with an exact transition and reward table. It supplies analytic or exactly enumerated values for dynamic programming, sampled tabular learning, DQN, termination, and truncation tests.

### Continuous point-mass fixture

Use one bounded planar point mass with state $\mathbf x_t=[p_x,p_y,v_x,v_y]^\mathsf T$ and continuous action $\mathbf u_t=[a_x,a_y]^\mathsf T$. Freeze discrete dynamics, time step, units, action and state limits, objectives, termination, and invalid-input behaviour. Its analytical transition is the independent oracle for TD3, SAC, PPO, imitation learning, offline RL, learned dynamics, imagined rollouts, random shooting, CEM, and conditional action-sequence diffusion.

### Synthetic text-world fixture

Use one procedurally generated hidden symbolic world containing bounded agents, objects, rooms, containers, relations, and deterministic actions such as move, take, place, open, and close. Expose only controlled natural-language observations and questions to the language model. Freeze schema versions, procedural seeds, controlled vocabulary and grammar, authoritative state transitions, train/validation/evaluation splits, longer-horizon splits, and unseen state-composition splits.

Keep external environment actions $a_t^{\mathrm{env}}$, internal reasoning actions $c_k^{\mathrm{reason}}$, and generated language tokens $x_j^{\mathrm{token}}$ as different types and contracts. Only an environment action changes authoritative world state.

## Package boundary and minimum reliability contract

- `intelligence_foundations` owns algorithm interfaces, update rules, neural modules, replay, trajectories, checkpoints, planning, bounded memory interfaces, and evaluation utilities; it does not own fixture semantics or robot schemas.
- `synthetic_intelligence_engine` owns fixture state and action meanings, controlled language, procedural datasets, semantic evaluators, experiment configuration, and adapters into foundation targets.
- Validate state and action schemas, tensor ranks and shapes, finiteness, bounds, device and scalar type, terminal and truncation semantics, dataset and checkpoint compatibility, step budgets, reasoning budgets, and invalid learned output.
- Provide a declared deterministic fallback for invalid output, timeout, stale or incompatible state, and unsupported action. These checks support trustworthy experiments but make no physical-safety or functional-safety claim.
- Freeze seeds, dataset versions, episode-level splits, configurations, checkpoint metadata, parameter count, training time, inference latency, peak memory, and interaction count wherever they affect a comparison. Use a small feasibility pilot to set numerical model, dataset, training, tuning, and evaluation budgets before a campaign; record actual focused work in the effort log separately from unattended compute.

## Retired standalone prerequisites — former `IF.1`–`IF.3`

These identifiers remain only to preserve roadmap history. They have no completion gate, status, package, or standalone survey chapter. Their topics form a just-in-time pool: probability and stochastic processes are filed in Part II, dynamics and system-identification results in Parts I or XI, and optimisation results in Parts V or VI according to the concept's final ownership.

- Former `IF.1` topics include probability spaces, conditioning, random variables, expectations, joint and sequential distributions, finite Markov chains, information measures, and sampling. Write only the block first required by an active consumer.
- Former `IF.2` topics include state, transition and observation models, discretisation, rollout error, system identification, excitation, and model mismatch. Reuse reviewed Phase 1 dynamics material before adding intelligence-specific content.
- Former `IF.3` topics include objectives, derivatives, gradient methods, automatic differentiation, constraints, likelihood objectives, and trust regions. Introduce each result immediately before the learner or planner that uses it.
- A consuming capability may not cite assumed knowledge that is absent from the declared prerequisites. It must either cite an already reviewed block or create and review the smallest missing block before proceeding.

## `IF.4` — Finite MDPs, Bellman equations, and exact dynamic programming

**Main-phase destination:** Phase 5.

### Cycle 1 — Finite Markov chains and the tabular MDP model

- [x] Route the block through Sutton and Barto, Chapter 3, and Bertsekas, Chapter 1, using the probability sources routed by `textbooks/INDEX.md` only for a missing prerequisite derivation.
- [x] Write and review the smallest Part II prerequisite block defining a finite stochastic state sequence, the first-order Markov property, time-homogeneous transition probabilities, a row-stochastic transition matrix, one-step distribution propagation, multi-step trajectory probability, and absorbing states. Do not expand it into the retired probability survey.
- [x] Write and review the consuming Part V block that adds actions, rewards, policies, terminal states, truncation, finite horizons, and discounting to form a finite MDP; distinguish the uncontrolled Markov chain from the policy-induced chain and the controlled MDP.
- [x] Specify the simulator-neutral environment result, finite-MDP table representation, state and action domains, transition and reward validation, policy representation, terminal versus truncation semantics, deterministic fixture, and invalid-input behaviour in the [finite-MDP model capability specification](docs/05_optimisation_planning_decisions/finite_mdp_model.md).
- Automatic terminal completion verification (2026-09-05): the package build and all 31 [finite-MDP tests](ros_ws/src/intelligence_foundations/test/test_finite_mdp.cpp) passed; the package test summary reported 80 checks, zero errors or failures, and 11 skipped cppcheck checks due to the installed-version warning. Reproduce from `ros_ws` with the documented ROS/environment setup, `colcon build --packages-select intelligence_foundations --symlink-install`, `colcon test --packages-select intelligence_foundations`, and `colcon test-result --test-result-base build/intelligence_foundations --verbose`. Learner review remains active; this does not close Cycle 1.
- [ ] Create the minimum `intelligence_foundations` and `synthetic_intelligence_engine` package skeletons when the reviewed model and first exact fixture are ready; implement the finite table, validation, deterministic sparse-to-dense compute conversion, policy-induced transition calculation, and deterministic fixture in C++20. The dense compute view shall declare the flattened state--action row order, transition and expected-reward shapes, feasible-action mask, terminal mask, ownership, and scalar type; masked padding shall never become invented dynamics for an infeasible pair.
- [ ] Verify stochastic rows, invalid probabilities, non-finite values, incompatible dimensions, absorbing and terminal cases, entrywise sparse-to-dense equivalence, flattening order, feasible-action masking, policy-induced transitions, and hand-calculated trajectory probabilities; then write and review the retrospective companion and close Cycle 1.

### Cycle 2 — Returns, value functions, and Bellman expectation equations

- [ ] Write and review only the conditional-expectation and tower-property prerequisite needed to derive returns and Bellman recursion, filing it in Part II and citing it from Part V.
- [ ] Define finite-horizon and discounted returns, state and action values under a policy, and the policy-induced Markov reward process; derive Bellman expectation equations without introducing optimality prematurely.
- [ ] Extend the specification with horizon and discount domains, policy-evaluation inputs and outputs, stopping behaviour, and independent analytic acceptance values.
- [ ] Implement finite-horizon backward induction and exact or converged policy evaluation in C++20 using the verified dense compute view for repeated matrix--vector work; verify the results against hand-calculated values and direct sparse joint-outcome acceptance cases, including terminal handling, horizon boundaries, discount boundaries, stopping error, and invalid policies.
- [ ] Write and review the retrospective companion and close Cycle 2.

### Cycle 3 — Bellman optimality and exact control

- [ ] Define optimal state and action values; derive Bellman optimality equations, greedy policy improvement, value iteration, and policy iteration, introducing contraction results only to the depth required for convergence and stopping error.
- [ ] Extend the specification with tie handling, deterministic output policy, convergence tolerance, iteration limits, non-convergence reporting, and exact optimal acceptance values.
- [ ] Implement value iteration and policy iteration in C++20 by reusing the verified dense table and evaluation operations, vectorising independent state--action Bellman work where the selected backend supports it while preserving deterministic tie and update semantics.
- [ ] Verify exact small-problem solutions, policy improvement, stable ties, contraction and stopping bounds where applicable, iteration limits, terminal cases, and agreement between independent exact methods.
- [ ] Write and review the retrospective companion, verify installed-target consumption from the synthetic package, run the focused package tests, link the evidence, render the reviewed notes, and close `IF.4`.

## `IF.5` — Sampled tabular reinforcement learning

**Main-phase destination:** Phase 6.

- [ ] Learn and review bandits, exploration versus exploitation, Monte Carlo prediction and control, TD(0), bootstrapping, n-step returns at comparative depth, SARSA, expected SARSA, Q-learning, on-policy versus off-policy learning, and convergence assumptions using Sutton and Barto, Chapters 2 and 5–8, supported by Bertsekas, Section 3.3.
- [ ] Derive every sample update from the corresponding Bellman target and distinguish episode termination from time-limit truncation in the return and bootstrap equations.
- [ ] Specify tabular policy, value, exploration, seeding, episode, trace, and evaluation contracts against the `IF.4` exact oracle.
- [ ] Implement seeded bandit estimates, Monte Carlo prediction/control, TD(0), SARSA, and Q-learning in C++20 through the shared environment contract. Preserve sequential sample-update order whenever repeated state--action entries make the update order part of the algorithm; parallel collection or evaluation requires independent seeded streams and a deterministic merge.
- [ ] Verify hand-calculated updates, terminal handling, exploration schedules, reproducible traces, and convergence toward the exact finite-MDP reference where assumptions permit.
- [ ] Write and review the retrospective companion, run focused tests and frozen multiple-seed learning evidence, and close `IF.5`.

## `IF.6` — Function approximation and DQN

**Main-phase destination:** Phase 6.

- [ ] Learn and review linear value approximation, neural value functions, semi-gradient updates, replay, target networks, Double-DQN as a comparative correction, the deadly triad, and known instability cases using Sutton and Barto, Chapters 9–10, Bertsekas, Chapter 3, and the primary DQN sources.
- [ ] Specify the tensor encoder, replay transition, sampling, target-network update, optimiser, checkpoint, seeding, device, evaluation, invalid-schema, batch-shape, contiguous-layout, and host/device-transfer contracts.
- [ ] Accept the shared LibTorch toolchain and neural-update cycle, creating it here only if no earlier consumer owns it: minimal CMake build, CPU and CUDA tensor operations where available, automatic differentiation, optimiser step, and checkpoint round trip. Close and expose that cycle before continuing to DQN-specific work.
- [ ] Implement one compact DQN in C++20 and LibTorch against the finite MDP, reusing accepted episode, seed, and evaluation operations and introducing replay here only if no earlier consumer owns it; construct replay batches, network inference, target values, losses, and optimiser inputs with LibTorch tensor operations rather than per-transition scalar training loops.
- [ ] Test exact tensor shapes, contiguous-layout assumptions, replay sampling, batched target equivalence to independent scalar cases, gradient isolation, terminal and truncation masks, target synchronisation, host/device transfer, serialisation, and deterministic tiny-network updates; then run frozen multiple-seed learning evidence against the exact optimum and record batch throughput where it affects the selected configuration.
- [ ] Write and review the companion, build through installed targets, record training cost and known instability, and close `IF.6`.

## `IF.7` — Policy gradients, actor–critic, and PPO

**Main-phase destination:** Phase 6.

- [ ] Learn and review the policy-gradient theorem, likelihood-ratio estimator, return-to-go, baselines, advantage functions, actor–critic, generalised advantage estimation, importance ratios, trust-region motivation, PPO clipping, entropy bonuses, and bias–variance trade-offs using Sutton and Barto, Chapter 11, Bertsekas, Section 3.5, and the primary PPO paper.
- [ ] Derive the score-function gradient and PPO surrogate from the declared objective; state what clipping does and does not guarantee.
- [ ] Specify stochastic-policy distributions, action bounds, log-probabilities, rollout batches, advantage normalisation, update epochs, old-policy identity, checkpoints, and evaluation.
- [ ] Implement REINFORCE as an equation-level reference, one bounded actor–critic, and PPO in C++20 and LibTorch; exercise discrete actions on the finite MDP and continuous actions on the point-mass fixture when the latter becomes available.
- [ ] Verify distributions, log-probabilities, entropy, advantages, clipping branches, value loss, gradient flow, old-policy freezing, minibatch coverage, terminal handling, and seeded updates before comparing learning curves.
- [ ] Write and review the companion, record multiple-seed evidence and compute, and close `IF.7`.

## `IF.8` — Deterministic policy gradients, TD3, and SAC

**Main-phase destination:** Phase 6.

- [ ] Learn and review deterministic policy gradients and DDPG foundations, critic approximation error, clipped double critics, delayed actor updates, target-policy smoothing, maximum-entropy RL, soft Bellman equations, reparameterised stochastic policies, squashed Gaussian actions, and entropy-temperature tuning using the primary DDPG, TD3, and SAC papers.
- [ ] Accept the bounded continuous point-mass fixture, creating its cycle here only if no earlier continuous consumer owns it: analytical dynamics, SI units, state and action bounds, reward, success, termination, truncation, invalid action, seeded disturbance if admitted, and frozen acceptance trajectories.
- [ ] Extend the shared replay, target-network, actor, critic, action-distribution, checkpoint, and evaluation contracts only where continuous semantics require it.
- [ ] Implement TD3 and SAC as durable C++20 and LibTorch learner cores; keep DDPG as the derivation and bounded reference needed to explain the extensions rather than another mandatory performance contender.
- [ ] Verify target construction, twin-critic minimum, delayed update schedule, target smoothing and clipping, squashed-action log-probability correction, entropy target and temperature update, action bounds, gradient ownership, checkpoint round trips, and deterministic tiny-batch calculations.
- [ ] Compare PPO, TD3, and SAC under frozen point-mass scenarios, interaction budgets, seeds, evaluators, and compute accounting; preserve negative results.
- [ ] Write and review the companion, run installed-target and focused tests, and close `IF.8`.

## `IF.9` — Attention, Transformers, and a tiny autoregressive language model

**Main-phase destination:** Phase 6.

- [ ] Learn and review token embeddings, queries, keys, values, scaled dot-product attention, softmax, self- and cross-attention, causal and padding masks, multiple heads, positional information, residual paths, normalisation, feed-forward blocks, tensor shapes, autoregressive factorisation, teacher forcing, and quadratic sequence cost through *Attention Is All You Need* and selected supporting primary sources.
- [ ] Derive the $1/\sqrt{d_k}$ scale, mask effect, head concatenation, cross-entropy language loss, and autoregressive generation relation; distinguish sequence position, model state, memory, and world state.
- [ ] Specify a controlled vocabulary and tokenizer, sequence and mask layouts, Transformer configuration, initialisation, random sources, checkpoints, generation limits, invalid tokens, and exact tiny-tensor acceptance cases.
- [ ] Implement attention and a complete Transformer block directly from LibTorch tensor operations, followed by a tiny decoder-only autoregressive language model trained only on versioned project-generated controlled text.
- [ ] Verify attention against hand calculations, mask semantics, shape contracts, head recombination, positional sensitivity, causal non-leakage, batching and padding, deterministic generation, checkpoint round trips, and overfitting of a deliberately tiny fixture before larger training.
- [ ] Write and review the companion, record parameters, training time, inference latency, memory, and language-scope limits, and close `IF.9`.

## `IF.10` — Learned dynamics, world models, imagination, and model-based RL

**Main-phase destination:** Phase 11A basic models and planning; Phase 11B selected extensions.

Accept transition prediction, uncertainty, rollout, and CEM as separate completed cycles when ready. Introduce advanced recurrent/latent/Transformer theory only when its consumer needs it; neither that theory nor Dyna integration blocks the basic cores used by Phase 11A. Reuse the accepted point-mass analytical oracle rather than creating a second fixture implementation.

- [ ] Learn and review analytical and learned transitions, deterministic and probabilistic prediction, ensembles, recurrent and latent state models, action-conditioned Transformer dynamics, one-step versus rollout loss, compounding error, model exploitation, Dyna, random shooting, CEM, and model predictive control using Sutton and Barto, Chapter 8, Bertsekas's model-based sections, and primary world-model sources.
- [ ] Study PETS, Dreamer, MuZero, TD-MPC, TransDreamer, and related architectures comparatively without creating a separate implementation of each system.
- [ ] Specify shared semantic transition records, model-specific encoders, deterministic and distributional predictions, rollout requests, uncertainty outputs, planner objectives, horizons, action bounds, fallback, latency budgets, and evaluation splits.
- [ ] Implement the analytical point-mass transition oracle, a deterministic learned transition model, one probabilistic or ensemble model, action-conditioned multi-step rollout, random shooting, and CEM in C++20 and LibTorch.
- [ ] Add Dyna-style simulated updates to one already verified learner without allowing model-generated data to become indistinguishable from real fixture transitions.
- [ ] Verify one-step and rollout error by horizon, likelihood or calibration for probabilistic outputs, ensemble aggregation, deterministic seeded rollouts, planner constraints, CEM elite selection and refitting, model exploitation, invalid predictions, horizon and compute limits, and fallback.
- [ ] Compare analytical and learned models on prediction and planning utility under matched data and compute; write and review the companion; preserve negative conclusions; and close `IF.10`.

## `IF.11` — Behavioural cloning, IQL, and Decision Transformer

**Main-phase destination:** Phase 9A cloning; Phase 9B selected advanced learner.

Complete and expose the data/supervised-learning and BC cycles first. Learn, specify, implement, and verify IQL and Decision Transformer in later separate cycles; Transformer contracts and return conditioning are prerequisites only for the sequence learner. The common educational comparison waits for the three verified learners, while Phase 9A may consume BC immediately.

- [ ] Learn and review imitation learning, behavioural cloning, covariate shift, DAgger, dataset support and provenance, offline-RL distribution shift, off-policy evaluation limits, expectile regression, IQL, conservative methods at comparative depth, and return-conditioned sequence modelling through PoRA, Chapter 19, and the primary IQL and Decision Transformer papers.
- [ ] Generate and freeze mixed-quality point-mass trajectories from declared expert, intermediate, poor, intervention, and failure policies without using evaluation episodes for training.
- [ ] Specify dataset schema and provenance, episode splits, observation and action encodings, behaviour policy metadata, target returns, context length, unsupported-return handling, training budgets, and exact online evaluator isolation.
- [ ] Implement behavioural cloning, IQL, and Decision Transformer in C++20 and LibTorch by reusing the Transformer, trajectory, replay, checkpoint, and evaluator targets.
- [ ] Verify supervised targets, expectile-loss branches, Q and value targets, advantage-weighted policy extraction, return-to-go construction, causal sequence alignment, context truncation, padding masks, unsupported requested returns, and checkpoint metadata.
- [ ] Compare BC, IQL, and Decision Transformer on identical dataset versions and splits; report dataset-quality sensitivity, return, success, support failures, multiple seeds, latency, and compute without presenting unmatched online learners as a fair leaderboard.
- [ ] Write and review the companion, run installed-target tests, and close `IF.11`.

## `IF.12` — Diffusion and score-based generative models

**Main-phase destinations:** Phases 6, 9, and 11.

- [ ] Learn and review the discrete Gaussian forward process, closed-form noising, reverse parameterisation, variational and noise-prediction objectives, denoising score matching, conditioning, guidance, inpainting, trajectory diffusion, and the continuous-time score-SDE connection using the DDPM, score-SDE, and Diffuser primary papers.
- [ ] Derive every schedule quantity, the arbitrary-time sample, the denoising target, and the reverse mean used by the bounded implementation; distinguish DDPM sampling, score interpretation, and trajectory-level planning.
- [ ] Specify schedules, scalar type, tensor layouts, timestep sampling, random generators, conditional inputs, guidance hook, sampler, checkpoint, deterministic replay, invalid configuration, and bounded compute.
- [ ] Implement a small DDPM and score-model core directly from LibTorch tensor operations, recover a seeded toy multimodal distribution, and add one conditional point-mass action-sequence generator with declared start and goal conditions.
- [ ] Verify schedule identities, closed-form noising, seeded noise, loss targets, reverse-step invariants, conditioning and inpainting preservation, guidance-off equivalence, action bounds, sequence shapes, deterministic replay where declared, and sensitivity to sampling steps.
- [ ] Evaluate distribution coverage and conditional sequence validity; keep image generation, large U-Nets, raw-language diffusion, robot Diffusion Policy, and opaque end-to-end libraries excluded.
- [ ] Write and review the companion, record parameter, time, latency, and memory budgets, and close `IF.12`.

## `IF.13` — Memory, grounded latent state, and intelligence-engine integration

**Main-phase destination:** Phase 7, with later Phase 11 reuse.

Begin from the accepted PPO, Transformer, basic predictive-model, and required memory operations. Create only missing memory/integration cycles here. Completion of `IF.8`, `IF.11`, `IF.12`, or any robotics phase is not an entry gate; this capability's longer outline is decomposed only when it approaches activation.

- [ ] Learn and review bounded working and episodic memory, retrieval, recurrent state, latent-state prediction, latent reasoning, action-conditioned internal simulation, and RL control of reasoning using the verified Transformer and world-model foundations; treat Coconut, TransDreamer, and RLP as comparative prior work rather than evidence for this project.
- [ ] Create the synthetic text-world specification with hidden symbolic state, controlled observations, authoritative transitions, external actions, internal reasoning actions, token generation, procedural splits, episode and reasoning budgets, validity, and failure behaviour.
- [ ] Specify a bounded episodic memory with typed versioned events, timestamps, episode identity, fixed capacity, explicit eviction, deterministic recency/relevance baseline, one learned retrieval scorer, stale or invalid records, and no external vector database.
- [ ] Define $z_t$ as a learned latent state that must support independent probes of current symbolic state, next state, next observation, and reward; freeze the model-specific encoder and decoder adapters without exposing latent coordinates as a semantic public contract.
- [ ] Keep $a_t^{\mathrm{env}}$, $c_k^{\mathrm{reason}}\in\{\text{RETRIEVE},\text{IMAGINE},\text{COMPARE},\text{ANSWER}\}$, and $x_j^{\mathrm{token}}$ separate. Only validated environment actions mutate authoritative state.
- [ ] Implement the text world, procedural data generator, semantic parser and evaluator, memory baseline and learned retrieval scorer, text encoder, grounded latent-state probes, action-conditioned latent world model, multi-step imagination, and autoregressive answer decoder by consuming installed foundation targets.
- [ ] Implement a scripted fixed-budget reasoner before the learned controller; then implement one PPO reasoning controller with reward $R_{\mathrm{semantic}}-\lambda C_{\mathrm{reasoning}}-\mu C_{\mathrm{invalid}}$. Admit prediction-improvement only as a declared potential-based shaping comparison.
- [ ] Verify authoritative transitions, controlled-language round trips, split isolation, memory capacity and eviction, retrieval, latent probes, one-step and multi-step imagined state, reasoning-action budgets, answer parsing, invalid output fallback, and deterministic small fixtures.
- [ ] Write and review the integration companion, build the synthetic consumer only through installed foundation targets, and close `IF.13`.

## `IF.14` — Synthetic language-engine capstone

**Main-phase destination:** Phase 7.

- [ ] Freeze five task families: current-state tracking, delayed recall, one- and multi-step action-conditioned prediction, goal-directed action-sequence planning, and counterfactual question answering.
- [ ] Freeze in-distribution, longer-horizon, and unseen-composition evaluation splits; semantic state, rollout-by-horizon, plan-validity, task-success, answer-semantic, token, reasoning-step, latency, memory, parameter, training-time, and peak-memory measures; seeds; compute; and practical-effect thresholds.
- [ ] Compare: autoregressive Transformer without persistent memory; Transformer with memory but no learned world model; grounded latent state and world model without imagined rollout; fixed-depth scripted imagination; learned RL reasoning controller; and the oracle symbolic model and planner.
- [ ] Require genuine next-token training and autoregressive answer generation. A classifier or template selector may be a baseline but cannot satisfy the capstone.
- [ ] Verify five task families, invalid and excessive reasoning, unsupported actions, stale or incompatible memory and checkpoints, context truncation, rollout compounding error, world-model exploitation, deterministic fallback, and evaluator isolation.
- [ ] Run multiple held-out seeds and record positive and negative results. Capstone completion requires correct implementation, reproducibility, and attribution, not superiority of learned reasoning over the scripted baseline.
- [ ] Publish `docs/reports/phase_07_synthetic_intelligence_engine_verification.md` with the architecture, data, baselines, ablations, metrics, compute, limitations, negative results, and exact foundation-core versions; review it and close `IF.14`.

## `IF.15` — Independent-track closure and main-track convergence

This closes the remaining independent track, not access to its earlier artifacts. Extend the handoff manifest at each actual consumer and complete the remaining checks here when the retained foundation and capstone outcomes are ready. If work transfers to the main tracker before independent closure, record the uncompleted outcomes as transferred rather than claiming they passed.

- [ ] Review all intelligence chapters in their final main-book parts, complete cumulative tests, reconcile cross-references and prerequisites, then update the shared notation and glossary only for reviewed material.
- [ ] Run the declared package builds, deterministic suites, installed-target downstream checks, frozen statistical evaluations, and clean-environment reproduction for the principal synthetic result.
- [ ] Confirm that `intelligence_foundations` has no ROS, Gazebo, fixture-semantic, synthetic-language, or robot dependency, and that `synthetic_intelligence_engine` remains a downstream reference consumer.
- [ ] Record source identity, public targets, configuration-only reuse, adapters, retraining, compatible extension, semantic incompatibility, and replacement separately in a versioned convergence manifest.
- [ ] Label the outputs candidate reusable cores until the owning main phase exercises the installed targets through its required robotics consumers. Do not use synthetic evidence to claim robot reuse, physical safety, general language intelligence, or embodied transfer.
- [ ] Update the owning Phase 6, Phase 7, Phase 9A/9B, and Phase 11A/11B checklist work from first implementation to acceptance, adaptation, and application evidence at each handoff, without duplicating completed cores.
- [ ] Give each unfinished shared cycle one owner. Retire this checklist as merged only when all remaining independent work has been completed or explicitly transferred to the owning main phases; preserve the final evidence links and transferred-work status.
- [ ] Render and inspect the complete affected book, inspect the final diff and artifacts, publish the track closure report, and close `IF.15`.

## Deferred and excluded work

- Epistemic autonomy, model-set adequacy, active diagnosis, metacognitive governance, and safe probing form a separate optional future capstone. They do not define this engine or block this track.
- Robotics, ROS, Gazebo, perception, manipulation, road-agent, UAV, and hardware implementations remain in the main track.
- Pretrained or large language models, unrestricted natural language, web-scale corpora, external foundation-model APIs, downloaded policies, VLA models, and general-intelligence claims are excluded.
- Image diffusion, large U-Nets, custom tensor libraries, custom autograd, custom optimisers, custom CUDA kernels, complete Dreamer, MuZero, PETS, or TD-MPC reproductions, and duplicate Python production engines are excluded.
- Safe-RL research, physical-safety claims, functional-safety certification, multi-agent RL, adversarial imitation, inverse RL, CQL implementation, and unrestricted online tool use are outside this track unless the programme is deliberately revised after closure.

## Backlog

- Optional research study: test whether explicitly predictive grounding of latent reasoning states improves held-out composition, horizon, interaction efficiency, or reasoning cost relative to matched continuous-latent and non-grounded baselines.
- Optional separate capstone: task-bounded epistemic autonomy with model-set inadequacy, active evidence gathering, fallback, and abstention.
- Optional post-core extensions require a frozen observed limitation, a simpler baseline, a bounded data and compute budget, and a declared main-track consumer before admission.
