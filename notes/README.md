# Project Intelligence Notes

This directory is the single source for the project's theory book. Its chapters combine mathematics, physics, algorithms, and engineering interpretation around autonomous-system problems instead of separating those subjects into different folders.

## Organisation

- Organise source chapters into the phase-aligned part folders defined by `PLAN.md`, with `notes/intelligence_engine_laboratory` as the cross-phase synthetic-engine laboratory.
- A phase may own more than one coherent book part, and an integration-only phase may own a laboratory or results part rather than a new theory survey.
- Keep filenames descriptive within each part. Quarto order, not a global filename number, determines the rendered chapter number.
- Keep the migrated Phase 1 chapters under `notes/part_01_motion_mechanics_control`. Migrate later reviewed chapters only through a deliberate batch that updates cross-references and `_quarto.yml` together.
- Write each just-in-time prerequisite block and early intelligence-foundations chapter directly in its final main-book part. When the main robotics track reaches it, review and extend that file in place rather than creating a parallel note.
- Use [the textbook index](../textbooks/INDEX.md) to find relevant sources.
- Add a chapter to this table of contents only when its note file exists.
- Keep generated HTML and PDF output outside this source directory.

## Planned book structure

### Front matter

- Purpose, prerequisites, notation, units, and conventions

### Part I — Phase 1: Motion, mechanics, and control

- Linear algebra foundations
- Geometry and coordinate frames
- Kinematics and numerical integration
- Dynamics, forces, torque, friction, and actuator limits
- Feedback, stability, PID control, and constraint handling

### Part II — Phase 2: Probability and state estimation

- Probability and stochastic models
- Bayesian inference and filtering
- Observability, Kalman filtering, and sensor fusion

### Part III — Phase 3: Perception and semantic observations

- Image formation and camera geometry
- Features, recognition, depth, and motion

### Part IV — Phase 4: Mapping, SLAM, and memory

- Registration and factor graphs
- Spatial-semantic world state
- Episodic memory, retrieval, provenance, and invalidation

### Part V — Phase 5: Optimisation, planning, and decisions

- Optimisation foundations and optimal control
- Finite MDPs and exact dynamic programming
- Search, motion planning, POMDPs, and risk-aware decisions

### Part VI — Phase 6: Learning foundations

- Tabular and deep reinforcement learning
- Policy gradients, PPO, TD3, and SAC
- Attention, Transformers, diffusion, and score models
- Reproducible learning, data, checkpoint, and evaluation contracts

### Synthetic intelligence-engine laboratory — Phase 7

- Tiny autoregressive language modelling
- Bounded episodic memory and predictively grounded latent state
- World-model imagination and learned reasoning control
- Capstone implementation companion, ablations, and results

### Part VIII — Phase 8: Classical manipulation and contact

- Spatial arm modelling, grasping, collision, and contact
- Manipulation planning, control, recovery, and benchmark design

### Part IX — Phase 9: Imitation, offline RL, and manipulation learning

- Behavioural cloning, IQL, and Decision Transformer
- Diffusion Policy and manipulation-specific learning evidence

### Part X — Phase 10: Generalization and adaptation

- Embodiment, environment, task, learner, and adaptation contracts
- Held-out transfer, target readiness, and negative transfer

### Part XI — Phase 11: World models and dynamic agents

- Analytical, learned, hybrid, latent, and Transformer world models
- Model-based RL, CEM, Diffuser, manipulation, and road-agent evidence

### Part XII — Phase 12: Classical UAV systems

- Multirotor dynamics, estimation, planning, control, and energy
- Flight envelopes, inspection-and-return missions, and failsafes

### Part XIII — Phase 13: UAV intelligence transfer

- Flight-specific learned dynamics, adaptation, uncertainty, and planning
- Policy and cross-embodiment transfer evidence

### Part XIV — Phase 14: Multi-embodiment release and research results

- Experimental design, benchmark definitions, and capstone conclusions
- Cross-embodiment evidence, ablations, limitations, and negative results

An optional epistemic-autonomy capstone receives a clearly labelled supplemental part only if it is later admitted. It is not reserved in advance and does not block the main book.

## Chapter structure

Each chapter should be self-contained:

1. motivating capability and intended robot behaviour;
2. prerequisites, assumptions, notation, units, and frames;
3. mathematical and physical model with step-by-step derivations;
4. algorithms and pseudocode after the model is understood;
5. worked examples and engineering interpretation;
6. limitations, failure conditions, and implementation implications;
7. short retrieval tests after major concepts;
8. a cumulative chapter test with answers or hints;
9. cited textbook sections and any supplementary authoritative sources.

## Compilation

The chapters are Quarto Markdown (`.qmd`) sources ordered by `_quarto.yml`. Render the complete book from this directory:

```zsh
cd /home/maxwell/Repos/robotics_autonomous/notes
quarto render
```

The configured output directory is `output/book` at the repository root. Treat that directory as generated output rather than source material.

