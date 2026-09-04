# Finite-MDP Model Capability Specification

## Capability and active cycle

This specification owns `IF.4`, Cycle 1: a simulator-neutral representation of a finite Markov decision process and the exact calculations needed to validate the model and apply a stationary Markov policy. Given finite state and action domains, feasible actions, joint next-state--reward probabilities, terminal states, and a policy, the capability shall either construct a valid immutable model and calculate its induced Markov reward process or report an explicit failure.

The reviewed theory is documented in [Finite Markov Chains](../../notes/part_02_probability_estimation/finite_markov_chains.qmd) and [Finite Markov Decision Processes](../../notes/part_05_optimisation_planning_decisions/finite_markov_decision_processes.qmd).

## Intended behaviour

The Cycle 1 capability shall:

1. represent finite state and global action domains without assigning fixture or robot semantics to them;
2. represent the feasible action set of every state and the finite joint distribution over next states and rewards for every feasible state--action pair;
3. validate probability, domain, feasibility, terminal-state, and policy invariants before calculations begin;
4. calculate controlled next-state probabilities and expected one-step rewards;
5. calculate the transition matrix and expected-reward row induced by a stationary Markov policy;
6. preserve the distinction between task termination and externally imposed truncation in a simulator-neutral transition result; and
7. reject invalid models, policies, identifiers, and numeric values without silently repairing them.

## Mathematical model

The controlled environment is the finite MDP

$$
\mathcal M
=
\left(
\mathcal S,
\mathcal A,
\{\mathcal A(s)\}_{s\in\mathcal S},
\mathcal R,
p
\right),
$$

where each state has a nonempty feasible action set $\mathcal A(s)\subseteq\mathcal A$. For every feasible pair $(s,a)$, the controlled kernel is the joint probability mass function

$$
p(s',r\mid s,a)
=
\Pr(S_{t+1}=s',R_{t+1}=r\mid S_t=s,A_t=a).
$$

It satisfies

$$
p(s',r\mid s,a)\geq0,
\qquad
\sum_{s'\in\mathcal S}
\sum_{r\in\mathcal R}
p(s',r\mid s,a)
=1.
$$

The controlled next-state probability and expected one-step reward are

$$
p(s'\mid s,a)
=
\sum_{r\in\mathcal R}p(s',r\mid s,a),
$$

and

$$
\bar r(s,a)
=
\sum_{s'\in\mathcal S}
\sum_{r\in\mathcal R}
r\,p(s',r\mid s,a).
$$

A stationary Markov policy assigns a probability to each globally identified action:

$$
\pi(a\mid s)\geq0,
\qquad
\pi(a\mid s)=0
\quad\text{when }a\notin\mathcal A(s),
\qquad
\sum_{a\in\mathcal A(s)}\pi(a\mid s)=1.
$$

The policy-induced transition and expected reward are

$$
p_\pi(s'\mid s)
=
\sum_{a\in\mathcal A(s)}
\pi(a\mid s)p(s'\mid s,a),
$$

and

$$
\bar r_\pi(s)
=
\sum_{a\in\mathcal A(s)}
\pi(a\mid s)\bar r(s,a).
$$

For declared state order $(s_1,\ldots,s_n)$, the resulting matrix and reward row satisfy

$$
[\mathbf P^\pi]_{ij}
=
p_\pi(s_j\mid s_i),
\qquad
[\bar{\mathbf r}^{\,\pi}]_i
=
\bar r_\pi(s_i).
$$

## Representation and contracts

### State and action indices

`StateIndex` and `ActionIndex` shall be distinct strong types backed by non-negative integers. For a model with $n\geq1$ states and $m\geq1$ global action labels, valid indices belong to the contiguous ranges $0,\ldots,n-1$ and $0,\ldots,m-1$. Their ordering defines matrix row, matrix column, and policy-column order. The foundations package shall not attach names, units, coordinates, or robot meanings to these indices.

The model shall store a nonempty feasible action list for every state. Each list shall contain valid, unique action identifiers in increasing global action order.

### Sparse joint-outcome table

For each feasible pair $(s,a)$, the model shall store a nonempty ordered collection of `OutcomeProbability` records. Each record contains:

- a valid next-state identifier $s'$;
- one finite scalar reward $r$; and
- one finite probability $q$ satisfying $0<q\leq1$.

The stored records are the positive support of $p(s',r\mid s,a)$; omitted outcomes have probability zero. The same next state may appear in several records when it is paired with different reward values. An exact pair $(s',r)$ shall appear at most once in one state--action row, so every stored record denotes one unambiguous joint event.

Reward values use `double`. They are scalar task evaluations with no physical unit imposed by this generic capability. The finite reward set $\mathcal R$ is the set of distinct reward values present in the validated table. Rewards shall not be compared, merged, or quantised using an approximate-equality rule.

### Terminal states

The model shall store a set $\mathcal S_{\mathrm{term}}\subseteq\mathcal S$. Entering one of these states terminates the modelled episode after retaining the reward produced by that transition.

For matrix calculations, each terminal state shall use one declared bookkeeping action whose only joint outcome is a probability-one, zero-reward self-transition. A bookkeeping action may be shared by several terminal states, but it shall not be feasible in a nonterminal state unless the model independently defines it there as a real action.

An episode runner shall not request another real decision after termination. The absorbing row exists to keep stochastic-matrix and post-termination sequence calculations defined; it does not create another task decision.

### Stationary Markov policy

A `FinitePolicy` shall store a dense $n\times m$ table. Entry $(i,\ell)$ represents $\pi(a_\ell\mid s_i)$ using the model's global state and action order. Every entry shall be finite and lie in $[0,1]$. Entries for infeasible actions shall be exactly zero, and the entries over the feasible actions of each state shall sum to one within the declared probability-sum tolerance.

The policy shall contain a probability-one choice of the bookkeeping action in each terminal state. The capability shall not renormalise a policy row or infer missing probabilities.

### Simulator-neutral transition result

The shared `StepResult` contract shall contain:

- `next_state`: the valid resulting state identifier;
- `reward`: the finite reward produced by the transition;
- `terminated`: whether the modelled task ended on this transition; and
- `truncated`: whether an external collector stopped the record on this transition.

`terminated` and `truncated` are separate facts and may both be true. Entering $\mathcal S_{\mathrm{term}}$ sets `terminated` to true. A task-defined finite horizon also produces termination when its final decision step is completed, but horizon bookkeeping belongs to the task or environment adapter because the time-homogeneous table contains no decision-step counter. A storage, evaluation, or wall-clock limit sets `truncated` without changing the MDP.

Cycle 1 defines and validates this result contract but does not yet require stochastic sampling. Sampled stepping begins when `IF.5` introduces controlled random generators and episode traces.

## Cycle 1 operations

### Validated construction

Construction accepts the state count, action count, feasible-action lists, terminal-state set, sparse joint-outcome table, and a probability-sum tolerance. It returns one immutable valid model or an explicit construction failure. No partially valid model shall be observable.

The probability-sum tolerance shall be finite and satisfy $0<\varepsilon_p\leq10^{-12}$. This upper bound is a deliberate package validation policy: the tolerance accounts for floating-point summation error rather than permitting materially non-normalised probability data. For each feasible pair, construction shall calculate the row sum using a numerically stable summation method and require

$$
\left|
\sum_{s',r}p(s',r\mid s,a)-1
\right|
\leq\varepsilon_p.
$$

Passing this check shall not cause the stored probabilities to be rescaled.

### Controlled queries

Given a valid model and a feasible pair $(s,a)$, the controlled-transition query shall return the $n$-entry row

$$
\begin{bmatrix}
p(s_1\mid s,a)&\cdots&p(s_n\mid s,a)
\end{bmatrix},
$$

and the expected-reward query shall return $\bar r(s,a)$. Both results shall accumulate directly from the validated joint-outcome records.

### Policy-induced calculation

Given a valid model and valid `FinitePolicy`, the policy-induced operation shall return:

$$
\mathbf P^\pi\in\mathbb R^{n\times n}
\quad\text{and}\quad
\bar{\mathbf r}^{\,\pi}\in\mathbb R^{1\times n}.
$$

The operation shall average only feasible controlled rows with their declared policy probabilities. It shall preserve state order, be deterministic, mutate neither input, and introduce no random sampling.

## Validation and failure behaviour

Construction or calculation shall fail explicitly for:

1. zero states or zero global actions;
2. an invalid, duplicate, or missing state--action row;
3. an empty feasible-action list or duplicate feasible action;
4. an outcome containing an invalid next state, non-finite reward, non-finite probability, non-positive stored probability, or probability greater than one;
5. duplicate exact $(s',r)$ outcomes in one row;
6. a joint-outcome row whose probability sum violates $\varepsilon_p$;
7. an invalid terminal-state identifier or an invalid terminal absorbing row;
8. an invalid probability-sum tolerance;
9. a policy with incompatible dimensions, non-finite entries, entries outside $[0,1]$, nonzero probability on an infeasible action, or an invalid row sum; or
10. a query containing an invalid state, invalid action, or infeasible state--action pair.

Failures shall distinguish invalid domain identifiers, invalid model structure, invalid probabilities, invalid terminal structure, and invalid policy structure. The implementation shall not clamp probabilities, discard invalid outcomes, create missing rows, insert actions, normalise rows, replace non-finite values, or return a numeric result alongside a failure.

## Frozen finite-MDP fixture

The `synthetic_intelligence_engine` package shall own one frozen fixture with states $(s_1,s_2,s_\dagger)$, global actions $(a_1,a_2,a_\bot)$, and terminal set $\{s_\dagger\}$. Its positive joint outcomes are:

| Current state | Action | Next state | Reward | Probability |
|---|---|---|---:|---:|
| $s_1$ | $a_1$ | $s_1$ | $0$ | $0.5$ |
| $s_1$ | $a_1$ | $s_2$ | $1$ | $0.5$ |
| $s_1$ | $a_2$ | $s_\dagger$ | $2$ | $1$ |
| $s_2$ | $a_1$ | $s_\dagger$ | $4$ | $1$ |
| $s_2$ | $a_2$ | $s_1$ | $-1$ | $1$ |
| $s_\dagger$ | $a_\bot$ | $s_\dagger$ | $0$ | $1$ |

The stationary policy is

$$
\begin{aligned}
\pi(a_1\mid s_1)&=0.6,
&\pi(a_2\mid s_1)&=0.4,\\
\pi(a_1\mid s_2)&=0.75,
&\pi(a_2\mid s_2)&=0.25,\\
\pi(a_\bot\mid s_\dagger)&=1.
\end{aligned}
$$

This fixture is an exact mathematical acceptance case. Fixture labels and task meaning remain downstream of the foundations package.

## Requirements

| ID | Requirement |
|---|---|
| IF-MDP-MOD-001 | The model shall represent every finite domain, feasible-action list, terminal state, and positive joint outcome declared by a valid input table. |
| IF-MDP-MOD-002 | The model shall remain immutable after successful validation. |
| IF-MDP-CAL-001 | Controlled-transition queries shall marginalise the joint kernel over reward. |
| IF-MDP-CAL-002 | Expected-reward queries shall calculate the probability-weighted reward over all joint outcomes. |
| IF-MDP-POL-001 | Policy validation shall enforce dimensions, probability bounds, feasibility, terminal bookkeeping, and row normalization. |
| IF-MDP-POL-002 | Policy-induced calculations shall implement the action-weighted transition and reward equations in declared state order. |
| IF-MDP-EP-001 | The shared step result shall preserve reward, termination, and truncation as separate fields with the declared meanings. |
| IF-MDP-VAL-001 | Model construction and queries shall reject every invalid condition listed in this specification explicitly. |
| IF-MDP-VAL-002 | The implementation shall neither repair nor silently normalise invalid model or policy data. |
| IF-MDP-API-001 | Public foundations interfaces shall contain no ROS, simulator, fixture-semantic, robot, or model-specific tensor dependency. |
| IF-MDP-API-002 | Exact calculations shall be deterministic, shall not mutate their inputs, and shall expose their declared ordering. |

## Acceptance cases

Use the frozen fixture and $\varepsilon_p=10^{-12}$ unless a case states otherwise.

| ID | Case | Expected result |
|---|---|---|
| IF-MDP-ACC-001 | Construct the complete frozen fixture. | Construction succeeds and preserves three states, three global actions, all feasible pairs, six positive joint outcomes, and one terminal state. |
| IF-MDP-ACC-002 | Query $(s_1,a_1)$. | The controlled-transition row is $[0.5\;0.5\;0]$ and $\bar r(s_1,a_1)=0.5$. |
| IF-MDP-ACC-003 | Apply the frozen policy. | $\mathbf P^\pi=\begin{bmatrix}0.30&0.30&0.40\\0.25&0&0.75\\0&0&1\end{bmatrix}$ and $\bar{\mathbf r}^{\,\pi}=[1.10\;2.75\;0]$. |
| IF-MDP-ACC-004 | Multiply the factors for the declared two-step event $s_1,a_1,1,s_2,a_2,-1,s_1$. | The independently calculated trajectory probability is $0.075$. |
| IF-MDP-ACC-005 | Classify the outcome entering $s_\dagger$ with reward $2$ and no collector stop. | The result retains reward $2$, sets `terminated=true`, and sets `truncated=false`. |
| IF-MDP-ACC-006 | Stop collection after a transition into nonterminal $s_2$ without a task ending. | The result sets `terminated=false` and `truncated=true`; the model still defines subsequent actions from $s_2$. |
| IF-MDP-ACC-007 | Replace one fixture probability by a negative, non-finite, or greater-than-one value, or make a row sum violate $\varepsilon_p$. | An invalid entry fails with `invalid_probability`; an invalid row sum fails with `invalid_probability_sum`; construction returns no model. |
| IF-MDP-ACC-008 | Give a policy positive mass on an infeasible action or an invalid row sum. | Policy validation fails and no induced matrix or reward row is returned. |
| IF-MDP-ACC-009 | Query an invalid state, invalid action, or infeasible pair. | The query fails explicitly and returns no probability or reward result. |
| IF-MDP-ACC-010 | Repeat a valid policy-induced calculation. | Every returned entry is identical across calls on the same platform and configuration. |

Floating-point comparisons shall use the tolerance declared by each test. Tests shall compare the fixture results with the analytic values above rather than with a second implementation of the same production calculation.

## Exclusions

Cycle 1 does not include:

- stochastic sampling, random-number ownership, episode traces, or exploration;
- returns, value functions, Bellman equations, policy evaluation, policy improvement, value iteration, or policy iteration;
- continuous state or action spaces;
- neural networks, tensors, replay, datasets, training, or checkpoints;
- task-specific state and action schemas inside the foundations package;
- ROS messages, nodes, Gazebo, robot adapters, or physical-safety claims; or
- automatic conversion of task horizons into augmented time-indexed states.

These operations belong to later declared cycles and shall not expand the first implementation.

## Traceability

| Requirement | Reviewed theory | Planned verification |
|---|---|---|
| IF-MDP-MOD-001, IF-MDP-CAL-001, IF-MDP-CAL-002 | Controlled transition probabilities and joint rewards in the finite-MDP chapter | IF-MDP-ACC-001 and IF-MDP-ACC-002 |
| IF-MDP-POL-001, IF-MDP-POL-002 | Policies and policy-induced dynamics | IF-MDP-ACC-003, IF-MDP-ACC-008, and IF-MDP-ACC-010 |
| IF-MDP-EP-001 | Termination, absorbing extensions, truncation, and horizons | IF-MDP-ACC-005 and IF-MDP-ACC-006 |
| IF-MDP-VAL-001, IF-MDP-VAL-002 | Probability and model validity conditions | IF-MDP-ACC-007 through IF-MDP-ACC-009 |
| IF-MDP-API-001, IF-MDP-API-002 | Intelligence-track package boundary and reproducibility rules | Interface review and installed-target consumer test |

## Gate to implementation

Implementation may begin only after this minimum specification is reviewed. The first buildable block shall create the `intelligence_foundations` package and its strong state and action identifiers. The immutable joint-outcome table follows in manageable blocks; the `synthetic_intelligence_engine` package begins when the frozen fixture is ready to consume the installed foundations target. Policy-induced calculations follow only after model construction and validation tests pass.
