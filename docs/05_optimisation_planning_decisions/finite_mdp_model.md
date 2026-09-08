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

For matrix calculations, construction shall generate one shared bookkeeping action whose only joint outcome at each terminal state is a probability-one, zero-reward self-transition. This generated action shall be feasible only at terminal states.

`FiniteMdp::from_rows(state_count, task_action_count, rows, terminal_states = {})` shall accept only task-action rows, with action indices in `[0, task_action_count)`. If any terminal states are declared, construction shall append the bookkeeping action at `ActionIndex{task_action_count}` without renumbering task actions. The complete model's `action_count()` shall equal `task_action_count() + 1` in this case and `task_action_count()` otherwise. `bookkeeping_action()` shall return `std::optional<ActionIndex>` containing the generated index, or `std::nullopt` when there are no terminal states. Adding the action shall fail with `invalid_action_count` if the complete count cannot be represented by `std::size_t`.

Construction shall reject every caller-supplied outgoing terminal row with `invalid_terminal_structure`, including a zero-reward self-transition. It shall generate exactly one row per declared terminal state using the shared bookkeeping action, with a probability-one, zero-reward self-transition. Generated rows shall enter the same canonical state--action order as supplied rows. Nonterminal feasibility is defined by supplied rows, and every nonterminal state must have at least one such row. Incoming transitions and their rewards shall remain unchanged. A zero task-action count is valid only when all states are terminal; otherwise construction shall fail with `invalid_action_count`. The state count must remain positive.

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

Construction accepts the state count, task-action count, nonterminal feasible-action rows, and terminal-state set. Each supplied row contains a joint distribution already validated with its declared probability-sum tolerance. Construction generates the bookkeeping action and terminal rows under the terminal-state contract and returns one immutable valid model or an explicit construction failure. No partially valid model shall be observable.

The probability-sum tolerance shall be finite and satisfy $0<\varepsilon_p\leq10^{-12}$. This upper bound is a deliberate package validation policy: the tolerance accounts for floating-point summation error rather than permitting materially non-normalised probability data. For each feasible pair, construction shall calculate the row sum using a numerically stable summation method and require

$$
\left|
\sum_{s',r}p(s',r\mid s,a)-1
\right|
\leq\varepsilon_p.
$$

Passing this check shall not cause the stored probabilities to be rescaled.

### Joint-distribution lookup

`FiniteMdp::find_outcome_distribution(StateIndex state, ActionIndex action) const` shall return `std::optional<std::reference_wrapper<const JointOutcomeDistribution>>`. For valid state and global action indices representing $(s,a)$, the result shall contain a read-only reference wrapper to the exact stored distribution $p(s',r\mid s,a)$ if $a\in\mathcal A(s)$, and shall be `std::nullopt` otherwise. An infeasible pair is ordinary absence, not an exception or a zero-valued distribution. This rule includes a task action queried at a terminal state and the generated bookkeeping action queried at a nonterminal state.

The lookup shall first reject a state index at or beyond `state_count()` with `FiniteMdpException` code `invalid_current_state`, then reject an action index at or beyond the complete `action_count()` with code `invalid_action`. These domain errors shall not be converted to empty optionals.

Lookup shall be deterministic, shall not mutate the model, and shall not copy the stored distribution or invent missing dynamics. The optional owns only its reference wrapper, not the referenced distribution. Callers shall check presence before dereferencing the optional and shall keep the referenced model storage alive and its reference valid throughout use. Returning or copying the wrapper shall not extend that storage's lifetime.

### Terminal-membership query

`FiniteMdp::is_terminal(StateIndex state) const` shall return `true` exactly when the valid queried state belongs to the declared terminal-state set, and `false` otherwise. An index outside `[0, state_count)` shall throw `FiniteMdpException` with code `invalid_current_state`, including when the terminal set is empty. The query shall be deterministic, shall not mutate the model, and shall use declared terminal membership rather than infer termination from self-transitions, rewards, or action counts.

### Controlled queries

Given a valid model and a feasible pair $(s,a)$, the controlled-transition query shall return the $n$-entry row

$$
\begin{bmatrix}
p(s_1\mid s,a)&\cdots&p(s_n\mid s,a)
\end{bmatrix},
$$

and the expected-reward query shall return $\bar r(s,a)$. Both results shall accumulate directly from the validated joint-outcome records.

### Dense compute view

Repeated matrix--vector calculations shall not reconstruct controlled rows from sparse joint outcomes on every use. A deterministic one-time conversion shall produce an immutable dense compute view. Here $m$ is the complete model's `action_count()`, including the generated bookkeeping action when present; policy dimensions use the same count. For state index $i\in\{0,\ldots,n-1\}$ and action index $\ell\in\{0,\ldots,m-1\}$, define the flattened row index

$$
k=im+\ell.
$$

The view shall contain a row-major controlled-transition matrix $\mathbf P_{\mathrm{ctl}}\in\mathbb R^{(nm)\times n}$, an expected-reward vector $\bar{\mathbf r}_{\mathrm{ctl}}\in\mathbb R^{nm}$, a feasible-action mask $\mathbf f\in\{0,1\}^{nm}$, and a terminal-state mask $\mathbf d\in\{0,1\}^{n}$. For every feasible pair $(s_i,a_\ell)$,

$$
\begin{aligned}
[\mathbf P_{\mathrm{ctl}}]_{k,j}
&=p(s_j\mid s_i,a_\ell),\\
[\bar{\mathbf r}_{\mathrm{ctl}}]_k
&=\bar r(s_i,a_\ell),\\
f_k&=1.
\end{aligned}
$$

For an infeasible pair, $f_k=0$ and the corresponding transition and reward storage shall be zero-filled deterministic padding. These zeros are not an environment response and shall never be queried, maximised over, or used in a policy-induced calculation without applying the feasible-action mask. The view shall declare `double` scalar storage, row-major layout, ownership, and lifetime; its conversion shall preserve the validated sparse model and introduce no random sampling. Cycle 1 verifies this representation and its equations but does not require threaded, SIMD, or GPU execution.

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

1. zero states, zero task actions when a nonterminal state exists, or overflow of the complete action count;
2. an invalid or duplicate state--action row, or a nonterminal state with no supplied feasible row;
3. an empty feasible-action list or duplicate feasible action;
4. an outcome containing an invalid next state, non-finite reward, non-finite probability, non-positive stored probability, or probability greater than one;
5. duplicate exact $(s',r)$ outcomes in one row;
6. a joint-outcome row whose probability sum violates $\varepsilon_p$;
7. an invalid or duplicate terminal-state identifier, or any caller-supplied outgoing terminal row;
8. an invalid probability-sum tolerance;
9. a policy with incompatible dimensions, non-finite entries, entries outside $[0,1]$, nonzero probability on an infeasible action, or an invalid row sum; or
10. a query containing an invalid state or invalid global action index.

A joint-distribution lookup with valid indices but an infeasible state--action pair shall return `std::nullopt` under the lookup contract above. It shall not throw an infeasibility exception or supply a probability or reward result for that pair.

Failures shall distinguish invalid domain identifiers, invalid model structure, invalid probabilities, invalid terminal structure, and invalid policy structure. The implementation shall not clamp probabilities, discard invalid outcomes, normalise rows, replace non-finite values, or return a numeric result alongside a failure. The only permitted action and row generation is the terminal completion above; missing nonterminal dynamics shall not be invented.

### Unresolved expected-reward numerical domain

The 2026-09-05 review found the following gap in the accepted-input domain. This is an unresolved contract finding, not an approved change to the reward domain or failure behaviour. Resolve the policy before changing production code or claiming full-domain numerical acceptance.

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

## Frozen finite-MDP fixture

The `synthetic_intelligence_engine` package shall own one frozen fixture with states $(s_1,s_2,s_\dagger)$, task actions $(a_1,a_2)$, and terminal set $\{s_\dagger\}$. Construction receives `task_action_count = 2` and only the nonterminal rows. It appends $a_\bot$ at action index 2, giving the complete global action order $(a_1,a_2,a_\bot)$. The completed model's positive joint outcomes are listed below; the final row is generated, not supplied:

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
| IF-MDP-MOD-003 | Construction shall automatically append one terminal-only bookkeeping action when needed, generate all terminal rows, reject supplied terminal rows, and preserve task-action indices and incoming rewards while exposing task and complete action counts. |
| IF-MDP-MOD-004 | The terminal-membership query shall report declared membership for valid states and explicitly reject invalid state indices. |
| IF-MDP-MOD-005 | Joint-distribution lookup shall return an optional borrowed read-only distribution for the exact feasible pair, an empty optional for an in-domain infeasible pair, and the specified exception for an invalid index, without copying distributions or generating dynamics. |
| IF-MDP-CAL-001 | Controlled-transition queries shall marginalise the joint kernel over reward. |
| IF-MDP-CAL-002 | Expected-reward queries shall calculate the probability-weighted reward over all joint outcomes. |
| IF-MDP-CAL-003 | One deterministic conversion shall produce the declared dense controlled-transition, expected-reward, feasible-action-mask, and terminal-mask representation without assigning dynamics to infeasible pairs. |
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
| IF-MDP-ACC-001 | Construct the frozen fixture from two task actions and its nonterminal rows. | Construction succeeds with three states, two task actions, three complete global actions, all feasible pairs, six positive joint outcomes, and one terminal state. |
| IF-MDP-ACC-002 | Look up $(s_1,a_1)$, then calculate its marginal and expected reward. | The optional is nonempty and refers to the matching stored distribution. The controlled-transition row is $[0.5\;0.5\;0]$ and $\bar r(s_1,a_1)=0.5$. |
| IF-MDP-ACC-003 | Apply the frozen policy. | $\mathbf P^\pi=\begin{bmatrix}0.30&0.30&0.40\\0.25&0&0.75\\0&0&1\end{bmatrix}$ and $\bar{\mathbf r}^{\,\pi}=[1.10\;2.75\;0]$. |
| IF-MDP-ACC-004 | Multiply the factors for the declared two-step event $s_1,a_1,1,s_2,a_2,-1,s_1$. | The independently calculated trajectory probability is $0.075$. |
| IF-MDP-ACC-005 | Classify the outcome entering $s_\dagger$ with reward $2$ and no collector stop. | The result retains reward $2$, sets `terminated=true`, and sets `truncated=false`. |
| IF-MDP-ACC-006 | Stop collection after a transition into nonterminal $s_2$ without a task ending. | The result sets `terminated=false` and `truncated=true`; the model still defines subsequent actions from $s_2$. |
| IF-MDP-ACC-007 | Replace one fixture probability by a negative, non-finite, or greater-than-one value, or make a row sum violate $\varepsilon_p$. | An invalid entry fails with `invalid_probability`; an invalid row sum fails with `invalid_probability_sum`; construction returns no model. |
| IF-MDP-ACC-008 | Give a policy positive mass on an infeasible action or an invalid row sum. | Policy validation fails and no induced matrix or reward row is returned. |
| IF-MDP-ACC-009 | Look up an out-of-domain state with a valid action; separately look up a valid state with an out-of-domain global action. | The lookup throws `FiniteMdpException` with `invalid_current_state` or `invalid_action`, respectively; it does not return an empty optional in place of the exception. |
| IF-MDP-ACC-010 | Repeat a valid policy-induced calculation. | Every returned entry is identical across calls on the same platform and configuration. |
| IF-MDP-ACC-011 | Convert the frozen sparse model to the dense compute view. | Every feasible transition and reward entry equals its direct sparse query in flattened order $k=im+\ell$; terminal and feasible masks match the model; infeasible storage is zero padding and remains excluded from calculations. |
| IF-MDP-ACC-012 | Construct with terminal states and only nonterminal rows, including an incoming terminal reward; repeat with shuffled rows and terminal indices and with multiple terminals. | Exactly one shared action is appended at the task-action count; each terminal gets an exact zero-reward, probability-one self-loop in canonical order. Task-action indices and every incoming outcome are preserved. The generated action is infeasible at nonterminals, and task actions are infeasible at terminals. |
| IF-MDP-ACC-013 | Supply an outgoing terminal row (including an absorbing one), use the generated index in a supplied nonterminal row, or append to the maximum representable task-action count. | Construction rejects with `invalid_terminal_structure`, `invalid_action`, or `invalid_action_count`, respectively, without replacing supplied data or overflowing. |
| IF-MDP-ACC-014 | Declare no terminal states; separately omit nonterminal rows in a model with a terminal state. | Without terminals, no action is appended and `bookkeeping_action()` is empty. Missing nonterminal feasibility still fails rather than generating nonterminal rows. |
| IF-MDP-ACC-015 | Declare every state terminal with zero task actions and no supplied rows; separately leave a nonterminal state with zero task actions. | The all-terminal model has task-action count zero, complete action count one, bookkeeping index zero, and one generated self-loop per state. The nonterminal case fails with `invalid_action_count`. |
| IF-MDP-ACC-016 | Query terminal and nonterminal states in a mixed model, valid states in empty-terminal and all-terminal models, and indices at or beyond the state count (including with no terminals). | Results match declared membership; a nonterminal zero-reward self-loop remains nonterminal. Every invalid index fails with `invalid_current_state`; repeated valid queries leave model data unchanged. |
| IF-MDP-ACC-017 | Use a two-state, three-task-action model without terminals and stored index pairs `(0,0)`, `(0,2)`, `(1,0)` with certain rewards 10, 20, 30. Look up each stored pair, then missing pairs `(0,1)` and `(1,2)`. In the frozen terminal fixture, also query a task action at the terminal state and the bookkeeping action at a nonterminal state. | Stored pairs return nonempty optionals whose distributions have the corresponding expected rewards. Every missing or terminal-infeasible pair returns an empty optional without throwing. Lookup neither substitutes a greater stored key nor accesses the end iterator as a row. |

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
| IF-MDP-MOD-001, IF-MDP-CAL-001, IF-MDP-CAL-002, IF-MDP-CAL-003 | Controlled transition probabilities and joint rewards in the finite-MDP chapter | IF-MDP-ACC-001, IF-MDP-ACC-002, and IF-MDP-ACC-011 |
| IF-MDP-POL-001, IF-MDP-POL-002 | Policies and policy-induced dynamics | IF-MDP-ACC-003, IF-MDP-ACC-008, and IF-MDP-ACC-010 |
| IF-MDP-EP-001 | Termination, absorbing extensions, truncation, and horizons | IF-MDP-ACC-005 and IF-MDP-ACC-006 |
| IF-MDP-MOD-003 | Terminal absorbing extension and the automatic factory completion contract | IF-MDP-ACC-012 through IF-MDP-ACC-015 |
| IF-MDP-MOD-004 | Declared terminal-state set and the distinction between terminal and absorbing states | IF-MDP-ACC-016 |
| IF-MDP-VAL-001, IF-MDP-VAL-002 | Probability and model validity conditions | IF-MDP-ACC-007 through IF-MDP-ACC-009 |
| IF-MDP-API-001, IF-MDP-API-002 | Intelligence-track package boundary and reproducibility rules | Interface review and installed-target consumer test |

## Gate to implementation

Implementation may begin only after this minimum specification is reviewed. The first buildable block shall create the `intelligence_foundations` package and its strong state and action identifiers. The immutable joint-outcome table follows in manageable blocks; the `synthetic_intelligence_engine` package begins when the frozen fixture is ready to consume the installed foundations target. Policy-induced calculations follow only after model construction and validation tests pass.
