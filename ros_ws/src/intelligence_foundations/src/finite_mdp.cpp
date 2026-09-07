#include "intelligence_foundations/finite_mdp.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "intelligence_foundations/finite_mdp_error.hpp"
#include "intelligence_foundations/outcome_probability.hpp"

namespace intelligence_foundations
{

FiniteMdp FiniteMdp::from_rows(
  std::size_t state_count, std::size_t task_action_count,
  std::vector<StateActionDistribution> rows,
  std::vector<StateIndex> terminal_states)
{
  // State indices belong to [0, state_count). Supplied action indices belong to
  // [0, task_action_count); the generated action is outside that input range.
  if (state_count == 0U) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_state_count, "The state count must be positive."};
  }

  // Validate terminal indices before indexing per-state storage. The set detects
  // duplicates here and supports terminal membership checks later.
  std::set<StateIndex> observed_terminal_states;

  for (const StateIndex state : terminal_states) {
    if (state.value() >= state_count) {
      throw FiniteMdpException{
        FiniteMdpError::invalid_terminal_state,
        "A terminal-state index is outside the model's state domain."};
    }

    const bool inserted = observed_terminal_states.emplace(state).second;

    if (!inserted) {
      throw FiniteMdpException{
        FiniteMdpError::duplicate_terminal_state,
        "The terminal-state list contains a duplicate index."};
    }
  }

  // Store terminal indices in a deterministic order independent of input order.
  std::ranges::sort(terminal_states);

  // Reject a model that has a nonterminal state but no task actions.
  // "terminal_states.size() != state_count" indicates a state is nonterminal.
  // It must has a task action.
  if (task_action_count == 0U && terminal_states.size() != state_count) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_action_count,
      "A model with nonterminal states requires a positive task-action count."};
  }

  // Check before adding one: unsigned wraparound would corrupt the action domain.
  if (
    !terminal_states.empty() &&
    task_action_count == std::numeric_limits<std::size_t>::max()) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_action_count,
      "Adding the bookkeeping action would overflow the action count."};
  }

  // Validate supplied rows before generating any terminal rows. Each distribution
  // already validates its own outcomes; here we check its domain against this MDP.
  std::set<std::pair<StateIndex, ActionIndex>> observed_state_actions;
  std::vector<std::size_t> feasible_action_counts(state_count, 0U);

  for (const StateActionDistribution & row : rows) {
    if (row.state.value() >= state_count) {
      throw FiniteMdpException{
        FiniteMdpError::invalid_current_state,
        "A row contains an invalid current-state index."};
    }

    // Terminal dynamics have one owner: the factory. Reject rather than replace
    // caller-supplied terminal rows, even if they already describe a self-loop.
    if (observed_terminal_states.contains(row.state)) {
      throw FiniteMdpException{
        FiniteMdpError::invalid_terminal_structure,
        "Outgoing terminal rows must be generated, not supplied."};
    }

    if (row.action.value() >= task_action_count) {
      throw FiniteMdpException{
        FiniteMdpError::invalid_action,
        "A supplied row contains an action outside the task-action domain."};
    }

    if (row.outcome_distribution.state_count() != state_count) {
      throw FiniteMdpException{
        FiniteMdpError::incompatible_outcome_state_count,
        "A row's outcome distribution uses an incompatible state count."};
    }

    const bool inserted =
      observed_state_actions.emplace(row.state, row.action).second;

    if (!inserted) {
      throw FiniteMdpException{
        FiniteMdpError::duplicate_state_action,
        "The model contains a duplicate state-action row."};
    }

    ++feasible_action_counts[row.state.value()];
  }

  // All terminal rows are generated. One new index is shared by every terminal;
  // existing task indices, incoming transitions, and rewards are untouched.
  for (const StateIndex state : terminal_states) {
    // p(state, 0 | state, bookkeeping_action) = 1 exactly.
    // Appending to rows is safe because this loop iterates over terminal_states.
    rows.push_back(StateActionDistribution{
      state,
      ActionIndex{task_action_count},
      JointOutcomeDistribution::from_outcomes(
        state_count, {{state, 0.0, 1.0}}, 1.0e-12),
    });

    feasible_action_counts[state.value()] = 1U;
  }

  // Every state must have a feasible row after terminal completion.
  for (std::size_t state_index = 0U; state_index < state_count; ++state_index) {
    if (feasible_action_counts[state_index] == 0U) {
      throw FiniteMdpException{
        FiniteMdpError::state_without_feasible_action,
        "Every state must have at least one feasible action."};
    }
  }

  // Canonical state-first, action-second order supports binary-search lookup.
  std::ranges::sort(
    rows, [](
            const StateActionDistribution & left,
            const StateActionDistribution & right) {
      if (left.state != right.state) {
        return left.state < right.state;
      }

      return left.action < right.action;
    });

  // Publish the model only after validation. std::move permits its members to
  // take ownership of these local vectors without copying their elements.
  return FiniteMdp{
    state_count,
    task_action_count,
    std::move(rows),
    std::move(terminal_states),
  };
}

FiniteMdp::FiniteMdp(
  std::size_t state_count, std::size_t task_action_count,
  std::vector<StateActionDistribution> rows,
  std::vector<StateIndex> terminal_states)
: state_count_{state_count},
  task_action_count_{task_action_count},
  rows_{std::move(rows)},
  terminal_states_{std::move(terminal_states)}
{
}

std::size_t FiniteMdp::state_count() const noexcept { return state_count_; }

std::size_t FiniteMdp::task_action_count() const noexcept
{
  return task_action_count_;
}

std::size_t FiniteMdp::action_count() const noexcept
{
  // The factory checked that this addition is representable.
  return task_action_count_ + (terminal_states_.empty() ? 0U : 1U);
}

std::optional<ActionIndex> FiniteMdp::bookkeeping_action() const noexcept
{
  if (terminal_states_.empty()) {
    return std::nullopt;
  }

  return ActionIndex{task_action_count_};
}

std::span<const StateActionDistribution> FiniteMdp::rows() const noexcept
{
  return rows_;
}

std::optional<std::reference_wrapper<const JointOutcomeDistribution>>
FiniteMdp::find_outcome_distribution(
  const StateIndex state, const ActionIndex action) const
{
  if (state.value() >= state_count_) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_current_state,
      "The query contains an invalid current-state index."};
  }

  if (action.value() >= action_count()) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_action,
      "The query contains an invalid action index."};
  }

  const std::pair<StateIndex, ActionIndex> target{state, action};

  /*
  Search the sorted range [rows_.begin(), rows_.end()) for target = (state, action).
  The end iterator marks the position after the last row and is excluded.

  The comparator returns true when a candidate precedes target in state-action order:
  - true: discard the candidate and all earlier rows; search to its right.
  - false: the candidate may be the answer; search earlier rows for the boundary.

  lower_bound uses binary search to find the first row for which the comparator
  returns false, or returns rows_.end() if every row precedes target.

  The returned row may follow target without matching it, so check for an exact
  state-action match before using its distribution.
  */
  const auto row = std::lower_bound(
    rows_.begin(), rows_.end(), target,
    [](
      const StateActionDistribution & candidate,
      const std::pair<StateIndex, ActionIndex> & key) {
      if (candidate.state != key.first) {
        return candidate.state < key.first;
      }

      return candidate.action < key.second;
    });

  if (row == rows_.end() || row->state != state || row->action != action) {
    return std::nullopt;
  }

  return std::cref(row->outcome_distribution);
}

bool FiniteMdp::is_terminal(StateIndex state) const
{
  if (state.value() >= state_count_) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_current_state,
      "The query contains an invalid state index."};
  }

  return std::ranges::binary_search(terminal_states_, state);
}

}  // namespace intelligence_foundations
