#ifndef INTELLIGENCE_FOUNDATIONS_FINITE_MDP_HPP_
#define INTELLIGENCE_FOUNDATIONS_FINITE_MDP_HPP_

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "intelligence_foundations/state_action_distribution.hpp"
#include "intelligence_foundations/state_action_index.hpp"

namespace intelligence_foundations
{
class FiniteMdp
{
private:
  FiniteMdp(
    std::size_t state_count, std::size_t task_action_count,
    std::vector<StateActionDistribution> rows,
    std::vector<StateIndex> terminal_states);

  std::size_t state_count_;
  std::size_t task_action_count_;
  std::vector<StateActionDistribution> rows_;
  std::vector<StateIndex> terminal_states_;

public:
  // Supply only task-action rows from nonterminal states. Terminal rows and one
  // shared bookkeeping action are generated when terminal_states is nonempty.
  // task_action_count excludes the bookkeeeping action.
  [[nodiscard]] static FiniteMdp from_rows(
    std::size_t state_count, std::size_t task_action_count,
    std::vector<StateActionDistribution> rows,
    std::vector<StateIndex> terminal_states = {});

  [[nodiscard]] std::size_t state_count() const noexcept;

  [[nodiscard]] std::size_t task_action_count() const noexcept;

  // Includes the generated bookkeeping action, if present.
  [[nodiscard]] std::size_t action_count() const noexcept;

  [[nodiscard]] std::optional<ActionIndex> bookkeeping_action() const noexcept;

  [[nodiscard]] std::span<const StateActionDistribution> rows() const noexcept;

  // Query p(s', r| s, a) given (s, a).
  [[nodiscard]] const JointOutcomeDistribution & outcome_distribution(
    StateIndex state, ActionIndex action) const;

  [[nodiscard]] bool is_terminal(StateIndex state) const;
};
}  // namespace intelligence_foundations

#endif  // INTELLIGENCE_FOUNDATIONS_FINITE_MDP_HPP_
