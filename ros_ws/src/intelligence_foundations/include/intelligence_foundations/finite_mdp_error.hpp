#ifndef INTELLIGENCE_FOUNDATIONS_FINITE_MDP_ERROR_HPP_
#define INTELLIGENCE_FOUNDATIONS_FINITE_MDP_ERROR_HPP_

#include <stdexcept>
#include <string>

namespace intelligence_foundations
{

enum class FiniteMdpError
{
  invalid_state_count,
  invalid_action_count,
  invalid_current_state,
  invalid_action,
  incompatible_outcome_state_count,
  invalid_probability_sum_tolerance,
  empty_outcome_distribution,
  invalid_next_state,
  non_finite_reward,
  invalid_probability,
  invalid_probability_sum,
  duplicate_outcome,
  duplicate_state_action,
  state_without_feasible_action,
  infeasible_state_action,
  invalid_terminal_state,
  duplicate_terminal_state,
  invalid_terminal_structure,
};

class FiniteMdpException : public std::invalid_argument
{
private:
  FiniteMdpError code_;

public:
  FiniteMdpException(FiniteMdpError code, const std::string & message)
  : std::invalid_argument{message}, code_{code}
  {
  }

  [[nodiscard]] FiniteMdpError code() const noexcept { return code_; }
};

}  // namespace intelligence_foundations

#endif  // INTELLIGENCE_FOUNDATIONS_FINITE_MDP_ERROR_HPP_
