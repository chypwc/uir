#ifndef INTELLIGENCE_FOUNDATIONS_JOINT_OUTCOME_DISTRIBUTION_HPP_
#define INTELLIGENCE_FOUNDATIONS_JOINT_OUTCOME_DISTRIBUTION_HPP_

#include <cstddef>
#include <span>
#include <vector>

#include "intelligence_foundations/outcome_probability.hpp"

namespace intelligence_foundations
{

class JointOutcomeDistribution
{
private:
  JointOutcomeDistribution(
    std::size_t state_count, std::vector<OutcomeProbability> outcomes);

  std::size_t state_count_;
  std::vector<OutcomeProbability> outcomes_;

public:
  [[nodiscard]] static JointOutcomeDistribution from_outcomes(
    std::size_t state_count, std::vector<OutcomeProbability> outcomes,
    double probability_sum_tolerance);

  [[nodiscard]] std::size_t state_count() const noexcept;

  [[nodiscard]] std::span<const OutcomeProbability> outcomes() const noexcept;
};

}  // namespace intelligence_foundations

#endif  // INTELLIGENCE_FOUNDATIONS_JOINT_OUTCOME_DISTRIBUTION_HPP_
