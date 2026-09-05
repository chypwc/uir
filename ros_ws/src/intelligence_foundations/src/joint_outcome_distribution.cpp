#include "intelligence_foundations/joint_outcome_distribution.hpp"

#include <cmath>    // std::isfinite
#include <set>      // std::set
#include <utility>  // std::move
#include <vector>

#include "intelligence_foundations/finite_mdp_error.hpp"

namespace intelligence_foundations
{
namespace
{
constexpr double kMaximumProbabilitySumTolerance = 1.0e-12;

void validate_distribution(
  std::size_t state_count, const std::vector<OutcomeProbability> & outcomes,
  double probability_sum_tolerance)
{
  if (state_count == 0U) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_state_count, "The state count must be positive."};
  }

  if (
    !std::isfinite(probability_sum_tolerance) ||
    probability_sum_tolerance <= 0.0 ||
    probability_sum_tolerance > kMaximumProbabilitySumTolerance) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_probability_sum_tolerance,
      "The probability-sum tolerance must be finite and lie in (0, 1e-12]."};
  }

  if (outcomes.empty()) {
    throw FiniteMdpException{
      FiniteMdpError::empty_outcome_distribution,
      "A joint outcome distribution must not be empty."};
  }

  std::set<std::pair<StateIndex, double>> observed_outcomes;
  double probability_sum = 0.0;
  double compensation = 0.0;

  for (const OutcomeProbability & outcome : outcomes) {
    if (outcome.next_state.value() >= state_count) {
      throw FiniteMdpException{
        FiniteMdpError::invalid_next_state,
        "An outcome contains an invalid next-state identifier."};
    }

    if (!std::isfinite(outcome.reward)) {
      throw FiniteMdpException{
        FiniteMdpError::non_finite_reward,
        "An outcome contains a non-finite reward."};
    }

    // Sparse representation: only positive probabilities are stored.
    if (
      !std::isfinite(outcome.probability) || outcome.probability <= 0.0 ||
      outcome.probability > 1.0) {
      throw FiniteMdpException{
        FiniteMdpError::invalid_probability,
        "A stored outcome probability must be finite and lie in (0, 1]."};
    }

    // set.emplace() returns [iterator, inserted]
    const bool inserted =
      observed_outcomes.emplace(outcome.next_state, outcome.reward).second;

    if (!inserted) {
      throw FiniteMdpException{
        FiniteMdpError::duplicate_outcome,
        "A joint outcome distribution contains a duplicate event."};
    }

    // Kahan compensated summation
    // Correct probability by the previous rounding error.
    const double corrected_probability = outcome.probability - compensation;
    // This sum may incur rounding error.
    const double updated_sum = probability_sum + corrected_probability;

    // (updated_sum - probability_sum) recovers the portion of corrected_probability
    // that actually entered the stored sum.
    // It then compares that recovered increment with the requested increment.
    compensation = (updated_sum - probability_sum) - corrected_probability;

    // Update probability sum.
    probability_sum = updated_sum;
  }  // for

  if (std::abs(probability_sum - 1.0) > probability_sum_tolerance) {
    throw FiniteMdpException{
      FiniteMdpError::invalid_probability_sum,
      "The outcome probabilities do not sum to one."};
  }
}

}  // namespace

JointOutcomeDistribution JointOutcomeDistribution::from_outcomes(
  std::size_t state_count, std::vector<OutcomeProbability> outcomes,
  double probability_sum_tolerance)
{
  validate_distribution(state_count, outcomes, probability_sum_tolerance);

  return JointOutcomeDistribution(state_count, std::move(outcomes));
}

JointOutcomeDistribution::JointOutcomeDistribution(
  std::size_t state_count, std::vector<OutcomeProbability> outcomes)
: state_count_{state_count}, outcomes_{std::move(outcomes)}
{
}

std::size_t JointOutcomeDistribution::state_count() const noexcept
{
  return state_count_;
}

std::span<const OutcomeProbability> JointOutcomeDistribution::outcomes()
  const noexcept
{
  return std::span<const OutcomeProbability>{outcomes_};
}

std::vector<double> JointOutcomeDistribution::next_state_probabilities() const
{
  std::vector<double> probabilities(state_count_, 0.0);
  std::vector<double> compensations(state_count_, 0.0);

  // Sum p(s', r| s, a) over reward to get p(s'| s, a).
  for (const OutcomeProbability & outcome : outcomes_) {
    const std::size_t next_state_index = outcome.next_state.value();

    /*
    This is the Kahan correction for destination state (j), where j = next_state_index.
    If the previous addition into probabilities[j] lost a small amount through rounding,
    compensations[j] records that error.
    Subtracting it adjusts the next probability before addition.
    */
    const double corrected_probability =
      outcome.probability - compensations[next_state_index];

    // This sum may incur rounding error.
    const double updated_probability =
      probabilities[next_state_index] + corrected_probability;

    // (updated_probability - probabilities[next_state_index])
    // recovers the amount that actually entered the floating-point sum.
    // Comparing that amount with corrected_probability estimates the rounding error.
    compensations[next_state_index] =
      (updated_probability - probabilities[next_state_index]) -
      corrected_probability;

    probabilities[next_state_index] = updated_probability;
  }

  return probabilities;
}

double JointOutcomeDistribution::expected_reward() const noexcept
{
  double expected_reward = 0.0;
  double compensation = 0.0;

  for (const OutcomeProbability & outcome : outcomes_) {
    const double weighted_reward = outcome.reward * outcome.probability;

    const double corrected_weighted_reward = weighted_reward - compensation;

    const double updated_expected_reward =
      expected_reward + corrected_weighted_reward;

    compensation =
      (updated_expected_reward - expected_reward) - corrected_weighted_reward;

    expected_reward = updated_expected_reward;
  }

  return expected_reward;
}

}  // namespace intelligence_foundations
