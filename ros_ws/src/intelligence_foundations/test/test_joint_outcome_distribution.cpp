#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <vector>

#include "intelligence_foundations/finite_mdp_error.hpp"
#include "intelligence_foundations/joint_outcome_distribution.hpp"
#include "intelligence_foundations/outcome_probability.hpp"
#include "intelligence_foundations/state_action_index.hpp"

using intelligence_foundations::FiniteMdpError;
using intelligence_foundations::FiniteMdpException;
using intelligence_foundations::JointOutcomeDistribution;
using intelligence_foundations::OutcomeProbability;
using intelligence_foundations::StateIndex;

TEST(JointOutcomeDistributionTest, AcceptsValidSparseDistribution)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, -1.0, 0.2},
    {StateIndex{1}, 4.0, 0.3},
    {StateIndex{1}, 5.0, 0.5},
  };

  const JointOutcomeDistribution distribution =
    JointOutcomeDistribution::from_outcomes(3U, outcomes, 1.0e-12);

  EXPECT_EQ(distribution.state_count(), 3U);

  const auto stored_outcomes = distribution.outcomes();
  ASSERT_EQ(stored_outcomes.size(), outcomes.size());

  for (std::size_t index = 0; index < outcomes.size(); ++index) {
    EXPECT_EQ(stored_outcomes[index].next_state, outcomes[index].next_state);
    EXPECT_DOUBLE_EQ(stored_outcomes[index].reward, outcomes[index].reward);
    EXPECT_DOUBLE_EQ(
      stored_outcomes[index].probability, outcomes[index].probability);
  }
}

TEST(JointOutcomeDistributionTest, RejectsProbabilitySumDifferentFromOne)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, -1.0, 0.4},
    {StateIndex{1}, 2.0, 0.5},
  };

  try {
    static_cast<void>(
      JointOutcomeDistribution::from_outcomes(2U, outcomes, 1.0e-12));

    FAIL() << "Expected construction to reject a probability sum of 0.9.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_probability_sum);
  }
}

TEST(JointOutcomeDistributionTest, RejectsZeroStateCount)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, 0.0, 1.0},
  };

  try {
    static_cast<void>(
      JointOutcomeDistribution::from_outcomes(0U, outcomes, 1e-12));

    FAIL() << "Expected construction to reject a zero state count.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_state_count);
  }
}

TEST(JointOutcomeDistributionTest, RejectsEmptyOutcomeDistribution)
{
  const std::vector<OutcomeProbability> outcomes{};

  try {
    static_cast<void>(
      JointOutcomeDistribution::from_outcomes(2U, outcomes, 1.0e-12));

    FAIL() << "Expected construction to reject an empty distribution.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::empty_outcome_distribution);
  }
}

TEST(JointOutcomeDistributionTest, RejectsOutOfRangeNextState)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{2}, 0.0, 1.0},
  };

  try {
    static_cast<void>(
      JointOutcomeDistribution::from_outcomes(2U, outcomes, 1.0e-12));

    FAIL() << "Expected construction to reject next-state index 2 "
              "for a two-state model.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_next_state);
  }
}

TEST(JointOutcomeDistributionTest, RejectsNonFiniteReward)
{
  const std::vector<OutcomeProbability> outcomes{
    {
      StateIndex{0},
      std::numeric_limits<double>::quiet_NaN(),
      1.0,
    },
  };

  try {
    static_cast<void>(
      JointOutcomeDistribution::from_outcomes(1U, outcomes, 1.0e-12));

    FAIL() << "Expected construction to reject a NaN reward.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::non_finite_reward);
  }
}

TEST(JointOutcomeDistributionTest, RejectsZeroProbability)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, 0.0, 0.0},
  };

  try {
    static_cast<void>(
      JointOutcomeDistribution::from_outcomes(1U, outcomes, 1.0e-12));

    FAIL() << "Expected sparse storage to reject a zero probability.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_probability);
  }
}

TEST(JointOutcomeDistributionTest, RejectsDuplicateJointOutcome)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, 2.0, 0.4},
    {StateIndex{0}, 2.0, 0.6},
  };

  try {
    static_cast<void>(
      JointOutcomeDistribution::from_outcomes(1U, outcomes, 1.0e-12));

    FAIL() << "Expected construction to reject a duplicate joint outcome.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::duplicate_outcome);
  }
}

TEST(JointOutcomeDistributionTest, RejectsInvalidStoredProbabilities)
{
  const std::vector<double> invalid_probabilities{
    -0.1,
    1.1,
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN(),
  };

  for (const double probability : invalid_probabilities) {
    SCOPED_TRACE(::testing::Message() << "probability = " << probability);

    const std::vector<OutcomeProbability> outcomes{
      {StateIndex{0}, 0.0, probability},
    };

    try {
      static_cast<void>(
        JointOutcomeDistribution::from_outcomes(1U, outcomes, 1.0e-12));

      ADD_FAILURE() << "Expected construction to reject the probability.";
    } catch (const FiniteMdpException & exception) {
      EXPECT_EQ(exception.code(), FiniteMdpError::invalid_probability);
    }  // try
  }  // for
}

TEST(JointOutcomeDistributionTest, RejectsInvalidProbabilitySumTolerances)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, 0.0, 1.0},
  };

  const std::vector<double> invalid_tolerances{
    0.0,
    -1.0e-12,
    1.0e-11,
    1.0,
    1.1,
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN(),
  };

  for (const double tolerance : invalid_tolerances) {
    SCOPED_TRACE(::testing::Message() << "tolerance = " << tolerance);

    try {
      static_cast<void>(
        JointOutcomeDistribution::from_outcomes(1U, outcomes, tolerance));

      ADD_FAILURE() << "Expected construction to reject the tolerance.";
    } catch (const FiniteMdpException & exception) {
      EXPECT_EQ(
        exception.code(), FiniteMdpError::invalid_probability_sum_tolerance);
    }
  }
}

TEST(
  JointOutcomeDistributionTest,
  ComputesDenseNextStateProbabilitiesBySummingOverRewards)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, -1.0, 0.2},
    {StateIndex{1}, 4.0, 0.3},
    {StateIndex{1}, 5.0, 0.5},
  };

  const JointOutcomeDistribution distribution =
    JointOutcomeDistribution::from_outcomes(3U, outcomes, 1.0e-12);

  const std::vector<double> probabilities =
    distribution.next_state_probabilities();

  ASSERT_EQ(probabilities.size(), 3U);
  EXPECT_NEAR(probabilities[0], 0.2, 1.0e-12);
  EXPECT_NEAR(probabilities[1], 0.8, 1.0e-12);
  EXPECT_NEAR(probabilities[2], 0.0, 1.0e-12);
}

TEST(JointOutcomeDistributionTest, ComputesExpectedOneStepReward)
{
  const std::vector<OutcomeProbability> outcomes{
    {StateIndex{0}, -1.0, 0.2},
    {StateIndex{1}, 4.0, 0.3},
    {StateIndex{1}, 5.0, 0.5},
  };

  const JointOutcomeDistribution distribution =
    JointOutcomeDistribution::from_outcomes(3U, outcomes, 1.0e-12);

  EXPECT_NEAR(distribution.expected_reward(), 3.5, 1.0e-12);
}
