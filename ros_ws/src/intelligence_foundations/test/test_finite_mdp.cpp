#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

#include "intelligence_foundations/finite_mdp.hpp"
#include "intelligence_foundations/finite_mdp_error.hpp"
#include "intelligence_foundations/joint_outcome_distribution.hpp"
#include "intelligence_foundations/outcome_probability.hpp"
#include "intelligence_foundations/state_action_distribution.hpp"
#include "intelligence_foundations/state_action_index.hpp"

using intelligence_foundations::ActionIndex;
using intelligence_foundations::FiniteMdp;
using intelligence_foundations::FiniteMdpError;
using intelligence_foundations::FiniteMdpException;
using intelligence_foundations::JointOutcomeDistribution;
using intelligence_foundations::OutcomeProbability;
using intelligence_foundations::StateActionDistribution;
using intelligence_foundations::StateIndex;

namespace
{

std::vector<StateActionDistribution> make_single_valid_row()
{
  // For each pair, the next state is always s_0.
  const JointOutcomeDistribution outcome_distribution =
    JointOutcomeDistribution::from_outcomes(
      1U, {{StateIndex{0}, 0.0, 1.0}}, 1.0e-12);

  return {
    {StateIndex{0}, ActionIndex{0}, outcome_distribution},
  };
}

FiniteMdp make_lookup_fixture()
{
  /*
  (s_0, a_0) -> (s_0, 10)
  (s_0, a_2) -> (s_0, 20)
  (s_1, a_0) -> (s_0, 30)
  */
  const auto make_distribution = [](double reward) {
    return JointOutcomeDistribution::from_outcomes(
      2U, {{StateIndex{0}, reward, 1.0}}, 1.0e-12);
  };

  return FiniteMdp::from_rows(
    2U, 3U,
    {
      {StateIndex{1}, ActionIndex{0}, make_distribution(30.0)},
      {StateIndex{0}, ActionIndex{2}, make_distribution(20.0)},
      {StateIndex{0}, ActionIndex{0}, make_distribution(10.0)},
    });
}

}  // namespace

TEST(FiniteMdpTest, PreservesPositiveDomainSizesAndRows)
{
  const auto rows = make_single_valid_row();
  const FiniteMdp mdp = FiniteMdp::from_rows(1U, 1U, rows);

  EXPECT_EQ(mdp.state_count(), 1U);
  EXPECT_EQ(mdp.task_action_count(), 1U);
  EXPECT_EQ(mdp.action_count(), 1U);
  EXPECT_FALSE(mdp.bookkeeping_action().has_value());

  const auto stored_rows = mdp.rows();
  ASSERT_EQ(stored_rows.size(), 1U);
  EXPECT_EQ(stored_rows[0].state, StateIndex{0});
  EXPECT_EQ(stored_rows[0].action, ActionIndex{0});
}

TEST(FiniteMdpTest, RejectsZeroStateCount)
{
  const auto rows = make_single_valid_row();

  try {
    static_cast<void>(FiniteMdp::from_rows(0U, 1U, rows));

    FAIL() << "Expected construction to reject a zero state count.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_state_count);
  }
}

TEST(FiniteMdpTest, RejectsZeroActionCount)
{
  const auto rows = make_single_valid_row();

  try {
    static_cast<void>(FiniteMdp::from_rows(1U, 0U, rows));

    FAIL() << "Expected construction to reject a zero action count.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_action_count);
  }
}

TEST(FiniteMdpTest, RejectsOutOfRangeCurrentState)
{
  auto rows = make_single_valid_row();
  rows[0].state = StateIndex{1};

  try {
    static_cast<void>(FiniteMdp::from_rows(1U, 1U, rows));

    FAIL() << "Expected construction to reject current-state index 1 "
              "for a one-state model.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_current_state);
  }
}

TEST(FiniteMdpTest, RejectsOutOfRangeAction)
{
  auto rows = make_single_valid_row();
  rows[0].action = ActionIndex{1};

  try {
    static_cast<void>(FiniteMdp::from_rows(1U, 1U, rows));

    FAIL() << "Expected construction to reject action index 1 "
              "for a one-action model.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_action);
  }
}

TEST(FiniteMdpTest, RejectsIncompatibleOutcomeStateCount)
{
  const JointOutcomeDistribution outcome_distribution =
    JointOutcomeDistribution::from_outcomes(
      2U, {{StateIndex{0}, 0.0, 1.0}}, 1.0e-12);

  const std::vector<StateActionDistribution> rows{
    {StateIndex{0}, ActionIndex{0}, outcome_distribution},
  };

  try {
    static_cast<void>(FiniteMdp::from_rows(1U, 1U, rows));

    FAIL() << "Expected construction to reject a row built "
              "for a different state domain.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(
      exception.code(), FiniteMdpError::incompatible_outcome_state_count);
  }
}

TEST(FiniteMdpTest, RejectsDuplicateStateActionRow)
{
  auto rows = make_single_valid_row();

  const StateActionDistribution duplicate_row = rows[0];
  rows.push_back(duplicate_row);

  try {
    static_cast<void>(FiniteMdp::from_rows(1U, 1U, rows));

    FAIL() << "Expected construction to reject duplicate state-action rows.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::duplicate_state_action);
  }
}

TEST(FiniteMdpTest, RejectsStateWithoutFeasibleAction)
{
  const JointOutcomeDistribution outcome_distribution =
    JointOutcomeDistribution::from_outcomes(
      2U, {{StateIndex{0}, 0.0, 1.0}}, 1.0e-12);

  const std::vector<StateActionDistribution> rows{
    {StateIndex{0}, ActionIndex{0}, outcome_distribution},
  };

  try {
    static_cast<void>(FiniteMdp::from_rows(2U, 1U, rows));

    FAIL() << "Expected construction to reject state 1, "
              "which has no feasible action.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::state_without_feasible_action);
  }
}

TEST(FiniteMdpTest, DoesNotRequireEveryActionToBeFeasibleInEveryState)
{
  const auto rows = make_single_valid_row();

  // 2 action labels but only one feasible action a_0 in s_0.
  const FiniteMdp mdp = FiniteMdp::from_rows(1U, 2U, rows);

  EXPECT_EQ(mdp.rows().size(), 1U);
}

TEST(FiniteMdpTest, StoresRowsInCanonicalStateActionOrder)
{
  const JointOutcomeDistribution outcome_distribution =
    JointOutcomeDistribution::from_outcomes(
      2U, {{StateIndex{0}, 0.0, 1.0}}, 1.0e-12);

  const std::vector<StateActionDistribution> rows{
    {StateIndex{1}, ActionIndex{1}, outcome_distribution},
    {StateIndex{0}, ActionIndex{1}, outcome_distribution},
    {StateIndex{1}, ActionIndex{0}, outcome_distribution},
    {StateIndex{0}, ActionIndex{0}, outcome_distribution},
  };

  const FiniteMdp mdp = FiniteMdp::from_rows(2U, 2U, rows);

  const auto stored_rows = mdp.rows();
  ASSERT_EQ(stored_rows.size(), 4U);

  EXPECT_EQ(stored_rows[0].state, StateIndex{0});
  EXPECT_EQ(stored_rows[0].action, ActionIndex{0});

  EXPECT_EQ(stored_rows[1].state, StateIndex{0});
  EXPECT_EQ(stored_rows[1].action, ActionIndex{1});

  EXPECT_EQ(stored_rows[2].state, StateIndex{1});
  EXPECT_EQ(stored_rows[2].action, ActionIndex{0});

  EXPECT_EQ(stored_rows[3].state, StateIndex{1});
  EXPECT_EQ(stored_rows[3].action, ActionIndex{1});
}

TEST(FiniteMdpTest, ReturnsDistributionForFeasibleStateActionPair)
{
  const auto rows = make_single_valid_row();
  const FiniteMdp mdp = FiniteMdp::from_rows(1U, 1U, rows);

  const JointOutcomeDistribution & distribution =
    mdp.outcome_distribution(StateIndex{0}, ActionIndex{0});

  EXPECT_EQ(distribution.state_count(), 1U);
  EXPECT_NEAR(distribution.expected_reward(), 0.0, 1.0e-12);
}

TEST(FiniteMdpTest, RejectsInfeasibleStateActionQuery)
{
  const auto rows = make_single_valid_row();
  const FiniteMdp mdp = FiniteMdp::from_rows(1U, 2U, rows);

  try {
    static_cast<void>(mdp.outcome_distribution(StateIndex{0}, ActionIndex{1}));

    FAIL() << "Expected lookup to reject an infeasible action.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::infeasible_state_action);
  }
}

TEST(FiniteMdpTest, RejectsInvalidStateInDistributionQuery)
{
  const auto rows = make_single_valid_row();
  const FiniteMdp mdp = FiniteMdp::from_rows(1U, 1U, rows);

  try {
    static_cast<void>(mdp.outcome_distribution(StateIndex{1}, ActionIndex{0}));

    FAIL() << "Expected lookup to reject an invalid state index.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_current_state);
  }
}

TEST(FiniteMdpTest, RejectsInvalidActionInDistributionQuery)
{
  const auto rows = make_single_valid_row();
  const FiniteMdp mdp = FiniteMdp::from_rows(1U, 1U, rows);

  try {
    static_cast<void>(mdp.outcome_distribution(StateIndex{0}, ActionIndex{1}));

    FAIL() << "Expected lookup to reject an invalid action index.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_action);
  }
}

TEST(FiniteMdpTest, LooksUpFirstMiddleAndLastRows)
{
  const FiniteMdp mdp = make_lookup_fixture();

  EXPECT_DOUBLE_EQ(
    mdp.outcome_distribution(StateIndex{0}, ActionIndex{0}).expected_reward(),
    10.0);

  EXPECT_DOUBLE_EQ(
    mdp.outcome_distribution(StateIndex{0}, ActionIndex{2}).expected_reward(),
    20.0);

  EXPECT_DOUBLE_EQ(
    mdp.outcome_distribution(StateIndex{1}, ActionIndex{0}).expected_reward(),
    30.0);
}

TEST(FiniteMdpTest, RejectsMissingPairBetweenStoredKeys)
{
  const FiniteMdp mdp = make_lookup_fixture();

  try {
    static_cast<void>(mdp.outcome_distribution(StateIndex{0}, ActionIndex{1}));

    FAIL() << "Expected lookup to reject a missing pair between stored keys.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::infeasible_state_action);
  }
}

TEST(FiniteMdpTest, RejectsMissingPairAfterLastStoredKey)
{
  const FiniteMdp mdp = make_lookup_fixture();

  try {
    static_cast<void>(mdp.outcome_distribution(StateIndex{1}, ActionIndex{2}));

    FAIL() << "Expected lookup to reject a missing pair after the last key.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::infeasible_state_action);
  }
}

TEST(FiniteMdpTest, RejectsOutOfRangeTerminalState)
{
  const auto rows = make_single_valid_row();

  try {
    static_cast<void>(FiniteMdp::from_rows(1U, 1U, rows, {StateIndex{1}}));

    FAIL() << "Expected construction to reject terminal-state index 1 "
              "for a one-state model.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_terminal_state);
  }
}

TEST(FiniteMdpTest, RejectsDuplicateTerminalState)
{
  const auto rows = make_single_valid_row();

  try {
    static_cast<void>(
      FiniteMdp::from_rows(1U, 1U, rows, {StateIndex{0}, StateIndex{0}}));

    FAIL() << "Expected construction to reject duplicate terminal states.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::duplicate_terminal_state);
  }
}

TEST(FiniteMdpTest, RejectsSuppliedTerminalSelfTransition)
{
  const auto rows = make_single_valid_row();

  try {
    static_cast<void>(FiniteMdp::from_rows(1U, 1U, rows, {StateIndex{0}}));

    FAIL() << "Expected even an absorbing terminal row to be rejected.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_terminal_structure);
  }
}

TEST(FiniteMdpTest, GeneratesTerminalRowAndPreservesIncomingReward)
{
  const auto incoming = JointOutcomeDistribution::from_outcomes(
    2U, {{StateIndex{1}, 7.0, 1.0}}, 1.0e-12);

  const std::vector<StateActionDistribution> rows{
    {StateIndex{0}, ActionIndex{0}, incoming},
  };

  const FiniteMdp mdp = FiniteMdp::from_rows(2U, 1U, rows, {StateIndex{1}});

  EXPECT_EQ(mdp.task_action_count(), 1U);
  EXPECT_EQ(mdp.action_count(), 2U);
  ASSERT_TRUE(mdp.bookkeeping_action().has_value());
  EXPECT_EQ(*mdp.bookkeeping_action(), ActionIndex{1});
  ASSERT_EQ(mdp.rows().size(), 2U);

  const auto incoming_outcomes =
    mdp.outcome_distribution(StateIndex{0}, ActionIndex{0}).outcomes();
  ASSERT_EQ(incoming_outcomes.size(), 1U);
  EXPECT_EQ(incoming_outcomes[0].next_state, StateIndex{1});
  EXPECT_DOUBLE_EQ(incoming_outcomes[0].reward, 7.0);
  EXPECT_DOUBLE_EQ(incoming_outcomes[0].probability, 1.0);
  EXPECT_DOUBLE_EQ(
    mdp.outcome_distribution(StateIndex{0}, ActionIndex{0}).expected_reward(),
    7.0);

  const auto terminal_outcomes =
    mdp.outcome_distribution(StateIndex{1}, ActionIndex{1}).outcomes();

  ASSERT_EQ(terminal_outcomes.size(), 1U);
  EXPECT_EQ(terminal_outcomes[0].next_state, StateIndex{1});
  EXPECT_DOUBLE_EQ(terminal_outcomes[0].reward, 0.0);
  EXPECT_DOUBLE_EQ(terminal_outcomes[0].probability, 1.0);
}

TEST(FiniteMdpTest, SharesGeneratedActionAndCanonicalOrderAcrossTerminals)
{
  const auto incoming = JointOutcomeDistribution::from_outcomes(
    4U, {{StateIndex{0}, 7.0, 0.25}, {StateIndex{2}, -3.0, 0.75}}, 1.0e-12);
  std::vector<StateActionDistribution> rows{
    {StateIndex{3}, ActionIndex{1}, incoming},
    {StateIndex{1}, ActionIndex{0}, incoming},
  };
  const auto mdp =
    FiniteMdp::from_rows(4U, 2U, rows, {StateIndex{2}, StateIndex{0}});
  std::ranges::reverse(rows);
  const auto reordered =
    FiniteMdp::from_rows(4U, 2U, rows, {StateIndex{0}, StateIndex{2}});

  EXPECT_EQ(mdp.task_action_count(), 2U);
  EXPECT_EQ(mdp.action_count(), 3U);
  ASSERT_TRUE(mdp.bookkeeping_action().has_value());
  EXPECT_EQ(*mdp.bookkeeping_action(), ActionIndex{2});
  ASSERT_EQ(mdp.rows().size(), 4U);
  ASSERT_EQ(reordered.rows().size(), 4U);
  const std::vector<ActionIndex> expected_actions{
    ActionIndex{2}, ActionIndex{0}, ActionIndex{2}, ActionIndex{1}};

  for (std::size_t i = 0U; i < mdp.rows().size(); ++i) {
    SCOPED_TRACE(::testing::Message() << "state index = " << i);
    const auto & row = mdp.rows()[i];
    const auto & reordered_row = reordered.rows()[i];
    EXPECT_EQ(row.state, StateIndex{i});
    EXPECT_EQ(row.action, expected_actions[i]);
    EXPECT_EQ(row.state, reordered_row.state);
    EXPECT_EQ(row.action, reordered_row.action);
    const auto outcomes = row.outcome_distribution.outcomes();
    const auto reordered_outcomes =
      reordered_row.outcome_distribution.outcomes();
    ASSERT_EQ(outcomes.size(), reordered_outcomes.size());
    for (std::size_t j = 0U; j < outcomes.size(); ++j) {
      EXPECT_EQ(outcomes[j].next_state, reordered_outcomes[j].next_state);
      EXPECT_DOUBLE_EQ(outcomes[j].reward, reordered_outcomes[j].reward);
      EXPECT_DOUBLE_EQ(
        outcomes[j].probability, reordered_outcomes[j].probability);
    }

    if (i == 0U || i == 2U) {
      ASSERT_EQ(outcomes.size(), 1U);
      EXPECT_EQ(outcomes[0].next_state, StateIndex{i});
      EXPECT_DOUBLE_EQ(outcomes[0].reward, 0.0);
      EXPECT_DOUBLE_EQ(outcomes[0].probability, 1.0);
    } else {
      ASSERT_EQ(outcomes.size(), 2U);
      EXPECT_EQ(outcomes[0].next_state, StateIndex{0});
      EXPECT_DOUBLE_EQ(outcomes[0].reward, 7.0);
      EXPECT_DOUBLE_EQ(outcomes[0].probability, 0.25);
      EXPECT_EQ(outcomes[1].next_state, StateIndex{2});
      EXPECT_DOUBLE_EQ(outcomes[1].reward, -3.0);
      EXPECT_DOUBLE_EQ(outcomes[1].probability, 0.75);
    }
  }
}

TEST(FiniteMdpTest, RejectsGeneratedActionInSuppliedNonterminalRow)
{
  const auto distribution = JointOutcomeDistribution::from_outcomes(
    2U, {{StateIndex{1}, 0.0, 1.0}}, 1.0e-12);

  try {
    static_cast<void>(FiniteMdp::from_rows(
      2U, 1U, {{StateIndex{0}, ActionIndex{1}, distribution}},
      {StateIndex{1}}));

    FAIL() << "The generated action must not be accepted as a task action.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_action);
  }
}

TEST(FiniteMdpTest, RejectsActionCountOverflow)
{
  try {
    static_cast<void>(FiniteMdp::from_rows(
      1U, std::numeric_limits<std::size_t>::max(), {}, {StateIndex{0}}));

    FAIL() << "Expected rejection before adding one to the maximum count.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_action_count);
  }
}

TEST(FiniteMdpTest, AcceptsMaximumActionCountWithoutTerminals)
{
  const auto mdp = FiniteMdp::from_rows(
    1U, std::numeric_limits<std::size_t>::max(), make_single_valid_row());

  EXPECT_EQ(mdp.action_count(), std::numeric_limits<std::size_t>::max());
  EXPECT_EQ(mdp.task_action_count(), mdp.action_count());
  EXPECT_FALSE(mdp.bookkeeping_action().has_value());
}

TEST(FiniteMdpTest, AcceptsAllTerminalModelWithoutTaskActions)
{
  const auto mdp =
    FiniteMdp::from_rows(2U, 0U, {}, {StateIndex{1}, StateIndex{0}});

  EXPECT_EQ(mdp.task_action_count(), 0U);
  EXPECT_EQ(mdp.action_count(), 1U);
  ASSERT_TRUE(mdp.bookkeeping_action().has_value());
  EXPECT_EQ(*mdp.bookkeeping_action(), ActionIndex{0});
  ASSERT_EQ(mdp.rows().size(), 2U);
  for (std::size_t i = 0U; i < 2U; ++i) {
    EXPECT_TRUE(mdp.is_terminal(StateIndex{i}));

    const auto outcomes =
      mdp.outcome_distribution(StateIndex{i}, ActionIndex{0}).outcomes();

    ASSERT_EQ(outcomes.size(), 1U);
    EXPECT_EQ(outcomes[0].next_state, StateIndex{i});
    EXPECT_DOUBLE_EQ(outcomes[0].reward, 0.0);
    EXPECT_DOUBLE_EQ(outcomes[0].probability, 1.0);
  }
}

TEST(FiniteMdpTest, RejectsZeroTaskActionsWithNonterminalState)
{
  try {
    static_cast<void>(FiniteMdp::from_rows(2U, 0U, {}, {StateIndex{1}}));

    FAIL() << "A bookkeeping action cannot replace missing task actions.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::invalid_action_count);
  }
}

TEST(FiniteMdpTest, TerminalCompletionDoesNotFillMissingNonterminalRows)
{
  try {
    static_cast<void>(FiniteMdp::from_rows(2U, 1U, {}, {StateIndex{1}}));

    FAIL() << "Terminal completion must not invent nonterminal dynamics.";
  } catch (const FiniteMdpException & exception) {
    EXPECT_EQ(exception.code(), FiniteMdpError::state_without_feasible_action);
  }
}

TEST(FiniteMdpTest, KeepsTaskAndBookkeepingFeasibilitySeparate)
{
  const auto incoming = JointOutcomeDistribution::from_outcomes(
    2U, {{StateIndex{1}, 7.0, 1.0}}, 1.0e-12);
  const auto mdp = FiniteMdp::from_rows(
    2U, 1U, {{StateIndex{0}, ActionIndex{0}, incoming}}, {StateIndex{1}});

  for (const StateIndex state : {StateIndex{0}, StateIndex{1}}) {
    // At state 0 only task action 0 is feasible; at state 1 only action 1 is.
    const ActionIndex infeasible_action{1U - state.value()};
    SCOPED_TRACE(::testing::Message() << "state index = " << state.value());
    try {
      static_cast<void>(mdp.outcome_distribution(state, infeasible_action));

      FAIL() << "Expected rejection of an in-domain but infeasible action.";
    } catch (const FiniteMdpException & exception) {
      EXPECT_EQ(exception.code(), FiniteMdpError::infeasible_state_action);
    }
  }
}

TEST(FiniteMdpTest, ReportsDeclaredTerminalMembership)
{
  const auto self_loop = JointOutcomeDistribution::from_outcomes(
    3U, {{StateIndex{1}, 0.0, 1.0}}, 1.0e-12);

  const auto mdp = FiniteMdp::from_rows(
    3U, 1U, {{StateIndex{1}, ActionIndex{0}, self_loop}},
    {StateIndex{2}, StateIndex{0}});

  EXPECT_TRUE(mdp.is_terminal(StateIndex{0}));
  EXPECT_FALSE(mdp.is_terminal(StateIndex{1}));
  EXPECT_TRUE(mdp.is_terminal(StateIndex{2}));
}

TEST(FiniteMdpTest, RejectsInvalidStateInTerminalQuery)
{
  const auto mdp = FiniteMdp::from_rows(1U, 1U, make_single_valid_row());

  EXPECT_FALSE(mdp.is_terminal(StateIndex{0}));

  for (const StateIndex state : {StateIndex{1}, StateIndex{2}}) {
    SCOPED_TRACE(::testing::Message() << "state index = " << state.value());

    try {
      static_cast<void>(mdp.is_terminal(state));

      FAIL() << "Expected terminal query to reject an invalid state.";
    } catch (const FiniteMdpException & exception) {
      EXPECT_EQ(exception.code(), FiniteMdpError::invalid_current_state);
    }
  }
}
