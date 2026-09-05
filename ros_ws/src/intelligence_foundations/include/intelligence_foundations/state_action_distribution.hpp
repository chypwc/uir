#ifndef INTELLIGENCE_FOUNDATIONS_STATE_ACTION_DISTRIBUTION_HPP_
#define INTELLIGENCE_FOUNDATIONS_STATE_ACTION_DISTRIBUTION_HPP_

#include "intelligence_foundations/joint_outcome_distribution.hpp"
#include "intelligence_foundations/state_action_index.hpp"

namespace intelligence_foundations
{
struct StateActionDistribution
{
  StateIndex state;
  ActionIndex action;
  JointOutcomeDistribution outcome_distribution;
};

}  // namespace intelligence_foundations

#endif  // INTELLIGENCE_FOUNDATIONS_STATE_ACTION_DISTRIBUTION_HPP_
