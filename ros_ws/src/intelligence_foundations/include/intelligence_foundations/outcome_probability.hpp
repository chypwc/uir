#ifndef INTELLIGENCE_FOUNDATIONS_OUTCOME_PROBABILITY_HPP_
#define INTELLIGENCE_FOUNDATIONS_OUTCOME_PROBABILITY_HPP_

#include "intelligence_foundations/state_action_index.hpp"

namespace intelligence_foundations
{

struct OutcomeProbability
{
  StateIndex next_state;
  double reward;
  double probability;
};

}  // namespace intelligence_foundations

#endif  // INTELLIGENCE_FOUNDATIONS_OUTCOME_PROBABILITY_HPP_
