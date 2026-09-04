#include <gtest/gtest.h>

#include <cstddef>
#include <type_traits>

#include "intelligence_foundations/state_action_index.hpp"

using intelligence_foundations::ActionIndex;
using intelligence_foundations::StateIndex;

static_assert(std::is_constructible_v<StateIndex, std::size_t>);
static_assert(std::is_constructible_v<ActionIndex, std::size_t>);

static_assert(!std::is_convertible_v<std::size_t, StateIndex>);
static_assert(!std::is_convertible_v<std::size_t, ActionIndex>);
static_assert(!std::is_convertible_v<StateIndex, ActionIndex>);
static_assert(!std::is_convertible_v<ActionIndex, StateIndex>);

TEST(StateActionIndexTest, PreservesUnderlyingIndex)
{
  EXPECT_EQ(StateIndex{3}.value(), 3U);
  EXPECT_EQ(ActionIndex{5}.value(), 5U);
}

TEST(StateActionIndexTest, ComparesIdentifiersWithinTheirOwnDomain)
{
  EXPECT_EQ(StateIndex{2}, StateIndex{2});
  EXPECT_LT(StateIndex{1}, StateIndex{2});

  EXPECT_EQ(ActionIndex{5}, ActionIndex{5});
  EXPECT_GT(ActionIndex{6}, ActionIndex{3});
}
