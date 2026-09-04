#ifndef INTELLIGENCE_FOUNDATIONS_STATE_ACTION_INDEX_HPP_
#define INTELLIGENCE_FOUNDATIONS_STATE_ACTION_INDEX_HPP_

#include <compare>
#include <cstddef>

namespace intelligence_foundations
{
class StateIndex
{
private:
  std::size_t value_;

public:
  explicit constexpr StateIndex(std::size_t value) noexcept : value_{value} {}

  [[nodiscard]] constexpr std::size_t value() const noexcept { return value_; }

  constexpr auto operator<=>(const StateIndex &) const noexcept = default;
};

class ActionIndex
{
private:
  std::size_t value_;

public:
  explicit constexpr ActionIndex(std::size_t value) noexcept : value_{value} {}

  [[nodiscard]] constexpr std::size_t value() const noexcept { return value_; }

  constexpr auto operator<=>(const ActionIndex &) const noexcept = default;
};

}  // namespace intelligence_foundations

#endif  // INTELLIGENCE_FOUNDATIONS_STATE_ACTION_INDEX_HPP_
