#include "rigid_body_kinematics/joint_limits.hpp"

#include <cmath>
#include <optional>

#include "rigid_body_kinematics/serial_chain_error.hpp"

namespace rigid_body_kinematics
{
namespace
{

void validate_bounds(
  const std::optional<double> & lower_bound,
  const std::optional<double> & upper_bound)
{
  if (
    (lower_bound.has_value() && !std::isfinite(*lower_bound)) ||
    (upper_bound.has_value() && !std::isfinite(*upper_bound))) {
    throw SerialChainException(
      SerialChainError::invalid_joint_limits,
      "Present joint-limit bounds must be finite.");
  }

  if (
    lower_bound.has_value() && upper_bound.has_value() &&
    *lower_bound > *upper_bound) {
    throw SerialChainException(
      SerialChainError::invalid_joint_limits,
      "The lower joint limit must not exceed the upper joint limit.");
  }
}

bool interval_contains(
  const std::optional<double> & lower_bound,
  const std::optional<double> & upper_bound, double value) noexcept
{
  if (!std::isfinite(value)) {
    return false;
  }

  return (!lower_bound.has_value() || *lower_bound <= value) &&
         (!upper_bound.has_value() || value <= *upper_bound);
}

}  // namespace

RevoluteLimits RevoluteLimits::from_bounds(
  std::optional<double> lower_bound_radians,
  std::optional<double> upper_bound_radians)
{
  validate_bounds(lower_bound_radians, upper_bound_radians);

  return RevoluteLimits(lower_bound_radians, upper_bound_radians);
}

RevoluteLimits::RevoluteLimits(
  std::optional<double> lower_bound_radians,
  std::optional<double> upper_bound_radians)
: lower_bound_radians_(lower_bound_radians),
  upper_bound_radians_(upper_bound_radians)
{
}

std::optional<double> RevoluteLimits::lower_bound_radians() const noexcept
{
  return lower_bound_radians_;
}

std::optional<double> RevoluteLimits::upper_bound_radians() const noexcept
{
  return upper_bound_radians_;
}

bool RevoluteLimits::contains(double angle_radians) const noexcept
{
  return interval_contains(
    lower_bound_radians_, upper_bound_radians_, angle_radians);
}

PrismaticLimits PrismaticLimits::from_bounds(
  std::optional<double> lower_bound_metres,
  std::optional<double> upper_bound_metres)
{
  validate_bounds(lower_bound_metres, upper_bound_metres);

  return PrismaticLimits(lower_bound_metres, upper_bound_metres);
}

PrismaticLimits::PrismaticLimits(
  std::optional<double> lower_bound_metres,
  std::optional<double> upper_bound_metres)
: lower_bound_metres_(lower_bound_metres),
  upper_bound_metres_(upper_bound_metres)
{
}

std::optional<double> PrismaticLimits::lower_bound_metres() const noexcept
{
  return lower_bound_metres_;
}

std::optional<double> PrismaticLimits::upper_bound_metres() const noexcept
{
  return upper_bound_metres_;
}

bool PrismaticLimits::contains(double displacement_metres) const noexcept
{
  return interval_contains(
    lower_bound_metres_, upper_bound_metres_, displacement_metres);
}
}  // namespace rigid_body_kinematics
