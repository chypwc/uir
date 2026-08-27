#ifndef RIGID_BODY_KINEMATICS_JOINT_LIMITS_HPP_
#define RIGID_BODY_KINEMATICS_JOINT_LIMITS_HPP_

#include <optional>

namespace rigid_body_kinematics
{

class RevoluteLimits
{
public:
  static RevoluteLimits from_bounds(
    std::optional<double> lower_bound_radians,
    std::optional<double> upper_bound_radians);

  [[nodiscard]] std::optional<double> lower_bound_radians() const noexcept;
  [[nodiscard]] std::optional<double> upper_bound_radians() const noexcept;

  [[nodiscard]] bool contains(double angle_radians) const noexcept;

private:
  RevoluteLimits(
    std::optional<double> lower_bound_radians,
    std::optional<double> upper_bound_radians);

  std::optional<double> lower_bound_radians_;
  std::optional<double> upper_bound_radians_;
};

class PrismaticLimits
{
public:
  static PrismaticLimits from_bounds(
    std::optional<double> lower_bound_metres,
    std::optional<double> upper_bound_metres);

  [[nodiscard]] std::optional<double> lower_bound_metres() const noexcept;
  [[nodiscard]] std::optional<double> upper_bound_metres() const noexcept;

  [[nodiscard]] bool contains(double displacement_metres) const noexcept;

private:
  PrismaticLimits(
    std::optional<double> lower_bound_metres,
    std::optional<double> upper_bound_metres);

  std::optional<double> lower_bound_metres_;
  std::optional<double> upper_bound_metres_;
};

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_JOINT_LIMITS_HPP_
