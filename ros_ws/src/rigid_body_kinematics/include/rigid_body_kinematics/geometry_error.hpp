#ifndef RIGID_BODY_KINEMATICS__GEOMETRY_ERROR_HPP_
#define RIGID_BODY_KINEMATICS__GEOMETRY_ERROR_HPP_

#include <stdexcept>

namespace rigid_body_kinematics
{
enum class GeometryError
{
  non_finite,
  invalid_rotation,
  invalid_transform,
  invalid_tangent,
  invalid_policy,        // a tolerance is invalid.
  unsupported_magnitude  // exceeds the finite numerical domain
};

/*
        Exception carrying both a human-readable message and a machine-readable
        GeometryError code.

        Example:
        GeometryException e(
            GeometryError::non_finite,
            "Rotation matrix must contain only finite values"
        );

        e.what();
        // -> "Rotation matrix must contain only finite values"

        e.code();
        // -> GeometryError::non_finite
    */
class GeometryException : public std::runtime_error
{
public:
  GeometryException(GeometryError code, const char * message)
  : std::runtime_error(message), code_(code)
  {
  }

  [[nodiscard]] GeometryError code() const noexcept { return code_; }

private:
  GeometryError code_;
};
}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS__GEOMETRY_ERROR_HPP_
