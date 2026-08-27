#ifndef RIGID_BODY_KINEMATICS_SERIAL_CHAIN_ERROR_HPP_
#define RIGID_BODY_KINEMATICS_SERIAL_CHAIN_ERROR_HPP_

#include <stdexcept>
#include <string>

namespace rigid_body_kinematics
{

enum class SerialChainError
{
  non_finite,
  invalid_model,
  invalid_joint_limits,
  dimension_mismatch,
  joint_out_of_domain,
  invalid_policy,
  unsupported_magnitude,
};

class SerialChainException final : public std::runtime_error
{
public:
  SerialChainException(SerialChainError code, const std::string & message)
  : std::runtime_error(message), code_(code)
  {
  }

  [[nodiscard]] SerialChainError code() const noexcept { return code_; }

private:
  SerialChainError code_;
};

}  // namespace rigid_body_kinematics

#endif  // RIGID_BODY_KINEMATICS_SERIAL_CHAIN_ERROR_HPP_
