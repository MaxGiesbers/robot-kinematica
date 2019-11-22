#include <cmath>       
#include <iostream>
#include <optional>
#include "matrix_class/Matrix.hpp"
namespace Kinematics
{
class Kinematics
{
public:
  Kinematics() = default;
  ~Kinematics() = default;
  std::optional<Matrix<double, 3, 1>> forward_kinematics(Matrix<double, 4, 1>& angles) const;
  std::optional<Matrix<double, 4, 1>> inverse_kinematics(Matrix<double, 4, 1>& angles,
                                                         const Matrix<double, 3, 1>& goal) const;

private:
  Matrix<double, 3, 3> compute_jacobian(const Matrix<double, 4, 1>& angles) const;
  bool check_angles(const Matrix<double, 4, 1>& angles) const;
  Matrix<double, 3, 1> joint_lengths = { { 0.14605 }, { 0.187325 }, { 0.125725 } };  // laatste mss {0.09}};
  Matrix<double, 5, 2> servo_limits = { { -90.0, 90.0 },
                                        { -30.0, 90.0 },
                                        { 0.0, 135.0 },
                                        { -90.0, 90.0 },
                                        { -90.0, 90.0 } };
};
}  // namespace Kinematics
