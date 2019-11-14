#include "al5d_controller/Kinematics.h"

namespace Kinematics
{
std::optional<Matrix<double, 3, 1>> Kinematics::forward_kinematics(Matrix<double, 4, 1>& angles) const
{
  angles[3][0] = 157.5 - angles[1][0] - angles[2][0];

  if (!Kinematics::check_angles(angles))
  {
    return std::nullopt;
  }

  Matrix<double, 3, 1> fk_solution;
  Matrix<double, 4, 1> a = angles * (M_PI / 180);
  const double& l1 = joint_lengths[0][0];
  const double& l2 = joint_lengths[1][0];
  const double& l3 = joint_lengths[2][0];

  // x axis
  fk_solution[0][0] =
      ((l1 * std::sin(a[1][0])) + (l2 * std::sin(a[1][0] + a[2][0])) + (l3 * std::sin(a[1][0] + a[2][0] + a[3][0]))) *
      std::cos(a[0][0]);

  // y axis
  fk_solution[1][0] =
      (l1 * std::cos(a[1][0])) + (l2 * std::cos(a[1][0] + a[2][0])) + (l3 * std::cos(a[1][0] + a[2][0] + a[3][0]));

  // z axis
  fk_solution[2][0] =
      ((l1 * std::sin(a[1][0])) + (l2 * std::sin(a[1][0] + a[2][0])) + (l3 * std::sin(a[1][0] + a[2][0] + a[3][0]))) *
      std::sin(a[0][0]);

  return fk_solution;
}

std::optional<Matrix<double, 4, 1>> Kinematics::inverse_kinematics(Matrix<double, 4, 1>& current_angles,
                                                                   const Matrix<double, 3, 1>& goal) const
{
  Matrix<double, 4, 1> ik_solution;
  auto new_angles = current_angles;

  double beta = 0.1;
  const double precision = 0.0005;
  const double deviation = 0.0001;
  const unsigned long max_iterations = 1000000;
  unsigned long iteration = 0;

  // xyz to move
  auto g = goal;

  // current xyz
  auto e_opt = forward_kinematics(current_angles);
  auto e_opt_new = e_opt;
  if (!e_opt.has_value())
  {
    return std::nullopt;
  }
  auto e = e_opt.value();
  auto e_new = e;

  while ((g - e).get_magnitude() > precision && ++iteration < max_iterations)
  {
    // compute jacobian
    const auto jacobian = compute_jacobian(current_angles);

    // compute inverse jacobian
    const auto jacobian_inverse_opt = jacobian.inverse();

    if (!jacobian_inverse_opt.has_value())
    {
      return std::nullopt;
    }
    const auto& jacobian_inverse = jacobian_inverse_opt.value();

    // delta e = beta(g - e)
    const auto delta_e = beta * (g - e);

    // delta angle = jacobian inverse * delta e
    const auto delta_angles = jacobian_inverse * delta_e;

    // angle new = angle old + delta angle
    new_angles = current_angles + delta_angles.template slice<0, 0, 4, 1>();
    new_angles[3][0] = (M_PI / 8 * 7) - new_angles[1][0] - new_angles[2][0];

    // e = forward(angle new)
    e_opt_new = forward_kinematics(new_angles);

    if (!e_opt_new.has_value())
    {
      beta /= 2.0;
      continue;
    }
    e_new = e_opt_new.value();

    if ((e_new - e + delta_e).get_magnitude() > deviation)
    {
      beta /= 2.0;
    }
    else
    {
      beta *= 1.2;
      e = e_new;
      current_angles = new_angles;
    }
  }

  // check precision and set solution
  if ((g - e).get_magnitude() <= precision)
  {
    new_angles[3][0] = 157.5 - new_angles[1][0] - new_angles[2][0];
    ik_solution = new_angles;
  }
  else
  {
    return std::nullopt;
  }

  return ik_solution;
}

Matrix<double, 3, 3> Kinematics::compute_jacobian(const Matrix<double, 4, 1>& angles) const
{
  Matrix<double, 3, 3> jacobian_solution;
  Matrix<double, 4, 1> a = angles * (M_PI / 180);
  const double& l1 = joint_lengths[0][0];
  const double& l2 = joint_lengths[1][0];

  // x axis
  jacobian_solution[0][0] = (l1 * std::sin(a[1][0]) + l2 * std::sin(a[1][0] + a[2][0])) * -std::sin(a[0][0]);
  jacobian_solution[0][1] = (l1 * std::cos(a[1][0]) + l2 * std::cos(a[1][0] + a[2][0])) * std::cos(a[0][0]);
  jacobian_solution[0][2] = l2 * std::cos(a[1][0] + a[2][0]) * std::cos(a[0][0]);

  // y axis
  jacobian_solution[1][0] = 0;
  jacobian_solution[1][1] = -l1 * std::sin(a[1][0]) - l2 * std::sin(a[1][0] + a[2][0]);
  jacobian_solution[1][2] = -l2 * std::sin(a[1][0] + a[2][0]);

  // z axis
  jacobian_solution[2][0] = (l1 * std::sin(a[1][0]) + l2 * std::sin(a[1][0] + a[2][0])) * std::cos(a[0][0]);
  jacobian_solution[2][1] = (l1 * std::cos(a[1][0]) + l2 * std::cos(a[1][0] + a[2][0])) * std::sin(a[0][0]);
  jacobian_solution[2][2] = l2 * std::cos(a[1][0] + a[2][0]) * std::sin(a[0][0]);

  return jacobian_solution;
}

bool Kinematics::check_angles(const Matrix<double, 4, 1>& angles) const
{
  for (std::size_t i = 0; i < angles.get_m(); ++i)
  {
    if (angles[i][0] < servo_limits[i][0] || angles[i][0] > servo_limits[i][1])
    {
      return false;
    }
  }
  return true;
}
}  // namespace Kinematics