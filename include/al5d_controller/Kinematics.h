#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>       
#include <iostream>
#include <optional>
#include "matrix_class/Matrix.hpp"
namespace Kinematics
{
class Kinematics
{
public:
  /**
   * @brief Construct a new Kinematics object
   * 
   */
  Kinematics() = default;
  
  /**
   * @brief Destroy the Kinematics object
   * 
   */
  ~Kinematics() = default;
  
  /**
   * @brief Function for performing forward kinematics. Returns current position of the end-effector based on current servo angles
   * 
   * @param angles The angles the servos are currently set to
   * @return std::optional<Matrix<double, 3, 1>> The current position of the end-effector (x,y,z)
   */
  std::optional<Matrix<double, 3, 1>> forward_kinematics(Matrix<double, 4, 1>& angles) const;
  
  /**
   * @brief Function for performing inverse kinematics. Returns new servo angles based on the angles the servos are currently set to and the goal position
   * 
   * @param angles The angles the servos are currently set to
   * @param goal The goal for the end-effector (x,y,z)
   * @return std::optional<Matrix<double, 4, 1>> 
   */
  std::optional<Matrix<double, 4, 1>> inverse_kinematics(Matrix<double, 4, 1>& angles,
                                                         const Matrix<double, 3, 1>& goal) const;

private:
  /**
   * @brief Function for calculating the Jacobian matrix for the angles the servos are currently set to
   * 
   * @param angles The angles the servos are currently set to
   * @return Matrix<double, 3, 3> Jacobian matrix for the angles the servos are currently set to
   */
  Matrix<double, 3, 3> compute_jacobian(const Matrix<double, 4, 1>& angles) const;
  
  /**
   * @brief Function for checking whether the newly calculated angle configuration falls within the limits of the servos 
   * 
   * @param angles The newly calculated servo angles
   * @return true The al5d is capable of moving to the newly calculated configuration
   * @return false The al5d is mnot capable of moving to the newly calculated configuration
   */
  bool check_angles(const Matrix<double, 4, 1>& angles) const;
  
  /**
   * @brief Matrix containing the lengths of the robot between the joints (..., ..., ...)
   * 
   */
  Matrix<double, 3, 1> joint_lengths = { { 0.14605 }, { 0.187325 }, { 0.125725 } };  // laatste mss {0.09}};
  
  /**
   * @brief Matrix containing the limits of the servos (min, max), in the order Base, Shoulder, Elbow, Wrist, WristRotate
   * 
   */
  Matrix<double, 5, 2> servo_limits = { { -90.0, 90.0 },
                                        { -30.0, 90.0 },
                                        { 0.0, 135.0 },
                                        { -90.0, 90.0 },
                                        { -90.0, 90.0 } };
};
}  // namespace Kinematics

#endif  // KINEMATICS_H
