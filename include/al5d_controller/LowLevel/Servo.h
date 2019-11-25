#ifndef SERVO_H_
#define SERVO_H

#include <string>

/**
 * @brief Enum containing the al5d's servos
 * 
 */
enum SERVO_ID : int
{
  BASE = 0,
  SHOULDER,
  ELBOW,
  WRIST,
  WRIST_ROTATE,
  GRIPPER
};

class Servo
{
public:
  /**
   * @brief Construct a new Servo object
   * 
   * @param servo_id ID of the servo that is created (see enum above)
   * @param min_degrees The minimum amount of degrees the specific servo can be set to 
   * @param max_degrees The maximum amount of degrees the specific servo can be set to
   * @param min_pwm The required PWM to send the specific servo to its min_degrees position
   * @param max_pwm The required PWM to send the specific servo to its max_degrees position
   */
  Servo(SERVO_ID servo_id, int16_t min_degrees, int16_t max_degrees, int16_t min_pwm, int16_t max_pwm);
  
  /**
   * @brief Destroy the Servo object
   * 
   */
  virtual ~Servo();
  
  /**
   * @brief Function to convert the specified amount of degrees to the required PWM value
   * 
   * @param target_degrees The amount of degrees the servo should be set to
   * @return int16_t The required PWM value to set the servo to the corresponding target_degrees
   */
  int16_t degreesToPwm(int16_t target_degrees);
  
  /**
   * @brief Set the incoming degrees
   * 
   * @param incoming_degrees 
   */
  void setIncomingDegrees(int16_t incoming_degrees);
  
  /**
   * @brief Function to set whether this instance of servo should be moved
   * 
   * @param move_servo 
   */
  void setMoveServo(bool move_servo);
  
  /**
   * @brief Get the ID of this servo instance
   * 
   * @return int Servo ID
   */
  int getServoId() const;
  
  /**
   * @brief Get the incoming degrees
   * 
   * @return int16_t incoming degrees
   */
  int16_t getIncomingDegrees() const;
  
  /**
   * @brief Function to determin whether the servo can be moved to the required position
   * 
   * @return true Servo can be moved to the required position
   * @return false Servo can't be moved to the required position
   */
  bool canMoveServo() const;

private:
  /**
   * @brief Servo ID of this servo instance
   * 
   */
  SERVO_ID m_servo_id;
  
  /**
   * @brief Minimum amount of degrees of this servo instance
   * 
   */
  int16_t m_min_degrees;
  
  /**
   * @brief Maximum amount of degrees of this servo instance
   * 
   */
  int16_t m_max_degrees;
  
  /**
   * @brief PWM value for this servo instances m_min_degrees
   * 
   */
  int16_t m_min_pwm;
  
  /**
   * @brief PWM value for this servo instances m_max_degrees 
   * 
   */
  int16_t m_max_pwm;
  
  /**
   * @brief 
   * 
   */
  bool m_move_servo;
  
  /**
   * @brief Amount of incoming degrees
   * 
   */
  int16_t m_incoming_degrees;
};

#endif  // SERVO_H_