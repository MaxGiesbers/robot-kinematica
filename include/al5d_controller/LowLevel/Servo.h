#ifndef SERVO_H_
#define SERVO_H_

#include <string>

enum SERVO_ID: int
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
  Servo(SERVO_ID servo_id, int16_t min_degrees, int16_t max_degrees, int16_t min_pwm, int16_t max_pwm);
  virtual ~Servo();
  int16_t degreesToPwm(int16_t target_degrees);

  void setCurrentDegrees(int16_t current_degrees);
  void setIncomingDegrees(int16_t incoming_degrees);
  void setMoveServo(bool move_servo);

  int getServoId() const;
  int16_t getIncomingDegrees() const;
  bool canMoveServo() const;

private:
  SERVO_ID m_servo_id;
  int16_t m_min_degrees;
  int16_t m_max_degrees;
  int16_t m_min_pwm;
  int16_t m_max_pwm;
  bool m_move_servo;
  int16_t m_incoming_degrees;
  int16_t m_current_degrees;
};

#endif  // SERVO_H_