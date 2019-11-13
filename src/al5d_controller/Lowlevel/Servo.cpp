#include "ros/ros.h"
#include "al5d_controller/LowLevel/Servo.h"

Servo::Servo(SERVO_ID servo_id, int16_t min_degrees, int16_t max_degrees, int16_t min_pwm, int16_t max_pwm)
  : m_servo_id(servo_id), m_min_degrees(min_degrees), m_max_degrees(max_degrees), m_min_pwm(min_pwm), m_max_pwm(max_pwm), m_current_degrees(0), m_move_servo(false), m_incoming_degrees(0)
{
}

Servo::~Servo()
{
}

int Servo::getServoId() const 
{
  return m_servo_id;
}

bool Servo::canMoveServo() const
{
  return m_move_servo;
}

int16_t Servo::getIncomingDegrees() const
{
  return m_incoming_degrees;
}

void Servo::setIncomingDegrees(int16_t incoming_degrees) 
{
  m_incoming_degrees = incoming_degrees;
}

void Servo::setMoveServo(bool move_servo)
{
  m_move_servo = move_servo;
}

void Servo::setCurrentDegrees(int16_t current_degrees) 
{
  m_current_degrees = current_degrees;
}

int16_t Servo::degreesToPwm(int16_t target_degrees)
{
  if (target_degrees >= m_min_degrees && target_degrees <= m_max_degrees)
  {
    return (target_degrees - m_min_degrees) * (m_max_pwm - m_min_pwm) / (m_max_degrees - m_min_degrees) + m_min_pwm;
  }
  else
  {
    std::cout << target_degrees << std::endl;
    ROS_WARN("Degrees out of range, degrees must be between %d and %d", m_min_degrees, m_max_degrees);
    return false;
  }
}
