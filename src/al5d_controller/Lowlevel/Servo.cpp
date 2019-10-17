#include "ros/ros.h"
#include "al5d_controller/LowLevel/Servo.h"

Servo::Servo(const std::string name, int16_t  min_degrees, int16_t max_degrees, int16_t min_pwm, int16_t max_pwm): 
                m_name(name),
                m_min_degrees(min_degrees),
                m_max_degrees(max_degrees),
                m_min_pwm(min_pwm),
                m_max_pwm(max_pwm)            
{
}

Servo::~Servo()
{
}

int16_t Servo::degreesToPwm(int16_t target_degrees)
{
    if(target_degrees >= m_min_degrees && target_degrees <= m_max_degrees)
    {
        return (target_degrees - m_min_degrees) * (m_max_pwm - m_min_pwm) / (m_max_degrees - m_min_degrees) + m_min_pwm;
    } else 
    {
        ROS_WARN("Degrees out of range, degrees must be between %d and %d", m_min_degrees, m_max_degrees);
        return false;
    }
}
