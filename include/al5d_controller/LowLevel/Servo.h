#ifndef SERVO_H_
#define SERVO_H_

#include <string>

class Servo
{
public:

    Servo(std::string name, int16_t min_degrees, int16_t max_degrees, int16_t min_pwm, int16_t max_pwm);
    virtual ~Servo();
    int16_t degreesToPwm( int16_t target_degrees);

private:
    std::string m_name;
    int16_t m_min_degrees;
    int16_t m_max_degrees;
    int16_t m_min_pwm;
    int16_t m_max_pwm; 
};

#endif // SERVO_H_