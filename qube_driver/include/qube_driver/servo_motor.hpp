#ifndef QUBE_DRIVER_SERVO_MOTOR_HPP
#define QUBE_DRIVER_SERVO_MOTOR_HPP

#include <string>
#include <cmath>

/**
 * @brief The ServoMotor class represents a servo motor
 */
class ServoMotor
{
public:
    std::string name = "";
    double velocity_cmd = 0;
    double angle = 0;
    double angular_velocity = 0;

    ServoMotor() = default;

    ServoMotor(const std::string &motor_name)
    {
        setup(motor_name);
    }

    void setup(const std::string &motor_name)
    {
        name = motor_name;
    }
};

#endif // QUBE_DRIVER_SERVO_MOTOR_HPP