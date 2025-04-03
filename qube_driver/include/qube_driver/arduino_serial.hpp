#ifndef JOINT_DRIVER_JOINT_SERIAL_HPP
#define JOINT_DRIVER_JOINT_SERIAL_HPP

#include <sstream>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>

class ArduinoSerial
{
public:
    ArduinoSerial() = default;

    /**
     * @brief Initialize a new ArduinoSerial::ArduinoSerial object
     * @param serial_device The serial device to connect to
     * @param baud_rate The baud rate of the serial connection
     * @param timeout_ms The timeout of the serial connection
     */
    ArduinoSerial(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    { }

    /**
     * @brief Set up the serial connection
     * @param serial_device The serial device to connect to
     * @param baud_rate The baud rate of the serial connection
     * @param timeout_ms The timeout of the serial connection
     */ 
    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        serial_conn_.setPort(serial_device);
        serial_conn_.setBaudrate(baud_rate);
        serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
        serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
        serial_conn_.open();
    }

    /**
     * @brief Start the communication thread
     */
    void start_communication_thread()
    {
        communication_thread_ = std::thread(&ArduinoSerial::communication_loop, this);
    }

    /**
     * @brief Stop the communication thread
     */
    void stop_communication_thread()
    {
        stop_thread_ = true;
        if (communication_thread_.joinable())
        {
            communication_thread_.join();
        }
    }

    /**
     * @brief Check if the serial connection is open
     * @return True if the serial connection is open, false otherwise
     */
    bool connected() const
    {
        return serial_conn_.isOpen();
    }

    /**
     * @brief Set the speed of the motor
     * @param speed The speed of the motor is between -999 and 999
     */
    void set_motor_speed(int speed)
    {
        std::clamp(speed, -999, 999);
        motor_speed_ = speed + 999;
    }

    /**
     * @brief Get the angle of the motor
     * @return The angle of the motor in radians
     */
    float get_motor_angle() const
    {
        return motor_angle_;
    }

private:
    /**
     * @brief The communication loop for the serial connection
     */
    void communication_loop()
    {
        while (!stop_thread_)
        {
            if (connected())
            {
                // Write the motor speed to the Arduino
                uint8_t* buffer = new uint8_t[2];
                buffer[0] = motor_speed_ >> 8;
                buffer[1] = motor_speed_ & 0xFF;
                serial_conn_.write(buffer, 2);
                delete[] buffer;
                
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                // Read the motor angle from the Arduino
                uint8_t* read_buffer = new uint8_t[4];
                serial_conn_.read(read_buffer, 4);
                bool motor_revolutions_negative = read_buffer[0] & 0x80;
                int motor_revolutions = ((read_buffer[0] & 0x7F) << 8) | read_buffer[1];
                int motor_angle_int = ((buffer[2] << 1) | (buffer[3] >> 7));
                int motor_angle_dec = (read_buffer[3] & 0x7F);
                // std::cout << "Read buffer: " << (int)read_buffer[0] << " " << (int)read_buffer[1] << " " << (int)read_buffer[2] << " " << (int)read_buffer[3] << ", ";
                // std::cout << "Motor revolutions: " << motor_revolutions << ", ";
                // std::cout << "Motor angle: " << motor_angle_int << "." << motor_angle_dec << std::endl;
                serial_conn_.read(read_buffer, 2);
                delete[] read_buffer;
                
                // The motor angle is in radians and is contnuous
                motor_angle_ = motor_revolutions * 360;
                motor_angle_ += motor_angle_int + motor_angle_dec / 100.0f;
                motor_angle_ = motor_revolutions_negative ? -motor_angle_ : motor_angle_;
                motor_angle_ = motor_angle_ / 180.0f * 3.14159265359;
            }
            else
            {
                std::cout << "Serial port not connected" << std::endl;
            }
        }
    }

    std::thread communication_thread_;
    std::atomic<bool> stop_thread_{false};

    serial::Serial serial_conn_;
    float motor_angle_ = 0;
    int motor_speed_ = 0;
};

#endif // JOINT_DRIVER_JOINT_SERIAL_HPP
