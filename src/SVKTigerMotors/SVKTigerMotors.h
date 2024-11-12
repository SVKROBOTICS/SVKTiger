// Header file for controlling the motors of the SVK Tiger robot
#pragma once

/// @brief Class for controlling the motors
class SVKTigerMotors {
    public:
        /// @brief Class Constructor with default max speed, base speed.
        SVKTigerMotors() : _maxSpeed(255), _baseSpeed(50), _acceleration{5, 10} {}

        /// @brief Class Constructor with custom maxSpeed.
        /// @param maxSpeed Maximum speed motors will be able to reach (in PWM range 0-255).
        SVKTigerMotors(uint8_t maxSpeed) : _maxSpeed(maxSpeed), _baseSpeed(50), _acceleration{5, 10} {}

        /// @brief Class Constructor with custom maxSpeed and baseSpeed.
        /// @param maxSpeed Maximum speed motors will be able to reach (in PWM range 0-255).
        /// @param baseSpeed Starting speed motors will start with (in PWM range 0-255).
        SVKTigerMotors(uint8_t maxSpeed, uint8_t baseSpeed) : _maxSpeed(maxSpeed), _baseSpeed(baseSpeed), _acceleration{5, 10} {}

        /// @brief Class Destructor
        ~SVKTigerMotors() = default;

        /// @brief Returns max speed of motors.
        /// @return Max speed in PWM range (0-255).
        uint8_t getMaxSpeed() const { return _maxSpeed; }

        /// @brief Sets max speed of motors.
        /// @param maxSpeed uInt number of max speed in PWM range (0-255).
        void setMaxSpeed(uint8_t maxSpeed) { _maxSpeed = maxSpeed; } 

        /// @brief Returns base starting speed of motors.
        /// @return Base Speed in PWM range (0-255).
        uint8_t getBaseSpeed() const { return _baseSpeed; }

        /// @brief Sets base speed of motors.
        /// @param baseSpeed uInt number of base speed in PWM range (0-255).
        void setBaseSpeed(uint8_t baseSpeed) { _baseSpeed = baseSpeed; }

        /// @brief Returns Speed of left motor.
        /// @return Int speed of left motor.
        int getLeftMotorSpeed() const { return _leftMotorSpeed; }

        /// @brief Returns Speed of right motor.
        /// @return Int speed of right motor.
        int getRightMotorSpeed() const { return _rightMotorSpeed; }

        /// @brief Sets speed and directions of the motors based on PID output, and starts them.
        /// @param output PID loop output based on sensor feedback.
        /// @param error Error amount from PID feedback.
        void runMotors(int output, int error) { runMotorsPrivate(output, error); }

        /// @brief Sets both motors to specific (positive only) PWM value, works with direct register access and normal analogWrites.
        /// @param leftSpeed PWM value for speed of left motor.
        /// @param rightSpeed PWM value for speed of right motor.
        void setMotorSpeeds(uint8_t leftSpeed, uint8_t rightSpeed);

        /// @brief Returns minimum acceleration.
        uint8_t getMinAcceleration() const { return _acceleration.min; }

        /// @brief Sets minimum acceleration.
        /// @param minAcceleration Minimum acceleration value in PWM range (0-255).
        void setMinAcceleration(uint8_t minAcceleration) { _acceleration.min = minAcceleration; }

        /// @brief Returns maximum acceleration.
        uint8_t getMaxAcceleration() const { return _acceleration.max; }

        /// @brief Sets maximum acceleration.
        /// @param maxAcceleration Maximum acceleration value in PWM range (0-255).
        void setMaxAcceleration(uint8_t maxAcceleration) { _acceleration.max = maxAcceleration; }

    private:
        /// @brief Struct for minimum and maximum amount of acceleration. Min and Max are used to map the error to dynamic acceleration in the program, max is for sharp corners and straights, whereas min is for smooth turns.
        struct Acceleration {
            uint8_t min; // Minimum acceleration value, in PWM range (0-255).
            uint8_t max; // Maximum acceleration value, in PWM range (0-255).
        };

        /// @brief Motor and Direction Pins.
        static const uint8_t _PWMA = 3, _PWMB = 11, _DIRA = 13, _DIRB = A1;
        /// @brief Max Speed of motors.
        uint8_t _maxSpeed;
        /// @brief Base Starting Speed for motors.
        uint8_t _baseSpeed;
        /// @brief Speed of Left Motor, in PWM range (0-255).
        int _leftMotorSpeed = 0;
        /// @brief Speed of Right Motor, in PWM range (0-255).
        int _rightMotorSpeed = 0;
        /// @brief Acceleration struct instance for minimum and maximum acceleration values.
        Acceleration _acceleration;

        /// @brief Sets speed and directions of the motors based on outputs, and starts them, private method.
        /// @param output PID loop output based on sensor feedback.
        /// @param error Error amount from PID feedback.
        void runMotorsPrivate(int output, int error);
};
