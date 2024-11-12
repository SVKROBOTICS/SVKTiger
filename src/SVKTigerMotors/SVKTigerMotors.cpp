#include <SVKTigerMotors.h>

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

// Sets pwm register macro so when its defined, constructor and motors will run using Fast PWM using registers. If undefined, constructor and motors will use normal analogWrite.
// Comment this define if you want to disable direct register writing
// OR write #undef USE_PWM_REGISTERS at the start of your .ino sketch
#define USE_PWM_REGISTERS


SVKTigerMotors::SVKTigerMotors()
{
    // Set pinModes
    pinMode(_DIRA, OUTPUT);
    pinMode(_DIRB, OUTPUT);
    pinMode(_PWMA, OUTPUT);
    pinMode(_PWMB, OUTPUT);

    // Sets motor direction to front
    digitalWrite(_DIRA, LOW);
    digitalWrite(_DIRB, LOW);

    // Checks if register pwm macro is enabled and sets up fast pwm registers
    #ifdef USE_PWM_REGISTERS

        // Configure Timer 2 for Fast PWM mode (Pin 3)
        TCCR2A = 0;  // Clear Timer/Counter Control Register A
        TCCR2B = 0;  // Clear Timer/Counter Control Register B
        TCNT2 = 0;   // Clear the timer count

        // Set Fast PWM Mode (WGM22:0 = 111)
        TCCR2A |= (1 << WGM21) | (1 << WGM20);  // Set WGM21 and WGM20 for Fast PWM
        TCCR2B |= (1 << WGM22);                  // Set WGM22 for Fast PWM

        // Set prescaler to 64 (CS22:0 = 011)
        TCCR2B |= (1 << CS22);  // Set the prescaler to 64

        // Configure Timer 1 for Fast PWM mode (Pin 11)
        TCCR1A = 0;  // Clear Timer/Counter Control Register A
        TCCR1B = 0;  // Clear Timer/Counter Control Register B
        TCNT1 = 0;   // Clear the timer count

        // Set Fast PWM Mode (WGM13:0 = 1111)
        TCCR1A |= (1 << WGM11) | (1 << WGM10);  // Set WGM11 and WGM10 for Fast PWM
        TCCR1B |= (1 << WGM12) | (1 << WGM13);  // Set WGM12 and WGM13 for Fast PWM

        // Set prescaler to 64 (CS12:0 = 011)
        TCCR1B |= (1 << CS11);  // Set the prescaler to 64
    #endif
}

SVKTigerMotors::SVKTigerMotors(uint8_t maxSpeed) : _maxSpeed(maxSpeed)
{
    // Set pinModes
    pinMode(_DIRA, OUTPUT);
    pinMode(_DIRB, OUTPUT);
    pinMode(_PWMA, OUTPUT);
    pinMode(_PWMB, OUTPUT);

    // Sets motor direction to front
    digitalWrite(_DIRA, LOW);
    digitalWrite(_DIRB, LOW);

    // Checks if register pwm macro is enabled and sets up fast pwm registers
    #ifdef USE_PWM_REGISTERS

        // Configure Timer 2 for Fast PWM mode (Pin 3)
        TCCR2A = 0;  // Clear Timer/Counter Control Register A
        TCCR2B = 0;  // Clear Timer/Counter Control Register B
        TCNT2 = 0;   // Clear the timer count

        // Set Fast PWM Mode (WGM22:0 = 111)
        TCCR2A |= (1 << WGM21) | (1 << WGM20);  // Set WGM21 and WGM20 for Fast PWM
        TCCR2B |= (1 << WGM22);                  // Set WGM22 for Fast PWM

        // Set prescaler to 64 (CS22:0 = 011)
        TCCR2B |= (1 << CS22);  // Set the prescaler to 64

        // Configure Timer 1 for Fast PWM mode (Pin 11)
        TCCR1A = 0;  // Clear Timer/Counter Control Register A
        TCCR1B = 0;  // Clear Timer/Counter Control Register B
        TCNT1 = 0;   // Clear the timer count

        // Set Fast PWM Mode (WGM13:0 = 1111)
        TCCR1A |= (1 << WGM11) | (1 << WGM10);  // Set WGM11 and WGM10 for Fast PWM
        TCCR1B |= (1 << WGM12) | (1 << WGM13);  // Set WGM12 and WGM13 for Fast PWM

        // Set prescaler to 64 (CS12:0 = 011)
        TCCR1B |= (1 << CS11);  // Set the prescaler to 64
    #endif
}

SVKTigerMotors::SVKTigerMotors(uint8_t maxSpeed, uint8_t baseSpeed) : _maxSpeed(maxSpeed), _baseSpeed(baseSpeed)
{
    // Set pinModes
    pinMode(_DIRA, OUTPUT);
    pinMode(_DIRB, OUTPUT);
    pinMode(_PWMA, OUTPUT);
    pinMode(_PWMB, OUTPUT);

    // Sets motor direction to front
    digitalWrite(_DIRA, LOW);
    digitalWrite(_DIRB, LOW);

    // Checks if register pwm macro is enabled and sets up fast pwm registers
    #ifdef USE_PWM_REGISTERS

        // Configure Timer 2 for Fast PWM mode (Pin 3)
        TCCR2A = 0;  // Clear Timer/Counter Control Register A
        TCCR2B = 0;  // Clear Timer/Counter Control Register B
        TCNT2 = 0;   // Clear the timer count

        // Set Fast PWM Mode (WGM22:0 = 111)
        TCCR2A |= (1 << WGM21) | (1 << WGM20);  // Set WGM21 and WGM20 for Fast PWM
        TCCR2B |= (1 << WGM22);                  // Set WGM22 for Fast PWM

        // Set prescaler to 64 (CS22:0 = 011)
        TCCR2B |= (1 << CS22);  // Set the prescaler to 64

        // Configure Timer 1 for Fast PWM mode (Pin 11)
        TCCR1A = 0;  // Clear Timer/Counter Control Register A
        TCCR1B = 0;  // Clear Timer/Counter Control Register B
        TCNT1 = 0;   // Clear the timer count

        // Set Fast PWM Mode (WGM13:0 = 1111)
        TCCR1A |= (1 << WGM11) | (1 << WGM10);  // Set WGM11 and WGM10 for Fast PWM
        TCCR1B |= (1 << WGM12) | (1 << WGM13);  // Set WGM12 and WGM13 for Fast PWM

        // Set prescaler to 64 (CS12:0 = 011)
        TCCR1B |= (1 << CS11);  // Set the prescaler to 64
    #endif
}

void SVKTigerMotors::setMotorSpeeds(uint8_t leftSpeed, uint8_t rightSpeed)
{
    // Checks if direct register access is defined or not
    #ifdef USE_PWM_REGISTERS
        // Cast motor speeds to uint8_t
        uint8_t leftMotorSpeed8bit = static_cast<uint8_t>(leftSpeed);
        uint8_t rightMotorSpeed8bit = static_cast<uint8_t>(rightSpeed);

        // Set the PWM duty cycle for pin 3 (Timer 2)
        OCR2A = leftMotorSpeed8bit;

        // Set the PWM duty cycle for pin 11 (Timer 1)
        OCR1A = rightMotorSpeed8bit;
    #else
        analogWrite(_PWMA, leftSpeed);
        analogWrite(_PWMB, rightSpeed);
    #endif
}

void SVKTigerMotors::runMotorsPrivate(int output, int error) 
{
    // Calculate target speeds based on PID output
    int targetLeftSpeed = _baseSpeed - output;
    int targetRightSpeed = _baseSpeed + output;

    // Constrain target speeds to be within the Max Speed defined
    targetLeftSpeed = constrain(targetLeftSpeed, 0, _maxSpeed);
    targetRightSpeed = constrain(targetRightSpeed, 0, _maxSpeed);

    // Calculate the speed change (difference) from the current speed to the target speed
    int leftSpeedDiff = targetLeftSpeed - _leftMotorSpeed;
    int rightSpeedDiff = targetRightSpeed - _rightMotorSpeed;

    // Map error to acceleration: smoother turns (smaller error) use min acceleration, sharper turns use max acceleration
    uint8_t mappedAcceleration = map(abs(error), 0, 3500, _acceleration.min, _acceleration.max);

    // Apply acceleration limits: change the speed by up to `mappedAcceleration` per loop
    if (abs(leftSpeedDiff) > mappedAcceleration) {
        _leftMotorSpeed += (leftSpeedDiff > 0) ? mappedAcceleration : -mappedAcceleration;
    } else {
        _leftMotorSpeed = targetLeftSpeed;  // If the change is smaller than mappedAcceleration, go directly to target speed
    }

    if (abs(rightSpeedDiff) > mappedAcceleration) {
        _rightMotorSpeed += (rightSpeedDiff > 0) ? mappedAcceleration : -mappedAcceleration;
    } else {
        _rightMotorSpeed = targetRightSpeed;  // If the change is smaller than mappedAcceleration, go directly to target speed
    }

    // Constrain the final motor speeds to be within the max speed of the motors
    _leftMotorSpeed = constrain(_leftMotorSpeed, 0, _maxSpeed);
    _rightMotorSpeed = constrain(_rightMotorSpeed, 0, _maxSpeed);

    #ifdef USE_PWM_REGISTERS
        // Cast motor speeds to uint8_t
        uint8_t leftMotorSpeed8bit = static_cast<uint8_t>(_leftMotorSpeed);
        uint8_t rightMotorSpeed8bit = static_cast<uint8_t>(_rightMotorSpeed);

        // Set the PWM duty cycle for pin 3 (Timer 2)
        OCR2A = leftMotorSpeed8bit;

        // Set the PWM duty cycle for pin 11 (Timer 1)
        OCR1A = rightMotorSpeed8bit;
    #else
        // Use simple analogWrite to send PWM to motors
        analogWrite(_PWMA, _leftMotorSpeed);
        analogWrite(_PWMB, _rightMotorSpeed);
    #endif

}
