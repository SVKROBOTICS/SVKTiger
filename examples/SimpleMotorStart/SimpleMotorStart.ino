// Simple program to start motors and make robot go forward for 3 seconds

#include <SVKTigerMotors.h>

// You can setup this macro (is enabled by default) if you want to use direct register writing, making the code faster
// #define USE_PWM_REGISTERS

// Or if you want normal pwm used, uncomment this line
// #undef USE_PWM_REGISTERS

#define MAX_SPEED 100 // Robot Max Speed
#define BASE_SPEED 50 // Robot base starting speed
#define MIN_ACCEL 5 // Minimum acceleration
#define MAX_ACCEL 15 // Max acceleration

// Create motors object to use all motor functions
SVKTigerMotors motors(MAX_SPEED, BASE_SPEED);

// Or can create a simple constructor with default values, and add values in setup
SVKTigerMotors basicMotors;


void setup() {

    // Functions to add max and base speed to our motors
    basicMotors.setMaxSpeed(MAX_SPEED);
    basicMotors.setBaseSpeed(BASE_SPEED);

    // Add min and max acceleration to robot
    basicMotors.setMinAcceleration(MIN_ACCEL);
    basicMotors.setMaxAcceleration(MAX_ACCEL);

    // small delay before starting robots
    delay(1000);

    // fixed motor speed variables
    int leftSpeed = 70;
    int rightSpeed = 70;

    // start motors with fixed speed
    basicMotors.setMotorSpeeds(leftSpeed, rightSpeed);
}

void loop() {
    // motors run for 3 seconds
    delay(3000);

    // stop motors
    basicMotors.setMotorSpeeds(0, 0);
}
