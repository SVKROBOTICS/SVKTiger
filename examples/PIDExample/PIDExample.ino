#include <SVKTiger.h>

#define MAX_INTEGRAL 2800 // Maximun value of integral variable
#define MAX_SPEED 140 // Maximum allowed speed

// Create class object
IRSensorsTiger irSensors;

// Sets sensor amount and pins
const uint8_t sensorCount = 8;
const uint8_t muxPins[4] = { 7, 4, 2, A7};

// Creates array to store Ir sensor values
uint16_t sensorValues[sensorCount];

// PID constants
float Kp = 0.07;      // Proportional constant
float Ki = 0.09;    // Integral constant
float Kd = 0.3;     // Derivative constant

// Motor Pins
const uint8_t PWMA = 3;
const uint8_t PWMB = 11;
const uint8_t DIRA = 13;
const uint8_t DIRB = A1;

// PID variables
float lastError = 0;
float integral = 0;

// Motor Speed variables
const int baseSpeed = 70;
int leftSpeed = 0;
int rightSpeed = 0;

void setup() {
    irSensors.setMultiplexerPins(muxPins);

    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);

    // Sets samples taken in each loop for each sensor
    irSensors.setSamplesPerSensor(1);

    // Runs the calibrate 100 times for the robot to get max and min values read
    for (uint16_t i = 0; i < 100; i++) {
        irSensors.calibrate();
    }

    Serial.begin(9600);

    // Prints minimum and maximum values read by sensors
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(irSensors._calibration.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(irSensors._calibration.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();

    // Adds delay to be able to place in starting position
    delay(2000);
}

void loop() {
    // read calibrated sensors values and get position of black line from 0 to 7000 (8 sensors)
    float position = irSensors.readLineBlack(sensorValues);
    float error = 3500 - position; // Assuming the line is at the middle (3500)

    // integral += error;
    // integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float derivative = error - lastError;
    lastError = error;

    float output = Kp * error + Kd * derivative;

    // Adjust motor speeds based on PID output
    leftSpeed = baseSpeed - output;
    rightSpeed = baseSpeed + output;

    // Ensure motor speeds don't exceed maximum speed limit
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

    // Control the motors
    analogWrite(PWMA, leftSpeed); // Left motor speed control
    analogWrite(PWMB, rightSpeed); // Right motor speed control

    // Set motor directions
    digitalWrite(DIRA, leftSpeed > 0 ? LOW : HIGH); // Set left motor direction
    digitalWrite(DIRB, rightSpeed > 0 ? LOW : HIGH); // Set right motor direction

    // Add a small delay to allow motors to adjust
    delayMicroseconds(100);
}
