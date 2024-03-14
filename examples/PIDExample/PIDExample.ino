#include <SVKTiger.h>

#define MAX_INTEGRAL 1400
#define MAX_SPEED 255 // Maximum allowed speed

IRSensorsTiger irSensors;

const uint8_t sensorCount = 8;
const uint8_t muxPins[4] = { 7, 4, 2, A7};

uint16_t sensorValues[sensorCount];

// PID constants
float Kp = 0.1;      // Proportional constant
float Ki = 0.001;    // Integral constant
float Kd = 0.05;     // Derivative constant

// Motor Pins
const uint8_t PWMA = 3;
const uint8_t PMWB = 11;
const uint8_t DIRA = 13;
const uint8_t DIRB = A1;

// PID variables
float lastError = 0;
float integral = 0;

// Motor Speed variables
const int baseSpeed = 100;
int leftSpeed = 0;
int rightSpeed = 0;

void setup() {
    irSensors.setMultiplexerPins(muxPins);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);

    irSensors.setCalibrationMode(true);

    for (uint16_t i = 0; i < 100; i++) {
        irSensors.calibrate();
    }

    digitalWrite(LED_BUILTIN, LOW);
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
    delay(1000);
}

void loop() {
    // read calibrated sensors values and get position of black line from 0 to 7000 (8 sensors)
    float position = irSensors.readLineBlack(sensorValues);
    float error = 3500 - position; // Assuming the line is at the middle (3500)

    integral += error;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float derivative = error - lastError;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    // Adjust motor speeds based on PID output
    leftSpeed = baseSpeed + output;
    rightSpeed = baseSpeed - output;

    // Ensure motor speeds don't exceed maximum speed limit
    leftSpeed = min(leftSpeed, MAX_SPEED);
    rightSpeed = min(rightSpeed, MAX_SPEED);

    // Control the motors
    analogWrite(PWMA, leftSpeed); // Left motor speed control
    analogWrite(PMWB, rightSpeed); // Right motor speed control

    // Set motor directions
    digitalWrite(DIRA, leftSpeed > 0 ? HIGH : LOW); // Set left motor direction
    digitalWrite(DIRB, rightSpeed > 0 ? HIGH : LOW); // Set right motor direction

    // Add a small delay to allow motors to adjust
    delay(10);
}
