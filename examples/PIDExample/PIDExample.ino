#include <SVKTigerSensors.h>
#include <SVKTigerMotors.h>

#define MAX_INTEGRAL 2800 // Maximun value of integral variable
#define DEBUG_MODE 0 // Debug Mode for reading sensor values via serial, 0 for debug mode off, 1 for debug mode on

// This code example uses a macro `#define USE_PWM_REGISTERS` that is default on inside the source code,
// for the motors to use direct register writing, which achieves faster writing and better performance.
// If you want to disable direct register writing and instead want the SVKTigerMotos to use normal analogWrites, uncomment line below.

// #undef USE_PWM_REGISTERS


#define MAX_SPEED 200 // Robot Max Speed
#define BASE_SPEED 50 // Robot base starting speed
#define MIN_ACCEL 5 // Minimum acceleration
#define MAX_ACCEL 15 // Max acceleration


// Create Sensor class instance
SVKTigerSensors sensors;
// Create Motor class instance (pinModes done in constructor, no need for in setup)
// Motor Parameters are _maxSpeed, _baseSpeed. Check SVKTigerMotors->Examples, or the header files for more info
SVKTigerMotors motors(MAX_SPEED, BASE_SPEED);


// Sets sensor amount and pins
const uint8_t sensorCount = sensors.getSensorAmount();

// PID constants
float Kp = 0.07;      // Proportional constant
float Ki = 0.09;    // Integral constant
float Kd = 0.3;     // Derivative constant

// PID variables
int lastError = 0;
int integral = 0;


void setup() {
    sensors.setMultiplexerPins();

    // Sets motor accelerations
    motors.setMinAcceleration(MIN_ACCEL);
    motors.setMaxAcceleration(MAX_ACCEL);

    // Runs the calibrate 100 times for the robot to get max and min values read
    for (uint8_t i = 0; i < 100; i++) {
        sensors.calibrate();
    }


    #if DEBUG_MODE

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

    #endif

    // Adds delay to be able to place in starting position
    delay(2000);
}

void loop() {
    // read calibrated sensors values and get position of black line from 0 to 7000 (8 sensors)
    int position = sensors.readLineBlack();
    int error = 3500 - position; // Assuming the line is at the middle (3500)

    integral += error;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    int derivative = error - lastError;
    lastError = error;

    int output = Kp * error + Ki * integral + Kd * derivative;

    // Main motor method, takes PID output and error to calculate and run both motor speeds correctly
    // Pass output and error to motor methods to start running
    motors.runMotors(output, error);

}
