#include <SVKTiger.h>

// Comment out to run without Bluetooth module
#define USE_SVKTUNER

#ifdef USE_SVKTUNER
  #include <SVKTunerApp.h>
  #include <SoftwareSerial.h>
  #define BT_RX 5
  #define BT_TX 6
  // #define SVKTUNER_DEBUG
  SoftwareSerial bluetoothSerial(BT_RX, BT_TX);
  SVKTunerApp tuner(bluetoothSerial);
  bool robotRunning = false;
#endif

#define MAX_INTEGRAL 1400 // Maximun value of integral variable
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
float Ki = 0.000;    // Integral constant
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

    #ifdef USE_SVKTUNER
      bluetoothSerial.begin(9600);
      #ifdef SVKTUNER_DEBUG
        while (!Serial);
        Serial.println(F("=== Bluetooth Debug Monitor ==="));
      #endif
    #endif

    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);
    digitalWrite(DIRA, LOW); // Set left motor direction
    digitalWrite(DIRB, LOW); // Set right motor direction

    // Adds delay to be able to place in starting position
    delay(1500);
}

void loop() {

  #ifdef USE_SVKTUNER
    tuner.processStartStopCommands();
    if(tuner.getRobotState() == RUNNING) robotRunning = true;
    else if(tuner.getRobotState() == STOPPED) robotRunning = false;
  #endif

  #ifdef USE_SVKTUNER
    if(robotRunning) {
  #endif
      // read calibrated sensors values and get position of black line from 0 to 7000 (8 sensors)
      float position = irSensors.readLineBlack(sensorValues);
      float error = 3500 - position; // Assuming the line is at the middle (3500)

      integral += error;
      integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

      float derivative = error - lastError;
      lastError = error;

      float output = Kp * error + Ki * integral + Kd * derivative;

      // Adjust motor speeds based on PID output
      leftSpeed = baseSpeed - output;
      rightSpeed = baseSpeed + output;

      // Ensure motor speeds don't exceed maximum speed limit
      leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
      rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

      // Control the motors
      analogWrite(PWMA, leftSpeed); // Left motor speed control
      analogWrite(PWMB, rightSpeed); // Right motor speed control

      // Add a small delay to allow motors to adjust
      delayMicroseconds(500);
  #ifdef USE_SVKTUNER
    } else {
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);
    }
  #endif
}
