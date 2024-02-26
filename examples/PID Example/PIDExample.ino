#include <SVKSensors.h>



IRSensorMultiplexer irSensors;


const uint8_t sensorCount = 8;
const uint8_t muxPins[4] = { 7, 4, 2, A7};

uint16_t sensorValues[sensorCount];


// PID constants
uint64_t Kp = 0.1;      // Proportional constant
uint64_t Ki = 0.001;    // Integral constant
uint64_t Kd = 0.05;     // Derivative constant

// Motor Pins
const uint8_t PWMA = 3;
const uint8_t PMWB = 11;
const uint8_t DIRA = 13;
const uint8_t DIRB = A1;

// PID variables

uint64_t lastError = 0;
uint64_t integral = 0;


void setup()
{
    irSensors.setMultiplexerPins(muxPins);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(DIRA, OUTPUT);
    pinMode(DIRB, OUTPUT);


    irSensors.setCalibrationMode(true);

        // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
    // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
    // Call calibrate() 300 times to make calibration take about 10 seconds.
    for(uint16_t i = 0; i < 100; i++)
    {
        irSensors.calibrate();
    }

    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(9600);

    // Prints minimum and maximum values read by sensors
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(irSensors._calibration.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(irSensors._calibration.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);

}


void loop() {
  // read calibrated sensors values and get position of black line from 0 to 7000 (8 sensors)
  uint16_t position = irSensors.readLineBlack(sensorValues);
  double error = 3500 - position; // Assuming the line is at the middle (3500)
  integral += error;
  double derivative = error - lastError;
  lastError = error;

  double output = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds based on PID output
  int leftSpeed = 100; // base speed
  int rightSpeed = 100; // base speed

  leftSpeed += output;
  rightSpeed -= output;


    if(leftSpeed >= 0)
    {
        digitalWrite(DIRA, LOW);
        analogWrite(PWMA, constrain(leftSpeed, 0, 50));
    }
    else
    {
        digitalWrite(DIRA, HIGH);
        analogWrite(PWMA, abs(constrain(leftSpeed, 0, 50)));
    }

    if(rightSpeed >= 0)
    {
        digitalWrite(DIRB, LOW);
        analogWrite(PWMA, constrain(rightSpeed, 0, 50));
    }
    else
    {
        digitalWrite(DIRB, HIGH);
        analogWrite(PWMB, abs(constrain(rightSpeed, 0, 50)));
    }

    delay(200);

}
