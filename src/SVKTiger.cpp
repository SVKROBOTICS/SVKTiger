#include <SVKTiger.h>

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void IRSensorsTiger::setMultiplexerPins(const uint8_t *pins)
{
    // 4 Pins used for Multiplexer (3 Signal 1 Output)
    const uint8_t pinAmount = 4;

    uint8_t * oldMuxPins = _muxPins;
    /// (Re)Allocates space for dynamic array of Multiplexer 3 Signal Pins + 1 Output Pin (4 pins)
    _muxPins = (uint8_t *)realloc(_muxPins, sizeof(uint8_t) * pinAmount);

    if(_muxPins == nullptr)
    {
      free(oldMuxPins);
      return;
    }

    // Sets up pin array by copying pins given
    memcpy(_muxPins, pins, sizeof(uint8_t) * pinAmount);

    // sets up the pinModes for digital signal pins of multiplexer (first 3 pins)
    pinMode(_muxPins[0], OUTPUT);
    pinMode(_muxPins[1], OUTPUT);
    pinMode(_muxPins[2], OUTPUT);

    /// Re-initializes Calibration of robot since Pins have changed
    _calibration.initialized = false;
}


void IRSensorsTiger::setSamplesPerSensor(uint8_t samples)
{
    if(samples > 64) { samples = 64; }
    _samplesPerSensor = samples;
}

void IRSensorsTiger::calibrate()
{
    if(!_calibrateOn) { return; }
    calibratePrivate(_calibration);
}

void IRSensorsTiger::resetCalibration()
{
  for (uint8_t i = 0; i < _sensorAmount; i++)
  {
    if (_calibration.maximum)   { _calibration.maximum[i] = 0; }
    if (_calibration.minimum)   { _calibration.minimum[i] = _maxValue; }
  }
}

void IRSensorsTiger::read(uint16_t* sensorValues)
{
    readPrivate(sensorValues);
}

void IRSensorsTiger::readCalibrated(uint16_t* _sensorValues)
{
    if(!_calibration.initialized)
    {
        Serial.println("Not Calibrated");
        return;
    }

    read(_sensorValues);

    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
        uint16_t calmin, calmax;

        calmax = _calibration.maximum[i];
        calmin = _calibration.minimum[i];

        uint16_t denominator = calmax - calmin;
        int16_t value = 0;

        if (denominator != 0)
        {
        value = (((int32_t)_sensorValues[i]) - calmin) * 1000 / denominator;
        }

        if (value < 0) 
        { 
          value = 0; 
        }
        else if (value > 1000)
        {
           value = 1000; 
        }

        _sensorValues[i] = value;
    }
}

uint16_t IRSensorsTiger::readLineBlack(uint16_t* sensorValues)
{
    return readLinesPrivate(sensorValues);
}

void IRSensorsTiger::selectChannel(uint8_t sensorNum)
{
    /// This is the truth table for the multiplexer signal pins
    const uint8_t muxPinLayout[] = { 0b110, 0b111, 0b011, 0b010, 0b001, 0b100, 0b000, 0b101 };

    // This is channel C
    digitalWrite(_muxPins[0], bitRead(muxPinLayout[sensorNum], 2));
    // This is channel B
    digitalWrite(_muxPins[1], bitRead(muxPinLayout[sensorNum], 1));
    // this is channel A
    digitalWrite(_muxPins[2], bitRead(muxPinLayout[sensorNum], 0));
}

void IRSensorsTiger::calibratePrivate(CalibrationData &calibration)
{
    uint16_t sensorValues[_sensorAmount];
    uint16_t maxSensorValues[_sensorAmount];
    uint16_t minSensorValues[_sensorAmount];

    // (Re)allocate and initialize the arrays if necessary.
  if (!calibration.initialized)
  {
    uint16_t * oldMaximum = calibration.maximum;
    calibration.maximum = (uint16_t *)realloc(calibration.maximum,
                                              sizeof(uint16_t) * _sensorAmount);
    if (calibration.maximum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMaximum); // deallocate any memory used by old array
      return;
    }

    uint16_t * oldMinimum = calibration.minimum;
    calibration.minimum = (uint16_t *)realloc(calibration.minimum,
                                              sizeof(uint16_t) * _sensorAmount);
    if (calibration.minimum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMinimum); // deallocate any memory used by old array
      return;
    }

    // Initialize the max and min calibrated values to values that
    // will cause the first reading to update them.
    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
      calibration.maximum[i] = 0;
      calibration.minimum[i] = _maxValue;
    }

    calibration.initialized = true;
  }

  for (uint8_t j = 0; j < 10; j++)
  {
    read(sensorValues);

    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
      // set the max we found THIS time
      if ((j == 0) || (sensorValues[i] > maxSensorValues[i]))
      {
        maxSensorValues[i] = sensorValues[i];
      }

      // set the min we found THIS time
      if ((j == 0) || (sensorValues[i] < minSensorValues[i]))
      {
        minSensorValues[i] = sensorValues[i];
      }
    }
  }

  // record the min and max calibration values
  for (uint8_t i = 0; i < _sensorAmount; i++)
  {
    // Update maximum only if the min of 10 readings was still higher than it
    // (we got 10 readings in a row higher than the existing maximum).
    if (minSensorValues[i] > calibration.maximum[i])
    {
      calibration.maximum[i] = minSensorValues[i];
    }

    // Update minimum only if the max of 10 readings was still lower than it
    // (we got 10 readings in a row lower than the existing minimum).
    if (maxSensorValues[i] < calibration.minimum[i])
    {
      calibration.minimum[i] = maxSensorValues[i];
    }
  }
}

void IRSensorsTiger::readPrivate(uint16_t *_sensorValues)
{
    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
    _sensorValues[i] = 0;
    }

    for (uint8_t j = 0; j < _samplesPerSensor; j++)
    {
        for (uint8_t i = 0; i < _sensorAmount; i++)
        {
            // add the conversion result
            selectChannel(i);
            _sensorValues[i] += analogRead(_muxPins[3]);
        }
    }

    // get the rounded average of the readings for each sensor
    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
    _sensorValues[i] = (_sensorValues[i] + (_samplesPerSensor >> 1)) / _samplesPerSensor;
    }

}

uint16_t IRSensorsTiger::readLinesPrivate(uint16_t* _sensorValues)
{
    bool onLine = false;
    uint32_t avg = 0; // this is for the weighted total
    uint16_t sum = 0; // this is for the denominator, which is <= 64000


    readCalibrated(_sensorValues);

    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
        uint16_t value = _sensorValues[i];

        // keep track of whether we see the line at all
        if (value > 200) { onLine = true; }

        // only average in values that are above a noise threshold
        if (value > 50)
        {
        avg += (uint32_t)value * (i * 1000);
        sum += value;
        }
    }

    if (!onLine)
    {
        // If it last read to the left of center, return 0.
        if (_lastPosition < (_sensorAmount - 1) * 1000 / 2)
        {
          Serial.println("Lost line from left side");
          return 0;
        }
        // If it last read to the right of center, return the max.
        else
        {
          Serial.println("Lost line from right side");
          return (_sensorAmount - 1) * 1000;
        }
    }

    _lastPosition = avg / sum;
    return _lastPosition;
}

IRSensorsTiger::~IRSensorsTiger()
{
    if(_calibration.maximum)  { free(_calibration.maximum); }
    if(_calibration.minimum)  { free(_calibration.minimum); }
}
