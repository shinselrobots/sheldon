/*
SharpDistSensorCustomPower.ino
Source: https://github.com/DrGFreeman/SharpDistSensor

MIT License

Copyright (c) 2017 Julien de la Bruere-Terreault <drgfreeman@tuta.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This example shows how to use the SharpDistSensor library to continuously
read the sensor and display the analog value and the corresponding distance
using custom power fit curve and range.
*/

#include <SharpDistSensor.h>

// Analog pin to which the sensor is connected
const byte sensorPin0 = A0;
const byte sensorPin1 = A1;

// Window size of the median filter (odd number, 1 = no filtering)
const byte medianFilterWindowSize = 5;

// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor0(sensorPin0, medianFilterWindowSize);
SharpDistSensor sensor1(sensorPin1, medianFilterWindowSize);

/* Set the power fit curve coefficients and range
 * C and P: Coefficients in Distance = C*A^P relation
 * where A is the analog value read from the sensor.
 */
const float C = 100608;
const float P = -1.334;

/*
 * Minimum and maximum analog values for which to return a distance
 * These should represent a range of analog values within which the
 * power fit curve is valid.
 */
const unsigned int minVal = 50; // ~150 mm
const unsigned int maxVal = 750; // ~20 mm

void setup() {
  Serial.begin(9600);

  // Set custom power fit curve coefficients and range
  sensor0.setPowerFitCoeffs(C, P, minVal, maxVal);
  sensor1.setPowerFitCoeffs(C, P, minVal, maxVal);
}

void loop() {
  /* Print raw analog value. This step is not required to read the sensor,
     it is included here to show the relation between the raw analog value
     and the returned distance. */

  // Sensor0
  Serial.print(analogRead(sensorPin0));
  Serial.print(",   0 = ");

  // Get distance from sensor
  unsigned int distance = sensor0.getDist()+12;

  // Print distance to Serial
  Serial.println(distance);

  // Sensor1
  Serial.print(analogRead(sensorPin1));
  Serial.print(",   1 = ");

  // Get distance from sensor
  distance = sensor1.getDist();

  // Print distance to Serial
  Serial.println(distance);
  // Wait some time

  Serial.println("----------------");
  delay(250);

}
