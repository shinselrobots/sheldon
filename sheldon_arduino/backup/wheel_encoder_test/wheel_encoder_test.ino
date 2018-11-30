/* Encoder Library - TwoKnobs Example
   http://www.pjrc.com/teensy/td_libs_Encoder.html

   This example code is in the public domain.
*/

#include <Encoder.h>
#define NOT_SET -99999
const long SPEED_SAMPLE_TIME_INTERVAL = 250; // ms - too short and it wont work for slow motor speeds
const int SPEED_HYSTERYSIS = 4;   // Ticks

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

// Assign pins for quadrature encoder. Mega pins with interrupt are: 2,3, plus 18,19,(serial1) 20,21 (I2C)
Encoder wheelEncoderRight(2, 3);  // quadrature inputs for each motor
Encoder wheelEncoderLeft(18, 19); // block use of Serial1!


long wheelCountRight  = 0L;
long wheelCountLeft  = 0L;
unsigned long oldTime = 0L;
int speedRight = 0;
int speedLeft = 0;
long speedCounterRight = 0L;
long speedCounterLeft = 0L;
long lastWheelCountRight = 0L;
long lastWheelCountLeft = 0L;
int targetSpeedRight = 0;
int targetSpeedLeft = 0;
int currentMotorCommandRight = 0;
int currentMotorCommandLeft = 0;
//volatile boolean indexFound = false;
//volatile long indexPos = NOT_SET;

/*
  void indexISR()
  {
  indexFound = true; // tell regular loop that we saw an index pulse
  if (NOT_SET == indexPos) {
    indexPos = positionEncoder1; // save the position
  }
  }
*/
void setup()
{
  Serial.begin(9600);
  Serial.println("Encoder Test:");
  //pinMode(INDEX_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(INDEX_PIN), indexISR, LOW);

  oldTime = millis();

  
}


void loop() {

  long newValue;
  //bool newUpdate = false;

  // Read Motor Encoders
  wheelCountLeft = wheelEncoderLeft.read();
  wheelCountRight = wheelEncoderRight.read();

  // calculate speed
  unsigned long newTime = millis();
  int deltaTime = (signed int) (newTime - oldTime);
  if (deltaTime > SPEED_SAMPLE_TIME_INTERVAL) {
    speedLeft = ((wheelCountLeft - lastWheelCountLeft) * 1000) / deltaTime; // avoid floating point operation
    speedRight = ((wheelCountRight - lastWheelCountRight) * 1000) / deltaTime;

    if ((speedRight != 0) || (speedLeft != 0)){
      Serial.print("Speed Left = ");
      Serial.print(speedLeft);
      Serial.print("   Right = ");
      Serial.print(speedRight);

  
      Serial.print("   CountLeft = ");
      Serial.print(wheelCountLeft);
      Serial.print("   Right = ");
      Serial.print(wheelCountRight);

      Serial.print("     LastWheelCountR = ");
      Serial.print(lastWheelCountRight);
      Serial.print("   Delta = ");
      Serial.println(wheelCountRight - lastWheelCountRight);

      lastWheelCountRight = wheelCountRight;
      lastWheelCountLeft = wheelCountLeft;
      oldTime = newTime;  // reset timer
    }
  }

  /***
      // Speed Control
      if (0 == targetSpeedRight) {
        // STOP
        // sendMotorCommand(MOTOR_RIGHT, 0); TODO
      }
      else if (speedRight < (targetSpeedRight - SPEED_HYSTERYSIS)) {
        // need to go more positive (either faster forward or slower reverse)
        currentMotorCommandRight += 1; // TODO TUNE THIS
        // sendMotorCommand(MOTOR_RIGHT, currentMotorCommandRight);  TODO

      }
      else if (speedRight > (targetSpeedRight + SPEED_HYSTERYSIS)) {
        // need to go more negative (either slower forward or faster reverse)
        currentMotorCommandRight -= 1; // TODO TUNE THIS
        // sendMotorCommand(MOTOR_RIGHT, currentMotorCommandRight);  TODO

      }
    }
  ***/

  delay(10);
}

