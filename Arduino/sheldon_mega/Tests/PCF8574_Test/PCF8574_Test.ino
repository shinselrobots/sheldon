// Tests for verifying Loki Robot Arduino interface board I2C expansion chips
// Blink LEDS in the ARMS and robot EYES

#include <Wire.h>

const byte   I2C_PCF8574_LEFT_ARM =   0x21; // Left Arm Digital I/O
const byte   I2C_PCF8574_HEAD =       0x22; // Head Digital I/O
const byte   I2C_PCF8574_RIGHT_ARM =  0x23; // Right Arm Digital I/O

//#define LED_PIN    13
const int ledPin =  13;      // the number of the LED pin

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 500;           // interval at which to blink (milliseconds)

void setup(void) 
{ 
  Serial.begin(19200); 
  Wire.begin(); // join i2c bus (address optional for master)
  pinMode(ledPin, OUTPUT);      
  Serial.println("Init Complete.  Normal reading (not triggered) should be 7E for each hand");
}

void loop(void) 
{

  // Blink Arm LEDS
    SetArmLED( I2C_PCF8574_LEFT_ARM, HIGH ); // blink the arms too
    SetArmLED( I2C_PCF8574_RIGHT_ARM, HIGH );


  // Blink the LED 
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;   
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    digitalWrite(ledPin, ledState);
  }
  
  byte LeftArm = ReadPCF8574(I2C_PCF8574_LEFT_ARM);
  LeftArm = LeftArm ^ 0xC0; // flip the top two bits
  Serial.print("LeftArm = ");
  Serial.print(LeftArm, HEX);
 
  byte RightArm = ReadPCF8574(I2C_PCF8574_RIGHT_ARM);
  RightArm = RightArm ^ 0xC0; // flip the top two bits
  Serial.print("    RightArm = ");
  Serial.print(RightArm, HEX);
 
  Serial.println();
  delay(100);
}

//////////////////////////////////////////////////////////////////////////////////
byte ReadPCF8574(int deviceaddress)
{
  byte data = 0;
    Wire.requestFrom(deviceaddress, 1);
  delay(1);
  if (Wire.available()) {
    data = Wire.read();
  }

  return data;
}

/////////////////////////////////////////////////////////////////////////////////////
void SetArmLED( byte DeviceAddress, boolean LedOn)
{
// I2C device in Left Arm
  byte PortCmd = 0;
  if( LedOn )  
    PortCmd = 0xFE;
  else
    PortCmd = 0xFF;
  
  Wire.beginTransmission(DeviceAddress);
  Wire.write(PortCmd); 
  Wire.endTransmission();
}

