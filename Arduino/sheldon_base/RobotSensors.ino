// RobotSensors:  Functions for reading sensor data and copying to Status block for sending to the PC
#include <Wire.h>



/////////////////////////////////////////////////////////////////////////////////////
void ReadFastDigitalPorts()
{
  // read ports that might have fast (transient) values, like a bumper or switch pressed
  Status.PortC |= Read_Digital_Port_C();  // keep track of any high bits over time
  Status.PortA |= Read_Digital_Port_A(); // function handles bit flipping as needed
  
  //byte LeftArm = ReadPCF8574(I2C_PCF8574_LEFT_ARM);
  //LeftArm = LeftArm ^ 0xC0; // flip the top two bits
  //Status.ArmBumperL |= LeftArm; // keep track of any high bits over time
  
  //byte RightArm = ReadPCF8574(I2C_PCF8574_RIGHT_ARM);
  //RightArm = RightArm ^ 0xC0; // flip the top two bits
  //Status.ArmBumperR |= RightArm; // keep track of any high bits over time
  
  //Status.HeadSensor |= ReadPCF8574(I2C_PCF8574_HEAD); // read capacitive sensor in head (just a touch sensor)
  
}



/////////////////////////////////////////////////////////////////////////////////////
byte Read_Digital_Port_A()
{  
  // Read digital input port A on the Arduino board.  Mega 2560: Pin22=PA0, Pin29=PA7
  byte PortA = PINA; // PORTA;  // bulk read of the full port
  //PortA ^= 0b00111111;	   // Invert Active LOW sensors (IR sensors go low when objected detected, but PIR are active high)
  return PortA;
}


/////////////////////////////////////////////////////////////////////////////////////
byte Read_Digital_Port_C()
{  
  // Read digital input port C on the Arduino board. Mega 256:  Pin30=PC7, Pin37=PC0 (note reverse order!)
  byte PortC = PINC; // PORTC;  // note reversed pin numbers on this one
  //PortC ^= 0b11111111;	// Active LOW, so invert all bits (goes low when bumper hit).  Unused bits are pulled high by PORTC command above.
  return PortC;
}



/////////////////////////////////////////////////////////////////////////////////////
void Read_I2C_IT_IR_Rangers()
{  
  // Read I2C-IT IR Range Sensors via I2C, up to NUM_IR3_SENSORS
  //Status.IR3[0] = I2C_Read_Byte( I2C_IT_SENSOR_ADDR_0, I2C_IT_UNITS_INCHES );  // Left Hand IR sensor
  //Status.IR3[1] = HW_SENSOR_NOT_ATTACHED;                // I2C_IT_SENSOR_ADDR_1, Left Arm IR sensor - Removed!
  //Status.IR3[2] = I2C_Read_Byte( I2C_IT_SENSOR_ADDR_2, I2C_IT_UNITS_INCHES );  // Right Hand IR sensor
  //Status.IR3[3] = HW_SENSOR_NOT_ATTACHED;                // I2C_IT_SENSOR_ADDR_3, Right Arm IR sensor - Removed!

}

/////////////////////////////////////////////////////////////////////////////////////
/*
void Read_Hand_Pressure()
{  
  // Read Hand pressure sensors via I2C Adafruit ADS1015 4 channel A2D
  //int16_t adc0;
  int adc;

  adc = ads1015.readADC_SingleEnded(0);  
  Status.LeftHandPressureL = (byte)((adc >>3)& 0xFF);
  adc = ads1015.readADC_SingleEnded(1);  
  Status.LeftHandPressureR = (byte)((adc >>3)& 0xFF);

//  adc1 = ads1015.readADC_SingleEnded(1);  
//  adc2 = ads1015.readADC_SingleEnded(2);  
//  adc3 = ads1015.readADC_SingleEnded(3);  
//  Serial.print("AIN0: "); Serial.println(adc0);
//  Serial.print("AIN1: "); Serial.println(adc1);
//  Serial.println(" ");
//  int temp0 = 0;
  //temp0 = adc0 >>3;
  //Status.LeftHandPressureR = (byte)( temp0 & 0xFF);	// Low Byte
}
*/

/////////////////////////////////////////////////////////////////////////////////////
void Read_Next_Analog_Port()
{  
  // Read Analog Ports on the Arduino board
  // this gets called twice each 20ms, allowing all ports to be read within 100-200ms (depending upon the number of A/D ports enabled by NUM_AD_IR_SENSORS)

  static int SensorToRead = 0;
  if( SensorToRead < NUM_AD_IR_SENSORS )
  {
    Status.RangeSensor[SensorToRead] = analogRead(SensorToRead)/4; // divide by 4 to make the range 0-255  (ignore extra precision, the sensor is not that precise)
    // delay 10ms to let the ADC recover:
    //delay(10); // this delay removed, because the function is only called every so often
    SensorToRead++;
  }
  else
  {
    /*  
  }
    // Analog port 15 is the battery voltage
    Status.Battery0 = analogRead(15)/4;
    PowerIsOn = CheckPower(Status.Battery0); // set global flag if power is not on
    if( !PowerIsOn )
    {
        AllLedsOff();  // turn everyting off!  No power!
        SetLedEyes( 0x00  );
    }
    */
    
    SensorToRead = 0;
  }
}


/////////////////////////////////////////////////////////////////////////////////////
boolean CheckPower(byte BatteryByte)
{
  /*
  // See if power is turned on, returns true if on
  if( 0 == BatteryByte )
  {
    // No voltage passed in, so read the port
    BatteryByte = analogRead(15)/4;
  }
    float BatteryVoltage = ( (double)BatteryByte * 0.041 ) + 7.0;
    if( BatteryVoltage > 8.5 ) // USB only returns about 8.3v
    {
      return true; // Power is on
    }
    return false;
*/
}

/////////////////////////////////////////////////////////////////////////////////////
unsigned int Read_Compass()
{  
  // Read Devantech CMPS11 Compass via I2C
  // Result is in Tenth Degrees, so 3600 = 360.0 degrees
  // from example by James Henderson, Devantech

  unsigned char high_byte, low_byte, angle8;
  char pitch, roll;
  unsigned int angle16;


  Wire.beginTransmission(I2C_CMPS11_COMPASS_ADDR);  //starts communication with CMPS11
  Wire.write(COMPASS_ANGLE_8_REG);  //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS11
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(I2C_CMPS11_COMPASS_ADDR, 5);       
  
  while(Wire.available() < 5);        // Wait for all bytes to come back
  
  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  
  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
    
  //Serial.print("roll: ");               // Display roll data
  //Serial.print(roll, DEC);
  //Serial.print("    pitch: ");          // Display pitch data
  //Serial.print(pitch, DEC);
  //Serial.print("    angle full: ");     // Display 16 bit angle with decimal place
  //Serial.print(angle16 / 10, DEC);
  //Serial.print(".");
  //Serial.print(angle16 % 10, DEC);
  //Serial.print("    angle 8: ");        // Display 8bit angle
  //Serial.println(angle8, DEC);
  
  return angle16;
}


  


/////////////////////////////////////////////////////////////////////////////////////
byte I2C_Read_Byte( int deviceaddress, int datamode ) 
{
  byte rdata = 0;
  Wire.beginTransmission(deviceaddress);
  Wire.write(datamode); 
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1, true);
  delay(5);
  if (Wire.available()) rdata = Wire.read();
  Wire.endTransmission();
  return rdata;
}

/////////////////////////////////////////////////////////////////////////////////////
/*
void SetLedEyes( byte EyeState )
{
// I2C device #3 (in Loki's Head)
  byte PortCmd = 0;
  PortCmd = EyeState | 0b00000001; // P0 = US Read enable.  High = Enabled (TODO - Toggle as needed to coordinate with other US sensors)
  PortCmd = PortCmd  | 0b00000010; // P1 = Input Port for Capacitive Sensor.  Keep high (floating) to allow read
  PortCmd = PortCmd  & 0b11111011; // P2 = N/C Transistor.  Keep low (transistor off) by default
  
  Wire.beginTransmission(I2C_PCF8574_HEAD);
  Wire.write(PortCmd);  // 
  Wire.endTransmission();
}
*/

/////////////////////////////////////////////////////////////////////////////////////
void SetArmLED( byte DeviceAddress, boolean LedOn)
{
/*
// I2C device in Left Arm
  byte PortCmd = 0;
  if( LedOn )  
    PortCmd = 0xFE;
  else
    PortCmd = 0xFF;
  
  Wire.beginTransmission(DeviceAddress);
  Wire.write(PortCmd); 
  Wire.endTransmission();
*/
}

/////////////////////////////////////////////////////////////////////////////////////
byte ReadPCF8574(int deviceaddress)
{
/*
  byte data = 0;
    Wire.requestFrom(deviceaddress, 1);
  delay(1);
  if (Wire.available()) {
    data = Wire.read();
  }

  return data;
*/
}


/////////////////////////////////////////////////////////////////////////////////////
void ReadSensors()
{
//    unsigned long SensorStartTime = millis();
  
  // Read all Sensors, and update the status block for sending to the PC

  
//  ReadKneeEncoder();
  ReadFastDigitalPorts();  
  Read_I2C_IT_IR_Rangers();
  //Read_Hand_Pressure();
  //Read_Compass();
  //Status.StatusFlags = HW_STATUS_BRAKE_COMPLETE; 

}

void ResetSensors()
{
  // clear fast sensors that are sampled over time
  Status.PortA = 0;
  Status.PortC = 0;
  //Status.ArmBumperL = 0;
  //Status.ArmBumperR = 0;
  Status.HeadSensor = 0;
  
}

