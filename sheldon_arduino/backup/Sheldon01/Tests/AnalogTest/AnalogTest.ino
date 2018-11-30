// Tests for verifying Loki Robot Arduino interface board

const int ledPin =  13;      // the number of the LED pin

int ledState = LOW;             // ledState used to set the LED
long previousMillis = 0;        // will store last time LED was updated
long interval = 500;           // interval at which to blink (milliseconds)

void setup(void) 
{ 
  Serial.begin(19200); 
  pinMode(ledPin, OUTPUT);      

  Serial.println("Init Complete.");
}

void loop(void) 
{

///////////////////////////////////////////////////////////////////
// Arduino Board Voltage read (resistor on the Arduino shield)
  int BatteryInt = analogRead(15);
  delay(20);
  byte BatteryByte = analogRead(15)/4;
  float BatteryVoltage = ( (double)BatteryByte * 0.041 ) + 7.0;

///////////////////////////////////////////////////////////////////
// SHUNT READ
  delay(20);
  int ShuntReadV = analogRead(7);
  float ShuntV = ( (double)ShuntReadV * 0.202265 );
//  float ShuntV = ( (double)ShuntReadV /49.44 );

  delay(20);
  int ShuntReadI = analogRead(6);
  float ShuntI = ( (double)ShuntReadI * 0.30 );  
//  float ShuntI = ( (double)ShuntReadI /14.9 );

/*    
  //Battery = analogRead(15)/4;
  Serial.print("Battery: Byte: ");
  Serial.print(BatteryByte, DEC);
  Serial.print("  Int: ");
  Serial.print(BatteryInt, DEC);
  Serial.print("  Voltage: ");
  Serial.print(BatteryVoltage, 2);
*/
  
/*
  Serial.print("  ShuntV: ");
  Serial.print(ShuntV);

  Serial.print("  ShuntI: ");
  Serial.print(ShuntI);

  Serial.print("  ShuntIRaw: ");
  Serial.print(ShuntReadI);

  Serial.print("  ShuntVRaw: ");
  Serial.print(ShuntReadV);
*/
  Serial.println();

///////////////////////////////////////////////////////////////////
// General Analog read
  Serial.print("Analog Ports:  ");
  int AnalogPort = 0;
  for (int i=0; i<15; i++)
  {
    AnalogPort = analogRead(i);
    Serial.print(i, DEC);
    Serial.print(":");
    Serial.print(AnalogPort, DEC);
    Serial.print("  ");
    delay(20);
  }
  Serial.println();

///////////////////////////////////////////////////////////////////
  // Blink the LED 
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) 
  {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
  
  delay(100);
}



