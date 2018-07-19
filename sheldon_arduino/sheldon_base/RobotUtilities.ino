
/////////////////////////////////////////////////////////////////////////////////////
//  Robot Subroutines
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
boolean TimeToPublishStatus()
{
  static unsigned long LastStatusTime = 0;
  unsigned long CurrentTime = millis();
  if( (CurrentTime - LastStatusTime) > PublishInterval ) 
  {
    LastStatusTime = CurrentTime;    
    return true;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////
boolean TimeToBlinkHeartBeat()
{
  static unsigned long LastBlinkTime = 0;
  unsigned long CurrentTime = millis();
  if( CurrentTime - LastBlinkTime > BlinkInterval ) 
  {
    LastBlinkTime = CurrentTime;    
    return true;
  }
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////
void ToggleHeartBeatLed()
{
  // Blink the Heartbeat LED 
  static boolean LedState = 0;
  LedState = !LedState;
  digitalWrite(HEARTBEAT_LED_PIN, LedState); // Active High
  digitalWrite(STATUS_LED_PIN, !LedState);   // Active Low
  
  // blink the arms too
  //SetArmLED( I2C_PCF8574_LEFT_ARM, LedState );
  //SetArmLED( I2C_PCF8574_RIGHT_ARM, LedState );
  
}

void AllLedsOff()
{
  digitalWrite(HEARTBEAT_LED_PIN, LOW); // Active High
  digitalWrite(STATUS_LED_PIN, HIGH);   // Active Low
  digitalWrite(11, HIGH);   // Active Low
  //SetArmLED( I2C_PCF8574_LEFT_ARM, LOW ); // blink the arms too
  //SetArmLED( I2C_PCF8574_RIGHT_ARM, LOW );
}

void AllLedsOn()
{
  digitalWrite(HEARTBEAT_LED_PIN, HIGH); // Active High
  digitalWrite(STATUS_LED_PIN, LOW);   // Active Low
  digitalWrite(11, LOW);   // Active Low
  //SetArmLED( I2C_PCF8574_LEFT_ARM, HIGH ); // blink the arms too
  //SetArmLED( I2C_PCF8574_RIGHT_ARM, HIGH );
}

/////////////////////////////////////////////////////////////////////////////////////
void PrintDebug(char *Message)
{ 
  nh.loginfo("ANDROID DEBUG: ");\
  nh.loginfo(Message);
  
  /*
  // Sends debug message to the robot control code, which prints the message into the robot log
  int StrSize = strlen(Message);
  // Write the Header
  Serial.write("ZZ");                   // 2 Sync Characters, using Binary mode write 
  Serial.write(StrSize+2);              // size of data, plus header: version + message type
  Serial.write(ARDUINO_CODE_VERSION);   // Version sent with every message, as extra corruption detection   
  Serial.write(ARDUINO_MESSAGE_TEXT);   // Identify this as a Text message   
  // Write the Message  
  Serial.write(Message);
  */  
}

/////////////////////////////////////////////////////////////////////////////////////
void PrintDebugHex(char *Message, int Value)
{
  /*
  // Sends debug message plus a HEX value to the robot control code, which prints the message into the robot log
  // Write the Header
  int StrSize = strlen(Message);
  Serial.write("ZZ");                   // 2 Sync Characters, using Binary mode write 
  Serial.write(StrSize+4);              // size of data, plus header (version + message type), plus 2 HEX characters 
  Serial.write(ARDUINO_CODE_VERSION);   // Version sent with every message, as extra corruption detection   
  Serial.write(ARDUINO_MESSAGE_TEXT);   // Identify this as a Text message   
  // Write the Message  
  Serial.write(Message); 
  if(Value < 0x10)
  {
    Serial.print("0"); // Kludge - print does not print the leading zero! Doh!
  }
  Serial.print(Value, HEX);
  */
}


/////////////////////////////////////////////////////////////////////////////////////
void PrintDebugDec(char *Message, int Value)
{
  /*
  // Sends debug message plus a Decimal value to the robot control code, which prints the message into the robot log
  // Write the Header
  int StrSize = strlen(Message);
  Serial.write("ZZ");                   // 2 Sync Characters, using Binary mode write 
  Serial.write(StrSize+5);              // size of data, plus header (version + message type), plus 3 DEC characters 
  Serial.write(ARDUINO_CODE_VERSION);   // Version sent with every message, as extra corruption detection   
  Serial.write(ARDUINO_MESSAGE_TEXT);   // Identify this as a Text message   
  // Write the Message  
  Serial.write(Message); 
  if(Value < 100)
  {
    Serial.print(" "); // Kludge - need to print 3 chars
  }
  if(Value < 10)
  {
    Serial.print(" "); // Kludge - need to print 3 chars
  }
  Serial.print(Value, DEC);
  */
}









