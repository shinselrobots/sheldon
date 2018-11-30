// Bluetooth:  Functions for reading data from Android phone via Bluetooth



void ConnectionEvent(byte flag, byte numOfValues)
{
  int Value = meetAndroid.getInt();
  Android.Connected = Value;

  if (1 == Value)
  {
    PrintDebug("Android Connected");
  }
  else if (0 == Value)
  {
    PrintDebug("Android Disconnected");
  }
  else
  {
    PrintDebug("Android Connect Error!");
  }
  
  Android.Cmd = 0;
  
}


void EnableAccelerometerEvent(byte flag, byte numOfValues)
{
  int Value = meetAndroid.getInt();
  Android.AccEnabled = Value;

  if (1 == Value)
  {
    PrintDebug("Android Acc Enabled");
  }
  else if (0 == Value)
  {
    PrintDebug("Android Acc Disabled");

    bt_motor_cmd_msg.buttons[4] = 0;  // Disable Deadman Switch      
    bt_motor_cmd_msg.axes[0] = 0.0;
    bt_motor_cmd_msg.axes[1] = 0.0;
    bt_motor_cmd_msg.header.stamp = nh.now();
    pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
    debugString = String("DEBUG: BlueTooth Accelerometer Disabled, Motor Stopped");
    debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
    nh.loginfo(debugStringChar);
  
  }
  else
  {
    //PrintDebug("Android Acc Msg Error!");
  }
  // Send message back to Android Phone
  unsigned char charMsg[32];
  String strMsg = "Arduino: AccEn = " + String(Value, DEC);
  strMsg.getBytes(charMsg, 31);
  meetAndroid.send((char*)charMsg);


}

void eXecuteCmdEvent(byte flag, byte numOfValues)
{
  // Command received from the Android device
  /*
    // if this info is needed, convert to PrintDebug statements
    Serial.print("Received Data: ");
    Serial.print(numOfValues);
    Serial.print(" items.  Value = ");
  */
  int Value = meetAndroid.getInt();
  Android.Cmd = Value;

  // Send message back to Android Phone
  unsigned char charMsg[32];
  String strMsg = "Arduino: Received " + String(Value, DEC);
  strMsg.getBytes(charMsg, 31);
  meetAndroid.send((char*)charMsg);
  PrintDebug((char*)charMsg); // send to ROS
}

void AccelerometerEvent(byte flag, byte numOfValues)
{
  // Received Azimuth (compass), Pitch, Roll
  int AccelerometerValues[3];
  //Serial.print("Received Acc Az,Pitch,Roll: ");
  //Serial.print(numOfValues);
  //Serial.print(" items.  Values = ");

  meetAndroid.getIntValues(AccelerometerValues);

  // TODO: make the math work!
  // values should be in the range of -1.0 to +1.0 for Joystick messages 
  // NOTE: See LOKI:  HWInterfaceArduino.cpp, line 500 for scaling!
  Android.Compass = (float)AccelerometerValues[0];
  Android.Pitch = ((float)(AccelerometerValues[1]) * -0.02) + 0.1;
  Android.Roll =  (float)(AccelerometerValues[2]) * -0.02;

  // Small Deadband, with bias to allow phone to be tilted slightly up
  if ( (Android.Pitch > -0.11) && (Android.Pitch < 0.11) ) {
    Android.Pitch = 0.0;// Dead Band
  }
  // Move faster when going forward, slower when backing up
  if (Android.Pitch < -0.1) {
    // backing up
    Android.Pitch /= 2.0;
  }
  /*  
  if ( (Android.Roll > -0.02) && (Android.Roll < 0.02) ) {
    Android.Roll = 0; // Dead Band
  }
  */  
  Android.UpdatePending = true;		        // Indicate that a new value is available

  /*
    // To Debug, Send message back to Android Phone
    unsigned char charMsg[32];
    String strMsg = "Arduino: Pitch = " + String(Android.PitchLow, DEC);
    strMsg.getBytes(charMsg, 31);
    meetAndroid.send((char*)charMsg);
    // and Print message on the PC:
    PrintDebugDec("Pitch = ", Android.PitchLow);
  */

}
