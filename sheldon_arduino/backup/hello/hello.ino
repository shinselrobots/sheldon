
long odom_ticks_right = 0L;
long odom_ticks_left = 0L;
int speed_ticks_right = 50;
int speed_ticks_left = -50;

char odom_char[45] = "0123456789 0123456789 0123456789 0123456789";


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  Serial.println("hello");

  odom_ticks_right += 10;
  odom_ticks_left -= 10;
  String msgString = String(odom_ticks_right) + " " + String(odom_ticks_left) + " "  + String(speed_ticks_right) + " " + String(speed_ticks_left);
  msgString.toCharArray(odom_char,45);

Serial.println(odom_char);
  
  delay(100);        // delay in between reads for stability
}
