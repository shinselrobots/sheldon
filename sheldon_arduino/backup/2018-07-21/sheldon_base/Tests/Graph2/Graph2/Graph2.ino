int x = 0;
int row = 0;
void setup() {
  Serial.begin(128000); // opens serial port, sets data rate to 9600 bps
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,x,sin(x)");
}

void loop() {
  Serial.print("DATA,TIME,"); Serial.print(x); Serial.print(","); Serial.println(sin(x*PI/180));
  row++;
  x++;
  if (row > 360) 
   {
    row=0;
    Serial.println("ROW,SET,2");
   }
  delay(100);
}
