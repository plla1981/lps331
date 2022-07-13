#include "LPS331AP.h"

LPS331AP LPS331AP;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  LPS331AP.begin();
//  if(!LPS331AP.begin())
//  {
//      while(1);
//  }
}

void loop() {
  // put your main code here, to run repeatedly:
  LPS331AP.measure();
  Serial.print("pressure:");
  Serial.print(LPS331AP.pressure);
  Serial.println("mber");
  Serial.print("Temperature:");
  Serial.print(LPS331AP.temperature);
  Serial.println("C°");
  delay(1000);
}
