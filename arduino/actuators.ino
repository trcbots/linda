/**
 * Controls the linear actuators that adjusts the accelerator, the breaks
 * and the gear shifts 
 *
 */
#include <SoftwareSerial.h>
SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).


void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("M1:get");   // send an initial string
    delay(300);
  }
}

void actuatorsSetup() {
  SWSerial.begin(9600);
  Serial.begin(9600);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }

  // establishContact();
}

void actuatorsTestloop() {
  // if (Serial.available() > 0) {
  //   Serial.println("M1:30");
  //   delay(1000);
  //   Serial.println("M1:-30");
  //   delay(1000);
  //   Serial.println("M1:0");
  //   delay(2000);
  // }
  // int power;
  
  // Ramp from -127 to 127 (full reverse to full forward), waiting 20 ms (1/50th of a second) per value.
  // for (power = -30; power <= 30; power ++)
  // {
  //   SWSerial.println("M1:");
  //   Serial.println(power);
  //   delay(20);
  // }
  
  // // Now go back the way we came.
  // for (power = 30; power >= -30; power --)
  // {
  //   ST.motor(1, power);
  //   Serial.println(power);    
  //   delay(20);
  // }
  SWSerial.println("M1:127");
  Serial.println("M1:127");
  delay(500);
  SWSerial.println("M1:-127");
  Serial.println("M1:-127");
  delay(500);
  SWSerial.println("M1:0");
  Serial.println("M1:0");
  delay(500);
}