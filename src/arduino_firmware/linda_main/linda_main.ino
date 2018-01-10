#include <Servo.h>
#include <SabertoothSimplified.h>

//#include "serial_command.h"
#include "linda.h"

Linda l;

//SerialCommand sc;

void setup()
{
  
    Serial.begin(115200);
    Serial.println("Initialising!");
//    pinMode(IGNITION_PIN, OUTPUT);

//    delay(3000);
//    Serial.println("Ignition!");
//    l.set_current_state_ID(IGNITION_STATE);
//
//    delay(250);
//    Serial.println("Engine!");
//    l.set_current_state_ID(ENGINE_START_STATE);
//
//    Serial.println("Human Drive!");
//    l.set_current_state_ID(RC_TELEOP_STATE);


l.Init();
l.set_current_state_ID(RC_TELEOP_STATE);

}

void loop()
{
  
  /*
  sc.ReadData();

  if ( sc.message_type != -1 ) {
       Serial.print("valid message:");
       Serial.print(sc.message_type);
       Serial.print(",");
       Serial.print(sc.message_data1);
       Serial.print(",");
       Serial.println(sc.message_data2);
       sc.reset();
    }
  */

  l.process_command();
}
