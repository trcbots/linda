#include <Servo.h>
#include <SabertoothSimplified.h>

#include "stuff.h"

Linda l;

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


l.set_current_state_ID(RC_TELEOP_STATE);

}

void loop()
{
  l.process_command();
}
