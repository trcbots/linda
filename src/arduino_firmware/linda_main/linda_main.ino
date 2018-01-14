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

    /* ENGINE AUTOSTART (disabled for now..., RC operator can do this manually)
    delay(3000);

    // Turn on the ignition
    // (i.e. turn the ignition to the ON state, but not start the engine, yet)
    Serial.println("Ignition!");
    l.set_current_state_ID(IGNITION_STATE);

    delay(250);

    // Ok, lets start the engine!
    Serial.println("Engine!");
    l.set_current_state_ID(ENGINE_START_STATE);
    */

    // Lets go into the Remote Control Teleoperation state!
    // This control the car based on PWM commands read from the RC reciever
    l.Init();
    l.set_current_state_ID(RC_TELEOP_STATE);
    
    // Please note that AI state is currently untested!

}

void loop()
{
    
  /*
  // If AI mode is used, we read commands from the serial port
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


  // process_command() MUST be called in the main loop
  // This is where all of the driverless car goodness happens
  l.process_command();
}
