#include <Servo.h>
#include <SabertoothSimplified.h>

#include "stuff.h"

#define THROTTLE_PWM_PIN 8
#define STEERING_PWM_PIN 9

Linda l;

void setup()
{
  
    Serial.println("Initialising!");
    pinMode(IGNITION_PIN, OUTPUT);

    delay(3000);
    Serial.println("Ignition!");
    l.set_current_state_ID(IGNITION_STATE);

    delay(250);
    Serial.println("Engine!");
    l.set_current_state_ID(ENGINE_START_STATE);

    Serial.println("Human Drive!");
    l.set_current_state_ID(RC_TELEOP_STATE);
}

void loop()
{
    double x_velocity = get_pwm_duty_cycle(THROTTLE_PWM_PIN);
    double theta = 0.5 - get_pwm_duty_cycle(STEERING_PWM_PIN);

    l.process_command(x_velocity, theta);
}
