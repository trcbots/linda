#include "state_machine.ino"

#define THROTTLE_PWM_PIN 8
#define STEERING_PWM_PIN 9

float get_pwm_duty_cycle(pwm_pin)
{
    unsigned long highTime pulseIn(inputPin, HIGH);
    unsigned long lowTime pulseIn(inputPin, LOW);
    unsigned long cycleTime = highTime + lowTime;
    return (float)highTime / float(cycleTime);
}

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
    l.set_current_state_ID(HUMAN_DRIVE_STATE);
}

void loop()
{
    x_velocity = get_pwm_duty_cycle(THROTTLE_PWM_PIN);
    theta = 0.5 - get_pwm_duty_cycle(STEERING_PWM_PIN);

    l.process_command(x_velocity, theta);
}