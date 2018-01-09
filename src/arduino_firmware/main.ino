#include "state_machine.ino"

void setup()
{
    Serial.begin(9600);
    Serial.println("Initialising!");
    pinMode(IGNITION_PIN, OUTPUT);

    Linda();
}

void loop()
{

}