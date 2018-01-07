/*
  Ignition

  Turns on the ignition using two switches. The two switches are:
   - IGN1/Run Relay Turns on the power
   - IGN2/Start     Starts the engine
*/

// INPUT/OUTPUT PINS
int IGN1 = 13;
int IGN2 = 12;
int SWITCH = 11;

// Momentary Switch delay
int switchDelay = 200;

// The current switch value
int switchValue = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize the output pins.
  pinMode(IGN1, OUTPUT);
  pinMode(IGN2, OUTPUT);
  pinMode(SWITCH, INPUT);
}

void turnOn() {
  digitalWrite(IGN1, HIGH);
  digitalWrite(IGN2, HIGH);
  delay(switchDelay);
  digitalWrite(IGN2, LOW);
}

void turnOff() {
  digitalWrite(IGN1, LOW);
  digitalWrite(IGN2, LOW);
}

// the loop function runs over and over again forever
void loop() {
  // TODO: Add debouncer
  switchValue = digitalRead(SWITCH);
  // If the switch is on, turn the ignition on
  if (switchValue == 1) {
    turnOn();
  // Else turn it off
  } else {
    turnOff();
  }
}
