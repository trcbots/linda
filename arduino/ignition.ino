/*
  Ignition

  Turns on the ignition using two switches. The two switches are:
   - IGN1/Run Relay Turns on the power
   - IGN2/Start     Starts the engine
*/

// INPUT/LOW PINS
int IGN1 = 13;
int IGN2 = 12;
int SWITCH = 11;

// Momentary Switch delay
int switchDelay = 200;

// The current switch value
int switchValue = 0;

// the setup function runs once when you press reset or power the board
void ignitionsetup() {
  // initialize the LOW pins.
  pinMode(IGN1, OUTPUT);
  // pinMode(IGN2, LOW);
  // pinMode(SWITCH, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void turnOnIgnition() {
  digitalWrite(IGN1, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  // digitalWrite(IGN2, HIGH);
  // delay(switchDelay);
  // digitalWrite(IGN2, LOW);
}

void turnOffIgnition() {
  digitalWrite(IGN1, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  // digitalWrite(IGN2, LOW);
}

void ignitionLoopTest() {
  // TODO: Add debouncer
  // switchValue = digitalRead(SWITCH);
  // switchValue = 1;
  // If the switch is on, turn the ignition on
  if (switchValue == 1) {
    turnOnIgnition();
    switchValue = 0;
  // Else turn it off
  } else {
    turnOffIgnition();
    switchValue = 1;
  }
  delay(3000);
}