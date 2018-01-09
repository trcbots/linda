#define __ASSERT_USE_STDERR
#include <assert.h>

#define HALT_STATE 0
#define IGNITION_STATE 1
#define ENGINE_START_STATE 2
#define HUMAN_DRIVE_STATE 3
#define AI_READY_STATE 4
#define ENGINE_START_STATE 5
#define COAST_STATE 6

#define ENGINE_START_PIN 4
#define NUM_START_ATTEMPTS 4

#define IGNITION_PIN 5

#define RC_FAILSAFE_PIN 6

// If a command has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 500

// These sensitivity values will need to be changed.
#define BRAKE_SENSITIVITY 1.0
#define THROTTLE_SENSITIVITY 1.0
#define STEER_SENSITIVITY 1.0

class Linda
{

  private:
    int currentStateID;
    long lastCommandTimestamp;
    double theta;
    double x_velocity;
    int gear_position;
    bool ai_enabled;
    bool main_relay_on;

  public:
    Linda()
    {
        setcurrentStateID(HALT_STATE);
        pinMode(ENGINE_START_PIN, OUTPUT);
        digitalWrite(ENGINE_START_PIN, LOW);

        pinMode(IGNITION_PIN, OUTPUT);
        digitalWrite(IGNITION_PIN, LOW);

        pinMode(RC_FAILSAFE_PIN, INPUT);

        lastCommandTimestamp = 0.0;
        x_velocity = 0.0;
        ai_enabled = 0;

        delay(3000);
        Serial.println("Ignition!");
        setcurrentStateID(IGNITION_STATE);

        delay(250);
        Serial.println("Engine!");
        setcurrentStateID(ENGINE_START_STATE);
    }

    void startEngine()
    {
        for (int i = 1; i < NUM_START_ATTEMPTS; i++)
        {

            Serial.println("Attempting to crank!");

            digitalWrite(ENGINE_START_PIN, HIGH);
            delay(2000 + i * 500); // Increase cranking time by 500ms for each attempt
            digitalWrite(ENGINE_START_PIN, LOW);

            if (isEngineRunning())
                break;
            delay(2000);
        }
    }

    bool isEngineRunning() { return true; } // return true for now, we have no way of determining this YET

    int calculateThrottlePos(double x_velocity) { return int(x_velocity * THROTTLE_SENSITIVITY); }
    int calculateGearPos(double x_velocity) { return (x_velocity > 0) ? 1 : ((x_velocity < 0) ? -1 : 0); }
    int calculateBrakePos(double x_velocity) { return (x_velocity == 0.0) * BRAKE_SENSITIVITY; }
    int calculateSteerPos(double cmd_theta) { return int(cmd_theta * 1.0); }

    void process_command(double cmd_x_velocity, double cmd_theta)
    {
        long lastCommandTimestamp = millis();

        // This function is called every time a serial (or RC PWM) command is recieved
        switch (currentStateID)
        {
        case HALT_STATE:
            x_velocity = 0.0;
            theta = cmd_theta;
            break;
        // case (HUMAN_DRIVE_STATE || IGNITION_STATE || ENGINE_START_STATE):
        // x_velocity = null;
        // theta = null;
        // break;
        case AI_READY_STATE:
            x_velocity = cmd_x_velocity;
            theta = cmd_theta;
        }
    }

    void setcurrentStateID(int newStateID)
    {
        assert(newStateID <= 6);

        switch (newStateID)
        {
        case IGNITION_STATE:
            if (currentStateID == HALT_STATE)
            {
                Serial.println(":)");
                if (x_velocity != 0.0)
                {
                    return;
                }
                else if (ai_enabled)
                {
                    return;
                }
                else
                {
                    digitalWrite(IGNITION_PIN, HIGH);
                    gear_position = 0;
                    main_relay_on = 1;
                    //Serial.println("Delaying for 5...");
                }
            }
        case ENGINE_START_STATE:
            if (currentStateID == IGNITION_STATE)
            {
                startEngine();
            }
            break;
        case HALT_STATE:
            break;
        case HUMAN_DRIVE_STATE:
            break;
        case AI_READY_STATE:
            break;
        }

        Serial.print("Changing state to: ");
        Serial.println(newStateID);

        currentStateID = newStateID;
    }

    int getcurrentStateID()
    {
        return currentStateID;
    }

    bool checkFailsafes()
    {
        bool watchdogValid = ((millis() - lastCommandTimestamp) < WATCHDOG_TIMEOUT);
        bool rcFailsafeValid = digitalRead(RC_FAILSAFE_PIN);

        bool safeToDrive = (watchdogValid && rcFailsafeValid);

        if (!safeToDrive)
        {
            setcurrentStateID(HALT_STATE);
        }

        return safeToDrive;
    }
};

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