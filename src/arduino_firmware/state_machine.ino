#define __ASSERT_USE_STDERR
#include <assert.h>
#include <SabertoothSimplified.h>

#define HALT_STATE 0
#define COAST_STATE 1
#define IGNITION_STATE 2
#define ENGINE_START_STATE 3
#define RC_TELEOP_STATE 4
#define AI_READY_STATE 5

#define ENGINE_START_PIN 4
#define NUM_START_ATTEMPTS 4

#define IGNITION_PIN 5

#define RC_FAILSAFE_PIN 6
#define FAILSAFE_LED_PIN 13

#define THROTTLE_SERVO_PIN 9

// If a command has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 250

// These sensitivity values will need to be changed.
#define BRAKE_SENSITIVITY 1.0
#define THROTTLE_SENSITIVITY 0.1
#define STEER_SENSITIVITY 1.0

#define PARK_GEAR_POSITION 100
#define REVERSE_GEAR_POSITION 300
#define NEUTRAL_GEAR_POSITION 500
#define DRIVE_GEAR_POSITION 700

Servo throtte_servo;

float get_pwm_duty_cycle(pwm_pin)
{
    unsigned long highTime pulseIn(inputPin, HIGH);
    unsigned long lowTime pulseIn(inputPin, LOW);
    unsigned long cycleTime = highTime + lowTime;
    return (float)highTime / float(cycleTime);
}

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
    bool engine_currently_running;

    *MotorInterface sabertooth_60A;
    *MotorInterface sabertooth_32A;

    *MotorController brake_motor;
    *MotorController gear_motor;
    *MotorController throttle_motor;
    *MotorController steering_motor;

  public:
    Linda()
    {
        pinMode(FAILSAFE_LED_PIN, OUTPUT);

        set_current_state_ID(HALT_STATE);
        pinMode(ENGINE_START_PIN, OUTPUT);
        digitalWrite(ENGINE_START_PIN, LOW);

        pinMode(IGNITION_PIN, OUTPUT);
        digitalWrite(IGNITION_PIN, LOW);

        pinMode(RC_FAILSAFE_PIN, INPUT);

        lastCommandTimestamp = 0.0;
        x_velocity = 0.0;
        ai_enabled = 0;
        engine_currently_running = false;

        sabertooth_60A = new ST(9600);
        sabertooth_32A = new ST(9600);
        throtte_servo.attach(THROTTLE_SERVO_PIN);

        brake_motor = new MotorController(sabertooth_32A, 1);
        gear_motor = new MotorController(sabertooth_32A, 2);
        steer_motor = new MotorController(sabertooth_60A, 1);
        // throttle_motor = new MotorController(sabertooth_60A, 1); //Does not use sabertooth! Uses a servo motor
    }

    void startEngine()
    {
        for (int i = 1; i < NUM_START_ATTEMPTS; i++)
        {

            Serial.println("Attempting to crank!");

            digitalWrite(ENGINE_START_PIN, HIGH);
            delay(2000 + i * 500); // Increase cranking time by 500ms for each attempt
            digitalWrite(ENGINE_START_PIN, LOW);

            // set flag value for now, we have no way of determining the engine state YET
            engine_currently_running = true;

            if (is_engine_running())
                break;

            delay(2000);
        }
    }

    void stopEngine()
    {
        engine_currently_running = false;
        digitalWrite(IGNITION_PIN, LOW);
        set_current_state_ID(HALT_STATE);
    }

    bool is_engine_running() { return engine_currently_running; } // return flag value for now, we have no way of determining this YET

    int calculate_gear_pos(double x_velocity)
    {
        if (getcurrentStateID() >= ENGINE_START_STATE)
        {
            if (x_velocity => 0)
            {
                gear_position = DRIVE_GEAR_POSITION;
            }
            else
            {
                gear_position = REVERSE_GEAR_POSITION;
            }
        }
        else
        {
            gear_position = PARK_GEAR_POSITION;
        }

        return gear_position;
    }

    int calculate_throttle_pos(double x_velocity) { return int(x_velocity * THROTTLE_SENSITIVITY); }
    int calculate_brake_pos(double x_velocity) { return (x_velocity == 0.0) * BRAKE_SENSITIVITY; }
    int calculate_steer_pos(double cmd_theta) { return int(cmd_theta * STEER_SENSITIVITY); }

    void process_command(double cmd_x_velocity, double cmd_theta)
    {
        long lastCommandTimestamp = millis();
        
        // Will be changed into the HALT state if it is not safe to drive.
        checkFailsafes();

        // This function is called every time a serial (or RC PWM) command is recieved
        switch (currentStateID)
        {
        case HALT_STATE:
            x_velocity = 0.0;
            theta = cmd_theta;
            break;
        
        case RC_TELEOP_STATE:
            x_velocity = cmd_x_velocity;
            theta = cmd_theta;
            break;

        case IGNITION_STATE:
            x_velocity = 0.0;
            theta = 0.0;
            break;

        case ENGINE_START_STATE:
            x_velocity = 0.0;
            theta = 0.0;
            break;

        case AI_READY_STATE:
            x_velocity = cmd_x_velocity;
            theta = cmd_theta;
        }

        // NOW SEND THE COMMANDS TO THE MOTOR CONTROLLERS
        steer_motor->SetTargetPosition(calculate_steer_pos(theta));
        gear_motor->SetTargetPosition(calculate_gear_pos(x_velocity));
        brake_motor->SetTargetPosition(calculate_brake_pos(x_velocity));
        send_throttle_command(calculate_throttle_pos(x_velocity));
    }

    void set_current_state_ID(int newStateID)
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
        case RC_TELEOP_STATE:
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
        bool rcFailsafeValid = get_pwm_duty_cycle(RC_FAILSAFE_PIN) >= 0.5;

        bool safeToDrive = (watchdogValid && rcFailsafeValid);

        if (!safeToDrive)
        {
            set_current_state_ID(HALT_STATE);
        }

        digitalWrite(FAILSAFE_LED_PIN, safeToDrive);

        return safeToDrive;
    }

    void send_throttle_command(int throttle_command)
    {
        throttle_command = constrain(throttle_command, 0, 10);
        throtte_servo.write(throttle_command);
    }
};