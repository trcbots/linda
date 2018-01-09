
// Software Serial Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

// #include <SoftwareSerial.h>
// #include <SabertoothSimplified.h>

// SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
// SabertoothSimplified ST(SWSerial);      // Use SWSerial as the serial port.

class MotorController
{
    public:
      MotorController(SabertoothSimplified* _motor_interface, int _motor_id, int _feedback_pin, int _motor_min_pos, int _motor_max_pos, double _Kp = 0.5, double _Ki = 0.0, double _Kd = 0.0)
      {
          // init the motor controller here
          this->motor_id = _motor_id;
          this->motor_interface = _motor_interface;
          this->Kp = _Kp;
          this->Ki = _Ki;
          this->Kd = _Kd;
          this->feedback_pin = _feedback_pin;
          this->motor_min_pos = _motor_min_pos;
          this->motor_max_pos = _motor_max_pos;
        }

        void SetTargetPosition(double target_pos)
        {
            if (target_pos < motor_min_pos)
                target_pos = motor_min_pos;
            else if (target_pos > motor_max_pos)
                target_pos = motor_max_pos;

            double output = 0;

            do
            {
                double current_pos = double(analogRead(feedback_pin));

                double pTerm = current_pos - target_pos;
                // FIXME
                double iTerm = 0.0;
                double dTerm = 0.0;
                output = int(Kp * pTerm + Ki * iTerm + Kd * dTerm);
                output = constrain(output, -127, 127);

//                Serial.print(sprintf("Motor ID: %d, Current Pos: %d, Output Command: %d\n", motor_id, current_pos, output));

                motor_interface->motor(motor_id, output);
            } while (abs(output) > 10);
        }

    private:
      SabertoothSimplified* motor_interface;
      int motor_id;
      int feedback_pin;
      double Kp;
      double Kd;
      double Ki;
      int motor_min_pos;
      int motor_max_pos;
};

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

float get_pwm_duty_cycle(int pwm_pin)
{
    unsigned long highTime = pulseIn(pwm_pin, HIGH);
    unsigned long lowTime = pulseIn(pwm_pin, LOW);
    unsigned long cycleTime = highTime + lowTime;
    return (float)highTime / float(cycleTime);
}

class Linda
{
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

        Serial1.begin(9600);
        Serial2.begin(9600);

        sabertooth_60A = new SabertoothSimplified(Serial1);
        sabertooth_32A = new SabertoothSimplified(Serial2);
        throtte_servo.attach(THROTTLE_SERVO_PIN);

        brake_motor = new MotorController(sabertooth_32A, 1, A3, 100, 1023);
        gear_motor = new MotorController(sabertooth_32A, 2, A4, 100, 1023);
        steer_motor = new MotorController(sabertooth_60A, 1, A5, 100, 100);
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
        int gear_position = -1;
        if (getcurrentStateID() >= ENGINE_START_STATE)
        {
            if (x_velocity >= 0)
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
                    gear_motor->SetTargetPosition(calculate_gear_pos(x_velocity));
                    current_gear_position = PARK_GEAR_POSITION;
                    digitalWrite(IGNITION_PIN, HIGH);
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
    
private:
    int currentStateID;
    long lastCommandTimestamp;
    double theta;
    double x_velocity;
    int current_gear_position;
    bool ai_enabled;
    bool main_relay_on;
    bool engine_currently_running;
    Servo throtte_servo;

    SabertoothSimplified* sabertooth_60A;
    SabertoothSimplified* sabertooth_32A;

    MotorController* brake_motor;
    MotorController* gear_motor;
    MotorController* throttle_motor;
    MotorController* steer_motor;
    
};




