#include <Servo.h>
#include "motor_controller.h"
#include "serial_command.h"

// These are the names of the states that the car can be in
#define HALT_STATE              0
#define COAST_STATE             1
#define IGNITION_STATE          2
#define ENGINE_START_STATE      3
#define RC_TELEOP_STATE         4
#define AI_READY_STATE          5
#define NEUTRAL_SWITCH_STATE    6
#define AUTONOMOUS_SWITCH_STATE 7


/************************ ARDUINO PIN DEFINITIONS ********************************/
// PWM input pins from RC Reciever
#define RC_ENGINE_START_PWM_PIN             11 // RC PIN 8
#define RC_IGNITION_PWM_PIN                 10 // RC PIN 7
#define RC_FAILSAFE_PIN    RC_IGNITION_PWM_PIN // RC PIN 7
#define THROTTLE_PWM_PIN                     5 // RC PIN 2
#define STEERING_PWM_PIN                     6 // RC PIN 1
#define THROTTLE_SERVO_PIN                   3 // THROTTLE SERVO MOTOR SIGNAL
#define RC_GEAR_SWITCH_PIN                   9 // RC PIN 6

// Digital output pins
#define ENGINE_START_RELAY_PIN  8           // ENGINE START RELAY OUTPUT
#define IGNITION_RELAY_PIN      7           // IGNITION RELAY OUTPUT
#define FAILSAFE_LED_PIN       13           // OUTPUT TO LED ON THE ARDUINO BOARD

// Analog input pins
#define NEUTRAL_CONTROL_SWITCH_PIN            A0
#define AUTONOMOUS_CONTROL_SWITCH_PIN         A1
#define BRAKE_ACTUATOR_POSITION_SENSOR_PIN    A3
#define GEAR_ACTUATOR_POSITION_SENSOR_PIN     A4
#define STEERING_ACTUATOR_POSITION_SENSOR_PIN A5

// Motor driver Pins (UART Serial)
// S1 on the sabertooth 2x60A goes to Arduino Mega pin 12 (Serial1 TX)
// S1 on the sabertooth 2x32A goes to Arduino Mega pin 2 (Serial2 TX)

/*********************************************************************************/

/************************ DRIVE CONTROL DEFINEs **********************************/
// These parameters adjust how the car will behave.

// They will need to be changed according to the particular vehicle.
// However, most values provided should be fairly suitable for  configurations.

// Sensitivity values define how responsive the actuators are to a given input
#define BRAKE_SENSITIVITY     1.0
#define THROTTLE_SENSITIVITY  1.0
#define STEERING_SENSITIVITY  1.0

// Max power applies a constraint to the driver output speed.
// Important note: set these low for testing so you don't destroy anything
#define BRAKE_MOTOR_MAX_POWER    115
#define GEAR_MOTOR_MAX_POWER     125
#define STEERING_MOTOR_MAX_POWER 100

// PID values for each motor driver
// Important note: These values are optional
#define BRAKE_MOTOR_P_VALUE 0.5
#define BRAKE_MOTOR_I_VALUE 0
#define BRAKE_MOTOR_D_VALUE 0

#define GEAR_MOTOR_P_VALUE 0.8
#define GEAR_MOTOR_I_VALUE 0
#define GEAR_MOTOR_D_VALUE 0

#define STEERING_MOTOR_P_VALUE 0.5
#define STEERING_MOTOR_I_VALUE 0
#define STEERING_MOTOR_D_VALUE 0

// Gear positions define where the gear actuator has to travel to engage a specified gear
#define PARK_GEAR_POSITION    840
#define REVERSE_GEAR_POSITION 500
#define NEUTRAL_GEAR_POSITION 300
#define DRIVE_GEAR_POSITION   105

// How close should the analog feedback reading be to the actual position, as confirmation that we are actually in the specified gear
// An absolute difference threshold
#define GEAR_FEEDBACK_TOLERENCE 20

// Define the allowable range of motion for the brake actuator
#define BRAKE_FULLY_ENGAGED_POSITION  420 // 100
#define BRAKE_NOT_ENGAGED_POSITION    800 // 1023

// Define the allowable range of motion for the throttle servo actuator
#define THROTTLE_SERVO_ZERO_POSITION 0
#define THROTTLE_SERVO_FULL_POSITION 40

// Define the allowable range of motion for the steering actuator
#define STEERING_FULL_LEFT   50
#define STEERING_FULL_RIGHT  1080

// Define the limits on Steering PWM input from the RC Reciever
// In RC Mode: these values will get mapped to STEERING_FULL_LEFT and STEERING_FULL_RIGHT respectively
#define RC_STEERING_FULL_LEFT_POSITION   1025
#define RC_STEERING_FULL_RIGHT_POSITION  1975

// Define the limits on Throttle PWM input from the RC Reciever
// In RC Mode: these values will get mapped to THROTTLE_SERVO_ZERO_POSITION and THROTTLE_SERVO_FULL_POSITION respectively
#define RC_THROTTLE_FULL_REVERSE_POSITION   1500
#define RC_THROTTLE_FULL_FORWARD_POSITION   1975

// Define the limits on Brake PWM input from the RC Receiver
// In RC Mode: these values will get mapped to BRAKE_SERVO_ZERO_POSITION and BRAKE_SERVO_FULL_POSITION respectively
#define RC_BRAKE_FULL_REVERSE_POSITION   1500
#define RC_BRAKE_FULL_FORWARD_POSITION   1025

// RC stick DEADZONEs are optionally used to adjust the ergonomics of RC control
// 0.0 values will disable them
#define RC_STEERING_DEADZONE   0.0
#define RC_THROTTLE_DEADZONE   0.0

// PWM input thresholds on the RC 3-way switch, these will map to gear positions
#define RC_DUTY_THRESH_PARK     1100
#define RC_DUTY_THRESH_REVERSE  1400
#define RC_DUTY_THRESH_DRIVE    1600


// PWM input thresholds on the ignition and start switches, relays will be activated if the thresholds are reached
#define RC_DUTY_THRESH_IGNITION     1500
#define RC_DUTY_THRESH_START_ENGINE 1500

/**********************************************************************************/

//Used in AI mode only
#define AUTOSTART_NUM_START_ATTEMPTS 4 // These parameters adjust how the car will behave.

// If a command from the RC or AI has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 250

SerialCommand sc;

class Linda
{
  public:
    Linda()
    {
        // Initialise pins
        pinMode(RC_ENGINE_START_PWM_PIN, INPUT);
        pinMode(RC_IGNITION_PWM_PIN, INPUT);
        pinMode(RC_FAILSAFE_PIN, INPUT);
        pinMode(THROTTLE_PWM_PIN, INPUT);
        pinMode(STEERING_PWM_PIN, INPUT);
        pinMode(RC_GEAR_SWITCH_PIN, INPUT);


        // Initialise class member variables
        pinMode(FAILSAFE_LED_PIN, OUTPUT);
        pinMode(ENGINE_START_RELAY_PIN, OUTPUT);
        digitalWrite(ENGINE_START_RELAY_PIN, LOW);

        pinMode(IGNITION_RELAY_PIN, OUTPUT);
        digitalWrite(IGNITION_RELAY_PIN, LOW);

        lastCommandTimestamp = 0.0;
        x_velocity = 0.0;
        x_velocity_sensed = -1.0;
        ai_enabled = 0;
        engine_currently_running = false;
    }

    void Init() {

      // Seperate function for Initialising the motor and servo controllers
      // In Arduino, these init calls do not work from the class constructor

      // Initialise the servo motor
      throttle_servo.attach(THROTTLE_SERVO_PIN);
      delay(5);
      throttle_servo.write(0);

      // Initialise 9600 baud communication with the Sabertooth Motor Controllers
      SoftwareSerial ST32port(NOT_A_PIN, 2);  // RX on no pin (unused), TX on pin 2 (to S1).
      SoftwareSerial ST60port(NOT_A_PIN, 12); // RX on no pin (unused), TX on pin 12 (to S1).
      ST32port.begin(9600);
      ST60port.begin(9600);
      sabertooth_60A = new SabertoothSimplified(ST60port);
      sabertooth_32A = new SabertoothSimplified(ST32port);

      // Initialise Motor Controllers for Brake, Gear and Steering
      brake_motor = new MotorController(
                      "Brake motor", sabertooth_32A, 1,
                      BRAKE_ACTUATOR_POSITION_SENSOR_PIN,
                      BRAKE_FULLY_ENGAGED_POSITION, BRAKE_NOT_ENGAGED_POSITION,
                      BRAKE_MOTOR_MAX_POWER, BRAKE_MOTOR_P_VALUE,
                      BRAKE_MOTOR_I_VALUE, BRAKE_MOTOR_D_VALUE);

      gear_motor = new MotorController(
                      "Gear motor", sabertooth_32A, 2,
                      GEAR_ACTUATOR_POSITION_SENSOR_PIN,
                      DRIVE_GEAR_POSITION, PARK_GEAR_POSITION,
                      GEAR_MOTOR_MAX_POWER, GEAR_MOTOR_P_VALUE,
                      GEAR_MOTOR_I_VALUE, GEAR_MOTOR_D_VALUE);

      steer_motor = new MotorController(
                      "Steering motor", sabertooth_60A, 1,
                      STEERING_ACTUATOR_POSITION_SENSOR_PIN,
                      STEERING_FULL_LEFT, STEERING_FULL_RIGHT,
                      STEERING_MOTOR_MAX_POWER, STEERING_MOTOR_P_VALUE,
                      STEERING_MOTOR_I_VALUE, STEERING_MOTOR_D_VALUE);
    }

    void startEngine()
    {
        // Engine AUTOSTART functionallity
        // Used in AI mode ONLY
        // This will attempt to start the engine, with multiple attempts on failure to do so

        for (int i = 1 ; i < AUTOSTART_NUM_START_ATTEMPTS ; i++)
        {
            Serial.println("Attempting to crank!");

            digitalWrite(ENGINE_START_RELAY_PIN, HIGH);
            delay(650 + i * 150); // Increase cranking time by 500ms for each attempt
            digitalWrite(ENGINE_START_RELAY_PIN, LOW);

            // set flag value for now, we have no way of determining the engine state YET
            engine_currently_running = true;

            if (is_engine_running())
                break;

            delay(2000);
        }
    }

    void stopEngine()
    {
        // Will stop the engine

        Serial.println("In stopEngine");
        engine_currently_running = false;
        digitalWrite(IGNITION_RELAY_PIN, LOW);
        //set_current_state_ID(HALT_STATE);
    }

    bool is_engine_running()
    {
        // return flag value for now, we have no way of determining this YET
        // Should hook up some kind of sensor and return the digital reading

        return engine_currently_running;
    }

    int calculate_gear_pos(double x_velocity)
    {
        // USED IN AI MODE ONLY!
        // Determine what gear we should be in based on the velocity command from the serial port

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

    int rc_read_gear_pos()
    {

     // USED IN RC MODE ONLY!
     // Determine what gear we shold be in based on switch input from the RC reciever

      double duty = read_pwm_value(RC_GEAR_SWITCH_PIN);
      Serial.print("rc remote gear value: 1100,1400,1600: ");
      Serial.println(duty);

      if (duty < RC_DUTY_THRESH_PARK) // ~1050, < 1100
      {
        return PARK_GEAR_POSITION;
      }

      if (duty < RC_DUTY_THRESH_REVERSE) // ~1350 < 1400
      {
        return REVERSE_GEAR_POSITION;
      }

      return DRIVE_GEAR_POSITION;
    }

    double calculate_throttle_pos(double x_velocity) {
      // The throttle pos is calculated from RC commands

      if(x_velocity > 1550)
      {
        x_velocity = (x_velocity - RC_THROTTLE_FULL_REVERSE_POSITION) * (THROTTLE_SERVO_FULL_POSITION - THROTTLE_SERVO_ZERO_POSITION) /
          (RC_THROTTLE_FULL_FORWARD_POSITION - RC_THROTTLE_FULL_REVERSE_POSITION) + THROTTLE_SERVO_ZERO_POSITION;

        Serial.print("calculate_throttle_pos=");
        Serial.println(x_velocity * THROTTLE_SENSITIVITY);

        return x_velocity * THROTTLE_SENSITIVITY;
      }

      return 0;
    }

    double calculate_brake_pos(double x_velocity)
    {
      if(x_velocity < 1450)
      {
        x_velocity = BRAKE_NOT_ENGAGED_POSITION - (RC_BRAKE_FULL_REVERSE_POSITION - x_velocity);
        return x_velocity * BRAKE_SENSITIVITY;
      }

      return BRAKE_NOT_ENGAGED_POSITION;
    }

    double calculate_steer_pos(double cmd_theta) {
        // The throttle pos is calculated from RC commands
        cmd_theta = (cmd_theta - RC_STEERING_FULL_LEFT_POSITION) * (STEERING_FULL_RIGHT - STEERING_FULL_LEFT) /
                        (RC_STEERING_FULL_RIGHT_POSITION - RC_STEERING_FULL_LEFT_POSITION) +
                    STEERING_FULL_LEFT;

        //Serial.print("calculate_steer_pos=");
        //Serial.println(cmd_theta * STEERING_SENSITIVITY);

        return cmd_theta * STEERING_SENSITIVITY;
    }

    void process_command(int cmd_x_velocity = 0, int cmd_b_velocity = 0, int cmd_theta = 0, int cmd_gamma = 0)
    {
        // This is the main function for the RC car control
        // It decides what action to do based on the current state and command input
        // RUNS REPEATEDLY, IT MUST BE CALLED FROM THE MAIN LOOP

        // Note: if in RC_TELEOP_STATE, commanded velocities will be ignored, PWM values will be read instead

        lastCommandTimestamp = millis();
        Serial.println("Processing command");

        // Will be changed into the HALT state if it is not safe to drive.
        //checkFailsafes();

        int neutral_switch_pos = int(analogRead(NEUTRAL_CONTROL_SWITCH_PIN));
        int autonomous_switch_pos = int(analogRead(AUTONOMOUS_CONTROL_SWITCH_PIN));

        if(neutral_switch_pos > 750) {
          if(currentStateID != NEUTRAL_SWITCH_STATE)
            set_current_state_ID(NEUTRAL_SWITCH_STATE);
            //return;
        } else if(autonomous_switch_pos > 750) {
          if(currentStateID != AUTONOMOUS_SWITCH_STATE)
            set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
            //return;
        } else {
          if(currentStateID != RC_TELEOP_STATE)
            set_current_state_ID(RC_TELEOP_STATE);
            //return;
        }

        // State Machine
        switch (currentStateID)
        {
        case HALT_STATE:
            // We are in HALT_STATE

            x_velocity = 0.0;
            theta = cmd_theta;

            /* DISABLE
            // Lets fully engage the brake
            send_throttle_command(calculate_throttle_pos(x_velocity));
            brake_motor->SetTargetPosition(BRAKE_FULLY_ENGAGED_POSITION);
            steer_motor->SetTargetPosition(calculate_steer_pos(theta));
            */

            // Once we have slowed to a HALT, lets stop the engine
            if (abs(x_velocity_sensed) <= 0.1)
            {
              /* DISBALE
              gear_motor->SetTargetPosition(PARK_GEAR_POSITION);
              */
              //delay(1500);
              stopEngine();
            }

            // Check the Control State Switch on dash of car
            // Neutral - Puts the car gear into neutral
            // RC - Allows the car to be driven by Remote Controller
            // Autonomous - car is drive by Nvidia Jetson

            if(neutral_switch_pos > 750) {
              set_current_state_ID(NEUTRAL_SWITCH_STATE);
              return;
            } else if(autonomous_switch_pos > 750) {
              set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
              return;
            } else {
              set_current_state_ID(RC_TELEOP_STATE);
              // set_current_state_ID(AUTONOMOUS_SWITCH_STATE);
              return;
            }

            break;

        case RC_TELEOP_STATE:
        {
            //if(neutral_switch_pos > 750 || autonomous_switch_pos > 750){
            //  set_current_state_ID(HALT_STATE);
            //}
            // We are in RC_TELEOP_STATE
            // Lets act according to the PWM input commands from the RC reciever

            x_velocity = read_pwm_value(THROTTLE_PWM_PIN);
            theta      = read_pwm_value(STEERING_PWM_PIN);

            Serial.print("X Vel: ");
            Serial.print(x_velocity);
            Serial.print(", Theta: ");
            Serial.print(theta);

            double ignition_val = read_pwm_value(RC_IGNITION_PWM_PIN);
            //RC_IGNITION_INPUT_AVERAGE.addValue(read_pwm_value(RC_IGNITION_PWM_PIN));
            //double ignition_val = RC_IGNITION_INPUT_AVERAGE.getAverage();

            double starter_val  = read_pwm_value(RC_ENGINE_START_PWM_PIN);

            Serial.print(", Ignition_pwm= ");
            Serial.print(ignition_val);
            Serial.print(", start_pwm: ");
            Serial.print(starter_val);

            // Ignition and Starter Motor Control
            if (ignition_val > RC_DUTY_THRESH_IGNITION) {
              digitalWrite(IGNITION_RELAY_PIN, HIGH);

              Serial.print(", IGNITION=ON");

              if (starter_val > RC_DUTY_THRESH_START_ENGINE)
              {
                digitalWrite(ENGINE_START_RELAY_PIN, HIGH);
                Serial.print(", STARTER=ON");
              }
              else
              {
                digitalWrite(ENGINE_START_RELAY_PIN, LOW);
                Serial.print(", STARTER=OFF");
              }
            }
            else
            {
              Serial.println("STOPPING ENGINE!!!!!");
              stopEngine();
              return;
            }

            Serial.print(", desired_steering=");
            Serial.print(calculate_steer_pos(theta));

            /* the joystick DEADZONEs are DISABLED for now
                DEADZONEs are implemented purely for operator ergonomics

            // Joystick steering DEADZONE
            if (abs(theta - float(STEERING_FULL_LEFT + STEERING_FULL_RIGHT) / 2.0 ) < RC_STEERING_DEADZONE)
            {
                theta = float(STEERING_FULL_LEFT + STEERING_FULL_RIGHT) / 2.0;
            }

                Joystick throttle DEADZONE
            if (abs(x_velocity - float(THROTTLE_FULL_FORWARD_POSITION + THROTTLE_FULL_REVERSE_POSITION) / 2.0 ) < RC_THROTTLE_DEADZONE)
            {
                x_velocity = float(THROTTLE_FULL_FORWARD_POSITION + THROTTLE_FULL_REVERSE_POSITION)/2.0;
            }
            */


            // Send command to the steering controller
            steer_motor->SetTargetPosition(calculate_steer_pos(theta));

            // Calculate brake position
            double desired_brake_position = calculate_brake_pos(x_velocity);

            // Calculate throttle position
            double desired_throttle_position = calculate_throttle_pos(x_velocity);

            // if (desired_brake_position < (BRAKE_FULLY_ENGAGED_POSITION + BRAKE_NOT_ENGAGED_POSITION) / 3.0)
            // {
            //   desired_throttle_position = calculate_throttle_pos(x_velocity);
            // }

            Serial.print(", desired_throttle=");
            Serial.print(desired_throttle_position);

            Serial.print(", desired_brake=");
            Serial.print(desired_brake_position);

            // Send command to the brake motor controller
            brake_motor->SetTargetPosition(desired_brake_position);


            // Send command to the throttle controller
            send_throttle_command(int(desired_throttle_position));


            // Gear shift interlock (prevent shifting at speed)
            // FIXME
            // if (abs(x_velocity_sensed) <= 0.1)
            if (true)
            {

                gear_motor->SetTargetPosition(rc_read_gear_pos());

                Serial.print(", desired_gear_pos=");
                Serial.println(rc_read_gear_pos());
            }

            Serial.println("");

            // Output the steering angle to the console
            Serial.print("#");
            Serial.print(double(analogRead(STEERING_ACTUATOR_POSITION_SENSOR_PIN)));
            Serial.println("!");

            break;
        }
        case IGNITION_STATE:
            // We don't do anything repetedly in ignition state
            // (only once: on state change)
            break;

        case ENGINE_START_STATE:
            // We don't do anything repetedly in ignition state
            // (only once: on state change)
            break;

        case AI_READY_STATE:
            // AI STATE CURRENTLY UNTESTED/UNUSED!
            // This will use velocity commands from the serial port

            x_velocity = cmd_x_velocity;
            theta = cmd_theta;

            /* DISABLE
            steer_motor->SetTargetPosition(calculate_steer_pos(theta));
            */

            // double desired_brake_position = calculate_brake_pos(x_velocity);
            // double desired_throttle_position = RC_THROTTLE_FULL_FORWARD_POSITION - RC_THROTTLE_FULL_REVERSE_POSITION;
            // if (desired_brake_position < (BRAKE_FULLY_ENGAGED_POSITION + BRAKE_NOT_ENGAGED_POSITION) / 3.0)
            // {
            //   desired_throttle_position = calculate_throttle_pos(x_velocity);
            // }

            /* DISABLE
            brake_motor->SetTargetPosition(desired_brake_position);
            send_throttle_command(int(desired_throttle_position));

            if (abs(x_velocity_sensed) <= 0.1)
            {
              gear_motor->SetTargetPosition(calculate_gear_pos(x_velocity));
            }
            */

            break;
        case NEUTRAL_SWITCH_STATE:
            Serial.println("Neutral State Selected");
            Serial.println("STOPPING ENGINE!!!!!");
            stopEngine();
            //delay(2000);
            gear_motor->SetTargetPosition(NEUTRAL_GEAR_POSITION);
            set_current_state_ID(HALT_STATE);
            break;

        case AUTONOMOUS_SWITCH_STATE:
            Serial.println("Autonomous State Selected");

            // Output the steering angle to the console
            Serial.print("#");
            Serial.print(double(analogRead(STEERING_ACTUATOR_POSITION_SENSOR_PIN)));
            Serial.println("!");

            //check_ignition_starter();

            sc.ReadData();
            //if(sc.message_time - timeDiff > 500)
            if ( sc.message_type != -1 ) {
                 Serial.print("valid message:");
                 Serial.print(sc.message_type);
                 Serial.print(",");
                 Serial.print(sc.message_steering);
                 cmd_theta = int(sc.message_steering);
                 Serial.print(",");
                 Serial.print(sc.message_brakes);
                 cmd_b_velocity = int(sc.message_brakes);
                 Serial.print(",");
                 Serial.print(sc.message_accelerator);
                 cmd_x_velocity = int(sc.message_accelerator);
                 Serial.print(",");
                 Serial.println(sc.message_gear);
                 cmd_gamma = int(sc.message_gear);
                 sc.Reset();
            } else {
              Serial.println("No command from Jetson received");
              return;
            }

            // Calculate steering positions
            double desired_steering_position = calculate_steer_pos(cmd_theta);

            // Calculate brake position
            double desired_brake_position = calculate_brake_pos(cmd_b_velocity);

            // Calculate throttle position
            double desired_throttle_position = calculate_throttle_pos(cmd_x_velocity);


            Serial.print(", desired_steering=");
            Serial.print(desired_steering_position);

            Serial.print(", desired_brake=");
            Serial.print(desired_brake_position);

            Serial.print(", desired_throttle=");
            Serial.print(desired_throttle_position);

            Serial.println(rc_read_gear_pos());


            // Send command to the steering controller
            steer_motor->SetTargetPosition(desired_steering_position);

            // Send command to the brake motor controller
            brake_motor->SetTargetPosition(desired_brake_position);

            // Send command to the throttle controller
            send_throttle_command(int(desired_throttle_position));

            // Send command to the gear controller
            // FIX ME
            //
            // gear_motor->SetTargetPosition(cmd_gamma);
            gear_motor->SetTargetPosition(rc_read_gear_pos());

            break;
        }

    }

    int get_current_gear_position()
    {
      if(!gear_motor->isMotorMoving()) {
        int current_gear_position = gear_motor->GetCurrentPosition();
        if (abs((current_gear_position - PARK_GEAR_POSITION) < GEAR_FEEDBACK_TOLERENCE)) {
          return PARK_GEAR_POSITION;
        } else if (abs((current_gear_position - REVERSE_GEAR_POSITION) < GEAR_FEEDBACK_TOLERENCE)) {
          return REVERSE_GEAR_POSITION;
        } else if (abs((current_gear_position - NEUTRAL_GEAR_POSITION) < GEAR_FEEDBACK_TOLERENCE)) {
          return NEUTRAL_GEAR_POSITION;
        } else if (abs((current_gear_position - DRIVE_GEAR_POSITION) < GEAR_FEEDBACK_TOLERENCE)) {
          return DRIVE_GEAR_POSITION;
        } else {
          // if unknown, put it into Neutral??
          // Fix me
          return 0;
        }
      }
    }

    bool set_current_state_ID(int newStateID)
    {
        // This function gets called when a change state is requested
        // Returns true on a successful transition

        // Code blocks within this switch statement are ONLY CALLED ON STATE change
        switch (newStateID)
        {
        case IGNITION_STATE:

            // Only allowed to transistion from HALT STATE to IGNITION STATE
            // FIXME: add state to MotorController class so that we can request the current and last commanded position
            if (currentStateID == HALT_STATE)
            {
                // Ensure that we are in park before engaging ignition
                if (abs(gear_motor->GetCurrentPosition() - PARK_GEAR_POSITION) > GEAR_FEEDBACK_TOLERENCE)
                {
                    Serial.println("Ignition command received, not in park.");

                    //Put the car into park
                    // current_gear_position = PARK_GEAR_POSITION;
                    /* DISABLE
                    gear_motor->SetTargetPosition(current_gear_position);
                    */

                    return false;
                }
                else
                {
                    // Once the car is in park, we can start the ignition
                    Serial.println("Car in park, turning on ignition");
                    digitalWrite(IGNITION_RELAY_PIN, HIGH);
                    main_relay_on = 1;
                    return true;
                }
                break;
            }
        case ENGINE_START_STATE:

            // Only transistion to ENGINE_START_STATE if currently in ignition state
            if (currentStateID == IGNITION_STATE)
            {
                startEngine();
            }
            break;
        case HALT_STATE:
            // Do nothing on transition into HALT
            break;
        case RC_TELEOP_STATE:
            // Do nothing on transition into RC_TELEOP
            break;
        case AI_READY_STATE:
            // Do nothing on transition into AI
            break;
        case NEUTRAL_SWITCH_STATE:
            // Do nothing on transition into NEUTRAL_SWITCH_STATE
            break;
        case AUTONOMOUS_SWITCH_STATE:
            // Do nothing on transition into AUTONOMOUS_SWITCH_STATE
            break;
        }

        Serial.print("Changing state to: ");
        Serial.println(newStateID);

        currentStateID = newStateID;
        return true;
    }

    int getcurrentStateID()
    {
        // Return the ID of the state that we are currently in
        return currentStateID;
    }

    float read_pwm_value(int pwm_pin)
    {
        // Read a value from a PWM input
        // Used on RC control for all commands, and failsafe
        // Used on AI control for failsafe ONLY
        int pwm_time = pulseIn(pwm_pin, HIGH);
        return pwm_time;
    }

    float convert_jetson_serial(int jetson_min, int jetson_max)
    {

    }

    bool checkFailsafes()
    {
        // This function will check all failsafes
        // If it is not safe to drive: the car will be switched to HALT_STATE
        // Pin 13 will be ON when it is safe to drive, otherwise OFF.

        // The failsafes include: a watchdog timer (i.e. an automatic shutdown if a command hasn't been recieved within 250ms)
        // Also included is PWM switch from the RC reciever.

        // Note that: the RC PWM switch failsafe should be connected in series with the emergency stop switch at the rear of the car

        Serial.println("Checking failsafes!");
        bool watchdogValid = ((millis() - lastCommandTimestamp) < WATCHDOG_TIMEOUT);
        bool rcFailsafeValid = read_pwm_value(RC_FAILSAFE_PIN) >= 0.5;

        Serial.print("Dutycycle for failsafe=");
        Serial.print(read_pwm_value(RC_FAILSAFE_PIN));

        Serial.print(", watchdog_valid=");
        Serial.println(watchdogValid);

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
        // Send command to the throttle servo
        throttle_command = constrain(throttle_command, 0, 60);
        throttle_servo.write(throttle_command);
    }

    void check_ignition_starter()
    {
      double ignition_val = read_pwm_value(RC_IGNITION_PWM_PIN);
      // Ignition and Starter Motor Control

      double starter_val  = read_pwm_value(RC_ENGINE_START_PWM_PIN);

      if (ignition_val > RC_DUTY_THRESH_IGNITION) {
        digitalWrite(IGNITION_RELAY_PIN, HIGH);

        Serial.print(", IGNITION=ON");

        if (starter_val > RC_DUTY_THRESH_START_ENGINE)
        {
          digitalWrite(ENGINE_START_RELAY_PIN, HIGH);
          Serial.print(", STARTER=ON");
        }
        else
        {
          digitalWrite(ENGINE_START_RELAY_PIN, LOW);
          Serial.print(", STARTER=OFF");
        }
      }
      else
      {
        Serial.println("STOPPING ENGINE!!!!!");
        stopEngine();
        return;
      }
    }

private:
    int currentStateID;
    long lastCommandTimestamp;
    double theta;
    double x_velocity;
    double x_velocity_sensed;
    int current_gear_position;
    bool ai_enabled;
    bool main_relay_on;
    bool engine_currently_running;
    Servo throttle_servo;

    SabertoothSimplified* sabertooth_60A;
    SabertoothSimplified* sabertooth_32A;

    MotorController* brake_motor;
    MotorController* gear_motor;
    MotorController* steer_motor;
};
