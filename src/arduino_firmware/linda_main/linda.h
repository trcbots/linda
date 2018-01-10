#include <Servo.h>

#include "motor_controller.h"

#define HALT_STATE         0
#define COAST_STATE        1
#define IGNITION_STATE     2
#define ENGINE_START_STATE 3
#define RC_TELEOP_STATE    4
#define AI_READY_STATE     5

// PWM input pins from RC Reciever
#define RC_ENGINE_START_PWM_PIN              2 // RC PIN 8
#define RC_IGNITION_PWM_PIN                  3 // RC PIN 7
#define RC_FAILSAFE_PIN    RC_IGNITION_PWM_PIN // RC PIN 7
#define THROTTLE_PWM_PIN                     7 // RC PIN 3
#define STEERING_PWM_PIN                     8 // RC PIN 4
#define THROTTLE_SERVO_PIN                   9 // THROTTLE SERVO MOTOR SIGNAL
#define RC_GEAR_SWITCH_PIN                  12 // RC PIN 6

// Digital output pins
#define ENGINE_START_RELAY_PIN  4           // ENGINE START RELAY OUTPUT
#define IGNITION_RELAY_PIN      5           // IGNITION RELAY OUTPUT
#define FAILSAFE_LED_PIN       13           // OUTPUT TO LED ON THE ARDUINO BOARD

// Analog input pins
#define BRAKE_ACTUATOR_POSITION_SENSOR_PIN    A3
#define GEAR_ACTUATOR_POSITION_SENSOR_PIN     A4
#define STEERING_ACTUATOR_POSITION_SENSOR_PIN A5

//Used in AI mode only
#define AUTOSTART_NUM_START_ATTEMPTS 4

// If a command has not been recieved within WATCHDOG_TIMEOUT ms, will be switched to HALT state.
#define WATCHDOG_TIMEOUT 250

/************************ DRIVE CONTROL DEFINEs **********************************/
// These sensitivity values will need to be changed.
#define BRAKE_SENSITIVITY     1.0
#define THROTTLE_SENSITIVITY  1.0
#define STEERING_SENSITIVITY  1.0

#define BRAKE_MOTOR_MAX_POWER    30
#define GEAR_MOTOR_MAX_POWER     20
#define STEERING_MOTOR_MAX_POWER 60

#define PARK_GEAR_POSITION    400
#define REVERSE_GEAR_POSITION 500
#define NEUTRAL_GEAR_POSITION 600
#define DRIVE_GEAR_POSITION   700

#define BRAKE_FULLY_ENGAGED_POSITION  475 // 100
#define BRAKE_NOT_ENGAGED_POSITION    525 // 1023

#define STEERING_FULL_LEFT   50
#define STEERING_FULL_RIGHT  1080

#define RC_STEERING_FULL_LEFT_POSITION   1025
#define RC_STEERING_FULL_RIGHT_POSITION  1975

#define RC_THROTTLE_FULL_REVERSE_POSITION   1025
#define RC_THROTTLE_FULL_FORWARD_POSITION   1975

#define THROTTLE_SERVO_ZERO_POSITION   0
#define THROTTLE_SERVO_FULL_POSITION   40

#define RC_STEERING_DEADZONE   25.0
#define RC_THROTTLE_DEADZONE   25.0

#define RC_DUTY_THRESH_PARK     1100
#define RC_DUTY_THRESH_DRIVE    1400
#define RC_DUTY_THRESH_REVERSE  1600

#define RC_DUTY_THRESH_START_ENGINE 1500
#define RC_DUTY_THRESH_IGNITION     1500

/********************************************************************************/


class Linda
{
  public:
    Linda()
    {
        pinMode(FAILSAFE_LED_PIN, OUTPUT);
        pinMode(ENGINE_START_RELAY_PIN, OUTPUT);
        digitalWrite(ENGINE_START_RELAY_PIN, LOW);

        pinMode(IGNITION_RELAY_PIN, OUTPUT);
        digitalWrite(IGNITION_RELAY_PIN, LOW);

        pinMode(RC_FAILSAFE_PIN, INPUT);

        lastCommandTimestamp = 0.0;
        x_velocity = 0.0;
        x_velocity_sensed = -1.0;
        ai_enabled = 0;
        engine_currently_running = false;
    }

    void Init() {
      throttle_servo.attach(THROTTLE_SERVO_PIN);
      delay(5);
      throttle_servo.write(0);

      Serial1.begin(9600);
      Serial2.begin(9600);
      sabertooth_60A = new SabertoothSimplified(Serial1);
      sabertooth_32A = new SabertoothSimplified(Serial2);

      brake_motor = new MotorController(
                      "Brake motor", sabertooth_32A, 1,
                      BRAKE_ACTUATOR_POSITION_SENSOR_PIN,
                      BRAKE_FULLY_ENGAGED_POSITION, BRAKE_NOT_ENGAGED_POSITION,
                      BRAKE_MOTOR_MAX_POWER);

      gear_motor = new MotorController(
                      "Gear motor", sabertooth_32A, 2,
                      GEAR_ACTUATOR_POSITION_SENSOR_PIN,
                      DRIVE_GEAR_POSITION, PARK_GEAR_POSITION,
                      GEAR_MOTOR_MAX_POWER);
      
      steer_motor = new MotorController(
                      "Steering motor", sabertooth_60A, 1,
                      STEERING_ACTUATOR_POSITION_SENSOR_PIN,
                      STEERING_FULL_LEFT, STEERING_FULL_RIGHT,
                      STEERING_MOTOR_MAX_POWER);
    }

    void startEngine()
    {
        for (int i = 1; i < AUTOSTART_NUM_START_ATTEMPTS; i++)
        {
            Serial.println("Attempting to crank!");

            digitalWrite(ENGINE_START_RELAY_PIN, HIGH);
            delay(2000 + i * 500); // Increase cranking time by 500ms for each attempt
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
        Serial.println("In stopEngine");
        engine_currently_running = false;
        digitalWrite(IGNITION_RELAY_PIN, LOW);
        //set_current_state_ID(HALT_STATE);
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
    
    int rc_read_gear_pos()
    {
      double duty = read_pwm_value(RC_GEAR_SWITCH_PIN);

      if (duty < RC_DUTY_THRESH_PARK)
      {
        return PARK_GEAR_POSITION;
      }
      
      if (duty < RC_DUTY_THRESH_DRIVE)
      {
        return DRIVE_GEAR_POSITION;
      }
      
      return REVERSE_GEAR_POSITION;
    }

    double calculate_throttle_pos(double x_velocity) {
      x_velocity = (x_velocity - RC_THROTTLE_FULL_REVERSE_POSITION) * (THROTTLE_SERVO_FULL_POSITION - THROTTLE_SERVO_ZERO_POSITION) / 
        (RC_THROTTLE_FULL_FORWARD_POSITION - RC_THROTTLE_FULL_REVERSE_POSITION) + THROTTLE_SERVO_ZERO_POSITION;

      Serial.print("calculate_throttle_pos=");
      Serial.println(x_velocity * THROTTLE_SENSITIVITY);

      if (x_velocity * THROTTLE_SENSITIVITY < 0)
      {
        return 0;
      }
      
      return x_velocity * THROTTLE_SENSITIVITY;
    }
    
    double calculate_brake_pos(double x_velocity)
    {      
      return (x_velocity == 0.0) * BRAKE_SENSITIVITY;
    }
    
    double calculate_steer_pos(double cmd_theta) {
      cmd_theta = (cmd_theta - RC_STEERING_FULL_LEFT_POSITION) * (STEERING_FULL_RIGHT - STEERING_FULL_LEFT) / 
        (RC_STEERING_FULL_RIGHT_POSITION - RC_STEERING_FULL_LEFT_POSITION) + STEERING_FULL_LEFT;

      Serial.print("calculate_steer_pos=");
      Serial.println(cmd_theta * STEERING_SENSITIVITY);
      
      return cmd_theta * STEERING_SENSITIVITY;
    }

    void process_command(double cmd_x_velocity = 0.0, double cmd_theta = 0.0)
    {
        lastCommandTimestamp = millis();
        Serial.println("Processing command");
        
        // Will be changed into the HALT state if it is not safe to drive.
        //checkFailsafes();

        // This function is called every time a serial (or RC PWM) command is recieved
        switch (currentStateID)
        {
        case HALT_STATE:
            x_velocity = 0.0;
            theta = cmd_theta;

            /* DISABLE 
            send_throttle_command(calculate_throttle_pos(x_velocity));
            brake_motor->SetTargetPosition(BRAKE_FULLY_ENGAGED_POSITION);
            steer_motor->SetTargetPosition(calculate_steer_pos(theta));
            */
            if (abs(x_velocity_sensed) <= 0.1)
            {
              /* DISBALE
              gear_motor->SetTargetPosition(PARK_GEAR_POSITION);
              */
              delay(1500);
              stopEngine();
            }
            break;

        case RC_TELEOP_STATE:
        {
            x_velocity = read_pwm_value(THROTTLE_PWM_PIN);
            theta      = read_pwm_value(STEERING_PWM_PIN);
           
            Serial.print("X Vel: ");
            Serial.print(x_velocity);
            Serial.print(", Theta: ");
            Serial.print(theta);

            double ignition_val = read_pwm_value(RC_IGNITION_PWM_PIN);
            double starter_val  = read_pwm_value(RC_ENGINE_START_PWM_PIN);
            
            Serial.print(", Ignition_pwm= ");
            Serial.print(ignition_val);  
            Serial.print(", start_pwm: ");
            Serial.print(starter_val);  
        
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
            
            // Joystick steering DEADZONE
//            if (abs(theta - float(STEERING_FULL_LEFT + STEERING_FULL_RIGHT) / 2.0 ) < RC_STEERING_DEADZONE)
//            {
//              theta = float(STEERING_FULL_LEFT + STEERING_FULL_RIGHT) / 2.0;
//            }
            
            // Joystick throttle DEADZONE
//            if (abs(x_velocity - float(THROTTLE_FULL_FORWARD_POSITION + THROTTLE_FULL_REVERSE_POSITION) / 2.0 ) < RC_THROTTLE_DEADZONE)
//            {
//              x_velocity = float(THROTTLE_FULL_FORWARD_POSITION + THROTTLE_FULL_REVERSE_POSITION)/2.0;
//            }


            steer_motor->SetTargetPosition(calculate_steer_pos(theta));

            double desired_brake_position = calculate_brake_pos(x_velocity);
            double desired_throttle_position = RC_THROTTLE_FULL_FORWARD_POSITION - RC_THROTTLE_FULL_REVERSE_POSITION;
            
            if (desired_brake_position < (BRAKE_FULLY_ENGAGED_POSITION + BRAKE_NOT_ENGAGED_POSITION) / 3.0)
            {
              desired_throttle_position = calculate_throttle_pos(x_velocity);
            }
            Serial.print(", desired_throttle=");
            Serial.print(desired_throttle_position);
            
            Serial.print(", desired_brake=");
            Serial.print(desired_brake_position);

            /* DISABLE 
            brake_motor->SetTargetPosition(desired_brake_position);
            */
            
            send_throttle_command(int(desired_throttle_position));
            
   
// FIXME            
//           if (abs(x_velocity_sensed) <= 0.1)
            if (true)
            {

                gear_motor->SetTargetPosition(rc_read_gear_pos());
 
                Serial.print(", desired_gear_pos=");
                Serial.println(rc_read_gear_pos());
            }
            
            Serial.println("");
            break;
        }
        case IGNITION_STATE:
            break;

        case ENGINE_START_STATE:
            break;

        case AI_READY_STATE:
            x_velocity = cmd_x_velocity;
            theta = cmd_theta;
            
            x_velocity = cmd_x_velocity;
            theta = cmd_theta;

            /* DISABLE 
            steer_motor->SetTargetPosition(calculate_steer_pos(theta));
            */
            
            double desired_brake_position = calculate_brake_pos(x_velocity);
            double desired_throttle_position = RC_THROTTLE_FULL_FORWARD_POSITION - RC_THROTTLE_FULL_REVERSE_POSITION;
            if (desired_brake_position < (BRAKE_FULLY_ENGAGED_POSITION + BRAKE_NOT_ENGAGED_POSITION) / 3.0)
            {
              desired_throttle_position = calculate_throttle_pos(x_velocity);
            }

            /* DISABLE 
            brake_motor->SetTargetPosition(desired_brake_position);
            send_throttle_command(int(desired_throttle_position));
            
            if (abs(x_velocity_sensed) <= 0.1)
            {
              gear_motor->SetTargetPosition(calculate_gear_pos(x_velocity));
            }
            */
            
            break;
        }
   
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
                    current_gear_position = PARK_GEAR_POSITION;
                    /* DISABLE 
                    gear_motor->SetTargetPosition(current_gear_position);
                    */
                    
                    digitalWrite(IGNITION_RELAY_PIN, HIGH);
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

    float read_pwm_value(int pwm_pin)
    {
        unsigned long pwm_time = pulseIn(pwm_pin, HIGH);
        return (float)pwm_time;
    }

    bool checkFailsafes()
    {
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
        throttle_command = constrain(throttle_command, 0, 60);
        throttle_servo.write(throttle_command);
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

