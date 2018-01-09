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
