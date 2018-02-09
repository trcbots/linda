
// #include <SoftwareSerial.h>

#define MAX_CHARS       24
#define MESSAGE_START 0x23
#define MESSAGE_END   0x21

#define NO_MESSAGE -1

// Message Format:
// char MESSAGE_START '#'
// char message type
// char 0x2c ','
// char[9] data1
// char 0x2c ','
// char[9] data2
// char MESSAGE_END '!'

// #define JETSON_SERIAL_TX		NOT_A_PIN
// #define JETSON_SERIAL_RX		4
//
// SoftwareSerial jetson_serial(JETSON_SERIAL_RX, JETSON_SERIAL_TX); // RX, TX

class SerialCommand
{
	public:
		int message_type;
    unsigned long message_time;  // Time
    int message_steering; // Steering
		int message_brakes; // Brake
		int message_accelerator; // Accelerator
		int message_gear;	// Gear

		SerialCommand();
		void ReadData();
		void Reset();

		private:
			String cmd_string;
			int curr_pos;
			bool reading_message;

			bool haveValidMessage();
};


SerialCommand::SerialCommand()
{
  //Serial.begin(9600);
	//jetson_serial.begin(9600);
  Reset();
}

void SerialCommand::ReadData()
{
  if (Serial.available() <= 0)
  {
    return;
  }

  int read_byte = Serial.read();

  if (! reading_message) {
    if (read_byte == MESSAGE_START) {
      reading_message = true;
    }
    return;
  }

  if (read_byte != MESSAGE_END)
  {
    cmd_string += (char)read_byte;
    curr_pos++;
    if ( curr_pos >= MAX_CHARS ) {
      Reset();
    }
    return;
  }

  // Once we receive our MESSAGE_END, then
  // validate we have a valid message packet
  if ( ! haveValidMessage() ) {
    Reset();
    Serial.println("Not valid message");
    return;
  }


	// Data1
  message_type = cmd_string.substring(0).toInt();
  bool found = false;
  char p = 2;
  for (; p < MAX_CHARS; p++)
  {
    if ( ! ( isDigit( cmd_string.charAt(p) ) || cmd_string.charAt(p) == '.' ) ) {
      // If the char is a comma, then parse the data we have
      if ( cmd_string.charAt(p) == 0x2c ) {
        message_steering = cmd_string.substring(2, p).toInt();
        found = true;
        break;
      }
      // otherwise we bail out
      break;
    }
  }


  if ( ! found ) {
    Reset();
    return;
  }

	// Data2
  found = false;
	int q = p + 1;
  for (q; q < MAX_CHARS; q++)
  {
    if ( ! ( isDigit( cmd_string.charAt(q) ) || cmd_string.charAt(q) == '.' ) ) {
      // If the char is a comma, then parse the data we have
      if ( cmd_string.charAt(q) == 0x2c ) {
        message_accelerator = cmd_string.substring(p + 1, q).toInt();
        found = true;
        break;
      }
      // otherwise we bail out
      break;
    }
  }

  if ( ! found ) {
    Reset();
    return;
  }


	// Data3
  found = false;
	int r = q + 1;
  for (r; r < MAX_CHARS; r++)
  {
    if ( ! ( isDigit( cmd_string.charAt(r) ) || cmd_string.charAt(r) == '.' ) ) {
      // If the char is a comma, then parse the data we have
      if ( cmd_string.charAt(r) == 0x2c ) {
        message_brakes = cmd_string.substring(q + 1, r).toInt();
        found = true;
        break;
      }
      // otherwise we bail out
      break;
    }
  }

  if ( ! found ) {
    Reset();
    return;
  }

	// Data4
	found = false;
	int s = r + 1;
  for (int s = r + 1; s < MAX_CHARS; s++)
  {
    if ( ! ( isDigit( cmd_string.charAt(s) ) || cmd_string.charAt(s) == '.' ) ) {
      // If the char is a comma, then parse the data we have
      if ( cmd_string.charAt(s) == NULL ) {
        message_gear = cmd_string.substring(r + 1, s).toInt();
        found = true;
        break;
      }
      // otherwise we bail out
      break;
    }
  }

  message_time = millis();

  if ( ! found ) {
    Reset();
    return;
  }
}

void SerialCommand::Reset() {
  cmd_string      = "";
  reading_message = false;
  curr_pos        = 0;
  message_time    = -1;
  message_type    = -1;
  message_steering   = -1;
  message_brakes   = -1;
  message_accelerator   = -1;
  message_gear   = -1;
}

bool SerialCommand::haveValidMessage() {
  int comma_count = 0;
  for ( int p = 1; p < cmd_string.length(); p++ ) {
    if ( cmd_string.charAt(p) == ',' ) {
      comma_count += 1;
    }
  }
  // Did we find the expected 2 commas
  return ( comma_count == 4 );
}
