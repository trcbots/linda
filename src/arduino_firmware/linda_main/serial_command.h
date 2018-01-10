
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

class SerialCommand
{
	public:
		int message_type;
		double message_data1;
		double message_data2;

		SerialCommand()
		{
			Serial.begin(115200);
			reset();
		}

		void ReadData()
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
					reset();
				}
				return;
			}

			// Once we receive our MESSAGE_END, then
			// validate we have a valid message packet
			if ( ! have_valid_message() ) {
				reset();
				return;
			}

			message_type = cmd_string.substring(0).toInt();
			bool found = false;
			char p = 2;
			for (; p < MAX_CHARS; p++)
			{
				if ( ! ( isDigit( cmd_string.charAt(p) ) || cmd_string.charAt(p) == '.' ) ) {
					// If the char is a comma, then parse the data we have
					if ( cmd_string.charAt(p) == 0x2c ) {
						message_data1 = cmd_string.substring(2, p).toFloat();
						found = true;
						break;
					}
					// otherwise we bail out
					break;
				}
			}

			if ( ! found ) {
				reset();
				return;
			}

			found = false;
			for (int q = p + 1; q < MAX_CHARS; q++)
			{
				if ( ! ( isDigit( cmd_string.charAt(q) ) || cmd_string.charAt(q) == '.' ) ) {
					// If the char is a comma, then parse the data we have
					if ( cmd_string.charAt(q) == NULL ) {
						message_data2 = cmd_string.substring(p + 1, q).toFloat();
						found = true;
						break;
					}
					// otherwise we bail out
					break;
				}
			}

			if ( ! found ) {
				reset();
				return;
			}
		}

		void reset() {
			cmd_string = "";
			reading_message = false;
			curr_pos = 0;
			message_type = -1;
			message_data1 = -1;
			message_data2 = -1;
		}

		private:
			String cmd_string;
			int curr_pos;
			bool reading_message;

			bool have_valid_message() {
				int comma_count = 0;
				for ( int p = 1; p < cmd_string.length(); p++ ) {
					if ( cmd_string.charAt(p) == ',' ) {
						comma_count += 1;
					}
				}
				// Did we find the expected 2 commas
				return ( comma_count == 2 );
			}
};
