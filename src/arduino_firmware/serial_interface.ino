
#define MAX_CHARS       24;
#define MESSAGE_START 0x23;
#define MESSAGE_END   0x13;

// Message Format:
// char 0x23 '#'
// char message type
// char 0x2c ','
// char[9] data1
// char 0x2c ','
// char[9] data2
// char 0x13 'CR'

class SerialCommand()
{
    public:
        SerialInterface()
        {
            Serial.begin(115200);
            SWSerial.begin(115200);
            Serial.setTimeout(50);
        }

        parseCommand()
        {
            int message_id = -1;
            int setpoint = -1;
            int k, q, i;

            if (Serial.available() > 0)
            {
                read_byte = Serial.read();
                // maybe: delay(10);

                if (read_byte == MESSAGE_END)
                { //  follow the false code until you get a CR - this packs inString with the complete string
                    if ( cmd_string[0] != MESSAGE_START ) {
                        reset();
                        return;
                    }

                    command_message_type = cmd_string[0];

                    for (int p = 1; p < MAX_CHARS; p++)
                    {
                        next_char = cmd_string[p]; // get the next Char
                        q++;

                        if ( ! isDigit(next_char) ) {
                            if ( nextChar == 44 ) {
                                
                            }
                            reset();
                        }
                         || nextChar == 13)
                        { // if it's a comma (ASCII 44) then bail out
                            break;
                        }

                        arg0_String[k] = nextChar; // start packing the arg0_String

                        k++; // increment the pointer to arg1_String
                    }

                    /*******************************************************************************************/
                    // Now we process the arg1_String  checking whether we are b1 or b,1 format
                    /*******************************************************************************************/

                    k = 0; //reset the string pointer

                    for (int p = q + 1; p < 34; p++)
                    { // Now start at the location after the 2nd comma and strip out the arg2_String

                        nextChar = inString[p]; // get the next Char

                        q++;

                        if (nextChar == 44 || nextChar == 13)
                        { // if it's a comma (ASCII 44) then bail out

                            break;
                        }

                        arg1_String[k] = nextChar; // start packing the arg1_String

                        k++; // increment the pointer to arg2_String
                    }

                    /*******************************************************************************************/
                    // Now we process the arg2_String in the same way
                    /*******************************************************************************************/

                    k = 0; //reset the string pointer

                    for (int p = q + 1; p < 34; p++)
                    { // Now start at the location after the 2nd comma and strip out the arg2_String

                        nextChar = inString[p]; // get the next Char

                        q++;

                        if (nextChar == 44 || nextChar == 13)
                        { // if it's a comma (ASCII 44) then bail out

                            break;
                        }

                        arg2_String[k] = nextChar; // start packing the arg1_String

                        k++; // increment the pointer to arg2_String
                    }
                

                    // Parse values, Set command ready


                    reset_string();
                    i = 0;

                    return;
                } else {
                    dataReady = false; // No CR seen yet so put digits into array
                                    //    Serial.print(inByte);
                
                    cmd_string[i] = inByte;
                    i++;
                }
        }

        private:
          char cmd_string[MAX_CHARS];

         void reset_string() {
             for (int j = 0; j < MAX_CHARS; j++)
             {
                 cmd_string[j] = 0;
             }
         }
}