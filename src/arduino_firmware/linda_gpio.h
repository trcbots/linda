class LindaGPIO
{



public:
    Flasher(int pin, long on, long off)
    {
        ledPin = pin;
        pinMode(ledPin, OUTPUT);     
            
        OnTime = on;
        OffTime = off;

        ledState = LOW; 
        previousMillis = 0;
    }
};