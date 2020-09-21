//www.elegoo.com
//2016.12.08
#include "SR04.h"

#define TRIG_PIN 12
#define ECHO_PIN 10
#define BUZZER 9
#define ALARM_LENGTH_CM 20
#define ledPin1 11



inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

class Flasher
{
    // Class Member Variables
    // These are initialized at startup
    int ledPin;      // the number of the LED pin
    long OnTime;     // milliseconds of on-time
    long OffTime;    // milliseconds of off-time
    boolean active; //active or not

    // These maintain the current state
    int ledState;                 // ledState used to set the LED
    unsigned long previousMillis;   // will store last time LED was updated


  public: Flasher() {}
    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public: Flasher(int pin, long on, long off)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);

      OnTime = on;
      OffTime = off;

      ledState = LOW;
      previousMillis = 0;
      active = false;
    }

    void ToggleActive() {
      active = !active;
    }

    void Update()
    {
      // check to see if it's time to change the state of the LED
      unsigned long currentMillis = millis();
      if (active) {
        if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
        {
          ledState = LOW;  // Turn it off
          previousMillis = currentMillis;  // Remember the time
          digitalWrite(ledPin, ledState);  // Update the actual LED
        }
        else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
        {
          ledState = HIGH;  // turn it on
          previousMillis = currentMillis;   // Remember the time
          digitalWrite(ledPin, ledState);   // Update the actual LED
        }
      } else {
        previousMillis = 0;
      }
    }
};

class Buzzer
{
    // Class Member Variables
    // These are initialized at startup
    int buzzerPin;      // the number of the LED pin
    long SoundMs;     // milliseconds sound buzzer for give frequency
    long FreqStepHz;     // change in frequency in Hz
    boolean active; //active or not
    long freq; //frequency
    static const long FREQ_MAX = 13000;
    static const long FREQ_MIN = 11500;
    boolean upCount; //defines up or down count


    // These maintain the current state
    unsigned long previousMillis;   // will store last time Buzzer was updated

  public: Buzzer() {}

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public: Buzzer(int pin, long soundMs, long freqStepHz)
    {
      buzzerPin = pin;
      pinMode(buzzerPin, OUTPUT);

      SoundMs = soundMs;
      FreqStepHz = freqStepHz;
      previousMillis = 0;
      active = false;
      freq = FREQ_MIN;
    }

    void ToggleActive() {
      active = !active;
      if (!active) {
        noTone(buzzerPin);
         freq = FREQ_MIN;
      }
    }

    long NextFrquency() {
      upCount = freq >= FREQ_MAX ? false : (freq <= FREQ_MIN ? true : upCount);
      freq = upCount ? freq + FreqStepHz : freq - FreqStepHz;
      return freq;

    }

    void Update()
    {
      // check to see if it's time to change the state of the LED
      //Serial.println((String)"inactive buzzer: " + active);
      unsigned long currentMillis = millis();
      if (active) {
        if ((currentMillis - previousMillis >= SoundMs))
        {
          NextFrquency();
          tone(buzzerPin, freq);
          previousMillis = currentMillis;  // Remember the time
        }
      }
    }
};


class Alarm
{
    // Class Member Variables
    // These are initialized at startup
    Buzzer buzzer;      // buzzer
    Flasher flasher;     // led
    long DurationTime;     // milliseconds of on-time
    boolean active; //active or not


    // These maintain the current state
    unsigned long previousMillis;   // will store last time Buzzer was updated

    // Constructor - creates a Flasher
    // and initializes the member variables and state
  public: Alarm(Buzzer buz, Flasher flash, long durationTime)
    {
      buzzer = buz;
      flasher = flash;
      DurationTime = durationTime;
      active = false;
    }

    void Activate() {
      if (!active) {
        active = true;
        flasher.ToggleActive();
        buzzer.ToggleActive();
        previousMillis = millis();
        Serial.println((String)"alarm active: " + (active ? "true" : "false"));
      }
    }

    void Update()
    {
      // check to see if it's time to change the state of the Alarm

      unsigned long currentMillis = millis();
      if (active) {
        flasher.Update();
        buzzer.Update();
        //Serial.println((String)"timing: " + (currentMillis - previousMillis) + " " + DurationTime);
        if ((currentMillis - previousMillis) >= DurationTime)
        {
          //Serial.println((String)"deactivate: " + (currentMillis - previousMillis) + " " + DurationTime);
          flasher.ToggleActive();
          buzzer.ToggleActive();
          active = false;
          previousMillis = 0;  // Remember the time
        }
        //previousMillis = currentMillis;  // Remember the time
      }
    }
};




Flasher flasher(ledPin1, 50, 100);
Buzzer buzzer(BUZZER, 5, 80);
Alarm alarm(buzzer, flasher, 5000);

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long distanceCm;

void setup() {



  Serial.begin(9600);//Initialization of Serial Port
  delay(1000);
}


void loop() {
  distanceCm = sr04.Distance();
  Serial.print(distanceCm);
  Serial.println("cm");//The difference between "Serial.print" and "Serial.println"
  //delay(500);
  alarm.Update();
  if (distanceCm < ALARM_LENGTH_CM) {
    Serial.print(distanceCm);
    Serial.println("cm ALARM!!!");
    alarm.Activate();
  }
}
