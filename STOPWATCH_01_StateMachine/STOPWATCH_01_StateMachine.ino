//www.elegoo.com
//2016.12.08
#include "SR04.h"
#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);


#define TRIG_PIN 3
#define ECHO_PIN 4
#define BUZZER 2
#define ledPin1 5
#define SWITCH_PIN 6
#define DISTANCE_SWITCH_PIN 22

int ALARM_LENGTH_CM = 20;

class StopWatch
{
    boolean started1 = false;
    boolean started2 = false;
    unsigned long startTime = 0;
    unsigned long stopTime1 = 0;
    unsigned long stopTime2 = 0;
    unsigned long timerDelayMs = 1000;

  public: StopWatch(int i) {}


    char *  timeToString()
    {
      static char str[12];
      unsigned long now = millis() - startTime;
      int m = now / 1000 / 60;
      int sec = now / 1000 - (m * 60);
      int ms = now % 1000;
      sprintf(str, "%02d:%02d:%04d", m, sec, ms);
      return str;
    }
    void Start() {
      lcd.clear();
      started1 = true;
      started2 = true;
      startTime = millis();
    }

    bool Running() {
      return started1 || started2;
    }

    void Stop() {
      if (started1) {
        Serial.println("StopWatch: Stopped 1");
        stopTime1 = millis();
        started1 = false;
      } else if (started2 && (millis() - stopTime1) > timerDelayMs) {
        Serial.println("StopWatch: Stopped 2");
        stopTime2 = millis();
        started2 = false;
      }
    }

    void Reset() {
      lcd.clear();
      lcd.write("Ready?");
      started1 = false;
      started2 = false;
      startTime = 0;
      stopTime1 = 0;
      stopTime2 = 0;
    }

    void Update() {
      if (started1) {
        lcd.setCursor(0, 0);
        lcd.write(timeToString());
      }
      if (started2) {
        lcd.setCursor(0, 1);
        lcd.write(timeToString());
      }
    }

};

StopWatch watch(1);

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
        digitalWrite(ledPin, LOW);
      }
    }
};

class Buzzer
{
    // Class Member Variables
    // These are initialized at startup
    int buzzerPin;      // the number of the Buzzer pin
    long SoundMs;     // milliseconds sound buzzer for give frequency
    long FreqStepHz;     // change in frequency in Hz per step
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


class Switch
{
    // These maintain the current state
    int switchPin;      // the number of the Switch pin

    const int LONG_PRESS_TIME = 2000; // ms

    // Variables will change:
    int lastState = HIGH;  // the previous state from the input pin
    int currentState;     // the current reading from the input pin
    unsigned long pressedTime  = 0;
    unsigned long releasedTime = 0;


  public: Switch(int pin)
    {
      switchPin = pin;
      pinMode(pin, INPUT_PULLUP);
    }

    void Reset()
    {
      pressedTime = 0;
      releasedTime = 0;
    }

    void Update()
    {
      // read the state of the switch/button:
      currentState = digitalRead(switchPin);

      if (lastState == HIGH && currentState == LOW)   {    // button is pressed
        pressedTime = millis();
        Serial.println((String)"pressed " + pressedTime);
      } else if (lastState == LOW && currentState == HIGH) { // button is released
        releasedTime = millis();
        Serial.println((String)"released " + releasedTime);
        long pressDuration = releasedTime - pressedTime;

        if ( pressDuration >= LONG_PRESS_TIME ) {
          Reset();
          watch.Reset();
        } else {
          Reset();
          if (watch.Running()) {
            watch.Stop();
          } else {
            watch.Start();
          }
        }
      }
      // save the the last state
      lastState = currentState;
    }
};

class DistanceSwitch
{
    // These maintain the current state
    int switchPin;      // the number of the Switch pin

    const int LONG_PRESS_TIME = 2000; // ms

    // Variables will change:
    int lastState = HIGH;  // the previous state from the input pin
    int currentState;     // the current reading from the input pin
    int incCm; //increment in centimeters
    unsigned long pressedTime  = 0;
    unsigned long releasedTime = 0;



  public: DistanceSwitch(int pin, int inc)
    {
      switchPin = pin;
      incCm = inc;
      pinMode(pin, INPUT_PULLUP);
    }

    void Reset()
    {
      pressedTime = 0;
      releasedTime = 0;
    }

    char * DisplayText() {
      static char buffer[128];
      snprintf(buffer, sizeof(buffer), "%s%d%s", "Distance: ", ALARM_LENGTH_CM, " cm");
      return  buffer;
    }

    void Update()
    {
      // read the state of the switch/button:
      currentState = digitalRead(switchPin);

      if (lastState == HIGH && currentState == LOW)   {    // button is pressed
        pressedTime = millis();
      } else if (lastState == LOW && currentState == HIGH) { // button is released
        releasedTime = millis();
        long pressDuration = releasedTime - pressedTime;

        if ( pressDuration >= LONG_PRESS_TIME ) {
          Serial.println("Distance Reset");
          ALARM_LENGTH_CM = incCm;
          lcd.clear();
          lcd.write(DisplayText());
          Reset();
        } else {
          Serial.println("Distance increment");
          ALARM_LENGTH_CM += incCm;
          lcd.clear();
          lcd.write(DisplayText());
          Reset();
        }
      }

      // save the the last state
      lastState = currentState;
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
    unsigned long previousMillis;   // will store last time Alarm was updated

    // Constructor - creates a Alarm
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
Alarm alarm(buzzer, flasher, 3000);
Switch switcher(SWITCH_PIN);
DistanceSwitch distanceSwitch(DISTANCE_SWITCH_PIN, 10);


SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long distanceCm;

void setup() {
  Serial.begin(9600);//Initialization of Serial Port
  lcd.begin(16, 2);
  lcd.print("Ready?");
  delay(1000);
}


void loop() {
  distanceCm = sr04.Distance();

  alarm.Update();
  switcher.Update();
  distanceSwitch.Update();
  watch.Update();

  if (distanceCm < ALARM_LENGTH_CM && watch.Running()) {
    watch.Stop();
    Serial.print(distanceCm);
    Serial.println("cm ALARM!!!");
    alarm.Activate();
  }
}
