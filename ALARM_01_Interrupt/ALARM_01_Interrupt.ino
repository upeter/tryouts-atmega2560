//www.elegoo.com
//2016.12.08
#include "SR04.h"

#define TRIG_PIN 12
#define ECHO_PIN 10 
#define BUZZER 9 
#define ALARM_LENGTH_CM 20 
#define ledPin1 11

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
 
  // Constructor - creates a Flasher 
  // and initializes the member variables and state
  public:Flasher(int pin, long on, long off)
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
     if(active) {
    if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
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
     }
  }
};

Flasher led1(ledPin1, 200, 400);

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;
boolean peep;

void setup() {


  pinMode(ledPin1, OUTPUT);



 // initialize timer1 

  noInterrupts();           // disable all interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;


  TCCR1B |= _BV(WGM12);   // CTC mode
  TCCR1B |= _BV(CS12);    // 256 prescaler 
  OCR1A = 7400;            // compare match register 16MHz/256/2Hz
  TIMSK1 |= _BV(OCIE1A);  // enable timer compare interrupt



  interrupts();             
  
   Serial.begin(9600);//Initialization of Serial Port
   pinMode(BUZZER, OUTPUT);
   led1.ToggleActive();
   delay(1000);
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
if(peep) {
  digitalWrite(ledPin1, digitalRead(ledPin1) ^ 1);   // toggle LED pin
} else {
    digitalWrite(ledPin1, 0);  
}

}

void loop() {
   a=sr04.Distance();
   Serial.print(a);
   Serial.println("cm");//The difference between "Serial.print" and "Serial.println" 
                        //is that "Serial.println" can change lines.
                        led1.Update();
   if(a < ALARM_LENGTH_CM) {
    
    peep = true;
   } else {
    peep = false;
   }
   if(peep) {
    tone(BUZZER, 1000);
   } else {
    noTone(BUZZER);
   }
   //delay(500);
}
