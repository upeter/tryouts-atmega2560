//www.elegoo.com
//2016.12.8


#define ledPin1 13
#define ledPin2 12

void setup()
{


  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);



 // initialize timer1 

  noInterrupts();           // disable all interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;


  TCCR1B |= _BV(WGM12);   // CTC mode
  TCCR1B |= _BV(CS12);    // 256 prescaler 
  OCR1A = 31250;//15624            // compare match register 16MHz/256/2Hz
  TIMSK1 |= _BV(OCIE1A);  // enable timer compare interrupt

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3  = 0;

  TCCR3B |= _BV(WGM32);   // CTC mode
  TCCR3B |= _BV(CS32);    // 256 prescaler 
  OCR3A = 15624;            // compare match register 16MHz/256/4Hz
  TIMSK3 |= _BV(OCIE3A);  // enable timer compare interrupt


  interrupts();             


 
  // pinMode( 9, OUTPUT);
  
}


ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{

   digitalWrite(ledPin2, digitalRead(ledPin2) ^ 1); 

}


ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{

  digitalWrite(ledPin1, digitalRead(ledPin1) ^ 1);   // toggle LED pin

}



// main loop
void loop()
{

}
