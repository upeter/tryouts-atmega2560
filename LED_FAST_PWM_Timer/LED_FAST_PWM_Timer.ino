
#define LED 11
void setup() {
  //We want to make 38kHz, setup timer 1 (16 bit timer)
  // Clear interrupts
  cli();
  // Init TCCR1A
  TCCR1A = 0;
  // Init TCCR1B
  TCCR1B = 0;// same for TCCR1B
  // Init counter
  TCNT1  = 0;
  //Set counter 1 configuration - FAST PWM, no pre-sacling
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(CS12) | _BV(WGM12) | _BV(WGM13);
  // Set counter 1 frequency (16000000/freq=counts so 16.000.000/1024/4Hz=3906)
  ICR1 = 3906;
  // Set interrupts (not that we are using any, but arduino libraries will!
  sei();
  // Set 25% Duty cycle ICR1 * 25
  OCR1A = 976; 
  // Get the signal on pin 9
  pinMode( LED, OUTPUT);
}


void loop() {
  while(1)
  {
    // Pulse the LED. The receiver needs that to properly set it's AGC level
    // Normally any pulsing should be done in an ISR, not from within the main loop
    delay(500);          
    pinMode( LED, INPUT); // Does not drive the output pin; LED OFF
    delay(100);
    pinMode( LED, OUTPUT); // Drives the LED again.
  }

}
