#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL //fazer ifdef para diferentes cpus
uint64_t _millis = 0;
uint16_t _1000us = 0;
uint64_t old_millis = 0;

#if defined(__arm__)
  volatile uint12_t ADC_value = 0;
#else
  volatile uint16_t ADC_value = 0;
#endif

/* interrupts routines */

//End of ADC conversion
ISR(ADC_vect){
  ADC_value = ADC;
}

// timer overflow occur every 0.256 ms
ISR(TIM0_OVF_vect) {
  _1000us += 256;
  while (_1000us > 1000) {
    _millis++;
    _1000us -= 1000;
  }
}

// safe access to millis counter
uint64_t millis() {
  uint64_t m;
  cli();
  m = _millis;
  sei();
  return m;
}

#if defined(__arm__)
uint12_t analogRead(){
  sei()
  ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE);
  while (!(0b11101111 | ADCSRA) )
}

#else
uint10_t analogRead(){
  sei()
  ADCSRA |= _BV(ADSC) | _BV(ADIE);
  while ()
}
#endif


/*void id setup(void) {


  //interrup setup
  // prescale timer0 to 1/8th the clock rate
  // overflow timer0 every 0.256 ms
  TCCR0B |= (1<<CS01);
  // enable timer overflow interrupt
  TIMSK  |= 1<<TOIE0;

  // Enable global interrupts
  sei();
}

void loop(void) {
  // every 1000ms toggle LED
  if ((millis() - old_millis) > 100) {
   // Toggle Port B pin 3 output state
   PORTB ^= 1<<PB3;
   old_millis = millis();
  }
}*/