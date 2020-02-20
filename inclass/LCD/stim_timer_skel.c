//
// stim_timer_skel.c
//
// Created: 10/14/2014 12:19:20 PM
//  Author: Justin Goins
//  Skeletonized for class use: R. Traylor 11.3.2014
//  Modifed to use hd44780_driver, R. Traylor 11.29.2016

// Gives practice in using both 8 and 16 bit timer counters as well as
// the use of a state machine to keep track of how button resources
// are used. Interrupts are not used but interrupt flags are manually
// manipulated.
//
// TCNT0 is used to count out a random time to wait for a users input.
// TCNT1 is used to count the time before the user hits a button
//
// Lines with code to be supplied by student are marked with a "*" in col 1.
//

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "hd44780.h"

enum states {SR_WELCOME, SR_RANDOM_DELAY, SR_TIMING_USER, SR_RESULTS};

//******************************************************************************
//                            spi_init                               
//Initalizes the SPI port to allow LCD access.
//******************************************************************************
void spi_init(void){
    DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
    //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, no interrupts 
    SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
    SPSR=(1<<SPI2X); //SPI at 2x speed (8 MHz)  
}//spi_init

int main(void) {
  uint16_t numticks = 0;
  enum states state = SR_WELCOME;// set default state
	
  spi_init();     //set up SPI
  lcd_init();     //set up LCD

  DDRB |= 0x80;   //port B bit 7 LED is the user signal 
  PORTB &= 0x7f;  //clear the LED
  DDRD = 0x00;    //all the pushbutton switches are inputs

  while(1) {
    switch (state) {
      case SR_WELCOME: {
        clear_display();
        string2lcd("REFLEX TESTER");
        line2_col1(); 
        string2lcd("Press any button");
       	_delay_ms(100); // force minimum 100ms display time
       	while (PIND == 0xFF) {};
	state = SR_RANDOM_DELAY; // progress to RANDOM_DELAY state
	break;
     }

      case SR_RANDOM_DELAY: {
        lcd_init();
        string2lcd("Press any button");
        line2_col1(); 
        string2lcd("after LED lights");
        // Set up TC0
       TCCR0 &= (0<<WGM00) | (0<<WGM01) | (0<<CS00) | (0<<CS01) |(0<<CS02);     // set TC0 timer into normal mode and disable clock
       TIMSK &= (0<<OCIE0) | (0<<TOIE0);            // disable TC0 interrupts	
       TIFR |= (1<<TOV0);             // manually clear the TC0 overflow flag
        // start the timer with a 1024 prescaler, 16MHz/1024 = 15.625 KHz
       TCCR0 |= (1<<CS00) | (1<<CS01) | (1<<CS02);

        //Now we need to randomly wait between 2-10 seconds.  Since it takes 1.64 ms 
        //for the 8 bit timer to overflow, we need to loop between 122 - 610 times.
        //The following code is supposedly more random than other methods 
        uint16_t numIterations = rand() / (RAND_MAX / 488 + 1); // pick number between (0 - 487)
        numIterations += 122; // numIterations should now be between (122 - 610)
    
        do {
    	    while (bit_is_clear(TIFR,TOV0)) {}; // wait until the TC0 overflow flag is set
    	    TIFR |= (1<<TOV0);                    // manually clear the TC0 overflow flag
    	    // note that the counter will automatically keep counting upward again
    	    numIterations--; // decrement number of iterations
           } while (numIterations > 0);
       TCCR0 &=  (0<<CS00) | (0<<CS01) |(0<<CS02);      // disable the TC0 timer
        state = SR_TIMING_USER; // progress to TIMING_USER state
        break;
    }
    case SR_TIMING_USER: {
      // Use 16 bit TC1 to measure the user's reaction time
     TCCR1B &= (1<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);// disable noise canceler, set WGM1{3,2} to 0, and disable clock
     TCCR1A &= 0x00;              // disable all of the output compare pins and set WGM1{1,0} to 0
     TIMSK &= (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1);                      // disable TC1 interrupts in TIMSK
     ETIMSK &= (0<<OCIE1C);       // disable TC1 interrupts	in ETIMSK
     TIFR |= (1<<TOV1);           // manually clear the TC1 overflow flag
     TCNT1 = 0x0000;              // initialize the TC1 counter to 0

      //Count the number of ticks until a button is pressed. Start the timer with a 1024 prescaler.
      //16MHz / 1024 = 15.625 KHz
     TCCR1B |= (1<<CS12) | (0<<CS11) | (1<<CS10);                                           // start TC1 counter
      PORTB |= 0x80; // light MSB LED so the user knows to push the button

      while ( ((TIFR & (1 << TOV1)) == 0) && (PIND == 0xFF) ) {}; // wait until button pressed or TC1 OVF set
      numticks = TCNT1;
     TCCR1B |=  (0<<CS12) | (0<<CS11) | (0<<CS10);    // stop the TC1 counter
      // note that the count is now stored in TCNT1
      state = SR_RESULTS; // progress to RESULTS state
      break;
    }
			
    case SR_RESULTS: {
      // Now we compute the results without using floating point arithmetic. The timer runs at 15.625KHz so there 
      // are 15.625 ticks in a millisecond. We can use this information to determine the user's reaction time.
      PORTB &= 0x7F; //disable LED
      //To compute milliseconds, we multiply by 8/125. Since we are multiplying a 16 bit number,
      //be sure to perform the math using a 32 bit number.
      uint32_t numMilliseconds = ((uint32_t)numticks * 8) / 125;
	
      clear_display();
      if (((TIFR & (1 << TOV1)) == 0) && (numMilliseconds == 0)) {
      // overflow wasn't triggered but numMilliseconds = 0, the user held down the button
        string2lcd("Cheating!!");
        line2_col1(); 
        string2lcd("Retry?");
        _delay_ms(1000);
      } else if ((TIFR & (1 << TOV1)) == 0) {
        // overflow wasn't triggered
        // display the time
        string2lcd("Your time:");
        line2_col1(); 
        lcd_int16(numMilliseconds, 3, 0, 0, 0);
        string2lcd(" ms");
      } else {
        // overflow was triggered
        // user took too long
        string2lcd(" Timer expired.");
        line2_col1(); 
        string2lcd("Press btn to rst");
    }
	
    _delay_ms(300); // add delay to avoid switch bouncing issues
    while (PIND == 0xFF) {}; // wait until a button is pressed
    state = SR_RANDOM_DELAY; // move back to the random delay state
    break;
    }
  }
 }
}
