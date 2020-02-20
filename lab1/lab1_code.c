// lab1_code.c 
// John Behman
// 9/30/2019

//This program was modified from the original program on the ECE 473 website. It displays the value in BCD and
//increments values from 0-99 with roll over with LEDS L1-L4 displaying the lower digit and L5-L8 displaying the upper

#include <avr/io.h>
#include <util/delay.h>

uint8_t temp = 0;
uint8_t ones = 0;
uint8_t tens = 0;
uint8_t temp_num = 0;

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// the number and display it on PORTB in BCD. 
//*******************************************************************************
int main()
{
DDRB = 0xFF;  //set port B to all outputs
while(1){     //do forever
 if(debounce_switch()) {
	 if(temp == 99) {temp = 0;} //if at 99 make it 0
	 else temp++; 		    //otherwise increment 

	 ones = temp%10;	
	 temp_num = temp/10;
	 tens = temp_num << 4;
	 PORTB = (tens + ones);	 
 }  //if swith true increment num and output to PORTB in BCD
  _delay_ms(2);                    //keep in loop to debounce 24ms
  } //while 
} //main
