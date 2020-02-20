// lab2_skel.c 
// John Behman
// Last Modified: 10.14.2019

// This code was modified directly from the provided lab2_skel.c on the course website
//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {0b11000000, 0b11111001, 0b10100100, 0b10110000,\
       	0b10011001, 0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10011000,\
	0b11111111, 0b00000000};

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
//Modified from Gansells Guide to Debouncing to include debouncing for all 8 buttons
uint8_t chk_buttons(uint8_t button) {
  static uint16_t state[8] = {0}; //holds present state
  state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
  if (state[button] == 0xF000) return 1;
  return 0;
}

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit0|digit1|colon|digit2|digit3|
//                                0      1      2     3      4
void segsum(uint16_t sum) {
  int cnt = sum; //determine how many digits there are 
  int dig = 0;
  while(cnt!=0){
	  cnt = cnt/10;
	  dig++;
  }

  //break up decimal sum into 4 digit-segments
  for(int i=0;i < 4;i++){
	  segment_data[i] = sum % 10;
	  sum = sum / 10;
  }
  
  //now move data to right place for misplaced colon position
  segment_data[4] = segment_data[3];
  segment_data[3] = segment_data[2];
  segment_data[2] = 10;

  //blank out leading zero digits
  if(dig < 2) segment_data[1] = 10;
  if(dig < 3) segment_data[3] = 10;
  if(dig < 4) segment_data[4] = 10;

  for(int k=0;k<5;k++){ //sets segment data to the matching encoded value
	int temp = segment_data[k];
	segment_data[k] = dec_to_7seg[temp];
  }
}
//************************************************************************************

uint8_t main()
{
  DDRB = 0xF0;		//set port bits 4-7 B as outputs
  uint16_t cnt = 0;
while(1){
  _delay_ms(1);		//insert loop delay for debounce
  DDRA = 0x00;		//make PORTA an input port with pullups 
  PORTA = 0xFF;
  PORTB = 0xF0;		//enable tristate buffer for pushbutton switches 

  for(int n=0;n<8;n++){ //now check each button and increment the count as needed
	  if(chk_buttons(n) == 1){
		  cnt = cnt + (1<<n); 
	  }
  }

  PORTB = 0x00;		//disable tristate buffer for pushbutton switches
 
  if(cnt > 1023){
	cnt = cnt % 1023; //bound the count to 0 - 1023
  }

  segsum(cnt);		//break up the disp_value to 4, BCD digits in the array: call (segsum)
 
  DDRA = 0xFF;		//make PORTA an output

  for(int i = 0;i<5;i++){
	  PORTA = segment_data[i]; //send 7 segment code to LED segments
	
		if(i == 0){
		  PORTB = 0x00;    //display on digit 1
		}
		if(i == 1){
		  PORTB = 0x10;    //display on digit 2
		}

		if(i == 3){
		  PORTB = 0x30;    //display on digit 3
		}
	  
		if(i == 4){
		  PORTB = 0x40;    //display on digit 4
		}
	  _delay_ms(1);
  }
  }//while
  return 0;
}//main
