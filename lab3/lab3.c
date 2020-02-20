// lab3.c 
// John Behman
// Last Modified: 10.29.2019

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.
//  PORTB.0 goes to RCLK
//  PORTB.1 goes to SRCCLK and SCK
//  PORTB.2 goes to SDIN
//  PORTB.3 goes to SER_OUT
//  PORTD.1 goes to CLK_INH
//  PORTE.6 goes to SH/LD
//  OE_N is grounded

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {0b11000000, 0b11111001, 0b10100100, 0b10110000,\
       	0b10011001, 0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10011000,\
	0b11111111, 0b00000000};


volatile int16_t inc = 1; //holds increment value
volatile int16_t cnt = 0; //holds the current count
volatile uint16_t spi_preva; //previous state for encoder a
volatile uint16_t spi_prevb; //previous state for encoder b

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
//*****************************************************************************
uint8_t chk_buttons(uint8_t button) {
  static uint16_t state[8] = {0}; //holds present state
  state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
  if (state[button] == 0xF000) return 1;
  return 0;
}


//********************************************************************
//                            spi_read                               
//Reads the SPI port. This function was taken from the bar graph 
//in class assignment.
//********************************************************************
  uint8_t spi_read(void){
  SPDR = 0x00;                       //"dummy" write to SPDR
  while (bit_is_clear(SPSR,SPIF)){}  //wait till 8 clock cycles are done
  return(SPDR);                      //return incoming data from SPDR        
}//read_spi



//***********************************************************************************
//                            segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit0|digit1|colon|digit2|digit3|
//                                0      1      2     3      4
//This is the function I created from lab 2
//**********************************************************************************                                
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

//***********************************************************************************
//				encoder_data
//Collects the encoder data from the spi interface and then computes the logic for
//encoder direction. Was unable to get encoder b to work but the code is here 
//and commented out.
//***********************************************************************************
uint16_t encoder_data(){
	uint8_t spi_current;
	uint8_t a_data;
	//uint8_t b_data;
	PORTE &= 0x00; //Set SH/LD to low
	_delay_us(50); //delay to allow for encoder values to be shifted in
	PORTE = 0xFF;  //set SH/LD to high again
	PORTD = 0x00;  //set CLK_INH to low
	spi_current = spi_read(); //read the serial data from SPDR 
	PORTD = 0xFF;  //set CLK_INH back to high
	a_data = (spi_current & 0x03); //the encoder a data is reserved to the 
				       //lowest 2 bits in the spi data

	/*b_data = (spi_current & 0x0b00001100);
	b_data = (b_data >> 2);*/

	if(a_data != spi_preva){ //if the current and previous dont equate

		//if the previous state was 00 and the current state is 01,
		//encoder a moved clockwise, therfore increment
		if((a_data == 0x01) && (spi_preva == 0x00)){ 
			cnt = cnt + inc;
		}

		//if the previous state was 00 and the current state is 01,
		//encoder a moved ccw, therefore decrement
		if((a_data == 0x02) && (spi_preva == 0x00)){
			cnt = cnt - inc;
		}
	}

	/*if(b_data != spi_prevb){
		if((b_data == 0x01) && (spi_prevb == 0x00)){
			cnt = cnt + inc;
		}
		
		if((b_data == 0x02) && (spi_prevb == 0x00)){
			cnt = cnt - inc;
		}
	}*/	

	spi_preva = a_data; //set the current encoder value to the previous 
			    //to allow the function to check next time if the 
			    //encoder was turned
	//spi_prevb = b_data;

	return cnt;
}

//***********************************************************************************
//				bar
//This function was written based on the bar graph in class assigment
//**********************************************************************************
uint8_t bar(){
	SPDR = inc;
	while(bit_is_clear(SPSR, SPIF)){};	
	return 0;
}


//************************************************************************************
//				ISR(TIMER0_OVF_vect)
//Interrupt that when the counter overflows is ran. Checks the buttons and 
//which one is pressed, as well as calls the encoder function that looks for the
//behavior of the encoders and responds accordingly
//************************************************************************************
ISR(TIMER0_OVF_vect){
  DDRA = 0x00;		//make PORTA an input port with pullups 
  PORTA = 0xFF;		//assert all of PORTA
  PORTB |= 0x70;	//enable tristate buffer for pushbutton switches 

  for(int n=0;n<8;n++){ //now check each button and increment the count as needed
	  if(chk_buttons(n) == 1){	  
		  if(n == 0){
			  inc = 1;
		  }
		  if(n == 1){
			  inc = 2;
		  }
		  if(n == 2){
			  inc = 4;
		  }
	  }
  }
  DDRA = 0xFF;  //make PORTA outputs for the seven segment
  PORTB = 0x01; 

  cnt = encoder_data(); //new cnt value is return of encoder_data()

  bar(); 
}	




uint8_t main()
{
  //TIMER 0 Initialization taken from the class slides
  TIMSK |= (1<<TOIE0);
  TCCR0 |= (1<<CS02) | (1<<CS00);
 
  //PORT Initializations
  DDRB = 0xF0;		//set PORTB bits 4-7 B as outputs
  DDRD = 0x02;          //set PORTD bit 1 to an output
  DDRE = 0x40; 		//set PORTE bit 6 as an output
  PORTE = 0x40;		//assert PORTE bit 6

  //SPI INIT  
  DDRB   |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2); //output mode for SS, MOSI, SCLK
  SPCR   |= (1<<MSTR) | (0<<CPOL)| (0<<CPHA) | (1<<SPE); //master mode, clk low on idle, leading edge sample
  SPSR   |= (1<<SPI2X); //choose double speed operation
 
  sei(); //set global interrupts

while(1){
  PORTB = 0x01;		//disable tristate buffer for pushbutton switches
  
  segsum(cnt);		//break up the disp_value to 4, BCD digits in the array: call (segsum)

  if(cnt > 1023){
	  cnt = cnt % 1023; //if greater then 1023 %1023
  }
  if(cnt < 0){
	  cnt = 1023; //if count becomes negative set it to 1023
  }

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
		
	  _delay_us(710); //delay time to allow for minimal flickering when 
	  		  //displaying 1-4 digits
	  
  }
  }//while
  return 0;
}//main
