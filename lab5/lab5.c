// lab5.c 
// John Behman
// Last Modified: 12.11.2019

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
//  Volume is connected to PE3
//  Audio output connected to PD2
//  CdS Cell voltage value connected to PF7 

//#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = {0b11000000, 0b11111001, 0b10100100, 0b10110000,\
       	0b10011001, 0b10010010, 0b10000010, 0b11111000, 0b10000000, 0b10011000,\
	0b11111111, 0b00000000};

volatile uint16_t spi_preva; 	//previous state for encoder a
volatile uint16_t spi_prevb; 	//previous state for encoder b
volatile uint8_t temp1 = 1;	
volatile uint8_t sec = 0;	//seconds counter
volatile int8_t min = 0;	//minutes counter
volatile int8_t hour = 0;	//hours counter
volatile int8_t mina = 0;	//alarm minutes
volatile int8_t houra = 0;	//alarm hours
volatile uint8_t hprev;		//previous hour state
volatile uint8_t pm = 0;	//pm indicator
volatile uint8_t pma = 0;	//alarm pm indicator
volatile uint8_t mil = 1;	//military time indicator
volatile uint8_t a_flag = 0;	//alarm flag
volatile uint8_t a_set = 0;	//alarm set indicator
volatile uint8_t t_change = 0;	
volatile uint8_t hpreva;	//previous alarm hour state
volatile uint8_t set_prev = 0;	//previous set state
volatile uint8_t snooze = 0;	//snooze indicator
volatile uint8_t secprev;	//previous seconds counter
volatile uint8_t sblock = 0;	
volatile uint8_t scheck = 0;	//second equivalent indicator
char lcd_array[32] = "                                ";

extern uint8_t lm73_rd_buf[2];  //read buffer for lm73
extern uint8_t lm73_wr_buf[2];  //write buffer for lm73
char lcd_string_array[16];      

uint8_t i;
volatile uint8_t  rcv_rdy;      //indicates data is ready to be revieved 
char              rx_char;      //character that is transmitted
char              lcd_str_array[16];  //holds string to send to lcd
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number



extern enum radio_band{FM, AM, SW};    //radio band selection
extern volatile uint8_t STC_interrupt; 

volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

volatile uint16_t current_fm_freq = 9910; //initialize current freq and temp value
volatile uint16_t current_fm_freq_temp = 9910;
volatile uint16_t current_am_freq;
volatile uint16_t current_sw_freq;
uint8_t  current_volume;
volatile uint16_t freq_temp;
volatile uint8_t freq_cnt = 0;

volatile uint8_t freq_change = 0;
volatile uint8_t radio_on = 0;

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
//				encoder_data
//Collects the encoder data from the spi interface and then computes the logic for
//encoder direction. Encoder A increments hours and encoder B increments minutes.
//***********************************************************************************
uint16_t encoder_data(){
	uint8_t spi_current;
	uint8_t a_data;
	uint8_t b_data;
	PORTE &= 0xBF; //Set SH/LD to low
	_delay_us(50); //delay to allow for encoder values to be shifted in
	PORTE |= 0x40;  //set SH/LD to high again
	PORTD &= 0b11110111;  //set CLK_INH to low
	spi_current = spi_read(); //read the serial data from SPDR 
	PORTD |= 0b00001000;  //set CLK_INH back to high
	a_data = (spi_current & 0x03); //the encoder a data is reserved to the 
				       //lowest 2 bits in the spi data

	b_data = (spi_current & 0b00001100); 
	b_data = (b_data >> 2); 



	//CHANGE TIME	
	if(t_change){
	if(b_data != spi_prevb){ //if the current and previous dont equate

		//if the previous state was 00 and the current state is 01,
		//encoder a moved clockwise, therfore increment
		if((b_data == 0x01) && (spi_prevb == 0x00)){ 
			min = min + 1; //minutes incremented on encoder cw turn
			if(min == 60){ //hours incremented if minutes is 60
				min = 0; 
				if(hour != 23) hour++;
				else hour = 0;
				
			}
		}

		//if the previous state was 00 and the current state is 01,
		//encoder a moved ccw, therefore decrement
		else if((b_data == 0x02) && (spi_prevb == 0x00)){
			min = min - 1;//minutes decremented if ccw turn
			if(min < 0){ //ours decremented is decremented past 0 minutes
				min = 59;
				hour = hour - 1;
			}

			//set hours based on encoder turn
			if(hour < 0) hour = 23;
			if(hour == 0 && !mil) hour = 12;
		}
	}

		//equivalent logic for hours encoder 
	else if(a_data != spi_preva){
		if((a_data == 0x01) && (spi_preva == 0x00)){
			if(hour != 23) hour++; //hours incremented if cw turn
			else hour = 0;
				
		}
		
		else if((a_data == 0x02) && (spi_preva == 0x00)){
			hour = hour - 1; //hours decremented if ccw turn
			if(hour < 0) hour = 23;
			if(hour == 0 && !mil) hour = 12;
		}
	}	
	}

	//SET ALARM TIME
	//This block contains the equivalent encoder logic
	//to the block above. This block instead modifies 
	//the alarm time instead of the current time.
	else if(a_flag){
		
        	if(b_data != spi_prevb){ //if the current and previous dont equate
                	//if the previous state was 00 and the current state is 01,
                	//encoder a moved clockwise, therfore increment
                	if((b_data == 0x01) && (spi_prevb == 0x00)){
                        mina = mina + 1;
                        if(mina == 60){
                                mina = 0;
                                if(houra != 23) houra++;
                                else houra = 0;

                        }
                }

                //if the previous state was 00 and the current state is 01,
                //encoder a moved ccw, therefore decrement
                	else if((b_data == 0x02) && (spi_prevb == 0x00)){
                        	mina = mina - 1;
                        	if(mina < 0){
                                	mina = 59;
                                	houra = houra - 1;
                        	}
                        	if(houra < 0) houra = 23;
                        	if(houra == 0 && !mil) houra = 12;
                }
        }

        	else if(a_data != spi_preva){
                	if((a_data == 0x01) && (spi_preva == 0x00)){
                        	if(houra != 23) houra++;
                        	else houra = 0;

                	}

                	else if((a_data == 0x02) && (spi_preva == 0x00)){
                        	houra = houra - 1;
                        	if(houra < 0) houra = 23;
                        	if(houra == 0 && !mil) houra = 12;
                	}
        	}	
        }

	//FREQUENCY CHANGING
	//If the frequency changed flag is set, then based on encoder direction
	//change the frequency of the radio
        else if(freq_change){
                 if(b_data != spi_prevb){ //if the current and previous dont equate

                //if the previous state was 00 and the current state is 01,
                //encoder a moved clockwise, therfore increment
                if((b_data == 0x01) && (spi_prevb == 0x00)){
                        if(current_fm_freq_temp <= 10770) current_fm_freq_temp = current_fm_freq_temp + 20;
                        else current_fm_freq_temp = current_fm_freq_temp;
			current_fm_freq = current_fm_freq_temp;
			if(radio_on) fm_tune_freq();
                        }


                //if the previous state was 00 and the current state is 01,
                //encoder a moved ccw, therefore decrement
                else if((b_data == 0x02) && (spi_prevb == 0x00)){
                        if(current_fm_freq_temp >= 8830) current_fm_freq_temp = current_fm_freq_temp - 20;
                        else current_fm_freq_temp = current_fm_freq_temp;
			current_fm_freq = current_fm_freq_temp; 
			if(radio_on) fm_tune_freq();
                }
                }

        }


	//If there are no modes selected, default to changing the volume
	else{
		 if(b_data != spi_prevb){ //if the current and previous dont equate

                //if the previous state was 00 and the current state is 01,
                //encoder a moved clockwise, therfore increment
                if((b_data == 0x01) && (spi_prevb == 0x00)){
                        if(OCR3A < 148) OCR3A = OCR3A + 10;
                        else OCR3A = OCR3A + (158 - OCR3A);

                        }


                //if the previous state was 00 and the current state is 01,
                //encoder a moved ccw, therefore decrement
                else if((b_data == 0x02) && (spi_prevb == 0x00)){
                        if(OCR3A > 10) OCR3A = OCR3A - 10;
                        else OCR3A = 0;

                }
 		}

	}

	spi_preva = a_data; //set the current encoder value to the previous 
			    //to allow the function to check next time if the 
			    //encoder was turned
	spi_prevb = b_data;

	return 0;

}


//***********************************************************************************
//                              clk_count
//Deals with clock logic. Based on seconds counter constantly modifies current time,
//deals with am/pm logic for time and alarm time, modifies output based on military
//or standard time selection, assigns proper values to segment_data output register
//**********************************************************************************
uint8_t clk_count(){
	//increment minutes, seconds, and hours based on current values
        if(sec == 60){
		sec = 0;
		min++;
		if(min == 60){
			min = 0;
			hour++;	
			if(hour == 24){
				hour = 0;
			}
		}
	}

	//pm logic, if there is ever a change from am to pm in either
	//military or normal time, the value changes from am to pm 
	//or vice versa
	if(a_flag == 0){
		if((hprev == 23 && hour == 0) || (hprev == 11 && hour == 12) ||\
		(hprev == 0 && hour == 23) || (hprev == 12 && hour == 11) ){
			pm = !pm;
		}
	}
	
	//if alarm is set, same logic for the alarm pm indicator
	else if(a_flag){
        	if((hpreva == 23 && houra == 0) || (hpreva == 11 && houra == 12) ||\
        	(hpreva == 0 && houra == 23) || (hpreva == 12 && houra == 11) ){
                	pma = !pma;
        	}
	}

	//if not in military time, set the time to the correct
	//standard time relative to military
	if(!mil && a_flag == 0){
		if(hour > 11){
		hour = hour - 12;
		}
		if(hour == 0){
			hour = 12;
		}
	}

	//if alarm is set, same logic for the standard time set
	else if(!mil && a_flag){

                if(houra > 11){
                houra = houra - 12;
                }
                if(houra == 0){
                        houra = 12;
                }
        }

	//if even seconds, display the colon
	if((sec%2) == 0){
		segment_data[2] = 0b100;
	}	
	//otherwise turn it off
	else segment_data[2] |= 0b011;

	
	if(a_flag == 0){
		if(pm && !mil){ //Lights up L3 if its pm
	       		segment_data[2] &= 0b011;
		}
		else segment_data[2] |= 0b100;
	}

	//equivalent logic for the alarm time
	else if(a_flag){
                if(pma && !mil){
                        segment_data[2] &= 0b011;
                }
                else segment_data[2] |= 0b100;
        }
	

	//if alarm flag asserted, displays the 
	//current alarm time instead
	if(a_flag){
        segment_data[4] = dec_to_7seg[houra/10];
        segment_data[3] = dec_to_7seg[houra%10];
        segment_data[1] = dec_to_7seg[mina/10];
        segment_data[0] = dec_to_7seg[mina%10];
	}

	else if(freq_change){
	freq_temp = current_fm_freq / 10;	
        segment_data[4] = dec_to_7seg[(freq_temp/1000)%10];
        segment_data[3] = dec_to_7seg[(freq_temp/100)%10];
        segment_data[1] = dec_to_7seg[(freq_temp/10)%10];
        segment_data[0] = dec_to_7seg[freq_temp%10];
	segment_data[1] &= 0b01111111; 
	segment_data[2] = 0b111;
	}

        //displays the current time set 
	else{
        segment_data[4] = dec_to_7seg[hour/10];
        segment_data[3] = dec_to_7seg[hour%10];
        segment_data[1] = dec_to_7seg[min/10];
        segment_data[0] = dec_to_7seg[min%10];
        }


	//saves the previous time and alarm state for
	//keeping track of am and pm
	hprev = hour;
	hpreva = houra;

	return 0;
}

//**********************************************************************************
//				adc_data
//Takes the data from the adc, converts it, and assigns it to OCR2 for PWM 
//brightness control. This block was influenced by the adc_skel in class code.
//**********************************************************************************
uint8_t adc_data(){
	uint8_t data; 
	ADCSRA |= (1<<ADSC); //poke ADSC and start conversion
	while(bit_is_clear(ADCSRA, ADIF)){} //spin while interrupt flag not set
	ACSR |= (1<<ACI); //clear flag now that it is complete
	data = ADC/4; //the data must be divided by 4 to get the correct scaling
		      //since OCR2 is an 8 bit register
	data =  256 - data; //data must be flipped before going into OCR2.
			    //high values lower the duty cycle while larger
			    //values increase it
	if(data > 205) data = 205; //stops the display from getting too dim.
	OCR2 = data; //write the value to OCR2 to modify the PWM duty cycle

	return 0;
}


//***********************************************************************************
//				bar
//This function was written based on the bar graph in class assigment. Writes the
//modes to the bar graph display
//**********************************************************************************
uint8_t bar(){
	SPDR = (!mil<<4) |(sblock<<3) | (t_change<<0) | (a_flag<<1) | (a_set<<2) | (freq_change<<5) | (radio_on<<6); 
	while(bit_is_clear(SPSR, SPIF)){};	
	return 0;
}

//******************************************************************************
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){
	STC_interrupt = TRUE;
	//PORTF ^= (1 << PF1);
}


//***********************************************************************************
//				ISR(TIMER1_COMPA_vect)
//Timer counter 1 ISR. If alarm is on, this strobes portD pin 2 to create a tone.
//also manages the snooze logic to avoid playing a tone of the snooze button is hit
//***********************************************************************************
ISR(TIMER1_COMPA_vect){
if((hour == houra && min == mina) && a_set && !a_flag && !sblock){
	PORTD = PIND ^ 0b0000100;
	
}
if(snooze){
	snooze = 0; //reset snooze flag
	secprev = sec; //set current seconds state
	sblock = 1;    //set value to block alarm from going off
	if(sec > 49){  //increment the alarm minutes if you snooze past 49 seconds
		secprev = sec - 49;
		mina = mina + 1;
		scheck = 1; 
		if(min == 59){ //increment hours alarm if you snooze past 
			       //59 minutes 49 seconds
			mina = 0;
			houra = houra + 1;

			//logic to avoid going past 23 and 12 hours for military
			//and standard time
			if(mil){ 
				if(houra == 24) houra = 0;
			}
			else if(!mil){
				if(houra == 13) houra = 1;
			}
		}
	}
}
if(secprev + 10 == sec) sblock = 0; //play the alarm if it has been 10 seconds
else if(scheck && secprev == sec){  //this case covers cases where snooze is 
				    //pressed after 49 seconds
	sblock = 0;
	scheck = 0;
}	

}

//************************************************************************************
//				ISR(TIMER0_comp_vect)
//ISR for reading TWI data from the LM73 local temperature sensor. Based on the in
//class TWI code
//************************************************************************************
ISR(TIMER0_COMP_vect){
	static uint16_t cnt = 0;
	cnt++;
	uint16_t lm73_temp;

	if((cnt % 128)==0){ //after 1 second, increment the seconds counter
  	twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
  	_delay_ms(2);    //wait for it to finish
  	lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  	lm73_temp = lm73_temp << 8; //shift it into upper byte 
  	lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
  	itoa(lm73_temp >> 7, lcd_string_array, 10); //convert to string in array with itoa() from avr-libc 

	//display the local temperature
	lcd_array[16] = 'L';	
  	lcd_array[17] = lcd_string_array[0];
	lcd_array[18] = lcd_string_array[1];
	lcd_array[19] = 'C';

  	}
        


}


//************************************************************************************
//				ISR(TIMER0_OVF_vect)
//Interrupt that when the counter overflows is ran. Checks the buttons and 
//which one is pressed to respond accordingly. Also handles incrementing 
//of the seconds counter. Incrementing it once a second.
//************************************************************************************
ISR(TIMER0_OVF_vect){
  DDRA = 0x00;		//make PORTA an input port with pullups 
  PORTA = 0xFF;		//assert all of PORTA
  PORTB |= 0x70;	//enable tristate buffer for pushbutton switches 


  static uint16_t sec_cnt = 0;

  sec_cnt ++;
  if((sec_cnt % 128)==0){ //after 1 second, increment the seconds counter
	sec++;
  }

  for(int n=0;n<8;n++){ //now check each button and respond accordingly
	  if(chk_buttons(n) == 1){
		  if(n == 0){
			  t_change = !t_change; //assert time change variable
			  a_flag = 0;
			  freq_change = 0;
		  }

		  if(n == 1){
			  a_flag = !a_flag; 
			  a_set = 1; //akarm is set
			  set_prev = 0;
			  t_change = 0;
			  freq_change = 0;

			  //if military time and alarm time is pm and hours<12
			  if(a_flag){ 
				  if(mil && pma && houra <12){
					  houra = houra + 12;
			  	  }
			  }
			  
			  //if military time and time is pm and hours<12
			  else if(a_flag == 0){
				  if(mil && pm && hour < 12){
					  hour = hour + 12;
				  }
			  }

			
		  }

		  if(n == 4){
			mil = !mil; //change time mode
			
			//if not in alarm mode change normal time 
			if(a_flag == 0){
			//if mil time and pm hours = hours + 12
	       		if(mil && pm && hour != 12) hour = hour + 12;
			//if not pm then subtract 12
			if(mil && !pm && hour == 12) hour = hour - 12;
			}

			//if in alarm mode, change alarm time
			else if(a_flag){
			//equivalent logic for changing alarm time 
                        if(mil && pma && houra != 12) houra = houra + 12;
                        if(mil && !pma && houra == 12) houra = houra - 12;	
			}

		  }
		  if(n == 2){
			  //clears alarm and sets back to zero
			  a_flag = 0;
			  a_set = 0;
			  houra = 0;
			  mina = 0;
			  pma = 0;
			  sblock = 0;
		  }

		  if(n == 3){
			  //if an alarm is set, set the snooze on press
			  if(a_set) snooze = 1;

		  } 

		  if(n == 5){
			  freq_change = !freq_change; //put it into the fequency change mode
			  t_change = 0;
			  a_flag = 0;
		  }

		  if(n == 6){
			//engage radio mode  
			radio_on = !radio_on;
			 
		  }

		  if(n == 7){
			//modifies the frequency to be 3 preset values  
			if(freq_cnt == 0)  {
				current_fm_freq_temp = 9910;
				current_fm_freq = current_fm_freq_temp;
				freq_cnt++;
			}
			else if(freq_cnt == 1)  {
				current_fm_freq_temp = 9990;
				current_fm_freq = current_fm_freq_temp;
				freq_cnt++;
				
			}
			else if(freq_cnt == 2){
				current_fm_freq_temp = 10630;
				current_fm_freq = current_fm_freq_temp;
				freq_cnt = 0;
			}
		  }


	  }
  }
  DDRA = 0xFF;  //make PORTA outputs for the seven segment
  PORTB = 0x01;  
}	




int main()
{
  //TIMER 0 Initialization taken from the class slides
  ASSR |= (1<<AS0);
  TIMSK |= (1<<TOIE0) | (1<<OCIE0);
  OCR0 = 0xFF;
  TCCR0 |= (1<<CS00);

  //TIMER 1 Initialization 
  TIMSK |= (1<<OCIE1A); //enable output compare match on A
  DDRC |= 0x80; 
  TCCR1A = 0x00; 
  //CTC mode with prescale 64
  TCCR1B |= (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
  TCCR1C = 0x00;
  OCR1A = 0x80; //set output compare value to 0x80;
  OCR1B = 0xFFFF;


  //TIMER 2 Initialization, Fast PWM, clear on compare match, no prescale
  TCCR2 |= (1<<WGM21) | (1<<WGM20) | (1<<CS20) | (1<<COM21);
 
  //TIMER 3 Initialization, Fast PWM, no prescale, clear on compare match 
  TCCR3A |= (1<<COM3A1) | (0<<COM3A0)| (1<<WGM31);
  TCCR3B |= (0<<CS32) |(0<<CS31)| (1<<CS30)|(1<<WGM32)|(1<<WGM33);
  TCCR3C = 0x00;
  ICR3 = 120;  //set top value based on: Prescale*(1+TOP) = 16MHz/100kHz so top is 120
  TCNT3 = 0;   //initialize tcnt3 to 0
  OCR3A = 90; //set OCR3A for max volume, can be modified where
  	       //(0 = no noise) (159 = max volume)



  //PORT Initializations
  DDRB = 0xF0;		//set PORTB bits 4-7 B as outputs
  DDRD = 0b00001100;    //set PORTD bit 1 to an output
  DDRE |= 0x4F; 	//set PORTE bit 6 as an output
  PORTE |= 0xFF;	//assert PORTE bit 6
  PORTD |= 0x0F;	//assert PORTD bits 0-3 as output

  //Initalize ADC and its ports taken from in class activity
  DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
  PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
  ADMUX |= (1<<REFS0) | (1<<MUX2)| (1<<MUX1)| (1<<MUX0); //single-ended, input PORTF bit 7, right adjusted, 10 bits
  ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //ADC enabled, don't start yet, single shot mode 

  DDRF |= 0x08;


  //SPI INIT  
  DDRB   |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2); //output mode for SS, MOSI, SCLK
  SPCR   |= (1<<MSTR) | (0<<CPOL)| (0<<CPHA) | (1<<SPE); //master mode, clk low on idle, leading edge sample
  SPSR   |= (1<<SPI2X); //choose double speed operation


  uart_init();
  lcd_init(); //initialize lcd display
  init_twi();

  //enable interrupt generation on rising edge and enable interrupt 7
  EICRB |= (1<<ISC71) | (1<ISC70);
  EIMSK |= (1<<INT7);

  _delay_ms(1000);

  sei(); //set global interrupts

  //set LM73 mode for reading temperature by loading pointer register
  lm73_wr_buf[0] = 0x00; //load lm73_wr_buf[0] with temperature pointer address
  twi_start_wr(LM73_ADDRESS, lm73_rd_buf, 1); //start the TWI write process
  _delay_ms(2);    //wait for the xfer to finish

  uint8_t rad_flag = 1;

while(1){
  PORTB = 0x01;	//disable tristate buffer for pushbutton switches

  clk_count();  
  adc_data();  
  
  //displays ALARM on the LCD if alarm is set
  if(a_set && set_prev == 0){  
	lcd_array[0] = 'A';
	lcd_array[1] = 'L';
	lcd_array[2] = 'A';
	lcd_array[3] = 'R';
	lcd_array[4] = 'M';
	set_prev = 1;
  }
  //if alarm is nor longer set clear the display
  else if(a_set == 0 ){
	lcd_array[0] = ' ';
	lcd_array[1] = ' ';
	lcd_array[2] = ' ';
	lcd_array[3] = ' ';
	lcd_array[4] = ' ';
  }
  else {};
  
  //set remote tempature indicators on lcd
  lcd_array[21] = 'R';
  lcd_array[24] = 'C';

//recieve portion based on in class activity
      if(rcv_rdy==1){
        lcd_array[22] = lcd_str_array[0];  //write out string if its ready
        lcd_array[23] = lcd_str_array[1];  //write out string if its ready
        rcv_rdy=0;
    }//if 

//transmit portion based on in class activity
    uart_puts("A");
    uart_putc('\0');

  refresh_lcd(lcd_array);

  encoder_data(); //manages changing of time and alarm time 


  bar(); //update mode indication on bar graph


  //RADIO****************************
  	if(radio_on && rad_flag){
	        for(int i=0; i<2; i++){	
 		DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
		PORTE |= 0x04; //radio reset on for powerup

		
		lcd_array[9] = 'R';  //write out string if its ready
        	lcd_array[10] = 'A';  //write out string if its ready
        	lcd_array[11] = 'D';  //write out string if its ready
        	lcd_array[12] = 'I';  //write out string if its ready
        	lcd_array[13] = 'O';  //write out string if its ready

		//reset si4734
		PORTE &= ~(0x80); //int7 low
		DDRE  |= 0x80;      //turn on PE bit 7
		PORTE |=  (0x04); //hardware reset of si4734
		_delay_us(200);     //hold for 200us         
		PORTE &= ~(0x04); //then release reset_n 
		_delay_us(30);      //delayed required for correct operation
		DDRE  &= ~(0x80);   //now make PE7 an input

		
		fm_pwr_up(); //powerup the radio
		current_fm_freq = current_fm_freq_temp; //set frequency for tuning
		fm_tune_freq(); //tune radio to set frequency
		rad_flag = 0;
		}
	}

	if(!radio_on && !rad_flag){ //if radio is turned off, then turn off radio and clear lcd
	       	radio_pwr_dwn();
                lcd_array[9] = ' ';  //write out string if its ready
                lcd_array[10] = ' ';  //write out string if its ready
                lcd_array[11] = ' ';  //write out string if its ready
                lcd_array[12] = ' ';  //write out string if its ready
                lcd_array[13] = ' ';  //write out string if its ready
		rad_flag = 1;
	}
  for(int i = 0;i<5;i++){
	  PORTA = segment_data[i]; //send 7 segment code to LED segments
		if(i == 0){
		  PORTB = 0x00;    //display on digit 1
		}
		if(i == 1){
		  PORTB = 0x10;    //display on digit 2
		}

		if(i == 2){
		  PORTB = 0x20;    //display semicolon
		}

		if(i == 3){
		  PORTB = 0x30;    //display on digit 3
		}
	  
		if(i == 4){
		  PORTB = 0x40;    //display on digit 4
		}

	  _delay_us(200); //delay time to allow for minimal flickering when 
		 	  //displaying 1-4 digits

	PORTA = 0xFF;	  //set PORTA to FF to avoid ghosting on the last digit		   
  }
  	PORTB = 0x40;
  }//while
  return 0;
}//main


//ISR reffered to from the in class uart activity
ISR(USART0_RX_vect){
static  uint8_t  i = 0;
  //lcd_array[9] = 'B'; 
  rx_char = UDR0;              //get character
  lcd_str_array[i++]=rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1;
    lcd_str_array[--i]  = (' ');     //clear the count field
    lcd_str_array[i+1]  = (' ');
    lcd_str_array[i+2]  = (' ');
    i=0;
  }
}

