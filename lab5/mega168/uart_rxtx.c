//UART Example for inclass coding
//Roger Traylor 12.4.12
//modified by John Behman
//Connect two mega128 boards via rs232 and they should end to each
//other a message and a sequence number.
//
//Change the message you send to your partner for checkoff.
//
//You can test this code by a "loopback" if you connect rx to tx
//on the DB9 connector.
//
//This code was modified to work with the atmega 168p for sending the remote
//temperature sensor data to the atmega 128

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions.h"
#include "hd44780.h"
#include "twi_master.h"
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t SHT21_rd_buf[2];
uint8_t SHT21_wr_buf[2];
uint8_t temp_cmd[2];// = 0b11100011;
uint8_t           i;
volatile uint8_t  rcv_rdy;
char              rx_char; 
char              lcd_str_array[16];  //holds string to send to lcd
uint8_t           send_seq=0;         //transmit sequence number
char              lcd_string[3];      //holds value of sequence number
char    lcd_string_array[16];  //holds a string to refresh the LCD

int main(){
	float temp;
	uint16_t SHT21_temp;
	uart_init();  
	init_twi();

	sei();
	while(1){
		if(rcv_rdy){//strlen(lcd_str_array)){
			SHT21_wr_buf[0] = 0xE3;
			twi_start_wr(0b10000001, SHT21_wr_buf, 1 ); //read temperature data from LM73 (2 bytes) 
			_delay_ms(2);    //wait for it to finish
			twi_start_rd(0b10000001, SHT21_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
			_delay_ms(2);    //wait for it to finish

			SHT21_temp = SHT21_rd_buf[0]; //save high temperature byte into lm73_temp
			SHT21_temp = SHT21_temp << 8; //shift it into upper byte 
			SHT21_temp |= SHT21_rd_buf[1]; //"OR" in the low temp byte to lm73_temp
			temp = (((float)SHT21_temp/65536)*175.72)-46.85;
			itoa(temp, lcd_string_array, 10); //convert to string in array with itoa() from avr-libc                           
			//**************  start tx portion *************** 
 			uart_puts(lcd_string_array);
			itoa(send_seq,lcd_string,10);
			uart_puts(lcd_string);
			uart_putc('\0');
			//**************  end tx portion ***************
			rcv_rdy=0;
		}
		}//while
	}//main

	ISR(USART_RX_vect){
		static  uint8_t  i;
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
