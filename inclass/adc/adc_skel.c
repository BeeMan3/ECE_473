// adc_skel.c 
//A simple voltmeter with the ADC operating in 10 bit mode with no interrupts, 
//coarse ADC accuracy, no quiet mode. Skeletonized by R. Traylor 11.1.2016

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"

uint8_t  i;              //dummy variable
uint16_t adc_result;     //holds adc result 
char     lcd_str_h[16];  //holds string to send to lcd  
char     lcd_str_l[16];  //holds string to send to lcd  
div_t    fp_adc_result, fp_low_result;  //double fp_adc_result; 

/*******************************************************/
void spi_init(void){
 /* Run this code before attempting to write to the LCD.*/
 DDRF  |= 0x08;  //port F bit 3 is enable for LCD
 PORTF &= 0xF7;  //port F bit 3 is initially low

 DDRB  |= 0x07;  //Turn on SS, MOSI, SCLK
 PORTB |= _BV(PB1);  //port B initalization for SPI, SS_n off
//see: /$install_path/avr/include/avr/iom128.h for bit definitions   

 //Master mode, Clock=clk/4, Cycle half phase, Low polarity, MSB first
 SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
 SPSR=(1<<SPI2X);           //SPI at 2x speed (8 MHz)  
 }
/*******************************************************/

int main()
{
//initalize the SPI port then the LCD
spi_init();
lcd_init(); 
clear_display();

//Initalize ADC and its ports
DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off

ADMUX |= (1<<REFS0) | (1<<MUX2)| (1<<MUX1)| (1<<MUX0); //single-ended, input PORTF bit 7, right adjusted, 10 bits

ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);//ADC enabled, don't start yet, single shot mode 
                             //division factor is 128 (125khz)
while(1){ 
  ADCSRA |= (1<<ADSC); //poke ADSC and start conversion

  while(bit_is_clear(ADCSRA, ADIF)){} //spin while interrupt flag not set

  ACSR |= (1<<ACI); //its done, clear flag by writing a one 

  adc_result = ADC;                      //read the ADC output as 16 bits

  //div() function computes the value num/denom and returns the quotient and
  //remainder in a structure called div_t that contains two members, quot and rem. 
  
  //now determine Vin, where Vin = (adc_result/204.8)
  fp_adc_result = div(adc_result, 205);              //do division by 205 (204.8 to be exact)
  itoa(fp_adc_result.quot, lcd_str_h, 10);           //convert non-fractional part to ascii string
  fp_low_result = div((fp_adc_result.rem*100), 205); //get the decimal fraction into non-fractional form 
  itoa(fp_low_result.quot, lcd_str_l, 10);           //convert fractional part to ascii string

  //send string to LCD
  string2lcd(lcd_str_h);  //write upper half
  char2lcd('.');          //write decimal point
  string2lcd(lcd_str_l);  //write lower half

  for(i=0;i<=10;i++){ _delay_ms(50);}  //delay 0.5 sec
  clear_display();
  cursor_home();
  } //while
}//main
