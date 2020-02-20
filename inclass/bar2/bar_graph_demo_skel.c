// bar_graph_demo_skel.c 
// R. Traylor
// 10.22.14
// demos interrups, counter timer and SPI

// !!! YOU MUST RENAME THIS FILE TO MATCH YOUR MAKEFILE   !!!
// !!! YOU MUST FILL IN THE AREAS MARKED WITH "@" SIGNS   !!!
// !!! DISCONNECT ALL OTHER CONNECTIONS TO YOUR AVR BOARD !!!

// This code implements a timer interrupt to update the bar graph display
// at 0.5 second intervals. Every half second, a new data value is sent to 
// the bargraph via SPI. The value is displayed as single led in a climbing
// pattern.

// Expected Connections:
// Bargraph board           Mega128 board 
// --------------      ----------------------    
//     reglck            PORTB bit 0 (ss_n)                      
//     srclk             PORTB bit 1 (sclk)
//     sdin              PORTB bit 2 (mosi)
//     oe_n                   ground
//     gnd2                   ground
//     vdd2                     vcc
//     sd_out               no connect

#include <avr/io.h>
#include <avr/interrupt.h>

/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void){
  DDRB   |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2); //output mode for SS, MOSI, SCLK
  
  SPCR   = (1<<MSTR) |(1<<CPOL) |(1<<CPHA) |(1<<SPE); //master mode, clk low on idle, leading edge sample

  SPSR   = (1<<SPI2X);  //choose double speed operation
 }//spi_init

/***********************************************************************/
//                              tcnt0_init                             
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void){
  ASSR   |=  (1<<AS0); //ext osc TOSC
  TIMSK  |=  (1<<TOIE0); //enable TCNT0 overflow interrupt
  TCCR0  |=  (1<<CS00); //normal mode, no prescale
}

/*************************************************************************/
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is    
//incremented. Every 7680 interrupts the minutes counter is incremented.
//TCNT0 interrupts come at 7.8125ms internals.
// 1/32768         = 30.517578uS
//(1/32768)*256    = 7.8125ms
//(1/32768)*256*64 = 500mS
/*************************************************************************/
ISR(TIMER0_OVF_vect){
  static uint8_t count_7ms = 0;        //holds 7ms tick count in binary
  static uint8_t display_count = 0x01; //holds count for display 

  count_7ms++;                //increment count every 7.8125 ms 
  if ((count_7ms % 64)==0){ //?? interrupts equals one half second 
    SPDR = display_count;               //send to display 
    while(bit_is_clear(SPSR, SPIF));               //wait till data sent out (while loop)
    PORTB |= (1<<PB0);          //HC595 output reg - rising edge...
    PORTB &= (0xFE);          //and falling edge
    display_count = (display_count<<1); //shift display bit for next time 
  }
  if (display_count == 0x00){display_count=0x01;} //back to 1st positon
}

/***********************************************************************/
//                                main                                 
/***********************************************************************/
int main(){     
tcnt0_init();  //initalize counter timer zero
spi_init();    //initalize SPI port
sei();         //enable interrupts before entering loop
while(1){}     //empty main while loop

} //main
