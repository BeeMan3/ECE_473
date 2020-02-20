Names:




// tcnt0_volatile_skel.c
// setup TCNT0 in output compare mode 
// create counting pattern on port B
// count update is every (32668)*(128)*(256) = 0.999975168 sec

#include <avr/io.h>
#include _____________________________

//declare 8-bit variable that is outside scope of main
_______________________________

//***********************************************************************
//                     ISR for timer counter zero
//***********************************************************************

_______________________________

_______________________________

_______________________________

//***********************************************************************
//                           init_tcnt0
//***********************************************************************
//initalize timer/counter zero to CTC mode

void init_tcnt0(){
  ASSR  |=  (1<<AS0);                //run off external 32khz osc (TOSC)
  //enable interrupts for output compare match 0
  
  _______________________________

  TCCR0 |=  (1<<WGM01) | (1<<CS00);  //CTC mode, no prescale
  OCR0  |=  0x07f;                   //compare at 128
}
//***********************************************************************

//***********************************************************************
//                              main
//***********************************************************************
int main() {

  DDRB = 0xFF;                     //set all port B pins to output
  init_tcnt0();                    //initalize timer counter zero
  _______________________________  //enable global interrupts
  
  while(1){
    if(ext_count == 255){     //blink light at 1sec intervals
      PORTB++;                //increment count pattern
      __________________      //hint: its an assignment statement
    } 
  }//while
}// main

