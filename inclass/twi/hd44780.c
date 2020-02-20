//hd44780.c
//A set of useful functions for writing LCD characer displays
//that utilize the Hitachi HD44780 or equivalent LCD controller.
//This code includes adaptation for 4 bit LCD interface.

//Original source by R. Traylor ~2006
//Refactored by R. Traylor 10.7.09
//New features added by Kirby Headrick 11.20.09
//lcd_send with delay added by William Dillon 10.8.10
//Cleaned up again by R. Traylor 12.28.2011

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "hd44780.h"

#define CMD_BYTE   0
#define CHAR_BYTE  1
#define NUM_LCD_CHARS 16

//If using an interrupt driven, one char at a time style lcd refresh
//for a 2x16 display, use this array to place anything to be displayed
char lcd_string_array[32];  //holds two strings to refresh the LCD

char  lcd_str[16];  //holds string to send to lcd  

//-----------------------------------------------------------------------------
//                               send_lcd
//
// Sends a command or character data to the lcd. First argument of 0x00 indicates 
// a command transfer while 0x01 indicates data transfer.  The next byte is the 
// command or character byte. Last argument is the delay in uS to be executed 
// after the byte is sent.  A zero delays are O.K., as delays can be added after 
// the return.
//
// This is a low-level function usually called by the other functions but may
// be called directly to provide more control, especially over the wait delay.
//
void send_lcd(uint8_t cmd_or_char, uint8_t byte, uint16_t wait){
uint8_t temp;

//TODO: uint32_t temp_long;
//TODO: temp_long = F_CPU;


#if SPI_MODE==1
  SPDR = (cmd_or_char)? 0x01 : 0x00;  //send the proper value for intent
  while (bit_is_clear(SPSR,SPIF)){}   //wait till byte is sent out
  SPDR = byte;                        //send payload
  while (bit_is_clear(SPSR,SPIF)){}   //wait till byte is sent out
  strobe_lcd();                       //strobe the LCD enable pin
  _delay_us(1000);                    //typ 1ms for CMDs, 100uS for CHARs  TODO: KLUDGE ALERT
#else //4-bit mode
  if(cmd_or_char==0x01){LCD_PORT |=  (1<<LCD_CMD_DATA_BIT);}
  else                 {LCD_PORT &= ~(1<<LCD_CMD_DATA_BIT);} 
  temp = LCD_PORT & 0x0F;             //perserve lower nibble, clear top nibble
  LCD_PORT   = temp | (byte & 0xF0);  //output upper nibble first
  strobe_lcd();                       //send to LCD
  LCD_PORT   = temp | (byte << 4);    //output lower nibble second
  strobe_lcd();                       //send to LCD
  if(cmd_or_char==0x01) {_delay_us(100);}  //typ 1ms for CMDs, 100uS for CHARs
  else                  {_delay_us(1000);}
#endif
}

//------------------------------------------------------------------
//                          refresh_lcd 
//
//When called, writes one character to the LCD.  On each call it     
//increments a pointer so that after 32 calls, the entire lcd is    
//written. The calling program is responsible for not calling this 
//function more often than every 1ms which is the maximum amount of 
//time it might take for any operation to take in this function.
//(homeline or homeline2). Its presently being called every 8ms as 
//determined by the setup for for TCNT1.
//
//The array is organized as one array of 32 char locations to make 
//the index handling easier.  To external functions that write into 
//the array it will appear as two separate 16 location arrays by 
//using an offset into the array address.
//
//        LCD display character index postions (2x16 display)
//  -----------------------------------------------------------------
//  |  0|  1|  2|  3|  4|  5|  6|  7|  8|  9| 10| 11| 12| 13| 14| 15| 
//  -----------------------------------------------------------------
//  | 16| 17| 18| 19| 20| 21| 22| 23| 24| 25| 26| 27| 28| 29| 30| 31|  
//  -----------------------------------------------------------------
//
//
//TODO: why is the delay 40us before hte homeline2 and cursor home?
//TODO: integrate the unified lcd code here.

void refresh_lcd(char lcd_string_array[]) {

    static uint8_t index=0;           // index into string array 

    SPDR = 0x01; //set SR for data
    while (!(SPSR & 0x80)) {}       //wait for SPI transfer to complete
    SPDR = lcd_string_array[index];
    while (!(SPSR & 0x80)) {}       //wait for SPI transfer to complete
    strobe_lcd();                   //write into LCD
    index++;                        //increment to next character
    if(index == 16) {_delay_us(40); home_line2();}           //on to 2nd line, 1st position 
    if(index == 32) {_delay_us(40); cursor_home(); index=0;} //back to 1st line, 1st position
}//refresh_lcd
/***********************************************************************/

//-----------------------------------------------------------------------------
//                          set_custom_character
//
//Loads a custom character into the CGRAM (character generation RAM) of the LCD
//data is an 8 element array of uint8_t, the last 5 digits of each element
//define the on/off pattern of each row starting at the top, 1 is on (dark) and
//0 is off (light)
//Example of a music quarter note
//uint8_t custom_LCD0[8] = {
//    0x04,             |  #  |
//    0x06,             |  ## |
//    0x05,  Generates  |  # #|
//    0x05,  -------->  |  # #|
//    0x04,             |  #  |
//    0x1C,             |###  |
//    0x1C,             |###  |
//    0x00,             |     |
//    };
//
//Variable "address" is the location to store the character where valid 
//addresses are 0x00 - 0x07 for a total of 8 custom characters (0x08 - 0x0F 
//map to 0x00 - 0x07) to display a custom character just refer to it's 
//address, such as send_lcd(CHAR_BYTE, 0x01, 1) or it can be insteretd into a string 
//via a forward slash escape as in string2lcd("this is my music note \1")

void set_custom_character(uint8_t data[], uint8_t address){
    uint8_t i;
    send_lcd(CMD_BYTE, 0x40 + (address << 3), 1000);  //only needs 40uS!
    for(i=0; i<8; i++){send_lcd(CHAR_BYTE, data[i], 100);}
}

//-----------------------------------------------------------------------------
//                          set_cursor 
//
//Sets the cursor to an arbitrary potition on the screen, row is either 1 or 2
//col is a number form 0-15, counting from left to right
void set_cursor(uint8_t row, uint8_t col){
    send_lcd(CMD_BYTE, 0x80 + col + ((row-1)*0x40), 1000);
}
//-----------------------------------------------------------------------------
//                          uint2lcd 
//
//Takes a 8bit unsigned and displays it in base ten on the LCD. Leading 0's are 
//not displayed.  
//TODO: optimize by removing the mod operators
//TODO: does not display the number zero!  (FIXED 8.30.2014)
//
void uint2lcd(uint8_t number){
    if  (number == 0)  {send_lcd(CHAR_BYTE, 0x30                , 100); }
    else{
      if(number >= 100){send_lcd(CHAR_BYTE, 0x30+number/100     , 100); }
      if(number >= 10) {send_lcd(CHAR_BYTE, 0x30+(number%100)/10, 100); }
      if(number >= 1)  {send_lcd(CHAR_BYTE, 0x30+(number%10)    , 100); }
    }
}

//-----------------------------------------------------------------------------
//                          int2lcd  
//
//Takes a 8bit signed and displays it in base ten on the LCD. Leading 0's are 
//not displayed.
//
void int2lcd(int8_t number){
    if(number < 0){send_lcd(CHAR_BYTE, '-', 100); uint2lcd(~number+1);}    //take 2's complement of number and display
    else                                    {uint2lcd(number);}
}

//-----------------------------------------------------------------------------
//                          cursor_on
//
//Sets the cursor to display
void cursor_on(void){send_lcd(CMD_BYTE, 0x0E, 1000);}

//-----------------------------------------------------------------------------
//                          cursor_off
//
//Turns the cursor display off
void cursor_off(void){send_lcd(CMD_BYTE, 0x0C, 1000);}

//-----------------------------------------------------------------------------
//                          shift_right 
//
//shifts the display right one character
void shift_right(void){send_lcd(CMD_BYTE, 0x1E, 1000);}

//-----------------------------------------------------------------------------
//                          shift_left  
//
//shifts the display left one character
void shift_left(void){send_lcd(CMD_BYTE, 0x18, 1000);}

//-----------------------------------------------------------------------------
//                          strobe_lcd  
//
void strobe_lcd(void){ 
#if SPI_MODE==1
 PORTF |=  0x08; PORTF &= ~0x08; //toggle port F bit 3, LCD strobe trigger
#else
//4-bit mode below
 LCD_PORT |= (1<<LCD_STROBE_BIT);           //set strobe bit
 asm("nop"); asm("nop"); asm("nop"); asm("nop"); //4 cycle wait 
 LCD_PORT &= ~(1<<LCD_STROBE_BIT);          //clear strobe bit
 asm("nop"); asm("nop"); asm("nop"); asm("nop"); //4 cycle wait 
#endif
}
 
//-----------------------------------------------------------------------------
//                          clear_display  
//
void clear_display(void){send_lcd(CMD_BYTE, 0x01,2000);} //2ms wait for LCD

//-----------------------------------------------------------------------------
//                          cursor_home    
//
//Set cursor to row 0, column 0.
void cursor_home(void){send_lcd(CMD_BYTE, 0x02,1500);} //1.5ms wait for LCD
  
//-----------------------------------------------------------------------------
//                          home_line2    
//
//Put cursor at row 1, column 0
void home_line2(void){send_lcd(CMD_BYTE, 0xC0,1500);} //1.5ms wait for LCD
 
//-----------------------------------------------------------------------------
//                          fill_spaces   
//
//Fill an entire line with spaces.
void fill_spaces(void){
	uint8_t i;
	for (i=0; i<=(NUM_LCD_CHARS-1); i++){
		send_lcd(CHAR_BYTE, ' ',100); //100us wait between characters
	}
}  
   
//----------------------------------------------------------------------------
//                            char2lcd
//                            
//Send a single char to the LCD.
//usage: char2lcd('H');  // send an H to the LCD
void char2lcd(char a_char){send_lcd(CHAR_BYTE, a_char, 100);} //100us wait after char
  

//----------------------------------------------------------------------------
//                            string2lcd
//                            
//Send a ascii string to the LCD.
void string2lcd(char *lcd_str){ 
  uint8_t i;
  for (i=0; i<=(strlen(lcd_str)-1); i++){send_lcd(CHAR_BYTE, lcd_str[i], 100);}                  
} 

//----------------------------------------------------------------------------
//                            lcd_int 
//
//Initalize the LCD 
//
void lcd_init(void){
  _delay_ms(16);      //power up delay
#if SPI_MODE==1       //assumption is that the SPI port is intialized
  //TODO: kludge alert! should not be here.
  DDRF=0x08;          //port F bit 3 is enable for LCD in SPI mode
  send_lcd(CMD_BYTE, 0x30, 7000); //send cmd sequence 3 times 
  send_lcd(CMD_BYTE, 0x30, 7000);
  send_lcd(CMD_BYTE, 0x30, 7000);
  send_lcd(CMD_BYTE, 0x38, 5000);
  send_lcd(CMD_BYTE, 0x08, 5000);
  send_lcd(CMD_BYTE, 0x01, 5000);
  send_lcd(CMD_BYTE, 0x06, 5000);
  send_lcd(CMD_BYTE, 0x0C + (CURSOR_VISIBLE<<1) + CURSOR_BLINK, 5);
#else //4-bit mode
  LCD_PORT_DDR = 0xF0                    | //initalize data pins
                 ((1<<LCD_CMD_DATA_BIT)  | //initalize control pins
                  (1<<LCD_STROBE_BIT  )  |
                  (1<<LCD_RDWR_BIT)    ); 
  //do first four writes in 8-bit mode assuming reset by instruction
  //command and write are asserted as they are initalized to zero
  LCD_PORT = 0x30; strobe_lcd(); _delay_ms(8);  //function set,   write lcd, delay > 4.1ms
  LCD_PORT = 0x30; strobe_lcd(); _delay_us(200);//function set,   write lcd, delay > 100us
  LCD_PORT = 0x30; strobe_lcd(); _delay_us(80); //function set,   write lcd, delay > 37us
  LCD_PORT = 0x20; strobe_lcd(); _delay_us(80); //set 4-bit mode, write lcd, delay > 37us
  //continue initalizing the LCD, but in 4-bit mode
  send_lcd(CMD_BYTE, 0x28, 7000); //function set: 4-bit, 2 lines, 5x8 font
  //send_lcd(CMD_BYTE, 0x08, 5000);
  send_lcd(CMD_BYTE, 0x01, 5000);  //clear display
  send_lcd(CMD_BYTE, 0x06, 5000);  //cursor moves to right, don't shift display
  send_lcd(CMD_BYTE, 0x0C | (CURSOR_VISIBLE<<1) | CURSOR_BLINK, 5);
#endif
}


//************************************************************************
//                                lcd_int32
// Displays a 32-bit value at the current position on the display. If a 
// fieldwidth is specified, the field will be cleared and the number right 
// justified within the field.  If a decimal position is specified, the 
// data divided by 10^decpos will be displayed with decpos decimal positions
// displayed.  If bSigned is YES, the data will be treated as signed data.  
// If bZeroFill and fieldwidth are specified, the number will be displayed 
// right justified in the field and all positions in the field to the left 
// of the number will be filled with zeros.
// This code courtesy of David Naviaux.
//************************************************************************
void  lcd_int32(int32_t l,          //number to display
                uint8_t fieldwidth, //width of the field for display
                uint8_t decpos,     //0 if no decimal point, otherwise
                uint8_t bSigned,    //non-zero if the number should be treated as signed 
                uint8_t bZeroFill)  //non-zero if a specified fieldwidth is to be zero filled
{
      char    sline[NUM_LCD_CHARS+1];
      uint8_t i=0;
      char    fillch;
      ldiv_t  qr;

      qr.quot = l; // initialize the quotient 

      if (bSigned){
        bSigned = (qr.quot<0);
        qr.quot = labs(qr.quot);
      }

      // convert the digits to the right of the decimal point 
      if (decpos){
        for (; decpos ; decpos--){
          qr = ldiv(qr.quot, 10);
          sline[i++] = qr.rem + '0';
        }
        sline[i++] = '.';
      }

      // convert the digits to the left of the decimal point 
      do{
          qr = ldiv(qr.quot, 10);
          sline[i++] = qr.rem + '0';
        }while(qr.quot);

      // fill the whole field if a width was specified
      if (fieldwidth){
        fillch = bZeroFill? '0': ' '; // determine the fill character
        for (; i<fieldwidth ; ){sline[i++] = fillch;}
      }

      // output the sign, if we need to
      if (bSigned){sline[i++] = '-';}

      // now output the formatted number
      do{send_lcd(CHAR_BYTE, sline[--i] , 1000);} while(i);

}

//***************************************************************************************
//                                        lcd_int16
//
// Displays a 16-bit value at the current position on the display.  If a fieldwidth 
// is specified, the field will be cleard and the number right justified within the 
// field.  If a decimal position is specified, the data divided by 10^decpos will be 
// displayed with decpos decimal positions displayed.  If bZeroFill and fieldwidth 
// are specified, the number will be displayed right justified in the field and all 
// positions in the field to the left of the number will be filled with zeros.
//
// IN:           l               - number to display
//               fieldwidth      - width of the field
//               decpos          - 0 if no decimal point, otherwise
//               bSigned         - non-zero if the number should be treated as signed
//               bZeroFill       - non-zero if a specified fieldwidth is to be zero filled
//
//**************************************************************************************
void    lcd_int16(int16_t l, 
                  uint8_t fieldwidth, 
                  uint8_t decpos, 
                  uint8_t bZeroFill)
{
        char    sline[NUM_LCD_CHARS+1];
        uint8_t i=0;
        char    fillch;
        div_t   qr;
        uint8_t bSigned;

        // initialize the quotient 
        qr.quot = l;

        if ( (bSigned=(qr.quot<0)) )
                qr.quot = -qr.quot;

        // convert the digits to the right of the decimal point 
        if (decpos){
          for (; decpos ; decpos--){
            qr = div(qr.quot, 10);
            sline[i++] = qr.rem + '0';
          }
          sline[i++] = '.';
        }

        // convert the digits to the left of the decimal point 
        do
        {
                qr = div(qr.quot, 10);
                sline[i++] = qr.rem + '0';
        }
        while(qr.quot);

        // add the sign now if we don't pad the number with zeros 
        if (!bZeroFill && bSigned)
        {
                sline[i++] = '-';
                bSigned = 0;
        }

        // fill the whole field if a width was specified 
        if (fieldwidth){
          // determine the fill character 
          fillch = bZeroFill? '0': ' ';
          for (; i<(fieldwidth-bSigned) ; ){ sline[i++] = fillch;}
        }

        // output the sign, if we need to 
        if (bSigned){sline[i++] = '-';}

        // now output the formatted number 
            do{send_lcd(CHAR_BYTE, sline[--i] , 1000);} while(i);
}

