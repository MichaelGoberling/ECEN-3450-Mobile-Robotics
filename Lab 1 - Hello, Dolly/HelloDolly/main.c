/*
 * HelloDolly.c
 *
 * Created: 1/17/2018 2:15:51 PM
 * Author : Michael Goberling
 */ 

#include "capi324v221.h"


void CBOT_main( void )
{
	// Open the LCD subsystem module.
	LCD_open();
   
	// Clear the display.
    LCD_clear();
	
	//Print, "Hello, Dolly!".
	LCD_printf( "Hello, Dolly!\n" );
	
	while (1);
}

