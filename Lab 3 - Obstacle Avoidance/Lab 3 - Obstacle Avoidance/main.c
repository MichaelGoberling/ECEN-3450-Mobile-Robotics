/*
 * Lab_3___Obstacle_Avoidance.c
 *
 * Created: 1/17/2018 2:15:51 PM
 * Author : Michael Goberling
 * Course : ECEN 3450 
 * Lab : 3
 */ 
// ======================= defines and includes ==================//
#include "capi324v221.h"
#define FORWARD 0
#define BACK 1

// ======================= prototypes ============================ //
void IR_AVOID( void );
void move_inch( unsigned char fwdbck, unsigned int distance );
void zero_point_turn( unsigned short direction );
void cruise( void );

// ======================= main ============================ //
void CBOT_main( void )
{
	//Open Required resources
	ATTINY_open();
	LCD_open();
    STEPPER_open();
	
    while (1) 
    {
		IR_AVOID();
    }
}


// ======================= functions ============================ //

/*
* Name		: IR_AVOID
* PURPOSE	: Avoids obstacles based on ATTINY->IR readings.
* RETURN	: Nothing
* NOTES		: Nothing
**/
void IR_AVOID( void )
{
	LCD_clear();

	if(ATTINY_get_IR_state(ATTINY_IR_BOTH))
	{
		//Debug
		//LCD_printf("Both sensors!");
		
		//Jump back
		//move_inch(BACK, 3);
		
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, 200, 300, 400, STEPPER_BRK_OFF,		//left
		STEPPER_REV, 200, 300, 375, STEPPER_BRK_OFF );	//right
		
		//Turn the f around
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, 260, 200, 200, STEPPER_BRK_OFF,	//left
		STEPPER_REV, 260, 200, 200, STEPPER_BRK_OFF );	//right
	
	}
	else if(ATTINY_get_IR_state(ATTINY_IR_LEFT) && !ATTINY_get_IR_state(ATTINY_IR_RIGHT))
	{
		//Debug
		//LCD_printf("Left sensor!");
		
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, 200, 300, 400, STEPPER_BRK_OFF,		//left
		STEPPER_REV, 200, 300, 375, STEPPER_BRK_OFF );	//right
		
		//turn the f left
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF,	//left
		STEPPER_FWD, 150, 200, 400, STEPPER_BRK_OFF );	//right
	}
	else if(ATTINY_get_IR_state(ATTINY_IR_RIGHT) && !ATTINY_get_IR_state(ATTINY_IR_LEFT))
	{
		//Debug
		//LCD_printf("Right sensor!");
		
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, 200, 300, 400, STEPPER_BRK_OFF,		//left
		STEPPER_REV, 200, 300, 375, STEPPER_BRK_OFF );	//right
		
		//turn the f right
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, 150, 200, 400, STEPPER_BRK_OFF,	//left
		STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF );	//right
	}
	else
	{
		LCD_printf("No sensors!");
		cruise();
	}
}

/*
* Name		: move_inch
* PURPOSE	: Move the ceenbot forward a number of inches passed as an unsigned integer.
* RETURN	: Nothing
* NOTES		: Nothing
**/
void move_inch( unsigned char fwdbck, unsigned int distance )
{
	//Figure out the number of steps needed to go that distance in inches
	unsigned int nstep = ( (distance)/(0.051) );
	
	//Delay so that robot does not start moving while still touching a button
	
	if(fwdbck == FORWARD)
	{
		LCD_clear();
		LCD_printf( "Moving %d inches", distance );
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, nstep, 300, 300, STEPPER_BRK_OFF,		//left
		STEPPER_FWD, nstep, 300, 275, STEPPER_BRK_OFF );	//right
		LCD_clear();
	}
	else if (fwdbck == BACK)
	{
		LCD_clear();
		LCD_printf( "Backin up %d inches", distance );
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_REV, nstep, 400, 300, STEPPER_BRK_OFF,		//left
		STEPPER_REV, nstep, 400, 275, STEPPER_BRK_OFF );	//right
		LCD_clear();
	}
	else
	{
		LCD_clear();
		LCD_printf( "That's not fwd/back" );
		TMRSRVC_delay( 500 );
		LCD_clear();
	}
}

/*
* Name		: Cruise
* PURPOSE	: Move the ceenbot forward, happily.
* RETURN	: Nothing
* NOTES		: Nothing
**/
void cruise ( void )
{
	STEPPER_move_stnb( STEPPER_BOTH,
	STEPPER_FWD, 5000, 200, 450, STEPPER_BRK_OFF, // Left
	STEPPER_FWD, 5000, 200, 450, STEPPER_BRK_OFF ); // Right	
}