/*
 * mgoberling_Lab2.c
 *
 * Created	: 1/17/2018 2:15:51 PM
 * Author	: Michael Goberling
 * Desc		: This program runs a series of routines decided by user input on the ceenbot. 
 * Course	: ECEN 3450 
 * Lab		: 2 
 */ 

#include "capi324v221.h"

#define LEFT 0
#define RIGHT 1
#define FORWARD 0 
#define REVERSE 1 
#define PART_ONE 0
#define PART_TWO_THREE_FOUR 1
#define PART_BONUS 2

// ======================= prototypes ============================ //
void part1( void );
void part2_3_4( void );
void set_LCD( unsigned short option );
void move_inch( unsigned char fwdbck, unsigned int distance );
void zero_point_turn( unsigned short direction );

// ======================= functions ============================ //
void CBOT_main( void )
{
	//Free up LCD, ATTINY, and STEPPER resources
	LCD_open();
	ATTINY_open();
	STEPPER_open();
	
	//Call function to perform Lab 2
	//part1();
	part2_3_4();
	
	//Infinite Loop
	while(1);	

}

/*
* Name		: part1
* PURPOSE	: Allows the user to execute part 1 of Lab 2 and select variable ranges of moving forward.
* RETURN	: Nothing
* NOTES		: SW3 = 6 inches
			  SW4 = 12 inches
			  SW5 = 18 inches/24 inches
**/
void part1( void )
{
	//Move in a straight line
	//With buttons as the distances 
	//S3 = 6 in
	//S4 = 12 in
	//S5 = 18 in/24 in 
	
	while(1)
	{	
		//Delay so Tiny is not overwhelmed
		TMRSRVC_delay( 100 );
		
		// Get state of all sensors
		unsigned char sensors = ATTINY_get_sensors();
		
		
		//Control structure for button selection
		if( sensors & SNSR_SW3_STATE )
		{
			//Move 6 inches forward
			move_inch( FORWARD, 6 );
		}
		else if( sensors & SNSR_SW4_STATE )
		{
			//Move 12 inches forward
			move_inch( FORWARD, 12 );
		}
		else if( sensors & SNSR_SW5_STATE )
		{
			//Move 18 inches forward
			move_inch(FORWARD, 18);
			
			for(int i = 0; i < 5; i++)
			{
				LCD_printf_RC(3, 0, "24 inches in %d", (5-i));
				TMRSRVC_delay( 1000 );
			}
			
			//Move 24 inches forward
			move_inch(FORWARD, 24);
		}
		else
		{	
			//Print part 1 menu
			set_LCD(PART_ONE);
		}
		
	}
	
}

/*
* Name		: part2_3_4
* PURPOSE	: Allows the user to execute part 2, 3, and 4 of Lab 2. 
* RETURN	: Nothing
* NOTES		: SW3 = Part 2
			  SW4 = Part 3
			  SW5 = Part 4	
**/
void part2_3_4( void )
{
	//S3 = start square
	
	while(1)
	{
		//Delay so Tiny is not overwhelmed
		TMRSRVC_delay( 100 );
		
		//Get state of all sensors
		unsigned char sensors = ATTINY_get_sensors();
		
		
		//Control structure for button selection
		if( sensors & SNSR_SW3_STATE )
		{
			LCD_clear();
			LCD_printf("Doing the square...");
			TMRSRVC_delay( 500 );
			
			//Move 4 ft forward
			move_inch(FORWARD, 48);
			
			//zero_point_turn 1
			zero_point_turn(RIGHT);
			
			//Move 4 ft forward
			move_inch(FORWARD, 48);
			
			//zero_point_turn 2
			zero_point_turn(RIGHT);
			
			//Move 4 ft forward
			move_inch(FORWARD, 48);
			
			//zero_point_turn 3
			zero_point_turn(RIGHT);
	
			//Move 4 ft forward
			move_inch(FORWARD, 48);
			
		}
		else if( sensors & SNSR_SW4_STATE )
		{
			LCD_clear();
			LCD_printf("Doing the maze...");
			TMRSRVC_delay( 500 );
			
			//Move 4 ft forward
			move_inch(FORWARD, 48);
			
			//zero_point_turn right 90
			zero_point_turn(RIGHT);
			
			//Move 5 ft forward
			move_inch(FORWARD, 60);
			
			//zero_point_turn right 90 
			zero_point_turn(RIGHT);
			
			//Move 2 ft forward
			move_inch(FORWARD, 24);
			
			//zero_point_turn left 90
			zero_point_turn(LEFT);
			
			//Move 2 ft forward
			move_inch(FORWARD, 24);
			
			//zero_point_turn left 90 
			zero_point_turn(LEFT);
			
			//Move 2 ft forward
			move_inch(FORWARD, 24);
			
			//zero_point_turn right 90
			zero_point_turn(RIGHT);
			
			//Move 2 ft forward
			move_inch(FORWARD, 24);
			
			//zero_point_turn right 90
			zero_point_turn(RIGHT);
			
			//Move 4 ft forward
			move_inch(FORWARD, 48);
			
			//Move right 90
			zero_point_turn(RIGHT);
			
			//Move 9 ft forward
			move_inch(FORWARD, 108);
				
		}
		else if( sensors & SNSR_SW5_STATE )
		{
			LCD_clear();
			LCD_printf( "Doing a donut..." );
			TMRSRVC_delay( 500 );
			
			STEPPER_move_stwt( STEPPER_BOTH, 
			STEPPER_FWD, 2000, 200, 400, STEPPER_BRK_OFF, 
			STEPPER_FWD, 1000, 100, 200, STEPPER_BRK_OFF);
			
			LCD_clear();
			
			for(int i = 0; i < 6; i++)
			{
				
				LCD_printf_RC(3, 0, "Bonus in %d", (6-i));
				TMRSRVC_delay( 1000 );
			}
			
			STEPPER_move_stwt( STEPPER_BOTH,
			STEPPER_FWD, 2000, 200, 400, STEPPER_BRK_OFF,
			STEPPER_FWD, 1000, 100, 200, STEPPER_BRK_OFF);
			
			STEPPER_move_stwt( STEPPER_BOTH,
			STEPPER_FWD, 1000, 100, 200, STEPPER_BRK_OFF,
			STEPPER_FWD, 2000, 200, 400, STEPPER_BRK_OFF);
		}
		else
		{
			//Set LCD menu options for Part 2, 3, and 4
			set_LCD( PART_TWO_THREE_FOUR );
		}
		
	}
	
}

/*
* Name		: set_LCD
* PURPOSE	: Sets the LCD with a certain number of options for the user to press. 
* RETURN	: Nothing
* NOTES		: Option 1 = Part 1
			  Option 2 = Part 2, 3, and 4	
**/
void set_LCD( unsigned short option )
{
	//Menu printing option for part 1
	if( option == 0 )
	{
		LCD_printf_RC( 3, 0, "Select a distance" );
		LCD_printf_RC( 2, 0, "S3: 6 inches" );
		LCD_printf_RC( 1, 0, "S4: 12 inches" );
		LCD_printf_RC( 0, 0, "S5: 18/24 inches" );
	}
	//Menu printing option for part 2, 3, and 4
	else if( option == 1 )
	{
		LCD_printf_RC( 3, 0, "Select an action" );
		LCD_printf_RC( 2, 0, "S3: 4x4 Square" );
		LCD_printf_RC( 1, 0, "S4: Maze Path" );
		LCD_printf_RC( 0, 0, "S5: Circle" );
	}
	//Menu printing option for an invalid menu option
	else
	{
		LCD_clear();
		LCD_printf( "Not a menu option" );
		TMRSRVC_delay( 500 );
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
	TMRSRVC_delay( 200 );
	
	if(fwdbck == FORWARD)
	{
		LCD_clear();
		LCD_printf( "Moving %d inches", distance );
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, nstep, 150, 300, STEPPER_BRK_OFF,		//left
		STEPPER_FWD, nstep, 150, 275, STEPPER_BRK_OFF );	//right
		LCD_clear();
	}
	else if (fwdbck == REVERSE)
	{
		LCD_clear();
		LCD_printf( "Backin up %d inches", distance );
		STEPPER_move_stwt( STEPPER_BOTH,
		STEPPER_FWD, nstep, 150, 300, STEPPER_BRK_OFF,		//left
		STEPPER_FWD, nstep, 150, 275, STEPPER_BRK_OFF );	//right
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
* Name		: zero_point_turn
* PURPOSE	: zero_point_turn the ceenbot in a certain direction at a certain angle
* RETURN	: Nothing
* NOTES		: 0 - Left
			  1 - Right
**/
void zero_point_turn( unsigned short direction )
{
	//Figure out number of steps needed for turn radius 
	
	if( direction == LEFT )
	{
		STEPPER_move_stwt( STEPPER_BOTH, 
				STEPPER_REV, 150, 200, 200, STEPPER_BRK_OFF,
				STEPPER_FWD, 150, 200, 200, STEPPER_BRK_OFF );	
	}
	else if (direction == RIGHT )
	{
		STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_FWD, 150, 200, 200, STEPPER_BRK_OFF,
				STEPPER_REV, 150, 200, 200, STEPPER_BRK_OFF );
	}
	else
	{
		LCD_clear();
		LCD_printf("That's not right or left.");
		TMRSRVC_delay( 500 );
	}
}