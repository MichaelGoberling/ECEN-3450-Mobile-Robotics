// Behavior-Based Control Skeleton code.
//
// Desc: Provides a C program structure that emulates multi-tasking and
//       modularity for Behavior-based control with easy scalability.
//
// Supplied for: Students of Mobile Robotics I, Fall 2013.
// University of Nebraska-Lincoln Dept. of Computer & Electronics Engineering
// Alisa N. Gilmore, P.E., Instructor, Course Developer.  Jose Santos, T.A.
// Version 1.3  Updated 10/11/2011.
// Version 1.4  Updated 12/2/2013
//
//      - Updated __MOTOR_ACTION() macro-function to invoke new functions
//        added to the API: `STEPPER_set_accel2()' and `STEPPER_run2()'.
//        In particular `STEPPER_run2()' now takes positive or negative
//        speed values to imply the direction of each wheel.
//
// Version 1.5  Updated 2/25/2015
//
//      - Fixed an error in __MOTOR_ACTION() and __RESET_ACTION() macros
//        where there was an extra curly brace ({) that should have been
//        removed.
//
// Version 1.6  Updated 2/24/2016
//
//      - In the 'IR_sense()' function, we now make use of the 'TIMER_ALARM()'
//        and 'TIMER_SNOOZE()' macros that were introduced in API version 2.x,
//        which makes usage of the timer object clear.   Before, students
//        manipulated the 'tc' flag inside the timer object directly, but this
//        always caused confusion, the 'TIMER_ALARM()' and 'TIMER_SNOOZE()'
//        macros achieve the same thing, but transparently.
//
//      - Also fixed __MOTOR_ACTION() macro, which previously invoked
//        'STEPPER_run2()', but the API has been modified so that this same
//        effect is now achieved with the function 'STEPPER_runn()', which
//        means to run the stepper motors and allow negative values to mean
//        reverse motion (that's what the second 'n' in 'runn' stands for).
//        This might change again in the future, because for consistency in the
//        API, it should have been called 'STEPPER_runn2()' since it takes two
//        parameters and not just one.
//

// Authors: Goberling & Knudtson
// Course: ECEN 3450-001
// Date: 3/5/18
// Lab: 5

#include "capi324v221.h"

// ---------------------- Defines:

#define DEG_90  123    /* Number of steps for a 90-degree (in place) turn. */


// Desc: This macro-function can be used to reset a motor-action structure
//       easily.  It is a helper macro-function.
#define __RESET_ACTION( motor_action )    \
do {                                  \
	( motor_action ).speed_L = 0;         \
	( motor_action ).speed_R = 0;         \
	( motor_action ).accel_L = 0;         \
	( motor_action ).accel_R = 0;         \
	( motor_action ).state = STARTUP;     \
	} while( 0 ) /* end __RESET_ACTION() */

	// Desc: This macro-fuction translates action to motion -- it is a helper
	//       macro-function.
	#define __MOTOR_ACTION( motor_action )   \
	do {                                 \
		STEPPER_set_accel2( ( motor_action ).accel_L, ( motor_action ).accel_R ); \
		STEPPER_runn( ( motor_action ).speed_L, ( motor_action ).speed_R ); \
		} while( 0 ) /* end __MOTOR_ACTION() */

		// Desc: This macro-function is used to set the action, in a more natural
		//       manner (as if it was a function).

		// ---------------------- Type Declarations:

		// Desc: The following custom enumerated type can be used to specify the
		//       current state of the robot.  This parameter can be expanded upon
		//       as complexity grows without intefering with the 'act()' function.
		//		 It is a new type which can take the values of 0, 1, or 2 using
		//		 the SYMBOLIC representations of STARTUP, EXPLORING, etc.
		typedef enum ROBOT_STATE_TYPE {

			STARTUP = 0,    // 'Startup' state -- initial state upon RESET.
			EXPLORING,      // 'Exploring' state -- the robot is 'roaming around'.
			FOLLOWING,
			WALL_FOLLOW, 
			LINE_FOLLOW, 
			LIGHT_OBSERVED,
			SONAR_AVOIDING,
			AVOIDING        // 'Avoiding' state -- the robot is avoiding a collision.

		} ROBOT_STATE;

		// Desc: Structure encapsulates a 'motor' action. It contains parameters that
		//       controls the motors 'down the line' with information depicting the
		//       current state of the robot.  The 'state' variable is useful to
		//       'print' information on the LCD based on the current 'state', for
		//       example.
		typedef struct MOTOR_ACTION_TYPE {

			ROBOT_STATE state;              // Holds the current STATE of the robot.
			signed short int speed_L;       // SPEED for LEFT  motor.
			signed short int speed_R;       // SPEED for RIGHT motor.
			unsigned short int accel_L;     // ACCELERATION for LEFT  motor.
			unsigned short int accel_R;     // ACCELERATION for RIGHT motor.
			
		} MOTOR_ACTION;

		// Desc: Structure encapsulates 'sensed' data.  Right now that only consists
		//       of the state of the left & right IR sensors when queried.  You can
		//       expand this structure and add additional custom fields as needed.
		typedef struct SENSOR_DATA_TYPE {

			BOOL left_IR;       // Holds the state of the left IR.
			BOOL right_IR;      // Holds the state of the right IR.
			BOOL IR_flag;		// Holds last IR state
			// 0 = right
			// 1 = left
			
			float light_L;
			float light_R;
			
			unsigned long int usonic_time_us;	//Holds 'trip" time in micro-seconds
			SWTIME usonic_time_ticks;			//Holds 'ticks' from STOPWATCH
			float distance_cm;					//Holds the distance-to-target in 'cm'

			signed long int err;
			signed long int last_err;
			signed long int derivative;
			
			float line_left;
			float line_right; 
			signed long int err_right;
			signed long int err_left;
			signed long int last_err_right;
			signed long int last_err_left;
			signed long int derivative_right;			
			signed long int derivative_left;	

		} SENSOR_DATA;

		// ---------------------- Globals:
		volatile MOTOR_ACTION action;  	// This variable holds parameters that determine
		// the current action that is taking place.
		// Here, a structure named "action" of type
		// MOTOR_ACTION is declared.

		// ---------------------- Prototypes:
		void IR_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms );
		void photo_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms );
		void SONAR_SENSE( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms );
		void Line_Sense(volatile SENSOR_DATA *pSensors, TIMER16 interval_ms );
		void explore( volatile MOTOR_ACTION *pAction );
		void light_follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void SONAR_AVOID( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void Wall_Follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void Line_Follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void IR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void light_observe( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void act( volatile MOTOR_ACTION *pAction );
		void info_display( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		BOOL compare_actions( volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b );

		// ---------------------- Convenience Functions:
		void info_display( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{

			// NOTE:  We keep track of the 'previous' state to prevent the LCD
			//        display from being needlessly written, if there's  nothing
			//        new to display.  Otherwise, the screen will 'flicker' from
			//        too many writes.
			static ROBOT_STATE previous_state = STARTUP;

			if ( ( pAction->state != previous_state ) || ( pAction->state == STARTUP ) )
			{

				LCD_clear();

				//  Display information based on the current 'ROBOT STATE'.
				switch( pAction->state )
				{

					case STARTUP:

					// Fill me in.

					break;

					case EXPLORING:

					LCD_printf_RC(3, 0, "Exploring...\n" );

					break;

					case AVOIDING:

					LCD_printf( "IR Avoiding!\n");

					break;
					
					case SONAR_AVOIDING:
					
					LCD_printf("Sonar Avoiding!\n");
					
					break;
					
					case WALL_FOLLOW:
					LCD_printf("Wall following!\n");

					break;
					
					case LINE_FOLLOW: 
					
					LCD_printf("Line Following!\n");
					
					break;
					
					case FOLLOWING:
					LCD_printf("Following!\n");

					break;
					
					case LIGHT_OBSERVED:
					
					LCD_printf("Light found.");
					
					default:

					LCD_printf( "Unknown state!\n" );

				} // end switch()

				// Note the new state in effect.
				previous_state = pAction->state;

			} // end if()

		} // end info_display()

		// ----------------------------------------------------- //
		BOOL compare_actions( volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b )
		{

			// NOTE:  The 'sole' purpose of this function is to
			//        compare the 'elements' of MOTOR_ACTION structures
			//        'a' and 'b' and see if 'any' differ.

			// Assume these actions are equal.
			BOOL rval = TRUE;

			if ( ( a->state   != b->state )   ||
			( a->speed_L != b->speed_L ) ||
			( a->speed_R != b->speed_R ) ||
			( a->accel_L != b->accel_L ) ||
			( a->accel_R != b->accel_R ) )

			rval = FALSE;

			// Return comparison result.
			return rval;

		} // end compare_actions()

		// ---------------------- Top-Level Behaviorals:
		void IR_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{

			// Sense must know if it's already sensing.
			//
			// NOTE: 'BOOL' is a custom data type offered by the CEENBoT API.
			//
			static BOOL timer_started = FALSE;
			
			// The 'sense' timer is used to control how often gathering sensor
			// data takes place.  The pace at which this happens needs to be
			// controlled.  So we're forced to use TIMER OBJECTS along with the
			// TIMER SERVICE.  It must be 'static' because the timer object must remain
			// 'alive' even when it is out of scope -- otherwise the program will crash.
			static TIMEROBJ sense_timer;
			
			// If this is the FIRST time that sense() is running, we need to start the
			// sense timer.  We do this ONLY ONCE!
			if ( timer_started == FALSE )
			{
				
				// Start the 'sense timer' to tick on every 'interval_ms'.
				//
				// NOTE:  You can adjust the delay value to suit your needs.
				//
				TMRSRVC_new( &sense_timer, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART,
				interval_ms );
				
				// Mark that the timer has already been started.
				timer_started = TRUE;
				
			} // end if()
			
			// Otherwise, just do the usual thing and just 'sense'.
			else
			{

				// Only read the sensors when it is time to do so (e.g., every
				// 125ms).  Otherwise, do nothing.
				if ( TIMER_ALARM( sense_timer ) )
				{

					// NOTE: Just as a 'debugging' feature, let's also toggle the green LED
					//       to know that this is working for sure.  The LED will only
					//       toggle when 'it's time'.
					LED_toggle( LED_Green );


					// Read the left and right sensors, and store this
					// data in the 'SENSOR_DATA' structure.
					pSensors->left_IR  = ATTINY_get_IR_state( ATTINY_IR_LEFT  );
					pSensors->right_IR = ATTINY_get_IR_state( ATTINY_IR_RIGHT );

					// NOTE: You can add more stuff to 'sense' here.
					
					// Snooze the alarm so it can trigger again.
					TIMER_SNOOZE( sense_timer );
					
				} // end if()

			} // end else.

		} // end sense()
		
		void photo_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{
			// Sense must know if it's already sensing.
			//
			// NOTE: 'BOOL' is a custom data type offered by the CEENBoT API.
			//
			static BOOL timer_started = FALSE;
			ADC_SAMPLE sample;
			
			// The 'sense' timer is used to control how often gathering sensor
			// data takes place.  The pace at which this happens needs to be
			// controlled.  So we're forced to use TIMER OBJECTS along with the
			// TIMER SERVICE.  It must be 'static' because the timer object must remain
			// 'alive' even when it is out of scope -- otherwise the program will crash.
			static TIMEROBJ sense_timer_2;
			
			// If this is the FIRST time that sense() is running, we need to start the
			// sense timer.  We do this ONLY ONCE!
			if ( timer_started == FALSE )
			{
				
				// Start the 'sense timer' to tick on every 'interval_ms'.
				//
				// NOTE:  You can adjust the delay value to suit your needs.
				//
				TMRSRVC_new( &sense_timer_2, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART,
				interval_ms );
				
				// Mark that the timer has already been started.
				timer_started = TRUE;
				
			} // end if()
			
			// Otherwise, just do the usual thing and just 'sense'.
			else
			{

				// Only read the sensors when it is time to do so (e.g., every
				// 125ms).  Otherwise, do nothing.
				if ( TIMER_ALARM( sense_timer_2 ) )
				{

					// NOTE: Just as a 'debugging' feature, let's also toggle the green LED
					//       to know that this is working for sure.  The LED will only
					//       toggle when 'it's time'.
					LED_toggle( LED_Green );
					
					// Read the left and right sensors, and store this
					// data in the 'SENSOR_DATA' structure.
					
					
					ADC_set_channel( ADC_CHAN4 );
					sample = ADC_sample();
					pSensors->light_R = ((sample * 5.0f) / 1024);
					
					TMRSRVC_delay(50);
					
					ADC_set_channel( ADC_CHAN5 );
					sample = ADC_sample();
					pSensors->light_L = ((sample * 5.0f) / 1024);
					
					// NOTE: You can add more stuff to 'sense' here.
					// Snooze the alarm so it can trigger again.
					TIMER_SNOOZE( sense_timer_2 );
					
				} // end if()

			} // end else.
		}
		// -------------------------------------------- //
		void explore( volatile MOTOR_ACTION *pAction )
		{
			
			// Nothing to do, but set the parameters to explore.  'act()' will do
			// the rest down the line.
			pAction->state = EXPLORING;
			pAction->speed_L = 200;
			pAction->speed_R = 200;
			pAction->accel_L = 400;
			pAction->accel_R = 400;
			
			// That's it -- let 'act()' do the rest.
			
		} // end explore()
		// -------------------------------------------- //
		
		void light_follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{
			float thisLight_R = pSensors->light_R;
			float thisLight_L = pSensors->light_L;
			
			//LCD_printf_RC(2, 0, "left: %.02f\n", thisLight_L );
			//LCD_printf_RC(1, 0, "right: %.02f\n", thisLight_R );
			
			if((pSensors->light_L + pSensors->light_R)/2 > 1)
			{
				pAction->state = FOLLOWING;
			}
			
			if(pAction->state == FOLLOWING)
			{
				
				pAction->speed_L = (200 - ((thisLight_L - thisLight_R)*100));
				pAction->speed_R = (200 + ((thisLight_L - thisLight_R)*100));

			}
		}
		
		void light_observe( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{
			LCD_printf_RC(2, 0, "left: %.02f\n", pSensors->light_L);
			LCD_printf_RC(1, 0, "right: %.02f\n", pSensors->light_R);
			
			if( (pSensors->light_L + pSensors->light_R)/2 > 4.3)
			{
				pAction->state = LIGHT_OBSERVED;
			}
			
			if( pAction->state == LIGHT_OBSERVED )
			{
				pAction->speed_L = 0;
				pAction->speed_R = 0;
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_ON );
				TMRSRVC_delay( 5000 );
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, 200, 300, 400, STEPPER_BRK_OFF,		//left
				STEPPER_REV, 200, 300, 375, STEPPER_BRK_OFF );	//right
				
				//Turn the f around
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_FWD, 260, 200, 200, STEPPER_BRK_OFF,	//left
				STEPPER_REV, 260, 200, 200, STEPPER_BRK_OFF );	//right
				pAction->state = EXPLORING;
			}
		}
		
		void SONAR_SENSE( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{
			// Sense must know if it's already sensing.
			//
			// NOTE: 'BOOL' is a custom data type offered by the CEENBoT API.
			//
			static BOOL timer_started = FALSE;
			
			// The 'sense' timer is used to control how often gathering sensor
			// data takes place.  The pace at which this happens needs to be
			// controlled.  So we're forced to use TIMER OBJECTS along with the
			// TIMER SERVICE.  It must be 'static' because the timer object must remain
			// 'alive' even when it is out of scope -- otherwise the program will crash.
			static TIMEROBJ sense_timer_3;
			
			// If this is the FIRST time that sense() is running, we need to start the
			// sense timer.  We do this ONLY ONCE!
			if ( timer_started == FALSE )
			{
				
				// Start the 'sense timer' to tick on every 'interval_ms'.
				//
				// NOTE:  You can adjust the delay value to suit your needs.
				//
				TMRSRVC_new( &sense_timer_3, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART,
				interval_ms );
				
				// Mark that the timer has already been started.
				timer_started = TRUE;
				
			} // end if()
			
			// Otherwise, just do the usual thing and just 'sense'.
			else
			{

				// Only read the sensors when it is time to do so (e.g., every
				// 125ms).  Otherwise, do nothing.
				if ( TIMER_ALARM( sense_timer_3) )
				{

					// NOTE: Just as a 'debugging' feature, let's also toggle the green LED
					//       to know that this is working for sure.  The LED will only
					//       toggle when 'it's time'.
					LED_toggle( LED_Green );
					
					pSensors->usonic_time_ticks = USONIC_ping();
					pSensors->usonic_time_us = 10 * ((unsigned long int)( pSensors->usonic_time_ticks ));
					pSensors->distance_cm = 0.01724 * pSensors->usonic_time_us;
					LCD_printf_RC(0,0, "Dist = %.3f\n", pSensors->distance_cm);
					
					// NOTE: You can add more stuff to 'sense' here.
					// Snooze the alarm so it can trigger again.
					TIMER_SNOOZE( sense_timer_3 );
					
				} // end if()

			} // end else.
		}
		
		void Wall_Follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{
			pSensors->err = (pSensors->distance_cm - 27.75 );
			pSensors->derivative = pSensors->err - pSensors->last_err;

			pAction->speed_L = ( 150 + (4*pSensors->err) + (69*pSensors->derivative ));
			pAction->speed_R = ( 150 - (4*pSensors->err) - (69*pSensors->derivative ));
			
			LCD_printf_RC( 3, 0, "Error: %d", pAction->speed_L );
			LCD_printf_RC( 2, 0, "Error: %d", pAction->speed_R );
			LCD_printf_RC( 1, 0, "Error: %ld", pSensors->err );
			
			// Store this sensor reading as the last value for the
			// next time the program comes back to the function
			pSensors->last_err = pSensors->err;
		}
		
		void Line_Sense(volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{
			// Sense must know if it's already sensing.
			//
			// NOTE: 'BOOL' is a custom data type offered by the CEENBoT API.
			//
			static BOOL timer_started = FALSE;
			ADC_SAMPLE sample;
			
			// The 'sense' timer is used to control how often gathering sensor
			// data takes place.  The pace at which this happens needs to be
			// controlled.  So we're forced to use TIMER OBJECTS along with the
			// TIMER SERVICE.  It must be 'static' because the timer object must remain
			// 'alive' even when it is out of scope -- otherwise the program will crash.
			static TIMEROBJ sense_timer_4;
			
			// If this is the FIRST time that sense() is running, we need to start the
			// sense timer.  We do this ONLY ONCE!
			if ( timer_started == FALSE )
			{
				
				// Start the 'sense timer' to tick on every 'interval_ms'.
				//
				// NOTE:  You can adjust the delay value to suit your needs.
				//
				TMRSRVC_new( &sense_timer_4, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART,
				interval_ms );
				
				// Mark that the timer has already been started.
				timer_started = TRUE;
				
			} // end if()
			
			// Otherwise, just do the usual thing and just 'sense'.
			else
			{

				// Only read the sensors when it is time to do so (e.g., every
				// 125ms).  Otherwise, do nothing.
				if ( TIMER_ALARM( sense_timer_4 ) )
				{

					// NOTE: Just as a 'debugging' feature, let's also toggle the green LED
					//       to know that this is working for sure.  The LED will only
					//       toggle when 'it's time'.
					LED_toggle( LED_Green );
					
					// Read the left and right sensors, and store this
					// data in the 'SENSOR_DATA' structure.
					
					
					// J3 pin 4
					ADC_set_channel( ADC_CHAN4 );
					sample = ADC_sample();
					pSensors->line_right = ((sample * 5.0f) / 1024);
					
					TMRSRVC_delay(50);
					
					// J3 pin 5
					ADC_set_channel( ADC_CHAN5 );
					sample = ADC_sample();
					pSensors->line_left = ((sample * 5.0f) / 1024);
					
					LCD_printf_RC(2, 0, "left: %.02f\n", line_left );
					LCD_printf_RC(1, 0, "right: %.02f\n", line_right );
					
					TIMER_SNOOZE( sense_timer_4 );
					
				} // end if()

			} // end else.
		}
		
		void Line_Follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{
			//stuff
			pAction->state = LINE_FOLLOW;
			
			LCD_printf_RC(2, 0, "left: %.02f\n", pSensors->line_left );
			LCD_printf_RC(1, 0, "right: %.02f\n", pSensors->line_right );
			
			float this_line_right = pSensors->line_right;
			float this_line_left = pSensors->line_left; 
			
			//Assuming target value is 4.5
			
			pSensors->err_right = ( this_line_right - 0.1 ); 
			pSensors->err_left = ( this_line_left - 0.1 );
			
			pSensors->derivative_right = pSensors->err_right - pSensors->last_err_right;
			pSensors->derivative_left = pSensors->err_left - pSensors->last_err_left;

			pAction->speed_R = ( 25 + (50*pSensors->err_right) + (20*pSensors->derivative_right));
			pAction->speed_L = ( 25 + (50*pSensors->err_left) + (20*pSensors->derivative_left));
				
			//pSensors->derivative_right = pSensors->err_right - pSensors->last_err_right;
			//pSensors->derivative_left = pSensors->err_left - pSensors->last_err_left;

			//pAction->speed_L = ( 150 + (4*pSensors->err_right) + (69*pSensors->derivative_right ));
			//pAction->speed_R = ( 150 - (4*pSensors->err_left) - (69*pSensors->derivative_left ));

			// Store this sensor reading as the last value for the
			// next time the program comes back to the function
			pSensors->last_err_right = pSensors->err_right;
			pSensors->last_err_left = pSensors->err_left;	
		}
	
		void SONAR_AVOID( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{
			if(pSensors->distance_cm < 75)
			{
				pAction->state = SONAR_AVOIDING;
			}
			
			if(pAction->state == SONAR_AVOIDING)
			{
				if(!pSensors->IR_flag)
				{
					pAction->speed_L = 200 + (5000/pSensors->distance_cm);
				}
				else if(pSensors->IR_flag)
				{
					pAction->speed_R = 200 + (5000/pSensors->distance_cm);
				}
			}
		}
		
		void IR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{

			// NOTE: Here we have NO CHOICE, but to do this 'ballistically'.
			//       **NOTHING** else can happen while we're 'avoiding'.
			
			// Example of ONE case (you can expand on this idea):
			
			// If the LEFT sensor tripped...
			
			if( (pSensors->right_IR == TRUE) || (pSensors->left_IR == TRUE) )
			{
				// Note that we're avoiding...
				pAction->state = AVOIDING;
				pSensors->IR_flag = TRUE;
				
				// STOP!
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				// Back up...
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF );
				
	
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, DEG_90, 200, 400, STEPPER_BRK_OFF,
				STEPPER_FWD, DEG_90, 200, 400, STEPPER_BRK_OFF );

				// ... and set the motor action structure with variables to move forward.
				
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
			}
			
			/*
			if( (pSensors->right_IR == TRUE) && (pSensors->left_IR == TRUE) )
			{
				pAction->state = AVOIDING;
				// STOP!
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, 200, 300, 400, STEPPER_BRK_OFF,		//left
				STEPPER_REV, 200, 300, 380, STEPPER_BRK_OFF );	//right
				
				//Turn the f around
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_FWD, 260, 200, 200, STEPPER_BRK_OFF,	//left
				STEPPER_REV, 260, 200, 200, STEPPER_BRK_OFF );	//right
				
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
			}
			else if( pSensors->left_IR == TRUE )
			{
				// Note that we're avoiding...
				pAction->state = AVOIDING;
				pSensors->IR_flag = TRUE;
				
				// STOP!
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				// Back up...
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF );
				
				// ... and turn RIGHT ~90-deg.
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_FWD, DEG_90, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, DEG_90, 200, 400, STEPPER_BRK_OFF );
				// ... and set the motor action structure with variables to move forward.
				
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
				
			}
			else if ( pSensors->right_IR == TRUE )
			{
				// Note that we're avoiding...
				pAction->state = AVOIDING;
				pSensors->IR_flag = FALSE;
				// STOP!
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				// Back up...
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF,
				STEPPER_REV, 150, 200, 400, STEPPER_BRK_OFF );
				
				// ... and turn RIGHT ~90-deg.
				STEPPER_move_stwt( STEPPER_BOTH,
				STEPPER_REV, DEG_90, 200, 400, STEPPER_BRK_OFF,
				STEPPER_FWD, DEG_90, 200, 400, STEPPER_BRK_OFF );
				// ... and set the motor action structure with variables to move forward.
				pAction->state = AVOIDING;
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
				
			}
			*/
			
		} // end avoid()
		// -------------------------------------------- //
		void act( volatile MOTOR_ACTION *pAction )
		{

			// 'act()' always keeps track of the PREVIOUS action to determine
			// if a new action must be executed, and to execute such action ONLY
			// if any parameters in the 'MOTOR_ACTION' structure have changed.
			// This is necessary to prevent motor 'jitter'.
			static MOTOR_ACTION previous_action = {

				STARTUP, 0, 0, 0, 0

			};

			if( compare_actions( pAction, &previous_action ) == FALSE )
			{

				// Perform the action.  Just call the 'free-running' version
				// of stepper move function and feed these same parameters.
				__MOTOR_ACTION( *pAction );

				// Save the previous action.
				previous_action = *pAction;

			} // end if()
			
		} // end act()
		// ---------------------- CBOT Main:
		void CBOT_main( void )
		{

			volatile SENSOR_DATA sensor_data;
			
			ADC_open();
			ADC_set_VREF( ADC_VREF_AVCC );
			
			// ** Open the needed modules.
			LED_open();     // Open the LED subsystem module.
			LCD_open();     // Open the LCD subsystem module.
			ATTINY_open();
			USONIC_open();
			STEPPER_open(); // Open the STEPPER subsystem module
			
			ATTINY_set_RC_servo( RC_SERVO0, 510 );
			// Reset the current motor action.
			__RESET_ACTION( action );
			
			// Nofify program is about to start.
			LCD_printf( "Starting...\n" );
			
			// Wait 3 seconds or so.
			TMRSRVC_delay( TMR_SECS( 3 ) );
			
			// Clear the screen and enter the arbitration loop.
			LCD_clear();
			
			// Enter the 'arbitration' while() loop -- it is important that NONE
			// of the behavior functions listed in the arbitration loop BLOCK!
			// Behaviors are listed in increasing order of priority, with the last
			// behavior having the greatest priority (because it has the last 'say'
			// regarding motor action (or any action)).
			while( 1 )
			{
				
				// Sense must always happen first.
				// (IR sense happens every 125ms).
				//IR_sense( &sensor_data, 125 );
				
				//SONAR_SENSE( &sensor_data, 200 );
				
				//photo_sense( &sensor_data, 75 );
				
				Line_Sense( &sensor_data, 50 );
				// Behaviors.
				//explore( &action );
				
				//light_follow( &action, &sensor_data );
				
				//light_observe( &action, &sensor_data );
				
				//Wall_Follow( &action, &sensor_data );
				
				Line_Follow( &action, &sensor_data );
				
				//SONAR_AVOID( &action, &sensor_data );
				
				// Note that 'avoidance' relies on sensor data to determine
				// whether or not 'avoidance' is necessary.
				//IR_avoid( &action, &sensor_data );
				
				// Perform the action of highest priority.
				act( &action );

				// Real-time display info, should happen last, if possible (
				// except for 'ballistic' behaviors).  Technically this is sort of
				// 'optional' as it does not constitute a 'behavior'.
				info_display( &action, &sensor_data );
				
			} // end while()
			
		} // end CBOT_main()
<<<<<<< HEAD
=======





>>>>>>> 97c7015774b9c82ab00a605e6c1c5b4de8b1e05d
