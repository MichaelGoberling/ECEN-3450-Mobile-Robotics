/*
 * Lab4.c
 *
 * Created: 1/17/2018 2:15:51 PM
 * Author : Michael Goberling
 * Course: ECEN 3450-001
 * Lab : 4
 * Demonstration #: 8 
 */ 
//====================== Defines, Includes, & Variables ===

#include "capi324v221.h"
#define cruise_speed 150

//====================== Prototypes =======================
void v2a( float v_left, float v_right );
void v2b( float v_left, float v_right );
void v3a( float v_left, float v_right);
void v3b( float v_left, float v_right );
void orbit( float v_left, float v_right );

//====================== Main =============================
void CBOT_main( void )
{
	LCD_open();
	ADC_open();
	STEPPER_open();
	ATTINY_open();
	
	ADC_set_VREF( ADC_VREF_AVCC );		ADC_SAMPLE sample;
	float v_left;
	float v_right;	
    /* Replace with your application code */
    while (1) 
    {
		TMRSRVC_delay(125);
		LCD_clear();
		
		ADC_set_channel( ADC_CHAN3 );
		sample = ADC_sample();
		v_left = ((sample * 5.0f) / 1024);
		
		ADC_set_channel( ADC_CHAN4 );
		sample = ADC_sample();
		v_right = ((sample * 5.0f) / 1024);
		
		LCD_printf("left: %.02f\n", v_left);
		LCD_printf("right: %.02f\n", v_right);
		
		//v2a(v_left, v_right);
		//v2b(v_left, v_right);
		//v3a(v_left, v_right);
		v3b(v_left, v_right);
		//orbit(v_left, v_right);
		
    }
	
}

void v2a( float v_left, float v_right )
{
	unsigned int speed_left = cruise_speed + 1.5 * ((cruise_speed) * (v_left/6.0));
	unsigned int speed_right = cruise_speed + 1.5 * ((cruise_speed) * (v_right/6.0));
	
	STEPPER_run2(	STEPPER_FWD, speed_left,
					STEPPER_FWD, speed_right);
	
}
	
void v2b( float v_left, float v_right )
{
	unsigned int speed_right = cruise_speed + 1.5 * ((cruise_speed) * (v_left/6.0));
	unsigned int speed_left = cruise_speed + 1.5 * ((cruise_speed) * (v_right/6.0));
	
	STEPPER_run2(	STEPPER_FWD, speed_left,
					STEPPER_FWD, speed_right);
	
}

void v3a( float v_left, float v_right )
{
	unsigned int speed_left = cruise_speed - ((cruise_speed) * (v_left/6.0));
	unsigned int speed_right = cruise_speed - ((cruise_speed) * (v_right/6.0));
	
	STEPPER_run2(	STEPPER_FWD, speed_left,
					STEPPER_FWD, speed_right);
	
}

void v3b( float v_left, float v_right )
{
	unsigned int speed_right = cruise_speed - ((cruise_speed) * (v_left/6.0));
	unsigned int speed_left = cruise_speed - ((cruise_speed) * (v_right/6.0));
	
	STEPPER_run2(	STEPPER_FWD, speed_left,
					STEPPER_FWD, speed_right);
	
}

void orbit( float v_left, float v_right )
{
	if(v_left > 2.00 || v_right > 2.00)
	{
		STEPPER_run2(	STEPPER_FWD, cruise_speed + ((cruise_speed) * (v_left/5.0)),
						STEPPER_FWD, cruise_speed + ((cruise_speed) * (v_right/5.0)));	
	}
	
}