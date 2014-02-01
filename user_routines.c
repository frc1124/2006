/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include <stdio.h>
#include <math.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "tracking.h"
#include "terminal.h"
#include "custom_vars.h"
#include "adc.h"
#include "gyro.h"

extern unsigned char aBreakerWasTripped;
//extern unsigned float bomb = '';
//extern string andy_ligotti = "AHAHAHA|AHAHAHA|KABOOM|cheese";

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
//  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */
 
  Init_Serial_Port_One();
  Init_Serial_Port_Two();

#ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
#endif

#ifdef TERMINAL_SERIAL_PORT_2    
  stdout_serial_port = SERIAL_PORT_TWO;
#endif

  Initialize_Gyro();
  Initialize_ADC();

  Putdata(&txdata);            /* DO NOT CHANGE! */

//  ***  IFI Code Starts Here***
//
//  Serial_Driver_Initialize();
//
//  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)

{
	Getdata(&rxdata);

	// send diagnostic information to the terminal
//	Tracking_Info_Terminal();

	// This function is responsable for camera initialization 
	// and camera serial data interpretation. Once the camera
	// is initialized and starts sending tracking data, this 
	// function will continuously update the global T_Packet_Data 
	// structure with the received tracking information.
	Camera_Handler();

	// This function reads data placed in the T_Packet_Data
	// structure by the Camera_Handler() function and if new
	// tracking data is available, attempts to keep the center
	// of the tracked object in the center of the camera's
	// image using two servos that drive a pan/tilt platform.
	// If the camera doesn't have the object within it's field 
	// of view, this function will execute a search algorithm 
	// in an attempt to find the object.
	//if (p4_sw_top > 0) {  //only track and waste battery if the switch is toggled.
		Servo_Track();
	//}

	Putdata(&txdata);       //DO NOT CHANGE! CHANGING IS A VIOLATION OF LAW 127.02.255 AND YOU WILL GET YOUR ASS KICKED SO HARD THAT YOU WONT BE ABLE TO FEEL IT FOR WEEKS
{
	static unsigned int i = 0;
	static unsigned int j = 0;
	//int temp_gyro_rate;
	long temp_gyro_angle;
	//int temp_gyro_bias;


	Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

	i++;
	j++; // this will rollover every ~1000 seconds

	if(j == 1)
	{
		printf("\rCalculating Gyro Bias...");
	}

	if(j == 6)
	{
		// start a gyro bias calculation
		Start_Gyro_Bias_Calc();
	}

	if(j == 200)
	{
		// terminate the gyro bias calculation
		Stop_Gyro_Bias_Calc();

		// reset the gyro heading angle
		Reset_Gyro_Angle();

		printf("Done\r");
	}


	if(i >= 30 && j >= 200)
	{
		//temp_gyro_rate = Get_Gyro_Rate();
		temp_gyro_angle = Get_Gyro_Angle();
		//printf(" Gyro Rate=%d\r\n", temp_gyro_rate);
		//printf("Gyro Angle=%d\r\n\r\n", (int)temp_gyro_angle);

		i = 0;
	}
}
//  ***  IFI Code Starts Here***
//
//  static unsigned char i;
//
//  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */
//
  Default_Routine();  /* Optional.  See below. */
//
//  /* Add your own code here. (a printf will not be displayed when connected to the breaker panel unless a Y cable is used) */
//
//  printf("Port1 Y %3d, X %3d, Fire %d, Top %d\r",p1_y,p1_x,p1_sw_trig,p1_sw_top);  /* printf EXAMPLE */
//
//  Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
//
//  /* Eample code to check if a breaker was ever tripped. */
//
//  if (aBreakerWasTripped)
//  {
//    for (i=1;i<29;i++)
//    {
//      if (Breaker_Tripped(i))
//        User_Byte1 = i;  /* Update the last breaker tripped on User_Byte1 (to demonstrate the use of a user byte) 
//                            Normally, you do something else if a breaker got tripped (ex: limit a PWM output)     */
//    }
//  }
//
//  Putdata(&txdata);             /* DO NOT CHANGE! */
}

//function FLIP_AXIS
unsigned char flip_axis(unsigned char valueToFlip, unsigned char axisFlipOver) {
	int returnValue;
	returnValue = (-1)*(valueToFlip - axisFlipOver);
	returnValue += axisFlipOver;
	return returnValue;
}

unsigned char amp_pwm(unsigned char valueToAccel, unsigned char factor) {
	int returnValue;
	returnValue = ((factor*(valueToAccel-127)))+127;
	if (returnValue > 255) {
		returnValue = 255;
	}else if (returnValue < 0) {
		returnValue = 0;
	}
	return returnValue;
}

unsigned char damp_pwm (unsigned char valueToDampen, unsigned char factor) {
	int returnValue;
	returnValue = ((valueToDampen-127)/factor)+127;
	return returnValue;
}

unsigned char set_pwm_deadband(unsigned char valueToSet, unsigned char deadband, unsigned char maxLimit, unsigned char minLimit) {
	int returnValue;
	//rev @ cokmpetetion: values spread by |13|
	if (valueToSet > 128) {  // around center point of 124
	//	returnValue = ((valueToSet-127)*((127-deadband)/127))+deadband+127;
	//	returnValue = valueToSet + deadband;
		returnValue = 158 + ((p4_x)/12);  //189| 162 for shopfloor, 167 for wresting floor - competition floor??
	}else if (valueToSet < 120){  //around center point of 124
	//	returnValue = ((valueToSet+127)*((127-deadband)/127))-deadband-127;
	//	returnValue = valueToSet - deadband;
		returnValue = 102 - ((p4_x)/12); //89 | 102 for shopfloor, 107 for wresting floor - competition floor??
	}else{
		returnValue = 127;
	}
	
	//printf("rval: %d\r\n", returnValue);
	/*if (returnValue > maxLimit) {
		returnValue = maxLimit;
	}else if (returnValue < minLimit) {
		returnValue = minLimit;
	}*/
	return returnValue;
}

unsigned char PIkDControl(char Kp, char Ki, char Kd, int error) {
	static int errorTotal = 0;
	static int prevError = 0;
	int P = 0;
	int I = 0;
	int D = 0;
	

	P = (error*Kp)/100;
	I = (errorTotal*Ki)/100;
	D = ((error - prevError)*Kd)/100;

	errorTotal += error;
	prevError = error;

	//Saturate the P value
	if (P > 60) {   //Value for matted gym floor 50 ; test 60 for carpet
		P = 60;
	}else if (P < -70) { //Value for matted gym floor -60; test -70 for Caarpet
		P = -70;
	}

	//Saturate the Total Error
	if (errorTotal > 600) {
		errorTotal = 600;
	}else if (errorTotal < -600) {
		errorTotal = -600;
	}
	
	//Kleggs Integrator	
	if (error < -2 || error > 2) {
		errorTotal = 0;
	}
	

	//printf("P: %d | ", P);
	printf("P: %d | I: %d | D: %d | error: %d | Kp: %d | Ki: %d\r\n", P, I, D, error, Kp, Ki);
	return  (Limit_Mix(2000+127+P+I+D));
}

unsigned char capPwm (unsigned int valueToCap, unsigned char cap) {
	int returnValue;
	if (valueToCap > cap) {
		returnValue = cap;
	}else if (valueToCap < 0) {
		returnValue = 0;
	}else {
		returnValue = valueToCap;
	}
	return returnValue;
}

//error is a value between 0 and 255, with 127 as NO ERROR
unsigned char track_quadratic(unsigned char deadZone, unsigned char maxCap, unsigned char error) {
	int returnVal;
	
	// y = widthFactor*(x-vertexX)*(x-vertexX) + vertexY
	if (error > 127) {
		returnVal = ((maxCap+127)/(16129))*((error-127)*(error-127)) + deadZone;
	}else if(error < 127) {
		returnVal = (-1)*((maxCap+127)/(16129))*((error-127)*(error-127)) - deadZone;
	}else{
		returnVal = 127;
	}

	return returnVal;
}


/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
//	pwm09 = pwm10 = p1_y;
//	pwm11 = pwm12 = p2_y;

	if(p3_sw_trig){
		pwm05 = 255;
		pwm04 = 255;
		pwm06 = p3_y;
	}else{
		pwm05 = 127;
		pwm04 = 127;
		pwm06 = 127;
	}

 /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *------------------------------------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
  
  if (user_display_mode == 0) /* User Mode is Off */
    
  { /* Check position of Port 1 Joystick */
    if (p1_y >= 0 && p1_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON  */
    }
    else if (p1_y >= 125 && p1_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON */
    }
    else if (p1_y >= 216 && p1_y <= 255)
    {                     /* Joystick is in full forward position*/
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON  */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }  /*END Check position of Port 1 Joystick
    
    /* Check position of Port 2 Y Joystick 
           (or Port 1 X in Single Joystick Drive Mode) */
    if (p2_y >= 0 && p2_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm2_green  = 0;    /* Turn pwm2 green LED - OFF */
      Pwm2_red  = 1;      /* Turn pwm2 red LED   - ON  */
    }
    else if (p2_y >= 125 && p2_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON */
      Pwm2_red  = 1;      /* Turn PWM2 red LED   - ON */
    }
    else if (p2_y >= 216 && p2_y <= 255)
    {                     /* Joystick is in full forward position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON  */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm2_green  = 0;    /* Turn PWM2 green LED - OFF */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }  /* END Check position of Port 2 Joystick */
    
    /* This drives the Relay 1 and Relay 2 "Robot Feedback" lights on the OI. */
    Relay1_green = relay1_fwd;    /* LED is ON when Relay 1 is FWD */
    Relay1_red = relay1_rev;      /* LED is ON when Relay 1 is REV */
    Relay2_green = relay2_fwd;    /* LED is ON when Relay 2 is FWD */
    Relay2_red = relay2_rev;      /* LED is ON when Relay 2 is REV */

    Switch1_LED = (T_Packet_Data.pixels > 1);
    Switch2_LED = (PAN_SERVO > 109 && PAN_SERVO < 139); //center is 124 in tracking.h
    Switch3_LED = (TILT_SERVO > 175 && TILT_SERVO < 185); //optimal shooting range99
    
  } /* (user_display_mode = 0) (User Mode is Off) */
  
  else  /* User Mode is On - displays data in OI 4-digit display*/
  {
    User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
  }   
  
} /* END Default_Routine(); */


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
