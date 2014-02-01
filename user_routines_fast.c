/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/
#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "tracking.h"
#include "adc.h"
#include "gyro.h"
// #include "user_Serialdrv.h"


/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
char autoOption = 0;			//option selected by the huge switches
char option1 = 0;				//a dipswitch option
unsigned int timer1 = 0;		//timer to count time for events
unsigned int counter1 = 0;
unsigned int turn_counter = 0;	//timer to counts turning time
unsigned int gyroAvg = 0;
unsigned int counter_pulse = 0;	
  
int robotTilt = 0;				//filler variable for the tilt of the robot. 0 assumed to be flush with floor
unsigned char cam_pan = 0;		//camera's pan
unsigned char cam_tilt = 0;		//camera's tilt
unsigned char cam_pixels = 0;	//camera target visibilty
unsigned char speed_offset = 0; //speed offset for the robot (0 = {255, 0})
unsigned char mode = 0;
unsigned char temp_p1_y = 127;	//Virtual joystick values, used for aiming while moving and cool stuff like that
unsigned char temp_p1_x = 127;
 
unsigned char maxShootAngle = 230;  //max shooting angle for the robot (how CLOSE it can be)
unsigned char minShootAngle = 50;	//min shooting angle (how FAR AWAY it can be)

char bAutoOpt01, bAutoOpt02, bOptBit01, bOptBit02, bOptBit03, bOptBit04, bOptBit05, bOptBit06, bOptBit07, bOptBit08;
/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata") /* You may want to save additional symbols. */

void InterruptHandlerLow ()     
{
	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}
	else if(PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
	{
		PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag [92]
		Timer_2_Int_Handler(); // call the timer 2 interrupt handler (in adc.c)
	}                     
	else if(PIR1bits.ADIF && PIE1bits.ADIE) // ADC interrupt
	{
		PIR1bits.ADIF = 0; // clear the ADC interrupt flag
		ADC_Int_Handler(); // call the ADC interrupt handler (in adc.c)
	}

//  ***  IFI Code Starts Here***
//                              
//  unsigned char int_byte;       
//  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
//  { 
//    INTCON3bits.INT2IF = 0;
//  }
//  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
//  {
//    INTCON3bits.INT3IF = 0;
//  }
//  else if (INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
//  {
//    int_byte = PORTB;          /* You must read or write to PORTB */
//    INTCONbits.RBIF = 0;     /*     and clear the interrupt flag         */
//  }                                        /*     to clear the interrupt condition.  */
//  else
//  { 
//    CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
//  }
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;
	counter1 = 0;
	counter1 = 0;
	counter_pulse = 0;
	mode = 0;

  autoOption = 0;
  

  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */
        /* Add your own autonomous code here. */
        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

		//option switches... all we really need is an on/off signal.
		bAutoOpt01 = rc_dig_in01;  //upper switch
		bAutoOpt02 = rc_dig_in03;  //lower switch
		//parameter dipswitches
		bOptBit01 = !rc_dig_in05;
		bOptBit02 = !rc_dig_in06;
		bOptBit03 = !rc_dig_in07;
		bOptBit04 = !rc_dig_in08;
		bOptBit05 = !rc_dig_in09;
		bOptBit06 = !rc_dig_in10;
		bOptBit07 = !rc_dig_in11;
		bOptBit08 = !rc_dig_in12;
		
		//update variables
		//FOR THE BINARY DEFICIENT: GG = 1, RG=2, GR = 3, RR = 4
		autoOption = (bAutoOpt01 + (bAutoOpt02*2))+1; //parse the binary input
		robotTilt = Get_Gyro_Angle();
		cam_pan = PAN_SERVO;
		cam_tilt = TILT_SERVO;
		cam_pixels = T_Packet_Data.pixels;
		

		//initialize camera use and tracking.
		Camera_Handler();
		
		

		//printf("cam_tilt: %d | pwm08: %d | pwm09: %d\r\n", cam_tilt, pwm08, pwm09);
		//printf("autoOption = %d\r\n", autoOption);
		//printf("Shooter(R,L) = (%d,%d)\r\nDrive(R,L) = (%d,%d)\r\nRokenbok(R,L) = (%d,%d)\r\nLoader = %d\r\nMode=%d\r\n\r\n",pwm04,pwm05, pwm08,pwm09, pwm03,pwm07, pwm06, autoOption);
		
	
		if (autoOption == 1) { //go to corner goal, detect tilt, shit the balls
			//printf("Corner Goal Auto Mode\r\n");
			if (robotTilt > 25 || robotTilt < -25) { //shit the balls :D
					pwm03 = 0;
					pwm07 = 255;

					pwm08 = 140;
					pwm09 = 109;
				
			}else{
				pwm03 = pwm07 = 127;
				counter1++;
				if (counter1 < 66) {
					pwm08 = 187; //move forwardS
					pwm09 = 89;  //(WHY IS THIS VALUE FLIPPED?!)
				}else if(counter1 < (22*5)) {
					pwm08 = 167;
					pwm09 = 99;
				}else{
					pwm08 = 137;
					pwm09 = 127;
				}
			}
		}else if (autoOption == 2) { //goto hotSpot, aim for light, fire in the hull
			static char takeAim = 0;
			static char autoStep = 0;
			static unsigned int timeToGo = 0;
			static int target;
			
			//RESET THE AUTO MODE!!
			//if (counter1 < 1) {
			//	autoStep = 0;
			//}
			printf(" counter1: %d | Phase: %d | time = %d | ", counter1, autoStep, timeToGo);
			Servo_Track();
			timeToGo = (int)(1000+(bOptBit01)+(bOptBit02*2)+(bOptBit03*4)-1000)*44;
			//Go out for one second, take aim, move forward till in range, take aim, fire.
			if (autoStep == 0) {
				counter1++;
				temp_p1_y = 96;
				if (counter1 >= timeToGo) {
					autoStep = 1;
				}
			}else if (autoStep == 1) {
				//if (takeAim == 0) {
				//	target = Get_Gyro_Angle() - 900;
				//	takeAim = 1;
				//}
				temp_p1_y = 127;
				autoStep = 2;  //The above code is flawed.  just skip on.
			}else if (autoStep == 2) {
				if (T_Packet_Data.confidence > 100) {
					if (cam_tilt < 175) {
						temp_p1_y = 96;
					}else if (cam_tilt > 185) {
						temp_p1_y = 160;
					}else{
						temp_p1_y = 127;
						autoStep = 3;
						takeAim = 0;
					}
				}else{
					temp_p1_y = 127;
				}
				//takeAim = 0;
			}else if (autoStep == 3) {
				temp_p1_y = 127;
				if (T_Packet_Data.confidence > 170) { // && cam_pixels > 1) {
					if (takeAim == 0) {
					target =  Get_Gyro_Angle() + (((( (float)PAN_SERVO - 124.0f) * 65.0f)/124.0f)*10.0f);
					}
					takeAim = 1;
					printf(" Finding Target ");
				}
				
				if (cam_pan < 137 && cam_pan > 117  && T_Packet_Data.confidence > 100) {
					pwm06 = 255;
				}else{
					pwm06 = 127;
				}
				if (cam_tilt > 185 || cam_tilt < 170) {
					autoStep = 2;
				}
			}
			/*else if (autoStep == 4) {
				if (cam_pan < 109 || cam_pan > 139) {
					autoStep = 3;
					takeAim = 0;
				}else{ 
					autoStep = 3;
				}
			}*/
			printf("AutoStoop : %d | Aim: %d | Confidence: %d | camPan: %d\r\n", autoStep, takeAim, T_Packet_Data.confidence, cam_pan);
			if (takeAim == 1) {
				temp_p1_x = flip_axis(PIkDControl(75,15 ,50, target-Get_Gyro_Angle()), 127);

				if ((cam_pan > 109 || cam_pan < 139))  {
					//autoStep++;
				}else if(autoStep == 3) {
					//takeAim = 0;
				}
			}
			//counter1++;  //take this out once we nail autonomous to perfection

			pwm08 = Limit_Mix(2000 + temp_p1_x + temp_p1_y - 127);
			pwm09 = Limit_Mix(2000 + temp_p1_x - temp_p1_y + 127);
			
			if (!rc_dig_in16) {
				pwm04 = pwm05 = 180;
			}else{
				pwm04 = pwm05 = sSpeed;
			}

		}else if (autoOption == 3) {
				counter1++;
				mode = (bOptBit01 + (2*bOptBit02));
				//pwm08 = Limit_Mix(2000+(150 - (counter1/10))); // Goes Right
				//pwm09 = Limit_Mix(2000+(104 - (counter1/2))); 
				if (mode == 0) { 
					pwm08 = Limit_Mix(2000+(130 + (counter1/1))); //Goes Left
					pwm09 = Limit_Mix(2000+(104 + (counter1/4))); //Goes Left 
				}else if (mode == 1) {
					printf("counter1 : %d\r\n", counter1);
					if (counter1 < 88) {
						pwm08 = 200;
						pwm09 = 54;
					}else{
						pwm08 = pwm09 = 127;
					}
				}else if (mode == 2) {
					printf("counter1 : %d\r\n", counter1);
					if (counter1 < 176) {
						pwm08 = 163;
						pwm09 = 90;
					}else{
						pwm08 = pwm09 = 127;
					}
				}
		}
		//printf("<----<->--------<->-------<->---->\r\n");
		if(Get_ADC_Result_Count()) {
			Process_Gyro_Data();
			Reset_ADC_Result_Count();
		}	

        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */

  // new ADC data available?
  if(Get_ADC_Result_Count())
  {
    Process_Gyro_Data();
	
    Reset_ADC_Result_Count();
  }	

}


/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
