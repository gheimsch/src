/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : ISRmotorcontrol.h
*****************************************************************************
*  Function   : Header for the interrupt service routine to regulate the
*  				drive motors.
*  				It contains a position-, speed- and current-controller.
*
* 			  	The position-controller compares the actual position in
* 			  	counts (read from the motor-encoders) with the target
* 			  	position in counts and compares the control deviation. This
* 			  	control deviation conducts the speed-controller as target
* 			  	value. The speed-controller compares the actual values (also
* 			  	read from the motor-encoders) with the target speed and
* 			  	calculates an other control deviation which conducts the
* 			  	current-controller as target value.The current-controller
* 			  	compares the actual current values of the motors (which are
* 			  	read from ADCs) with the target current value and calculates
* 			  	a control deviation. Due to the received control deviation
* 			  	from the current-controller, the PWM-outputs will be
* 			  	adjusted. The bridge driver converts the PWM-signals to the
* 			  	needed output-voltage to activate the drive motors.
*
*  Procedures : ADC_IRQHandler()
*
* 				CalcPID()
*				setMotorVoltageL()
*				setMotorVoltageR()
*				set_PID_PHI_L_w()
*				get_PID_PHI_L_w()
*				set_PID_PHI_R_w()
*				get_PID_PHI_R_w()
*				getCurrentL()
*				getCurrentR()
*
*  Author     : rufed1, haldj3
*
*  Version    : V0.1
*
*  History    : 01.07.2013  File created
*  				16.11.2013  Edited header, comments and ISR
*
*  TODO		  : Prevent angular deviation ("Winkelabweichung") by the use of
*  				int and not floating point variables.
*  				(ctrl->w and ctrl->y should be int, ctrl->e can be float).
*
*  Bug		  : The current-controller oscillates.
*
****************************************************************************/

#ifndef ISRMOTORCONTROL_H_
#define ISRMOTORCONTROL_H_

/*-------------------------------defines-----------------------------------*/
/* Voltage defines */
#define MotorVoltage ( 22.2f )		// Operating voltage in V

/* Max motor current defines */
#define MaxMotorCurrent ( 10.0f )		// Peak current of the motors in A (20W and 60W)
#define MinMotorCurrent ( -10.0f )

/* Controller defines */
#define KpPhi ( 0.01f )
#define KiPhi ( 0.0f )

#define KpV ( 1.7883f )
#define KiV ( 37.1783f )
#define TimeIntevalSpeedControl ( 0.1f )	// in ms

#define KpI ( 0.403f )
#define KiI ( 2203.0f )
#define TimeIntevalCurrentControl ( 0.1f )	// in ms

/* PWM DutyCycles */
#define PWM_A_DutyCycle	( TIM1->CCR1 )
#define PWM_B_DutyCycle	( TIM1->CCR2 )
#define PWM_C_DutyCycle	( TIM1->CCR3 )
#define PWM_D_DutyCycle	( TIM1->CCR4 )

/* Alt */
#define PWMScale	( 499.0f / 24.0f )	//convertss the controller output to a PWM signa... ??

/* Intermediate macros for current-calculation */
#define CurrentSenseOffset ( 1.65f )											// 3.3V / 2
#define CurrentSenseScale ( (float)(CurrentSenseOffset / MaxMotorCurrent) )		// 1.65V / 10A = 0.165V/A
#define getADCvoltage( adcVal ) ( (3.3f / 4096.0f) * (float)(adcVal) )			// 3.3V resolution value / (2^12 = 4096)
#define getCurrent( Uadc ) ( (Uadc - CurrentSenseOffset) / CurrentSenseScale )	// Transforms the ADC-voltage (0V to 3.3V) in a current (-10A to 10A)

/* Encoder defines */
#define MotL (0)
#define MotR (1)
#define OdoL (2)
#define OdoR (3)

/*----------------------exported (global) functions-------------------------*/
extern inline void set_PID_PHI_L_w(float value);
//extern inline float get_PID_PHI_L_w( void );

extern inline void set_PID_PHI_R_w(float value);
//extern inline float get_PID_PHI_R_w( void );

#endif
