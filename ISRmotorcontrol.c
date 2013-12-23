/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : ISRmotorcontrol.c
*****************************************************************************
*  Function   : This file contains the interrupt service routine to regulate
*  				the	drive motors.
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

/*-------------------------------includes----------------------------------*/
/* Hardware includes */
#include "stm32f4xx.h"

/* User includes */
#include "inits.h"
#include "config.h"
#include "ISRmotorcontrol.h"

/*-------------------------------variables----------------------------------*/
/* Control-Values typedef */
typedef struct {
	float w; 			// reference input			(Führungsgrösse)
	float y; 			// controlled condition 	(Regelgrösse)
	float e; 			// control deviation 		(Regelabweichung)
	float u; 			// correcting variable		(Stellgrösse)
	float I_acc; 		// integral-accumulator		(Integral-Accumulator)
	float Kp; 			// proportional gain		(Proportional-Verstärkung)
	float Ki; 			// integral gain			(Integral-Verstärkung)
	float SatMin; 		// lower saturation point	(Untere Sättigungsgrenze)
	float SatMax; 		// upper saturation point	(Obere Sättigungsgrenze)
} controller;

/* Struct for the left position-controller */
controller PID_PHI_L =
{
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	KpPhi, 				// Kp
	KiPhi,				// Ki
	-2147483647, 		// SatMin in counts (-(2^31-1) (singed int) but not really needed since no Ki)
	2147483647 			// SatMax in counts (2^31-1 (singed int) but not really needed since no Ki)
};

/* Struct for the right position-controller */
controller PID_PHI_R =
{
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	KpPhi, 				// Kp
	KiPhi,				// Ki
	-2147483647,		// SatMin in counts (-(2^31-1) (singed int) but not really needed since no Ki)
	2147483647			// SatMax in counts (2^31-1 (singed int) but not really needed since no Ki)
};

/* Struct for the left speed-controller */
controller PID_V_L =
{
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
	KpV,												// Kp calculated (Kp simulated with simulink = 0.153)
	( KiV * (TimeIntevalSpeedControl / 1000.0f) ),		// Ki calculated (Ki simulated with simulink = 636.125)
	( -((float)MaxNumberOfRevolutions / 60.0f) * (float)CountsPerTurnMotEnc ), 	// SatMin in counts/s (MaxNumberOfRevolutions and CountsPerTurnMotEnc are defined in config.h)
	( ((float)MaxNumberOfRevolutions / 60.0f) * (float)CountsPerTurnMotEnc )	// SatMax in counts/s (MaxNumberOfRevolutions and CountsPerTurnMotEnc are defined in config.h)
};

/* Struct for the right speed-controller */
controller PID_V_R =
{
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
	KpV,												// Kp calculated (Kp simulated with simulink = 0.153)
	( KiV * (TimeIntevalSpeedControl / 1000.0f) ),		// Ki calculated (Ki simulated with simulink = 636.125)
	( -((float)MaxNumberOfRevolutions / 60.0f) * (float)CountsPerTurnMotEnc ), 	// SatMin in counts/s (MaxNumberOfRevolutions and CountsPerTurnMotEnc are defined in config.h)
	( ((float)MaxNumberOfRevolutions / 60.0f) * (float)CountsPerTurnMotEnc )	// SatMax in counts/s (MaxNumberOfRevolutions and CountsPerTurnMotEnc are defined in config.h)
};

/* Struct for the left current-controller */
controller PID_I_L =
{
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
	KpI, 												// Kp calculated (Kp simulated with simulink = 0.153)
	( KiI * (TimeIntevalCurrentControl / 1000.0f) ),	// Ki calculated (Ki simulated with simulink = 636.125)
	MinMotorCurrent, 									// SatMin in A
	MaxMotorCurrent 									// SatMax in A
};

/* Struct for the right current-controller */
controller PID_I_R =
{
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
	KpI, 												// Kp calculated (Kp simulated with simulink = 0.153)
	( KiI * (TimeIntevalCurrentControl / 1000.0f) ),	// Ki calculated (Ki simulated with simulink = 636.125)
	MinMotorCurrent, 									// SatMin in A
	MaxMotorCurrent 									// SatMax in A
};

/* Array to store the actual encoder values */
int32_t Encoder[4];

/* Only for debugging */
int i = 0;
int j = 0;

//int32_t EncoderDataMotL[1250];
//int32_t EncoderDataMotR[1250];
//int32_t EncoderDataOdoL[1250];
//int32_t EncoderDataOdoR[1250];

int32_t CurrentDataL[1250];
int32_t CurrentDataR[1250];

//int32_t ControllerDataL_w[1250];
//int32_t ControllerDataL_u[1250];
//int32_t ControllerDataL_y[1250];
//int32_t ControllerDataR_w[1250];
//int32_t ControllerDataR_u[1250];
//int32_t ControllerDataR_y[1250];

/*-------------------------------prototypes-------------------------------*/
static void CalcPID( controller *ctrl );
static void setMotorVoltageL( float voltage );
static void setMotorVoltageR( float voltage );
static float getCurrentL( void );
static float getCurrentR( void );
static void setPWMMotorL( int8_t speed );
static void setPWMMotorL( int8_t speed );

/****************************************************************************
*  Procedure  : ADC_IRQHandler
*****************************************************************************
*  Function   : Timer interrupt service routine of the ADCs. This routine
*  				reads the encoder values and calculates the controller.
*
*  Input Para : -
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 22.11.2013
*
****************************************************************************/
void ADC_IRQHandler (void)
{
	/* Both ADCs have the same interrupt, we check first if both are set */
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC) == 1 && ADC_GetITStatus(ADC2, ADC_IT_EOC) == 1)	// 1 = SET, 0 = RESET
	{
		/* Reset the interrupt-flags to prevent multiple IRQ (means to prevent a deadlock) */
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);

		/* This IO is only for testing. You can measure the jitter of the ISR */
		GPIO_ToggleBits(GPIOC, GPIO_Pin_6);

/*			  ______	   _____
 *	w--+--e--|Regler|--u--|Motor|--+--y
 *	   |-y	  ``````	   `````   |
 *	   |___________________________|
*/

	/* Left wheel position-controller */
		Encoder[MotL] -= ( int32_t )(( int16_t )TIM_GetCounter( TIM2 ));	// get encoder value
		TIM_SetCounter (TIM2, 0);		// reset the encoder counter to prevent an overflow
		PID_PHI_L.y = -Encoder[MotL];	// set the controlled condition for the position-controller
		CalcPID( &PID_PHI_L );			// calculate this PID-controller
//		PID_V_L.w = PID_PHI_L.u / TimeIntevalSpeedControl;	// set the current reference input for the next control-loop (in counts/s)
	/* End of left wheel position-controller */

	/* Right wheel position-controller */
		Encoder[MotR] += ( int32_t )(( int16_t )TIM_GetCounter( TIM3 ));
		TIM_SetCounter (TIM3, 0);		// reset the encoder counter to prevent an overflow
		PID_PHI_R.y = -Encoder[MotR];	// set the controlled condition for the position-controller
		CalcPID( &PID_PHI_R );			// calculate this PID-controller
//		PID_V_R.w = PID_PHI_R.u / TimeIntevalSpeedControl;	// set the current reference input for the next control-loop (in counts/s)
	/* End of right wheel position-controller */

		/* Alt */
		/* To test the position-controller exclusively (the other controller have to be commented) */
		setMotorVoltageL(22);//PID_PHI_L.u );	// set output-voltage by modifying PWM duty-cycle
		setMotorVoltageR(22);//PID_PHI_R.u );	// set output-voltage by modifying PWM duty-cycle

//	/* Left motor speed-controller */
////		Encoder[MotL] -= ( int32_t )(( int16_t )TIM_GetCounter( TIM2 ));	// get encoder value
////		TIM_SetCounter (TIM2, 0);		// reset the encoder counter to prevent an overflow
//		PID_V_L.y = -Encoder[MotL] / TimeIntevalSpeedControl;	// set the controlled condition for the speed-controller
//		CalcPID( &PID_V_L );			// calculate this PID-controller
//		PID_I_L.w = PID_V_L.u / ( TorqueConstant * (CharacteristicCurvesSlope / 60) * CountsPerTurnMotEnc);	// set the current reference input for the next control-loop (in A)
//	/* End of left motor speed-controller */
//
//	/* Right motor speed-controller */
////		Encoder[MotR] += ( int32_t )(( int16_t )TIM_GetCounter( TIM3 ));
////		TIM_SetCounter (TIM3, 0);		// reset the encoder counter to prevent an overflow
//		PID_V_R.y = -Encoder[MotR] / TimeIntevalSpeedControl;	// set the controlled condition for the speed-controller
//		CalcPID( &PID_V_R );			// calculate this PID-controller
//		PID_I_R.w = PID_V_R.u / ( TorqueConstant * (CharacteristicCurvesSlope / 60) * CountsPerTurnMotEnc);	// set the current reference input for the next control-loop (in A)
//	/* End of right motor speed-controller */
//
//	/* Left motor current-controller */
//		PID_I_L.y = GetCurrentL();		// read current-value and set the controlled condition
//		CalcPID( &PID_I_L );			// calculate this PID-controller
//		setMotorVoltageL( PID_I_L.u * ArmatureResistor );	// set output-voltage (ArmatureResistor is defined in config.h)
//	/* End of left motor current-controller */
//
//	/* Right motor current-controller */
//		PID_I_R.y = GetCurrentR();		// read current-value and set the controlled condition
//		CalcPID( &PID_I_R );			// calculate this PID-controller
//		setMotorVoltageR( PID_I_R.u * ArmatureResistor );	// set output-voltage (ArmatureResistor is defined in config.h)
//	/* End of right motor current-controller */

	/* Only for debugging */
		/* Save Encoder-, Current- and Controller-data in arrays (250 values per second) */
		//if(j>=40 && i<1250)
		//{
			/* Get odometry data */
//			Encoder[OdoL] += ( int32_t )(( int16_t )TIM_GetCounter( TIM4 ));
//			Encoder[OdoR] += ( int32_t )(( int16_t )TIM_GetCounter( TIM5 ));
//
//			EncoderDataMotL[i] = Encoder[MotL];
//			EncoderDataMotR[i] = Encoder[MotR];
//			EncoderDataOdoL[i] = Encoder[OdoL];
//			EncoderDataOdoR[i] = Encoder[OdoR];

			CurrentDataL[i] = (int32_t)(getCurrentL() * 1000000);	// * 10^6 um die Stellen nach dem Komma aufzuzeigen
			CurrentDataR[i] = (int32_t)(getCurrentR() * 1000000);	// muss wieder geteilt werden!

//			ControllerDataL_w[i] = PID_PHI_L.w;
//			ControllerDataL_u[i] = PID_PHI_L.u;
//			ControllerDataL_y[i] = PID_PHI_L.y;
//			ControllerDataR_w[i] = PID_PHI_R.w;
//			ControllerDataR_u[i] = PID_PHI_R.u;
//			ControllerDataR_y[i] = PID_PHI_R.y;

			i++;
			//j = 0;
		//}
		j++;
	}
}

/****************************************************************************
*  Procedure  : CalcPID
*****************************************************************************
*  Function   : This function regulates the motors. It can be used for any
*  				controller.
*
*  Input Para : ctrl (controller struct typdef) contains the controller-values
*
*  Return	  : -
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
static void CalcPID( controller *ctrl )
{
	/* Calculate the error-signal */
	ctrl->e = ctrl->w - ctrl->y;
	
	/* Update-Integrator */
	ctrl->I_acc += ctrl->e * ctrl->Ki;	
	
	/* Anti-windup */
	if( ctrl->I_acc > ctrl->SatMax ) ctrl->I_acc = ctrl->SatMax;
	if( ctrl->I_acc < ctrl->SatMin ) ctrl->I_acc = ctrl->SatMin;
	
	/* Calculate output */
	ctrl->u = ( ctrl->e * ctrl->Kp ) + ctrl->I_acc;
	
	/* Clamp output (saturation) */
	if( ctrl->u > ctrl->SatMax ) ctrl->u = ctrl->SatMax;
	if( ctrl->u < ctrl->SatMin ) ctrl->u = ctrl->SatMin;
}

/****************************************************************************
*  Procedure  : setMotorVoltageL
*****************************************************************************
*  Function   : Sets the average motor-voltage ~(-23V..23V) of the left motor.
*  				If both duty cycles (A and B) are 500, the motor is in
*  				standstill. If PWM_A > PWM_B, the rotating direction is
*  				forward, otherwise backward.
*
*  Input Para : voltage in V
*
*  Return	  : -
*
*  Author     : rufed1, haldj3
*
*  Version    : V0.2
*
*  History    : 01.07.2013
*  				07.12.2013  eliminated magic numbers
*
****************************************************************************/
static void setMotorVoltageL( float voltage )
{
//	/* Scales the given voltage to a DutyCycle (PWMPulseValue is defined in inits.h) */
//	float DutyCycle;
//	DutyCycle = (voltage / MotorVoltage) * PWMPulseValue;	// -500 <= DutyCycle <= 500
//
//	/* Set PWM-DutyCycles */
//	PWM_A_DutyCycle = (uint16_t)( PWMPulseValue + (int16_t)DutyCycle );	// 0 <= PWM_A_DutyCycle <= 1000
//	PWM_B_DutyCycle = (uint16_t)( PWMPulseValue - (int16_t)DutyCycle );	// 0 <= PWM_B_DutyCycle <= 1000
	
	/* Alt: */
	/* Scale voltage to duty-cycle */
	voltage *= PWMScale;

	/* Set PWM-DutyCycles (PWMPulseValue is defined in inits.h) */
	PWM_A_DutyCycle = (uint16_t)( PWMPulseValue + (int16_t)voltage );
	PWM_B_DutyCycle = (uint16_t)( PWMPulseValue - (int16_t)voltage );
}
 
/****************************************************************************
*  Procedure  : setMotorVoltageR
*****************************************************************************
*  Function   : Sets the average motor-voltage ~(-23V..23V) of the right
*  				motor. If both duty cycles (C and D) are 500, the motor is in
*  				standstill. If PWM_C > PWM_D, the rotating direction is
*  				forward, otherwise backward.
*
*  Input Para : voltage in V
*
*  Return	  : -
*
*  Author     : rufed1, haldj3
*
*  Version    : V0.2
*
*  History    : 01.07.2013
*  				07.12.2013  eliminated magic numbers
*
****************************************************************************/
static void setMotorVoltageR( float voltage )
{
//	/* Scales the given voltage to a DutyCycle (PWMPulseValue is defined in inits.h) */
//	float DutyCycle;
//	DutyCycle = (voltage / MotorVoltage) * PWMPulseValue;	// -500 <= DutyCycle <= 500
//
//	/* Set PWM-DutyCycles */
//	PWM_C_DutyCycle = (uint16_t)( PWMPulseValue + (int16_t)DutyCycle );	// 0 <= PWM_C_DutyCycle <= 1000
//	PWM_D_DutyCycle = (uint16_t)( PWMPulseValue - (int16_t)DutyCycle );	// 0 <= PWM_D_DutyCycle <= 1000

	/* Alt: */
	/* Scale voltage to duty-cycle */
	voltage *= PWMScale;

	/* Set PWM-DutyCycles (PWMPulseValue is defined in inits.h) */
	PWM_C_DutyCycle = (uint16_t)( PWMPulseValue + (int16_t)voltage );
	PWM_D_DutyCycle = (uint16_t)( PWMPulseValue - (int16_t)voltage );
}
 
/****************************************************************************
*  Procedure  : set_PID_PHI_L_w
*****************************************************************************
*  Function   : Sets the variable PID_PHI_L.w.
*
*  Input Para : value is the reference input of the controller
*
*  Return	  : -
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
inline void set_PID_PHI_L_w(float value)
{
	PID_PHI_L.w = value;
}

/****************************************************************************
*  Procedure  : get_PID_PHI_L_w
*****************************************************************************
*  Function   : Gets the variable PID_PHI_L.w.
*
*  Input Para : -
*
*  Return	  : PID_PHI_L.w
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
inline float get_PID_PHI_L_w( void )
{
	return PID_PHI_L.w;
}

/****************************************************************************
*  Procedure  : set_PID_PHI_R_w
*****************************************************************************
*  Function   : Sets the variable PID_PHI_R.w.
*
*  Input Para : value is the reference input of the controller
*
*  Return	  : -
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
inline void set_PID_PHI_R_w(float value)
{
	PID_PHI_R.w = value;
}

/****************************************************************************
*  Procedure  : get_PID_PHI_R_w
*****************************************************************************
*  Function   : Gets the variable PID_PHI_R_w.
*
*  Input Para : -
*
*  Return	  : PID_PHI_R.w
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
inline float get_PID_PHI_R_w( void )
{
	return PID_PHI_R.w;
}

/****************************************************************************
*  Procedure  : getCurrentL
*****************************************************************************
*  Function   : This function gets the converted value of the ADC1 (value
*  				between 0 and 2^12), converts it in a current and returns it.
*
*  Input Para : -
*
*  Return	  : current in A
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 16.11.2013
*
****************************************************************************/
float getCurrentL( void )
{
	float current;
	int adc;
	adc = ADC_GetConversionValue(ADC1);
	current = getCurrent( getADCvoltage( ADC_GetConversionValue(ADC1) ) );
	return current;
}	

/****************************************************************************
*  Procedure  : getCurrentR
*****************************************************************************
*  Function   : This function gets the converted value of the ADC2 (value
*  				between 0 and 2^12), converts it in a current and returns it.
*
*  Input Para : -
*
*  Return	  : current in A
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 16.11.2013
*
****************************************************************************/
float getCurrentR( void )
{
	float current;
	current = getCurrent( getADCvoltage( ADC_GetConversionValue(ADC2) ) );
	return current;
}


/****************************************************************************
*  Procedure  : setPWMMotorL
*****************************************************************************
*  Function   : Comment here
*
*  Input Para : speed of the left Motor (-100 to 100)
*
*  Return     : -
*
*  Author     : meert1
*
*  Version    : V0.1
*
*  History    : 23.12.2013	meert1 - Function added to Project
*
****************************************************************************/
static void setPWMMotorL( int8_t speed )
{
	/* Check if the speedvalui is in the correct range */
	if(speed < -100)
	{
		speed = -100;
	}
	else if(speed > 100)
	{
		speed = 100;
	}

	/* Set PWM-DutyCycles (PWMPulseOffset is defined in inits.h) */
	PWM_A_DutyCycle = (uint16_t)( PWMPulseOffset + speed );
	PWM_B_DutyCycle = (uint16_t)( PWMPulseOffset - speed );
}


/****************************************************************************
*  Procedure  : setPWMMotorR
*****************************************************************************
*  Function   : Comment here
*
*  Input Para : speed of the right Motor (-100 to 100)
*
*  Return     : -
*
*  Author     : meert1
*
*  Version    : V0.1
*
*  History    : 23.12.2013	meert1 - Function added to Project
*
****************************************************************************/
static void setPWMMotorR( int8_t speed )
{
	/* Check if the speedvalui is in the correct range */
	if(speed < -100)
	{
		speed = -100;
	}
	else if(speed > 100)
	{
		speed = 100;
	}

	/* Set PWM-DutyCycles (PWMPulseOffset is defined in inits.h) */
	PWM_C_DutyCycle = (uint16_t)( PWMPulseOffset + speed );
	PWM_D_DutyCycle = (uint16_t)( PWMPulseOffset - speed );
}
