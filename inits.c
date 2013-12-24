/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : inits.c
*****************************************************************************
*  Function   : This file contains all the functions to initialize the I/O's
* 			  	of the uC.
*
* Procedures  : initLEDs()
* 				initCurrenMeasure()
* 				initPWMoutputs()
* 				initEncoder()
*
*  Author     : rufed1, haldj3
*
*  Version    : V0.1
*
*  History    : 01.07.2013  File created
*  				16.11.2013  Edited header, comments and initCurrentMeasure()
*
****************************************************************************/

/*------------------------------includes-----------------------------------*/
/* Kernel includes */
#include "FreeRTOS.h"		// must appear before #include task.h
#include "task.h"

/* Hardware and starter kit includes */
#include "stm32f4xx.h"

/* User includes */
#include "inits.h"

/*-------------------------------variables--------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;			//GPIO Init structure
DMA_InitTypeDef DMA_InitStructure;				//DMA Init structure
ADC_CommonInitTypeDef ADC_CommonInitStructure;	//ADC Common Init structure
ADC_InitTypeDef ADC_InitStructure;				//ADC Init structure
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	//TIM Time Base Init structure
TIM_OCInitTypeDef  TIM_OCInitStructure;			//TIM Output Compare Init structure
NVIC_InitTypeDef NVIC_InitStructure;			//NVIC Init structure

/****************************************************************************
*  Procedure  : initLEDs
*****************************************************************************
*  Function   : This function initializes the two flashing-LEDs for the
*  				status message of the uC, the two error-reporting-LEDs for
*  				the error status signal of the bridge driver and its Error-
*  				reporting-pins.
*       		(otw = over temperature warning)
*         		(fault = over current warning)
*
*  GPIOs  	  : flashing-LED green		=> GPIOB, GPIO_Pin_10 (output)
*         		flashing-LED red		=> GPIOB, GPIO_Pin_11 (output)
*         		error-LED (otw) red		=> GPIOB, GPIO_Pin_12 (output)
*         		error-LED (fault) green	=> GPIOB, GPIO_Pin_13 (output)
*
*         		otw-pin bridge driver	=> GPIOC, GPIO_Pin_10 (input)
*         		fault-pin bridge driver	=> GPIOC, GPIO_Pin_11 (input)
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
void initLEDs(void)
{
	/* Enable clocks for the GPIO-port */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE );		// EN PeriphClock for GPIOB

	/* GPIOB configuration: LED-pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( GPIOB, &GPIO_InitStructure ); 
	
	/* GPIOB configuration: TIM8 CH1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	/* GPIOC configuration: Error-reporting-pins of the bridge driver */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOC, &GPIO_InitStructure );


	/* Only for CAN debugging */
	/* tx pin *
	 */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	//en gpio B clock
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			// 9=tx
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init( GPIOB, &GPIO_InitStructure );
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;			// 8=rx
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init( GPIOB, &GPIO_InitStructure );
//
//	/* en pin */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;			// 2=en
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	//en gpio D clock
//
//	GPIO_Init( GPIOD, &GPIO_InitStructure );
//
//	GPIO_ResetBits(GPIOD,GPIO_Pin_2); /* set the RS pin of the CAN-transceiver to low -> no sleep-mode */

}

/****************************************************************************
*  Procedure  : initCurrentMeasure
*****************************************************************************
*  Function   : This function initializes two ADCs and a Timer to trigger
*  				them. The ADC-Values of the CurrentMeasure (between 0 and
*  				2^12) can be read with the ADC_GetConversionValue()-function.
*
*  GPIOs 	  : CurrentMeasure Motor1 	=> GPIOC, GPIO_Pin_4 (analog)
*  		  		CurrentMeasure Motor2 	=> GPIOC, GPIO_Pin_5 (analog)
*
*  Timer      : TIM8 triggers the ADCs	=> 10kHz
*
*  ADCs   	  : For GPIO_Pin_4			=> ADC1 Channel 14, single scan,
*  										   external trigger (TIM8)
* 		        For GPIO_Pin_5			=> ADC2 Channel 15, single scan,
*  										   external trigger (TIM8)
*
*  Author     : haldj3
*
*  Version    : V0.2
*
*  History    : 22.11.2013
*
****************************************************************************/
void initCurrentMeasure( void )
{
	/* Enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);		// EN PeriphClock for GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);		// EN PeriphClock for TIM8
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);		// EN PeriphClock for TIM8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);		// EN PeriphClock for ADC1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);		// EN PeriphClock for ADC2

	/* GPIOC configuration: CurrentMeasurement-pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//  /* Initialize TIM 8 as trigger for the ADCs */
//	/* Initialize Timebase structure (Fill with default values) */
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//
//	/* To achieve 10kHz with the 84MHz driven timer we divide by 8400 overall */
//	TIM_TimeBaseStructure.TIM_Period = 499;					// equals division by 420
//	TIM_TimeBaseStructure.TIM_Prescaler = 4;				// equals division by 10
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // equals division by 4
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_Cmd(TIM8, ENABLE);
//
//	/* Setup timer with given values */
//	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	/* Generate trigger, whenever counter reloads (Defined by period) */
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

	/* Start the timer */
	TIM_Cmd(TIM1, ENABLE);

	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 regular channel 14 configuration: */
	/* Single scan, 12 Bit resolution */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			// ENABLE = Scan mode; DISABLE = Single mode
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		// ENABLE = Continuous mode; DISABLE = Single mode
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	/* External Trigger, Rising-edge (T8_TRGO) */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_T1_TRGO;

	/* Right alignment (Result in lower 12 bit, highest 4 bits will be 0) */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

	/* Initialize ADC with given values */
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Use channel 14, 3-Cycle conversion */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_3Cycles);

  /* ADC2 regular channel 15 configuration: */
	/* Single scan, 12 Bit resolution */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	/* External Trigger, Rising-edge (T8_TRGO) */
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_T1_TRGO;

	/* Right alignment (Result in lower 12 bit, highest 4 bits will be 0) */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

	/* Initialize ADC with given values */
	ADC_Init(ADC2, &ADC_InitStructure);

	/* Use channel 15, 3-Cycle conversion */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);

	/* Enable ADC1 & ADC2 */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	/* Enable ADC interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

	/* Start ADC Conversion, not needed when external triggered*/
	//ADC_SoftwareStartConv(ADC1);
	//ADC_SoftwareStartConv(ADC2);
}

/****************************************************************************
*  Procedure  : initPWMoutputs
*****************************************************************************
*  Function   : This function initializes 4 PWM-outputs, 2 for each motor.
*		  		PWM A & B are used by the left motor.
*		  		PWM C & D are used by the right motor.
*		  		To generate the PWMs, TIM1 is used.
*
*  GPIOs 	  : PWM A 				=> GPIOA, GPIO_Pin_8  (alternate function)
* 		 		PWM B 				=> GPIOA, GPIO_Pin_9  (alternate function)
* 		  		PWM C				=> GPIOA, GPIO_Pin_10 (alternate function)
* 		 		PWM D 				=> GPIOA, GPIO_Pin_11 (alternate function)
*
* 		  		EN A&B				=> GPIOC, GPIO_Pin_12 (output)
* 		  		EN C&D				=> GPIOC, GPIO_Pin_13 (output)
*
*  Timer      : TIM1 for the PWMs	=> 18.667kHz
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
void initPWMoutputs( void )
{
	/* Enables peripheral clocks */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );		// EN PeriphClock for GPIOA
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );		// EN PeriphClock for TIM1

	/* GPIO Configuration: PWM Outputs ( PWM A, B, C, D ) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	/* GPIO Configuration: Enable-Signals for Full-Bridge ( EN A&B, EN C&D ) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	/* Route the correct peripherals (TIM1) to the pins */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

	/* Initialize the timer */
	/* On base of 168MHz we generate a 18.667kHz-PWM-signal with 1000-steps resolution */
	TIM_TimeBaseStructure.TIM_Period = 200; 	// Equals 1000 steps
	TIM_TimeBaseStructure.TIM_Prescaler = 15; 	// 168MHz / ( 8 + 1 ) / 1000 = 18.667kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	/* Setup timer with given values */
	TIM_TimeBaseInit( TIM1 , &TIM_TimeBaseStructure);

	/* Initialize the PWM-signal */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWMPulseValue;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	/* Enable all four Channels of the timer */
	TIM_OC1Init( TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig( TIM1 , TIM_OCPreload_Enable);
	TIM_OC2Init( TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig( TIM1 , TIM_OCPreload_Enable);
	TIM_OC3Init( TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig( TIM1 , TIM_OCPreload_Enable);
	TIM_OC4Init( TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig( TIM1 , TIM_OCPreload_Enable);

	/* for Advanced timer enable the arr-register */
	TIM_ARRPreloadConfig( TIM1, ENABLE );

	/* for Advanced timer enable the PWM-output */
	TIM_CtrlPWMOutputs( TIM1, ENABLE );

	/* Start the timer */
	TIM_Cmd( TIM1, ENABLE );

	/* Set the enable (RESET_AB, RESET_CD) pins of the full-bridges. The resets
	are active low. Therefore the enables have to be set for normal operation */
	GPIO_SetBits( GPIOC, GPIO_Pin_12 | GPIO_Pin_13 );
}

/****************************************************************************
*  Procedure  : initEncoder
*****************************************************************************
*  Function   : This function initializes 4 encoders, 2 for the motors and 2
*  				for the	odometry.
*		  		The encoder of the left motor uses TIM2.
*		  		The encoder of the right motor uses TIM3.
*		  		The encoder of the left odometry uses TIM4.
*		  		The encoder of the right odometry uses TIM5.
*
*  GPIOs 	  : Encoder Mot L TIM2_CH1	=> GPIOA, GPIO_Pin_15 (alternate function)
* 		 		Encoder Mot L TIM2_CH2	=> GPIOB, GPIO_Pin_3  (alternate function)
* 		  		(TIM2)
*
* 		  		Encoder Mot R TIM3_CH1	=> GPIOB, GPIO_Pin_4  (alternate function)
* 		  		Encoder Mot R TIM3_CH2	=> GPIOB, GPIO_Pin_5  (alternate function)
* 		  		(TIM3)
*
* 		  		Encoder Odo L TIM4_CH1	=> GPIOB, GPIO_Pin_6  (alternate function)
* 		  		Encoder Odo L TIM4_CH2	=> GPIOB, GPIO_Pin_7  (alternate function)
* 		  		(TIM4)
*
* 		  		Encoder Odo R TIM5_CH1	=> GPIOA, GPIO_Pin_0  (alternate function)
* 		  		Encoder Odo R TIM5_CH2	=> GPIOA, GPIO_Pin_1  (alternate function)
* 		  		(TIM5)
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
void initEncoder( void )
{
	/* Enables the AHB1 peripheral clock */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );		// EN PeriphClock for GPIOA
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );		// EN PeriphClock for GPIOB

	/* Enable timer-resources */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		// EN PeriphClock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);		// EN PeriphClock for TIM3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);		// EN PeriphClock for TIM4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);		// EN PeriphClock for TIM5

  /* Configure the Pins and the Timer for Encoder Motor Left */
	/* GPIO Configuration: Input-Pins */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource15, GPIO_AF_TIM2 );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource3, GPIO_AF_TIM2 );

	/* Configure the timer */
	TIM_EncoderInterfaceConfig( TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_SetAutoreload( TIM2, 0xFFFF );

	/* Timer counter enable */
	TIM_Cmd( TIM2, ENABLE );

	taskDISABLE_INTERRUPTS();
	{
		/* Reset the counting values */
		TIM_SetCounter( TIM2, 0);
	}
	taskENABLE_INTERRUPTS();

  /* Configure the Pins and the Timer for Encoder Motor Right */
	/* GPIO Configuration: Input-Pins */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource4, GPIO_AF_TIM3 );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource5, GPIO_AF_TIM3 );

	/* Configure the timer */
	TIM_EncoderInterfaceConfig( TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_SetAutoreload( TIM3, 0xFFFF );

	/* Timer counter enable */
	TIM_Cmd( TIM3, ENABLE );

	taskDISABLE_INTERRUPTS();
	{
		/* Reset the counting values */
		TIM_SetCounter( TIM3, 0);
	}
	taskENABLE_INTERRUPTS();

  /* Configure the Pins and the Timer for Encoder Odometry Left */
	/* GPIO Configuration: Input-Pins */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource6, GPIO_AF_TIM4 );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource7, GPIO_AF_TIM4 );

	/* Configure the timer */
	TIM_EncoderInterfaceConfig( TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_SetAutoreload( TIM4, 0xFFFF );

	/* Timer counter enable */
	TIM_Cmd( TIM4, ENABLE );

	taskDISABLE_INTERRUPTS();
	{
		/* Reset the counting values */
		TIM_SetCounter( TIM4, 0);
	}
	taskENABLE_INTERRUPTS();

  /* Configure the Pins and the Timer for Encoder Odometry Right */
	/* GPIO Configuration: Input-Pins */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource0, GPIO_AF_TIM5 );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource1, GPIO_AF_TIM5 );

	/* Configure the timer */
	TIM_EncoderInterfaceConfig( TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_SetAutoreload( TIM5, 0xFFFF );

	/* Timer counter enable */
	TIM_Cmd( TIM5, ENABLE );

	taskDISABLE_INTERRUPTS();
	{
		/* Reset the counting values */
		TIM_SetCounter( TIM5, 0);
	}
	taskENABLE_INTERRUPTS();
}
