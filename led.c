/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : led.c
*****************************************************************************
*  Function   : This file Contains tasks to initialize and handle LEDs.
*
*  Procedures : initLEDTasks()
*  				setLEDgreen()
*  				setLEDred()
*  				setLEDotw()
*  				setLEDfault()
*
*  Tasks 	  : vLEDgreen()
* 		  		vLEDred()
* 		  		vLEDotw()
* 		  		vLEDfault()
*
*  Author     : rufed1, haldj3
*
*  Version    : V0.2
*
*  History    : 01.07.2013  File created
*  				13.12.2013  startLEDTasks changed to initLEDTasks (Task to Function)
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
#include "led.h"

/*------------------------------variables----------------------------------*/
/* Start-condition of the error-LEDs = OFF */
static uint8_t statusOTW 	= Bit_SET;		//if the otw pin's set, the otw LED's off
static uint8_t statusFAULT 	= Bit_SET;		//if the fault pin's set, the fault LED's off

/*------------------------------prototypes---------------------------------*/
/* The LED-task that will be created four times */
static portTASK_FUNCTION_PROTO( vLEDgreen, pvParameters );
static portTASK_FUNCTION_PROTO( vLEDred,   pvParameters );
static portTASK_FUNCTION_PROTO( vLEDotw,   pvParameters );
static portTASK_FUNCTION_PROTO( vLEDfault, pvParameters );

/* The setLED-functions */
static void setLEDgreen( BitAction BitVal );
static void setLEDred(   BitAction BitVal );
static void setLEDotw(   BitAction BitVal );
static void setLEDfault( BitAction BitVal );

/****************************************************************************
*  Procedure  : initLEDTasks
*****************************************************************************
*  Function   : This function initializes the LED-flash-tasks for the status
*  				LEDs of the uC and the LED-tasks for the error status signal
*  				of the bridge driver. It also initializes the related
*		        peripherals. It will be only called once.
*
*  Input Para : LED task priority
*
*  Return	  : -
*
*  Author     : rufed1, haldj3
*
*  Version    : V0.2
*
*  History    : 01.07.2013  File created
*  				13.12.2013  startLEDTasks changed to initLEDTasks (Task to Function)
*
****************************************************************************/
void initLEDTasks( unsigned portBASE_TYPE uxPriority )
{
	/* Initializes the HW for the LEDs and the pins for the error report */
	initLEDs();		// declared in the file 'inits.c'

	/* Spawn the tasks */
	xTaskCreate( vLEDgreen, ( signed char * ) "LEDgreen", ledSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
	xTaskCreate( vLEDred,   ( signed char * ) "LEDred",   ledSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
	xTaskCreate( vLEDotw,   ( signed char * ) "LEDotw",   ledSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
	xTaskCreate( vLEDfault, ( signed char * ) "LEDfault", ledSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}

/****************************************************************************
*  Task 	  : vLEDgreen
*****************************************************************************
*  Function   : This task toggles the green uC-Status-LED. There is a long
*  				off-delay after two short flashes.
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
static portTASK_FUNCTION( vLEDgreen, pvParameters )
{
	/* The task-function-parameters are not used */
	( void ) pvParameters;

	/* xLastFlashTime needs to be initialized before the first call of vTaskDelayUntil() */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		setLEDgreen( Bit_SET );
		vTaskDelayUntil( &xLastFlashTime, 10 );
		setLEDgreen( Bit_RESET );
		vTaskDelayUntil( &xLastFlashTime, 90 );
		setLEDgreen( Bit_SET );
		vTaskDelayUntil( &xLastFlashTime, 10 );
		setLEDgreen( Bit_RESET );
		vTaskDelayUntil( &xLastFlashTime, 890 );		// Long off-delay
	}
}

/****************************************************************************
*  Task 	  : vLEDred
*****************************************************************************
*  Function   : This task toggles the red uC-Status-LED.
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
static portTASK_FUNCTION( vLEDred, pvParameters )
{
	/* The task-function-parameters are not used */
	( void ) pvParameters;

	/* xLastFlashTime needs to be initialized before the first call of vTaskDelayUntil() */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		setLEDred( Bit_SET );
		vTaskDelayUntil( &xLastFlashTime, 5 );	//300
		setLEDred( Bit_RESET );
		vTaskDelayUntil( &xLastFlashTime, 5 );	//300

		/* Only for CAN debugging */
//		GPIO_ToggleBits(GPIOD, GPIO_Pin_2);
//		GPIO_ToggleBits(GPIOB, GPIO_Pin_9);
	}
}

/****************************************************************************
*  Task 	  : vLEDotw
*****************************************************************************
*  Function   : This task activates the red otw LED, if an error status signal
* 		  		occurs at the otw pin of the bridge driver.
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
static portTASK_FUNCTION( vLEDotw, pvParameters )
{
	/* The task-function-parameters are not used */
	( void ) pvParameters;

	/* xLastFlashTime needs to be initialized before the first call of vTaskDelayUntil() */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		statusOTW = GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_10 );
		if ( statusOTW == Bit_SET )
		{
			setLEDotw( Bit_RESET );
		}
		
		if ( statusOTW == Bit_RESET )
		{
			setLEDotw( Bit_SET );
		}
		/* Cycle-Time */
		vTaskDelayUntil( &xLastFlashTime, 300 );
	}
}

/****************************************************************************
*  Task 	  : vLEDfault
*****************************************************************************
*  Function   : This task activates the green fault LED, if an error status
*  				signal occurs at the fault pin of the bridge driver.
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
static portTASK_FUNCTION( vLEDfault, pvParameters )
{
	/* The task-function-parameters are not used */
	( void ) pvParameters;

	/* xLastFlashTime needs to be initialized before the first call of vTaskDelayUntil() */
	portTickType xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for(;;)
	{
		statusFAULT = GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_11 );
		if ( statusFAULT == Bit_SET )
		{
			setLEDfault( Bit_RESET );
		}
		
		if ( statusFAULT == Bit_RESET )
		{
			setLEDfault( Bit_SET );
		}
		/* Cycle-Time */
		vTaskDelayUntil( &xLastFlashTime, 300 );
	}
}

/****************************************************************************
*  Procedure  : setLEDgreen
*****************************************************************************
*  Function   : This function sets the green LED.
*
*  Input Para : BitVal contains the bit action (Bit_SET or Bit_RESET)
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
static void setLEDgreen( BitAction BitVal )
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, BitVal);
}

/****************************************************************************
*  Procedure  : setLEDred
*****************************************************************************
*  Function   : This function sets the red LED.
*
*  Input Para : BitVal contains the bit action (Bit_SET or Bit_RESET)
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
static void setLEDred( BitAction BitVal )
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, BitVal);
}

/****************************************************************************
*  Procedure  : setLEDotw
*****************************************************************************
*  Function   : This function sets the red otw LED.
*	      		(otw = overtemperature warning)
*
*  Input Para : BitVal contains the bit action (Bit_SET or Bit_RESET)
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
static void setLEDotw( BitAction BitVal )
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, BitVal);
}

/****************************************************************************
*  Procedure  : setLEfault
*****************************************************************************
*  Function   : This function sets the green fault LED.
*         		(fault = overcurrent warning)
*
*  Input Para : BitVal contains the bit action (Bit_SET or Bit_RESET)
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
static void setLEDfault( BitAction BitVal )
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, BitVal);
}
