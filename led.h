/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : led.h
*****************************************************************************
*  Function   : Header for the tasks to initialize and handle LEDs.
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

#ifndef LED_H_
#define LED_H_

/*-------------------------------defines-----------------------------------*/
#define ledSTACK_SIZE ( configMINIMAL_STACK_SIZE )

/*----------------------exported (global) function-------------------------*/
extern void initLEDTasks( unsigned portBASE_TYPE uxPriority );

#endif
