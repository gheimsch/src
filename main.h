/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : main.h
*****************************************************************************
*  Function   : Startup file called at the startup of the controller.
*
*  Procedures : main()
*				prvSetupHardware()
*				vApplicationTickHook()
*				vApplicationMallocFailedHook()
*				vApplicationIdleHook()
*				vApplicationStackOverflowHook()
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 16.11.2013  File created
*
****************************************************************************/

#ifndef __MAIN_H__
#define __MAIN_H__

/*-------------------------------defines-----------------------------------*/
/* Priorities for the application tasks */
#define mainCONTROL_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1UL )
#define mainPERFORM_TASK_PRIORITY 	( tskIDLE_PRIORITY + 2UL )
#define mainFLASH_TASK_PRIORITY		( tskIDLE_PRIORITY + 3UL )

/*----------------------exported (global) function-------------------------*/
extern void SDIOtask(void *pvargs);

#endif
