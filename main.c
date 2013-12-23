/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : main.c
*****************************************************************************
*  Function   : This file initializes the basic HW and the OS (FreeRTOS). It
*  				starts the StartTasks and also contains some Hook-functions.
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

/*------------------------------includes-----------------------------------*/
/* Kernel includes */
#include "FreeRTOS.h"		// must appear before #include task.h
#include "task.h"

/* Hardware includes */
#include <memPoolService.h>		// Memory pool manager service
#include "stm32f4xx.h"

/* User includes */
#include "led.h"
#include "driving.h"
#include "drivemanager.h"
#include "CANGatekeeper.h"
#include "main.h"

/*------------------------------prototypes---------------------------------*/
static void prvSetupHardware( void );

/****************************************************************************
*  Procedure  : main
*****************************************************************************
*  Function   : Initializes the basic HW and the OS. It also starts the init-
*  				functions.
*
*  Input Para : -
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.2
*
*  History    : 16.11.2013
*  				06.12.2013 CAN
*
****************************************************************************/
int main(void)
{
	//int8_t points[2][2] = {{-1,-1},{-1,-1}};

	/* Configure the basic hardware */
	prvSetupHardware();

	/* Initialize LED-control */
	initLEDTasks( mainFLASH_TASK_PRIORITY );

	/* Initialize drive manager */
	initDrivemanagerTask( mainCONTROL_TASK_PRIORITY );

	/* Initialize motor control */
	initDrivingTask( mainPERFORM_TASK_PRIORITY );

	/* Initialize CAN gatekeeper */
	initCANGatekeeper();	// Have to be called before the scheduler and as the last initialization

	//txGotoXY(239,137,180,20,points);

	/* Start the scheduler */
	vTaskStartScheduler();
	
	/* Infinite loop */
	for( ;; );
}

/****************************************************************************
*  Procedure  : prvSetupHardware
*****************************************************************************
*  Function   : In this function the basic hardware-setup is done.
*			    Call this function before starting the scheduler!
*
****************************************************************************/
static void prvSetupHardware( void )
{
	/* The system-clock is already configured properly by the startup-code */

	/* Ensure all priority bits are assigned as preemption priority bits */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
}


/* Hooks: */
void vApplicationTickHook( void )
{

}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
