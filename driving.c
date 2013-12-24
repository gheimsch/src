/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : driving.c
*****************************************************************************
*  Function   : The function initDrivingTask initializes the hardware and
*  				creates the driving task. The driving task cuts the received
*  				distances from the task vDriveManagerTask into smaller pieces
*  				and gives them to the ISRmotorcontrol. Further it contains a
*  				start-, a break- and a stop-ramp.
*
*  Procedures : initDrivingTask()
*  				ConvDistanceToCounts()
*  				setStopFlag()
*  				getStopFlag()
*
*  Tasks 	  : vDrivingTask
*
*  Author     : haldj3
*
*  Version    : V0.2
*
*  History    : 30.11.2013  File created
*  				06.12.2013  getStopFlag()
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
#include "config.h"
#include "drivemanager.h"
#include "ISRmotorcontrol.h"
#include "driving.h"

#include <math.h>		// to use sqrt and fabs

/*-------------------------------variables----------------------------------*/
volatile static int StopFlag = pdFALSE;	// Stop flag to check if we received a stop command

/*-------------------------------prototypes-------------------------------*/
static portTASK_FUNCTION_PROTO( vDrivingTask, pvParameters );
static float ConvDistanceToCounts( float distance );

/****************************************************************************
*  Procedure  : initDrivingTask
*****************************************************************************
*  Function   : This task initializes the driving task and the related
*		  		peripherals. It will be only called once.
*
*  Input Para : Driving task priority
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.3
*
*  History    : 22.11.2013
*  				30.11.2013	Changed Task names
*  				13.12.2013  moved DrivemanagerTask to initDrivemanagerTask()
*
****************************************************************************/
void initDrivingTask( unsigned portBASE_TYPE uxPriority )
{
	/* Initialize current-measurement and the ADC interrupt */
	initCurrentMeasure();		// declared in the file 'inits.c'

	/* Initializes and enables PWM-generator for the motor-control-signals */
	initPWMoutputs();			// declared in the file 'inits.c'
	
	/* Initialize hardware for the encoders */
	initEncoder();				// declared in the file 'inits.c'

	/* Spawn the task */
	//TODO//xTaskCreate( vDrivingTask, ( signed char * ) "Driving", drivingSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}					
		
/****************************************************************************
*  Task 	  : vDrivingTask
*****************************************************************************
*  Function   : This task cuts the received distances GoalL and GoalR
*  				from the task vDriveManagerTask into smaller pieces and
*  				gives them to the ISRmotorcontrol. Further it contains a
*  				start-, a break- and a stop-ramp.
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 30.11.2013
*
****************************************************************************/
static portTASK_FUNCTION( vDrivingTask, pvParameters )
{
	/* Driving variables */
	static float GoalL;			// will be in m
	static float GoalR;			// will be in m
	static float MaxSpeed;		// will be in m/s
	static float SLeft;			// will be in m
	static float SRight;		// will be in m
	static float dS;			// will be in m
	static float SpeedLimit;	// will be in m/s
	static float SpeedL;		// will be in m/s
	static float SpeedR;		// will be in m/s
	static float DistanceInCountsL;
	static float DistanceInCountsR;

	static float DriveL = 0.0f;	// in m
	static float DriveR = 0.0f;	// in m
	static float Speed = 0.0f;	// in m/s

	/* The task-function-parameters are not used */
	( void ) pvParameters;

	/* xLastCycleStart needs to be initialized before the first call of vTaskDelayUntil() */
	portTickType xLastCycleStart;
	xLastCycleStart = xTaskGetTickCount();

	for( ;; )
	{
		/* Wait until new driving command */
		while(getDriveFinished() == pdTRUE)				// getDriveFinished() is declared in drivemanager.h
		{
			vTaskDelay( (portTickType)DrivingTaskDelay );	// wait for ~10ms
		}

		/* Check if StopFlag is set */
		if (StopFlag == pdTRUE)
		{
		/* Stopramp */
			/* Decrees speed */
			Speed -= Deceleration * dt;

			/* Check if the robot will stop (the StopFlag will be reset at the end of drivemanager.c) */
			if (Speed <= 0)
			{
				Speed = 0;

				taskENTER_CRITICAL();
				{
					setDriveFinished(pdTRUE);		// setDriveFinished() is declared in drivemanager.h
				}
				taskEXIT_CRITICAL();
			}

			/* Increase the distances the robot shall drive with the next little piece of the route */
			DriveL += Speed * dt;
			DriveR += Speed * dt;
		/* End of stopramp */
		}
		else
		{
			/* Get the total distance for the left and right wheel and the maximum speed */
			taskENTER_CRITICAL();
			{
				GoalL = getGlobalGoalL();		// in m (getGlobalGoalL() is declared in drivemanager.h)
				GoalR = getGlobalGoalR();		// in m (getGlobalGoalR() is declared in drivemanager.h)
				MaxSpeed = getGlobalSpeed();	// in m/s (getGlobalSpeed() is declared in drivemanager.h)
			}
			taskEXIT_CRITICAL();

		/* Startramp */
			/* Increase current Speed when it is lower then the Speed the robot should drive with */
			if (Speed < MaxSpeed)
			{
				Speed += Acceleration * dt;

				/* Set MaxSpeed when the Speed is higher then the MaxSpeed now */
				if (Speed > MaxSpeed) Speed = MaxSpeed;
			}
		/* End of startramp */

		/* Breakramp */
			/* Calculate a SpeedLimit with the average way to go (dS) */
			SLeft = fabs(GoalL - DriveL);
			SRight = fabs(GoalR - DriveR);
			dS = (SLeft + SRight) / 2.0f;
			SpeedLimit = sqrt(2.0f * Deceleration * dS);	// SpeedLimit will decrease every cycle (v=sqrt(2*a*s))

			/* Decrease current Speed by setting it equal the SpeedLimit when it is higher then the calculated SpeedLimit */
			if (Speed > SpeedLimit)
			{
				Speed = SpeedLimit;
				if (Speed < 0) Speed = 0;
			}
		/* End of breakramp */

			/* Calculate the speed each wheel shall drive with, on base of each way to go */
			SpeedL = 2.0f * Speed * SLeft / (SLeft + SRight);
			SpeedR = 2.0f * Speed * SRight / (SLeft + SRight);

		/* Left wheel distance */
			/* Check if the way driven so far is smaller then the total distance... */
			if (DriveL < GoalL)
			{
				/* Increase the distances the wheel shall drive with the next little piece of the route */
				DriveL += SpeedL * dt;		// The wheel will drive forward

				/* Set GoalL when the left wheel will drive over the goal to achieve an exact precision landing */
				if( DriveL > GoalL ) DriveL = GoalL;
			}
			/* ... or check if the way driven so far is bigger then the total distance */
			else if (DriveL > GoalL)
			{
				/* Decrees the distances the wheel shall drive with the next little piece of the route */
				DriveL -= SpeedL * dt;		// The wheel will drive backward

				/* Set GoalL when the left wheel will drive over the goal to achieve an exact precision landing */
				if( DriveL < GoalL ) DriveL = GoalL;
			}
		/*End of left wheel distance */

		/* Right wheel distance */
			/* Check if the way driven so far is smaller then the total distance... */
			if (DriveR < GoalR)
			{
				/* Increase the distances the wheel shall drive with the next little piece of the route */
				DriveR += SpeedR * dt;		// The wheel will drive forward

				/* Set GoalR when the right wheel will drive over the goal to achieve an exact precision landing */
				if( DriveR > GoalR ) DriveR = GoalR;
			}
			/* ... or check if the way driven so far is bigger then the total distance */
			else if (DriveR > GoalR)
			{
				/* Decrees the distances the wheel shall drive with the next little piece of the route */
				DriveR -= SpeedR * dt;		// The wheel will drive backward

				/* Set GoalR when the right wheel will drive over the goal to achieve an exact precision landing */
				if( DriveR < GoalR ) DriveR = GoalR;
			}
		/*End of right wheel distance */

			/* Check if the current driving command is finished now */
			if(DriveL == GoalL && DriveR == GoalR)
			{
				taskENTER_CRITICAL();
				{
					setDriveFinished(pdTRUE);		// setDriveFinished() is declared in drivemanager.h
				}
				taskEXIT_CRITICAL();
			}
		}

	/* Send the current reference input to the position-controller (ISRmotorcontrol) */
		/* The new distances have to be converted into counts first to give them to the position-controller */
		DistanceInCountsL = ConvDistanceToCounts(DriveL);
		DistanceInCountsR = ConvDistanceToCounts(DriveR);

		taskDISABLE_INTERRUPTS();
		{
			set_PID_PHI_L_w(DistanceInCountsL);
			set_PID_PHI_R_w(DistanceInCountsR);
		}
		taskENABLE_INTERRUPTS();

		/* Cycle-Time */
		vTaskDelayUntil( &xLastCycleStart, DrivingTaskDelay );	// wait for exactly 10ms
	}
}

/****************************************************************************
*  Procedure  : ConvDistanceToCounts
*****************************************************************************
*  Function   : This function converts a distance to counts on base of the
*  				defined robot parameters in config.h.
*  				(counts = NbrOfWheelTruns * CountsPerTrunWheel)
*
*  Input Para : distance in m
*
*  Return	  : counts
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 30.11.2013
*
****************************************************************************/
float ConvDistanceToCounts( float distance )
{
	float counts;
	return counts = (distance / (M_PI * WheelDiameter)) * TranslationWheelToMot * (float)CountsPerTurnMotEnc;
}

/****************************************************************************
*  Procedure  : setStopFlag
*****************************************************************************
*  Function   : This function sets or resets the StopFlag.
*
*  Input Para : Status of the StopFlag
*				(pdTRUE = 1 = set, pdFALSE = 0 = reset)
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 30.11.2013
*
****************************************************************************/
inline void setStopFlag( int status )
{
	StopFlag = status;
}

/****************************************************************************
*  Procedure  : getStopFlag
*****************************************************************************
*  Function   : This function gets the status of the StopFlag.
*
*  Input Para : -
*
*  Return	  : Status of the StopFlag
*				(pdTRUE = 1 = set, pdFALSE = 0 = reset)
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 06.12.2013
*
****************************************************************************/
inline int getStopFlag( void )
{
	return StopFlag;
}
