/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : drivemanager.c
*****************************************************************************
*  Function   : This file contains the task to manage the driving of the
*  				robot. It gets the required data (XGoal, YGoal, PhiGoal, V)
*  				via CAN message and	calculates the way each wheel have to lay
*  				back.
*
*  Procedures : Rotate()
*  				Drive()
*				convSpeedInPercentToSpeed()
*				getGlobalGoalL()
*				getGlobalGoalR()
*				getGlobalSpeed()
*				setDriveFinished()
*				getDriveFinished()
*				CANGoToHandler()
*				CANStopHandler()
*				CANStateRequestHandler()
*
*  Tasks 	  : vDriveManagerTask
*
*  Author     : ruefd1, haldj3
*
*  Version    : V0.3
*
*  History    : 01.07.2013  File created
*  				30.11.2013  new ManagerTask and functions
*  				06.12.2013	implemented CAN functions
*
*  TODO		  : Build flag status "DriveFinished" as a semaphore.
*  				Build data transfer with queues
*
****************************************************************************/

/*------------------------------includes------------------------------------*/
/* Kernel includes */
#include "FreeRTOS.h"		//must appear before #include task.h
#include "task.h"

/* Hardware and starter kit includes */
#include "stm32f4xx.h"

/* User includes */
#include "config.h"
#include "CANGatekeeper.h"
#include "driving.h"
#include "drivemanager.h"

#include <math.h>		// to use atan2, sqrt and M_PI

/*-------------------------------variables----------------------------------*/
volatile static float GlobalGoalL = 0.0f;		// Goal distance in m for left wheel
volatile static float GlobalGoalR = 0.0f;		// Goal distance in m for right wheel
volatile static float GlobalSpeed = 0.0f;		// in m/s
volatile static int DriveFinished = pdTRUE;		// Flag to check if the robot has reached the required end position

/* Number of GoTo commands receiving via CAN */
static int NumberOfGoTos = 0;

/* Arrays with space for new positions, orientations, speeds and support points receiving via CAN*/
static float NextXGoal[MaxNumberOfGoTos];
static float NextYGoal[MaxNumberOfGoTos];
static uint16_t NextPhiGoal[MaxNumberOfGoTos];
static uint8_t NextSpeed[MaxNumberOfGoTos];

static float NextSupportPointX1[MaxNumberOfGoTos];
static float NextSupportPointY1[MaxNumberOfGoTos];
static float NextSupportPointX2[MaxNumberOfGoTos];
static float NextSupportPointY2[MaxNumberOfGoTos];

/*-------------------------------prototypes---------------------------------*/
static portTASK_FUNCTION_PROTO( vDriveManagerTask, pvParameters );

static void Rotate(float, float, float);
static void Drive(float, float, float);

static float convSpeedInPercentToSpeed(float);

static void CANGoToHandler(uint16_t, CAN_data_t*);
static void CANStopHandler(uint16_t, CAN_data_t*);
static void CANStateRequestHandler(uint16_t, CAN_data_t*);

/*--------------------------------------------------------------------------*/

/****************************************************************************
*  Procedure  : initDrivemanagerTask
*****************************************************************************
*  Function   : This task initializes the drive manager task and sets the
*  				functions that shall be called wehn receiving a GoTo- or
*  				Stop-message. It will be only called once.
*
*  Input Para : Drivemanager task priority
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 13.12.2013
*
****************************************************************************/
void initDrivemanagerTask(unsigned portBASE_TYPE uxPriority )
{
	/* Set the functions that shall be called when receiving defined CAN-messages */
	setFunctionCANListener(CANGoToHandler, GOTO_XY);
	setFunctionCANListener(CANStopHandler, STOP_DRIVE);
	setFunctionCANListener(CANStopHandler, EMERGENCY_STOP);
	setFunctionCANListener(CANStateRequestHandler, GOTO_STATE_REQUEST);

	/* Spawn the task */
	xTaskCreate( vDriveManagerTask, ( signed char * ) "DriveManager", managerSTACK_SIZE, NULL, uxPriority, ( xTaskHandle * ) NULL );
}

/****************************************************************************
*  Task 	  : vDriveManagerTask
*****************************************************************************
*  Function   : This task gets the desired data (XGoal, YGoal, PhiGoal, V)
*  				via CAN	message. It scales the start and goal angle and calls
*  				the	functions Rotate() and Drive().
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 30.11.2013
*
****************************************************************************/
portTASK_FUNCTION( vDriveManagerTask, pvParameters )
{
	/* DriveManager variables */
	static float XStart = FirstXStart;		// in m (FirstXStart is defined in conifg.h)
	static float YStart = FirstYStart;		// in m (FirstYStart is defined in conifg.h)
	static float PhiStart = FirstPhiStart;	// in ° (FirstPhiStart is defined in conifg.h)
	static float XGoal;						// in m
	static float YGoal;						// in m
	static float PhiGoal;					// in °

	static float Speed;						// will be in m/s
	static float dX;						// will be in m
	static float dY;						// will be in m
	static float Phi;						// will be in rad

	/* The task-function-parameters are not used */
	( void ) pvParameters;

	/*--------------------------------------------------------------------------*/
	/* To drive a predefined route comment this for-loop in and the other out and
	 * call Rotate() and/or Drive() individually */
	for( ;; )
	{
		Speed = 0.5f;						// in m/s

		Drive(0.0f, 100.0f, Speed);			// Drive 1m straight in pos-y-direction
//		Rotate((M_PI/2), -(M_PI/2), Speed);	// Rotate 180° (from 90° to -90°)
//		Drive(0.0f, 1.0f, Speed);			// Drive 1m straight in neg-y-direction
//		Rotate(-(M_PI/2), (M_PI/2), Speed);	// Rotate -180° (from -90° to 90°)

/* geht so noch i-wie nicht -> kommt zwar zum breakpoint, aber fährt dann nicht */
		/*while(NumberOfGoTos == 0)
		{
			vTaskDelay( (portTickType)ManagerTaskDelay );		// wait for ~100ms
		}

		Rotate((1.57079637f), (0.98433423f), Speed);
		Drive(0.4f, 0.6f, Speed);
		NumberOfGoTos--;
		Rotate((0.98433423f), (3.14159274f), Speed);*/


//		while(1)
//		{
//			vTaskDelay((portTickType)10000);
//		}
	}
	/*--------------------------------------------------------------------------*/

//	for( ;; )
//	{
//		/* Wait until at least one GoTo-command */
//		while(NumberOfGoTos == 0)
//		{
//			vTaskDelay( (portTickType)ManagerTaskDelay );		// wait for ~100ms
//		}
//
//		/* Get next position, orientation and speed */
//		/* While saving current data, interrupts have to be disabled to prevent a change of NumberOfGoTos */
//		taskDISABLE_INTERRUPTS();
//		{
//			XGoal = NextXGoal[NumberOfGoTos-1];		// in m
//			YGoal = NextYGoal[NumberOfGoTos-1];		// in m
//			PhiGoal = NextPhiGoal[NumberOfGoTos-1];	// in °
//			Speed = NextSpeed[NumberOfGoTos-1];		// in %
//		}
//		taskENABLE_INTERRUPTS();
//
//		/* Convert SpeedInPercent to the actual Speed in m/s */
//		Speed = convSpeedInPercentToSpeed(Speed);
//
//		 /* Check if already at the goal */
//		if (XStart != XGoal && YStart != YGoal)
//		{
//			/* Calculate drive direction angle */
//			dX = XGoal - XStart;
//			dY = YGoal - YStart;
//			Phi = atan2f(dY, dX);						// in rad (-180° < Phi <= 180°)
//
//		/* Start-Rotation */
//			/* Scale start angle if necessary */
//			if (PhiStart > 180.0f) PhiStart -= 360.0f;	// -180° < PhiStart <= 180°
//
//			/* Rotate from PhiStart to Phi (convDegToRad is defined in drivemanager.h) */
//			Rotate(convDegToRad(PhiStart), Phi, Speed);
//		/* End Start-Rotation */
//
//		/* Drive */
//			/* Drive to the goal */
//			Drive(dX, dY, Speed);
//
//			/* Set new start point and decrease number of GoTo-commands, since the robot has reached the goal */
//			XStart = XGoal;
//			YStart = YGoal;
//			if (NumberOfGoTos > 0) NumberOfGoTos--;	// NumberOfGoTos could already be 0 after a Stopramp
//		/* End Drive */
//		}
//
//	/* Goal-Rotation */
//		/* Scale goal angle if necessary */
//		if (PhiGoal > 180.0f) PhiGoal -= 360.0f;	// -180° < PhiGoal <= 180°
//
//		/* Rotate from Phi to PhiGoal */
//		Rotate(Phi, convDegToRad(PhiGoal), Speed);
//
//		/* Set new start angle */
//		PhiStart = PhiGoal;
//	/* End Goal-Rotation */
//
//		/* Reset StopFlag */
//		taskENTER_CRITICAL();
//		{
//			setStopFlag(pdFALSE);	// Declared in driving.h
//		}
//		taskEXIT_CRITICAL();
//	}
}

/****************************************************************************
*  Procedure  : Rotate
*****************************************************************************
*  Function   : This function calculates the distance the right and the left
*  				wheel have to lay back when rotating. It sends these
*  				distances and the given speed to the DrivingTask and waits
*  				until this driving has finished.
*
*  				dPhi > 0 -> Rotate left (against clockwise)
*  				dPhi < 0 -> Rotate right (clockwise)
*
*  Input Para : 1. Angle in radiant the robot is looking at at the moment
*  				   (it have to be between -180° and 181°)
*  				2. Angle in radiant the robot shall look at
*  				   (it have to be between -180° and 181°)
*  				3. Speed in m/s
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.2
*
*  History    : 30.11.2013
*  				13.12.2013  changed dPhi in ° to rad
*
****************************************************************************/
static void Rotate( float Phi1, float Phi2, float V)
{
	static float distanceL;	// in m
	static float distanceR;	// in m
	static float dPhi;		// in rad

	dPhi = Phi2 - Phi1;		// in rad (-360° < dPhi < 360°)

	/* Do not rotate when dPhi = 0 or StopFlag is set */
	if (dPhi != 0 && !getStopFlag())
	{
		/* dPhi have to be > 0 when between -360° and -180° to rotate left
		 * and < 0 when between 180° and 360° to rate right */
		if (dPhi > M_PI) dPhi -= (2*M_PI);
		if (dPhi < -M_PI) dPhi += (2*M_PI);

		/* Calculates the distance in m the wheels have to lay back (WheelDistance is defined in config.h) */
		distanceR = (WheelDistance / 2.0f) * dPhi;		// b = r * dPhi[rad]
		distanceL = -distanceR;

		/* Send the new distance to the DrivingTask. The new distance has to
			be added to the old one to get the new absolute distance */
		taskENTER_CRITICAL();
		{
			GlobalGoalR += distanceR;
			GlobalGoalL += distanceL;

			GlobalSpeed = V;

			DriveFinished = pdFALSE;
		}
		taskEXIT_CRITICAL();

		/* Wait until drive finished */
		while(DriveFinished == pdFALSE)
		{
			vTaskDelay( (portTickType)DrivingTaskDelay );	// wait for ~10ms (DrivingTaskDelay is defined in drving.h)
		}
	}
}

/****************************************************************************
*  Procedure  : Drive
*****************************************************************************
*  Function   : This function calculates the distance the right and the left
*  				wheel have to lay back when driving straight to a defined
*  				point. It sends these distances and the speed to the
*  				DrivingTask and waits until the driving has finished.
*
*  Input Para : 1. X-component of a straight distance in m
*  				2. Y-component of a straight distance in m
*  				3. Speed in m/s
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
static void Drive( float dX, float dY, float V )
{
	static float distanceL;	// in m
	static float distanceR;	// in m

	/* Do not drive when already at the goal or StopFlag is set */
	if ((dX > 0 || dY > 0) && !getStopFlag())
	{
		/* Calculates the distance in m the wheels have to lay back */
		distanceR = sqrt(dX*dX + dY*dY);
		distanceL = distanceR;

		/* Send the new distance to the DrivingTask. The new distance has to
			be added to the old one to get the new absolute distance */
		taskENTER_CRITICAL();
		{
			GlobalGoalR += distanceR;
			GlobalGoalL += distanceL;

			GlobalSpeed = V;

			DriveFinished = pdFALSE;
		}
		taskEXIT_CRITICAL();

		/* Wait until drive finished */
		while(DriveFinished == pdFALSE)
		{
			vTaskDelay( (portTickType)DrivingTaskDelay );	// wait for ~10ms (DrivingTaskDelay is defined in drving.h)
		}
	}
}

/****************************************************************************
*  Procedure  : convSpeedInPercentToSpeed
*****************************************************************************
*  Function   : This function converts the SpeedInPercent given via CAN to
*  				the Speed the robot shall drive with in m/s. It's calculated
*  				with the TopSpeed the robot can max. drive with defined in
*  				config.h.
*
*  Input Para : Speed in %
*
*  Return	  : Speed in m/s
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 06.12.2013
*
****************************************************************************/
static float convSpeedInPercentToSpeed( float SpeedInPercent )
{
	static float Speed;
	Speed = TopSpeed * SpeedInPercent / 100.0f;		// TopSpeed is defined in config.h
	return Speed;
}

/****************************************************************************
*  Procedure  : getGlobalGoalL
*****************************************************************************
*  Function   : This function gets the variable GlobalGoalL.
*  				It's the new total distance the left wheel have to lay back.
*
*  Input Para : -
*
*  Return	  : GlobalGoalL in m
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
inline float getGlobalGoalL( void )
{
	return GlobalGoalL;
}

/****************************************************************************
*  Procedure  : getGlobalGoalR
*****************************************************************************
*  Function   : This function gets the variable GlobalGoalR.
*  				It's the new total distance the right wheel have to lay back.
*
*  Input Para : -
*
*  Return	  : GlobalGoalR in m
*
*  Author     : rufed1
*
*  Version    : V0.1
*
*  History    : 01.07.2013
*
****************************************************************************/
inline float getGlobalGoalR( void )
{
	return GlobalGoalR;
}

/****************************************************************************
*  Procedure  : getGlobalSpeed
*****************************************************************************
*  Function   : This function gets the variable GlobalSpeed.
*  				It's the speed the robot shall drive with.
*
*  Input Para : -
*
*  Return	  : GlobalSpeed in m/s
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 30.11.2013
*
****************************************************************************/
inline float getGlobalSpeed( void )
{
	return GlobalSpeed;
}

/****************************************************************************
*  Procedure  : setDriveFinished
*****************************************************************************
*  Function   : This function sets the Flag DriveFinished.
*
*  Input Para : Status that declares if the robot is still driving or not
*				(pdTRUE = 1 = driving, pdFALSE = 0 = standstill)
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
inline void setDriveFinished( int status )
{
	DriveFinished = status;
}

/****************************************************************************
*  Procedure  : getDriveFinished
*****************************************************************************
*  Function   : This function gets the status of the DriveFinished flag.
*
*  Input Para : -
*
*  Return	  : Status that declares if the robot is still driving or not
*				(pdTRUE = 1 = driving, pdFALSE = 0 = standstill)
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 30.11.2013
*
****************************************************************************/
inline int getDriveFinished( void )
{
	return DriveFinished;
}

/****************************************************************************
*  Procedure  : CANGoToHandler
*****************************************************************************
*  Function   : This function gets called whenever we receive a GoTo-command.
*  				It counts the number of GoTos, safes the received values
*  				in arrays and sends the GoTo-received-acknowledge to confirm
*  				a proper data transfer via CAN.
*
*  Input Para : 1. not used -> Corresponding CAN-message ID (GOTO_XY)
*  				2. Pointer on the address of the Data-packet of the
*  				   CAN-message
*  				(defined in CANGatekeeper.h)
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 06.12.2013
*
****************************************************************************/
static void CANGoToHandler(uint16_t id, CAN_data_t* data)
{
	/* Increase number of GoTos to trigger the drive manager */
	NumberOfGoTos++;

	/* Do not send GoTo-received-acknowledge when more then max. number of allowed GoTo-commands */
	if (NumberOfGoTos <= MaxNumberOfGoTos)
	{
		/* Write received data in the arrays they belong to (the 'DistancePerBit' are defined in drivemanager.h) */
		taskDISABLE_INTERRUPTS();
		{
			NextXGoal[NumberOfGoTos-1] = (float)(data->goto_x * DistancePerBit);	// in m
			NextYGoal[NumberOfGoTos-1] = (float)(data->goto_y * DistancePerBit);	// in m
			NextPhiGoal[NumberOfGoTos-1] = data->goto_angle;						// in °
			NextSpeed[NumberOfGoTos-1] = data->goto_speed;							// in %

			NextSupportPointX1[NumberOfGoTos-1] = (float)(data->goto_points[0][0] * DistancePerBitSupportPoint);	// in m
			NextSupportPointY1[NumberOfGoTos-1] = (float)(data->goto_points[0][1] * DistancePerBitSupportPoint);	// in m
			NextSupportPointX2[NumberOfGoTos-1] = (float)(data->goto_points[1][0] * DistancePerBitSupportPoint);	// in m
			NextSupportPointY2[NumberOfGoTos-1] = (float)(data->goto_points[1][1] * DistancePerBitSupportPoint);	// in m
		}
		taskENABLE_INTERRUPTS();

		/* GoTo-received-acknowledge */
		txGotoConfirm();
	}
}

/****************************************************************************
*  Procedure  : CANStopHandler
*****************************************************************************
*  Function   : This function gets called whenever we receive a Stop-command.
*  				It resets the NumberOfGoTos and sets the StopFlag.
*
*  Input Para : 1. not used -> Corresponding CAN-message ID (STOP_DRIVE)
*  				2. not used -> Pointer on the address of the Data-packet of
*  				               the CAN-message
*  							   (defined in CANGatekeeper.h)
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 06.12.2013
*
****************************************************************************/
static void CANStopHandler(uint16_t id, CAN_data_t* data)
{
	/* Reset number of GoTos */
	NumberOfGoTos = 0;

	/* Set StopFlag to trigger a Stopramp and to disable following drives or rotations */
	taskENTER_CRITICAL();
	{
		setStopFlag(pdTRUE);		// Declared in driving.h
	}
	taskEXIT_CRITICAL();

	/* Wait until stopped (MaxTimeToStop is defined in driving.h) */
	vTaskDelay( (portTickType)MaxTimeToStop );

	/* Get new position and direction after stopping (The goal values have to be set,
	 * because the start values get their values from the goals automatically after every drive */
	//XGoal = getNaviDataX();
	//YGoal = getNaviDataY();
	//PhiGoal = getNaviDataPhi();
}

/****************************************************************************
*  Procedure  : CANStateRequestHandler
*****************************************************************************
*  Function   : This function gets called whenever we receive a StateRequest-
*  				command. It calculates the remaining time the robot is
*  				driving and sends it via CAN.
*
*  Input Para : 1. not used -> Corresponding CAN-message ID (STOP_DRIVE)
*  				2. not used -> Pointer on the address of the Data-packet of
*  				               the CAN-message
*  							   (defined in CANGatekeeper.h)
*
*  Return	  : -
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 06.12.2013
*
****************************************************************************/
static void CANStateRequestHandler(uint16_t id, CAN_data_t* data)
{
	/* ... */
	//uint32_t time;

	/* ... */
	//txGotoStateResponse(time);
}
