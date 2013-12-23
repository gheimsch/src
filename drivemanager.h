/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : drivemanager.h
*****************************************************************************
*  Function   : Header for the task to manage the driving of the robot. It
*  				gets the required data (XGoal, YGoal, PhiGoal, V) via
*  				CAN message and	calculates the way each wheel have to lay
*  				back.
*
*  Procedures : initDrivemanagerTask()
*  				Rotate()
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

#ifndef DRIVEMANAGER_H_
#define DRIVEMANAGER_H_

/*-------------------------------defines-----------------------------------*/
/* Task stack size */
#define managerSTACK_SIZE ( configMINIMAL_STACK_SIZE )

/* Task delays */
#define ManagerTaskDelay ( 100 )	// in ms

/* Max. number of GoTo-commands we can receive */
#define MaxNumberOfGoTos ( 10 )

/* Degree to radiant conversion */
#define convDegToRad( x )  ( M_PI/180.0f * (x) )

/* Distance of 1 bit we receive via CAN */
#define DistancePerBit ( 0.00586f )				// 1Bit = 5.86mm
#define DistancePerBitSupportPoint ( 0.02343f )	// 1Bit = 23.43mm

/*----------------------exported (global) functions-------------------------*/
extern void initDrivemanagerTask( unsigned portBASE_TYPE );
extern inline float getGlobalGoalL( void );
extern inline float getGlobalGoalR( void );
extern inline float getGlobalSpeed( void );
extern inline void setDriveFinished( int );
extern inline int getDriveFinished( void );

#endif
