/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : driving.h
*****************************************************************************
*  Function   : Header for the driving task. It cuts the received distances
*  				from the task vDriveManagerTask into smaller pieces and gives
*  				them to the ISRmotorcontrol. Further it contains a start-, a
*			  	break- and a stop-ramp.
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

#ifndef DRIVING_H_
#define DRIVING_H_

/*-------------------------------defines-----------------------------------*/
/* Task stack size */
#define drivingSTACK_SIZE ( configMINIMAL_STACK_SIZE )

/* Maximum accelerations (StaticFrictionCoeff is defined in config.h) */
#define AccelerationOfGravity ( 9.81f )
#define Acceleration ( StaticFrictionCoeff * AccelerationOfGravity )	// a = u * g
#define Deceleration ( Acceleration )

/* Task delays */
#define DrivingTaskDelay ( 10 )						// in ms
#define dt ( (float)DrivingTaskDelay / 1000.0f )	// Time per DrivingCycle in s
#define MaxTimeToStop ( TopSpeed / Deceleration * 1000.0f )	// in ms (TopSpeed is defined in config.h)

/*----------------------exported (global) function-------------------------*/
extern void initDrivingTask( unsigned portBASE_TYPE );
extern inline void setStopFlag( int );
extern inline int getStopFlag( void );

#endif
