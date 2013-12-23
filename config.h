/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : config.h
*****************************************************************************
*  Function   : Header to define the configuration parameters. It
*  				contains all the essential mechanical parameters and start
*  				conditions. These parameters have to be modified for every
*  				robot individually.
*
*  Author     : haldj3
*
*  Version    : V0.1
*
*  History    : 30.11.2013  File created
*
****************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

/* First start point and start orientation of the robot */
/* RED */
#define FirstXStart ( 1.0f )			// in m
#define FirstYStart ( 0.2f )			// in m
#define FirstPhiStart ( 90.0f )			// in °

/* YELLOW */
//#define FirstXStart ( 1.5f )			// in m
//#define FirstYStart ( 0.02f )			// in m
//#define FirstPhiStart ( 90.0f )			// in °

/* Max. Speed the robot can drive with */
#define TopSpeed ( 1.0f )				// in m/s

/* Static friction coefficient of the wheels on the table to calculate the acceleration (a = u*g) */
#define StaticFrictionCoeff ( 0.5f )	// u = F/Fg ~= 0.7 (rubber on wood) -> we assume 0.5 to be on the safe side

/* Translation of the wheel to the motor (encoder) */
#define TranslationWheelToMot ( 17.4f )	// Translation of the motor gear * Translation of the drive unit

/* Distance between the two drive wheels in m */
#define WheelDistance ( 0.293f )

/* Diameter of the drive wheels in m */
#define WheelDiameter ( 0.072f )

/* Counts per turn of the motor encoder */
#define CountsPerTurnMotEnc ( 500 * 4 )		// Multiplied with 4 because there are 4 edges per count

///* Distance between the two odometry wheels in m */
//#define WheelDistanceOdo ( 0.271f )
//
///* Diameter of the odometry wheels in m */
//#define WheelDiameterOdo ( 0.0168f )
//
///* Counts per turn of the odometry encoder */
//#define CountsPerTurnOdoEnc ( 256 * 4 )	// Multiplied with 4 because there are 4 edges per count

/* Motor catalog values (for the controller) */
#define ArmatureResistor ( 1.53f )			// in Ohm 		(little robot: 0.98f) ?
#define MaxNumberOfRevolutions ( 7390 )		// in rpm 		(little robot: 10400)
#define TorqueConstant ( 39.8f )			// in mNm/A 	(little robot: 16.1f)
#define CharacteristicCurvesSlope ( 8.61f )	// in rpm/mNm 	(little robot: 35.9f)

#endif
