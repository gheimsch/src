/****************************************************************************
*  o o o o      Berner Fachhochschule
*        :...o  Technik und Informatik
*****************************************************************************
*  File       : inits.h
*****************************************************************************
*  Function   : Header for the functions to initialize the I/O's of the uC.
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

#ifndef INITS_H_
#define INITS_H_

/*-------------------------------defines-----------------------------------*/
#define PWMPulseValue ( 500 )
#define PWMPulseOffset	100

/*----------------------exported (global) functions-------------------------*/
extern void initLEDs( void );
extern void initCurrentMeasure( void );
extern void initPWMoutputs( void );
extern void initEncoder( void );

#endif
