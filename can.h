/**
 * @file    can.h
 * @author  Schüpbach Simon
 * @author  Simon Grossenbacher
 * @date    28.11.2013
 *
 * @version 2.0
 *  realise the id-switch with void pointer
 *
 * @version 1.0
 *  create this file
 *
 * @brief   this file contains function for the can handling. The function
 * initCanMessenger generate two Task, one for the transmitting and one for
 * the receiving of CAN-Messages. If a message is receiving the CAN_RX0_IRQHandler
 * is called.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H

/* exported typedef ----------------------------------------------------------*/
typedef void (*CANRxCatch_t)(CanRxMsg); /* function-handler to application-area (callback-function) */


/* exported define -----------------------------------------------------------*/
/* Pin and Port of the CAN-Interface */
#define CAN_PIN_TX_NUMBER       9
#define CAN_PIN_RX_NUMBER       8
#define CAN_PIN_EN_NUMBER		2
#define CAN_PINS_MODE           GPIO_Mode_AF
#define CAN_PIN_EN_MODE         GPIO_Mode_OUT
#define CAN_PINS_SPEED          GPIO_Speed_50MHz
#define CAN_PINS_TYPE           GPIO_OType_PP
#define CAN_PINS_PUPD           GPIO_PuPd_UP
#define CAN_PORT_LETTER         'B'
#define CAN_EN_PORT_LETTER		'D'
#define CAN_INTERFACE_NUMBER    1 /**< 1 or 2 */
#define CAN_BAUD                250 /*kBit/s Baudrate */
#define CAN_FILTER              14 /**< Amount of filter banks -> max. 14 */

#if CAN_INTERFACE_NUMBER == 1
    #ifndef CAN_INTERFACE
        #define CAN_INTERFACE     CAN1
    #endif
#endif
#if CAN_INTERFACE_NUMBER == 2
    #ifndef CAN_INTERFACE
        #define CAN_INTERFACE     CAN2
    #endif
#endif

/* exported types ------------------------------------------------------------*/


/* exported constants --------------------------------------------------------*/


/* exported macro ------------------------------------------------------------*/


/* exported functions --------------------------------------------------------*/
extern void initCAN(CANRxCatch_t);
extern uint8_t setCANFilter(uint16_t);

#endif
