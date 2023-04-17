/*
 * board1.h
 *
 *  Created on: 2023. 4. 17.
 *      Author: RIMLAB
 */

#ifndef BOARD1_H_
#define BOARD1_H_


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
//
// PinMux Configurations
//
//*****************************************************************************

//
// CANB -> myCAN0 Pinmux
//
//
// CANRXB - GPIO Settings
//
#define GPIO_PIN_CANRXB 17
#define myCAN0_CANRX_GPIO 17
#define myCAN0_CANRX_PIN_CONFIG GPIO_17_CANRXB
//
// CANTXB - GPIO Settings
//
#define GPIO_PIN_CANTXB 12
#define myCAN0_CANTX_GPIO 12
#define myCAN0_CANTX_PIN_CONFIG GPIO_12_CANTXB

//
// SCIA -> mySCI0 Pinmux
//
//
// SCIRXDA - GPIO Settings
//
#define GPIO_PIN_SCIRXDA 43
#define mySCI0_SCIRX_GPIO 43
#define mySCI0_SCIRX_PIN_CONFIG GPIO_43_SCIRXDA
//
// SCITXDA - GPIO Settings
//
#define GPIO_PIN_SCITXDA 42
#define mySCI0_SCITX_GPIO 42
#define mySCI0_SCITX_PIN_CONFIG GPIO_42_SCITXDA

//*****************************************************************************
//
// CAN Configurations
//
//*****************************************************************************
#define myCAN0_BASE CANB_BASE

#define myCAN0_MessageObj1_ID 1
#define myCAN0_MessageObj2_ID 0
void myCAN0_init();


//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
#define myCPUTIMER0_BASE CPUTIMER0_BASE
void myCPUTIMER0_init();

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************

// Interrupt Settings for INT_myCAN0_0
#define INT_myCAN0_0 INT_CANB0
#define INT_myCAN0_0_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP9
extern __interrupt void INT_myCAN0_0_ISR(void);

// Interrupt Settings for INT_myCAN0_1
#define INT_myCAN0_1 INT_CANB1
#define INT_myCAN0_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP9
extern __interrupt void INT_myCAN0_1_ISR(void);

// Interrupt Settings for INT_myCPUTIMER0
#define INT_myCPUTIMER0 INT_TIMER0
#define INT_myCPUTIMER0_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1
extern __interrupt void INT_myCPUTIMER0_ISR(void);

// Interrupt Settings for INT_mySCI0_RX
#define INT_mySCI0_RX INT_SCIA_RX
#define INT_mySCI0_RX_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP9
extern __interrupt void INT_mySCI0_RX_ISR(void);

// Interrupt Settings for INT_mySCI0_TX
#define INT_mySCI0_TX INT_SCIA_TX
#define INT_mySCI0_TX_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP9
extern __interrupt void INT_mySCI0_TX_ISR(void);

//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
#define mySCI0_BASE SCIA_BASE
#define mySCI0_BAUDRATE 115200
#define mySCI0_CONFIG_WLEN SCI_CONFIG_WLEN_8
#define mySCI0_CONFIG_STOP SCI_CONFIG_STOP_ONE
#define mySCI0_CONFIG_PAR SCI_CONFIG_PAR_NONE
#define mySCI0_FIFO_TX_LVL SCI_FIFO_TX0
#define mySCI0_FIFO_RX_LVL SCI_FIFO_RX1
void mySCI0_init();

//*****************************************************************************
//
// Board Configurations
//
//*****************************************************************************
void    Board_init();
void    CAN_init();
void    CPUTIMER_init();
void    INTERRUPT_init();
void    SCI_init();
void    PinMux_init();


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif




#endif /* BOARD1_H_ */
