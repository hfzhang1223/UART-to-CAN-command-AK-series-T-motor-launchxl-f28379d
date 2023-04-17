/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "board1.h"

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules.
// Call this function in your application if you wish to do all module
// initialization.
// If you wish to not use some of the initializations, instead of the
// Board_init use the individual Module_inits
//
//*****************************************************************************
void Board_init()
{
    EALLOW;

    PinMux_init();
    CAN_init();
    CPUTIMER_init();
    SCI_init();
    INTERRUPT_init();

    EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
    //
    // PinMux for modules assigned to CPU1
    //

    //
    // CANB -> myCAN0 Pinmux
    //
    GPIO_setPinConfig(GPIO_17_CANRXB);
    GPIO_setPadConfig(myCAN0_CANRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(myCAN0_CANRX_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_12_CANTXB);
    GPIO_setPadConfig(myCAN0_CANTX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(myCAN0_CANTX_GPIO, GPIO_QUAL_ASYNC);

    //
    // SCIA -> mySCI0 Pinmux
    //
    GPIO_setPinConfig(GPIO_43_SCIRXDA);
    GPIO_setPadConfig(mySCI0_SCIRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(mySCI0_SCIRX_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_42_SCITXDA);
    GPIO_setPadConfig(mySCI0_SCITX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(mySCI0_SCITX_GPIO, GPIO_QUAL_ASYNC);


}

//*****************************************************************************
//
// CAN Configurations
//
//*****************************************************************************
void CAN_init(){
    myCAN0_init();
}

void myCAN0_init(){
    CAN_initModule(myCAN0_BASE);
    //
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    //
    CAN_setBitTiming(myCAN0_BASE, 7, 0, 15, 7, 3);
    //
    // Enable CAN Interrupts
    //
    CAN_enableInterrupt(myCAN0_BASE, CAN_INT_IE0);
    CAN_enableGlobalInterrupt(myCAN0_BASE, CAN_GLOBAL_INT_CANINT0);
    //
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 1
    //      Message Identifier: 1
    //      Message Frame: CAN_MSG_FRAME_STD
    //      Message Type: CAN_MSG_OBJ_TYPE_TX
    //      Message ID Mask: 0
    //      Message Object Flags:
    //      Message Data Length: 8 Bytes
    //
    CAN_setupMessageObject(myCAN0_BASE, 1, myCAN0_MessageObj1_ID, CAN_MSG_FRAME_STD,CAN_MSG_OBJ_TYPE_TX, 0, 0,8);
    //
    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 2
    //      Message Identifier: 0
    //      Message Frame: CAN_MSG_FRAME_STD
    //      Message Type: CAN_MSG_OBJ_TYPE_RX
    //      Message ID Mask: 0
    //      Message Object Flags: CAN_MSG_OBJ_RX_INT_ENABLE
    //      Message Data Length: 0 Bytes
    //
    CAN_setupMessageObject(myCAN0_BASE, 2, myCAN0_MessageObj2_ID, CAN_MSG_FRAME_STD,CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,0);
    CAN_setInterruptMux(myCAN0_BASE, 2);
    //
    // Start CAN module operations
    //
    CAN_startModule(myCAN0_BASE);
}

//*****************************************************************************
//
// CPUTIMER Configurations
//
//*****************************************************************************
void CPUTIMER_init(){
    myCPUTIMER0_init();
}

void myCPUTIMER0_init(){
    CPUTimer_setEmulationMode(myCPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setPreScaler(myCPUTIMER0_BASE, 2U);
    CPUTimer_setPeriod(myCPUTIMER0_BASE, 2000000U);
    CPUTimer_enableInterrupt(myCPUTIMER0_BASE);
    CPUTimer_stopTimer(myCPUTIMER0_BASE);

    CPUTimer_reloadTimerCounter(myCPUTIMER0_BASE);
}

//*****************************************************************************
//
// INTERRUPT Configurations
//
//*****************************************************************************
void INTERRUPT_init(){

    // Interrupt Setings for INT_myCAN0_0
    Interrupt_register(INT_myCAN0_0, &INT_myCAN0_0_ISR);
    Interrupt_enable(INT_myCAN0_0);

    // Interrupt Setings for INT_myCAN0_1
    Interrupt_register(INT_myCAN0_1, &INT_myCAN0_1_ISR);
    Interrupt_disable(INT_myCAN0_1);

    // Interrupt Setings for INT_myCPUTIMER0
    Interrupt_register(INT_myCPUTIMER0, &INT_myCPUTIMER0_ISR);
    Interrupt_enable(INT_myCPUTIMER0);

    // Interrupt Setings for INT_mySCI0_RX
    Interrupt_register(INT_mySCI0_RX, &INT_mySCI0_RX_ISR);
    Interrupt_enable(INT_mySCI0_RX);

    // Interrupt Setings for INT_mySCI0_TX
    Interrupt_register(INT_mySCI0_TX, &INT_mySCI0_TX_ISR);
    Interrupt_disable(INT_mySCI0_TX);
}
//*****************************************************************************
//
// SCI Configurations
//
//*****************************************************************************
void SCI_init(){
    mySCI0_init();
}

void mySCI0_init(){
    SCI_clearInterruptStatus(mySCI0_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
    SCI_clearOverflowStatus(mySCI0_BASE);
    SCI_resetTxFIFO(mySCI0_BASE);
    SCI_resetRxFIFO(mySCI0_BASE);
    SCI_resetChannels(mySCI0_BASE);
    SCI_setConfig(mySCI0_BASE, DEVICE_LSPCLK_FREQ, mySCI0_BAUDRATE, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
    SCI_disableLoopback(mySCI0_BASE);
    SCI_performSoftwareReset(mySCI0_BASE);
    SCI_enableInterrupt(mySCI0_BASE, SCI_INT_RXFF);
    SCI_setFIFOInterruptLevel(mySCI0_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1);
    SCI_enableFIFO(mySCI0_BASE);
    SCI_enableModule(mySCI0_BASE);
}

