//#############################################################################
//
// FILE:   JW_HIGHLEVELControl_UI_UARTtoCAN_for_AKseries_T_motor
//         by JUNGWOO HUR_(RIM_LAB/ Sogang University)
//         23.04.09
//
// TITLE:  UART_to_CAN_for_AK_series_T_motor
//
//
//#############################################################################
//
// $Release Date: $
// $Copyright:
// Copyright (C) 2013-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "stdlib.h"
#include "math_ops.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define MSG_DATA_LENGTH_R    0   // "Don't care" for a Receive mailbox
#define MSG_DATA_LENGTH_T    8   // MSG_LENGTH for a Transmit mailbox
#define RX_MSG_OBJ_ID      2   // Use mailbox 2
#define TX_MSG_OBJ_ID      1   // Use mailbox 1

uint16_t rxMsgData[6];
uint16_t txMsgData[8];

volatile uint32_t status;

volatile float p_hat;
volatile float v_hat;
volatile float i_hat;

char* msg;                // Message sent through terminal window
char receivedChar;        // Variable used to track input from the terminal window
uint16_t rxStatus = 0U;   // Variable used to store the status of the SCI RX Register

char chararrp[20];
char chararrv[20];
char chararrk[20];
char chararrd[20];
char chararrt[20];
char* mp = chararrp;
char* mv = chararrv;
char* mk = chararrk;
char* md = chararrd;
char* mt = chararrt;

float result;
int cmdval;

float pbuff;
float vbuff;
float kbuff;
float dbuff;
float tbuff;

float pref;
float vref;
float kref;
float dref;
float tref;

float __float_reg[64];                                                          // Floats stored in flash
int __int_reg[256];


void pack_command(uint16_t* txMsgData, float p_des, float v_des, float kp, float kd, float t_ff){


    // Initialize the transmit message object data buffer to be sent
    //
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
    kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    /// convert floats to unsigned ints ///
    int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    /// pack ints into the can buffer ///
    txMsgData[0] = p_int>>8;
    txMsgData[1] = p_int&0xFF;
    txMsgData[2] = v_int>>4;
    txMsgData[3] = ((v_int&0xF)<<4)|(kp_int>>8);
    txMsgData[4] = kp_int&0xFF;
    txMsgData[5] = kd_int>>4;
    txMsgData[6] = ((kd_int&0xF)<<4)|(t_int>>8);
    txMsgData[7] = t_int&0xff;

    CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH_T, txMsgData);

    //
    // Poll TxOk bit in CAN_ES register to check completion of transmission
    //
    //while(((HWREGH(CANB_BASE + CAN_O_ES) & CAN_ES_TXOK)) !=  CAN_ES_TXOK)
    //{
    //}
}

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]

void unpack_reply(uint16_t * rxMsgData){

    int id = rxMsgData[0]; // ID

    int p_int = (rxMsgData[1]<<8)|rxMsgData[2];
    int v_int = (rxMsgData[3]<<4)|(rxMsgData[4]>>4);
    int i_int = ((rxMsgData[4]&0xF)<<8)|rxMsgData[5];
    /// convert ints to floats ///
    p_hat = uint_to_float(p_int, P_MIN, P_MAX, 16);
    v_hat = uint_to_float(v_int, V_MIN, V_MAX, 12);
    i_hat = uint_to_float(i_int, T_MIN, T_MAX, 12);
}


//initialize c2000 board
void initc2000() {
    //INITIALIZE -------------------------------------------------------------------------------------------------------------------------------------
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Board initialization
    //
    Board_init();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;
    //---------------------
};


__interrupt void INT_myCAN0_1_ISR(void){

    CAN_clearInterruptStatus(CANB_BASE, TX_MSG_OBJ_ID);
    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT1);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

__interrupt void INT_myCAN0_0_ISR(void){
    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(CANB_BASE);

    //
    // Check if the cause is the receive message object 1
    //
    if(status == RX_MSG_OBJ_ID)
    {
        if(((HWREGH(CANB_BASE + CAN_O_ES) & CAN_ES_RXOK)) == CAN_ES_RXOK)
            {
                //
                // Get the received message
                //
                CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID, rxMsgData);
            }
        unpack_reply(rxMsgData);
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID);
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}


__interrupt void INT_mySCI0_TX_ISR(void){
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
__interrupt void INT_mySCI0_RX_ISR(void){
    int i = 0;
    //
    // Read a character from the FIFO.
    //
    receivedChar = SCI_readCharBlockingFIFO(SCIA_BASE);
    //value = atoi(&receivedChar);
    rxStatus = SCI_getRxStatus(SCIA_BASE);
    if((rxStatus & SCI_RXSTATUS_ERROR) != 0)
    {
        //ESTOP0;
        msg = "\r\nError SCI!!!!!!!!!! \r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 25);
    }

    switch(receivedChar){
    case 'm':
        msg = "Enter a Motor Control mode\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 28);
        txMsgData[0] = 0xFF;
        txMsgData[1] = 0xFF;
        txMsgData[2] = 0xFF;
        txMsgData[3] = 0xFF;
        txMsgData[4] = 0xFF;
        txMsgData[5] = 0xFF;
        txMsgData[6] = 0xFF;
        txMsgData[7] = 0xFC;
        CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH_T, txMsgData);
        break;
    case 'e':
        msg = "Exit the Motor Control mode\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 29);
        txMsgData[0] = 0xFF;
        txMsgData[1] = 0xFF;
        txMsgData[2] = 0xFF;
        txMsgData[3] = 0xFF;
        txMsgData[4] = 0xFF;
        txMsgData[5] = 0xFF;
        txMsgData[6] = 0xFF;
        txMsgData[7] = 0xFD;
        CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH_T, txMsgData);
        break;
    case 'z':
        msg = "Set the current position ZERO\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 32);
        txMsgData[0] = 0xFF;
        txMsgData[1] = 0xFF;
        txMsgData[2] = 0xFF;
        txMsgData[3] = 0xFF;
        txMsgData[4] = 0xFF;
        txMsgData[5] = 0xFF;
        txMsgData[6] = 0xFF;
        txMsgData[7] = 0xFE;
        CAN_sendMessage(CANB_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH_T, txMsgData);
        break;
    case 'p':
        msg = "Position Reference[radian](-4pi to 4Pi): ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 42);
        do{
            chararrp[i] = SCI_readCharBlockingFIFO(SCIA_BASE);//save to the array
            //
            // Echo back the character.
            //
            SCI_writeCharNonBlocking(SCIA_BASE,chararrp[i]);
            i++;
        }while((int)(chararrp[i-1]) != 13);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);
        chararrp[i-1] = '\0';
        pbuff = atof(chararrp);
        break;
    case 'v':
        msg = "Velocity reference[radian/s](-30 to 30): ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 42);
        do{
            chararrv[i] = SCI_readCharBlockingFIFO(SCIA_BASE);//save to the array
            //
            // Echo back the character.
            //
            SCI_writeCharNonBlocking(SCIA_BASE,chararrv[i]);
            i++;
        }while((int)(chararrv[i-1]) != 13);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);
        chararrv[i-1] = '\0';
        vbuff = atof(chararrv);
        break;
    case 'k':
        msg = "Kp Impedance[N-m/rad](0 to 500): ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 34);
        do{
            chararrk[i] = SCI_readCharBlockingFIFO(SCIA_BASE);//save to the array
            //
            // Echo back the character.
            //
            SCI_writeCharNonBlocking(SCIA_BASE,chararrk[i]);
            i++;
        }while((int)(chararrk[i-1]) != 13);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);
        chararrk[i-1] = '\0';
        kbuff = atof(chararrk);
        break;
    case 'd':
        msg = "Kd Impedance[N-m*s/rad](0 to 100): ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 36);
        do{
            chararrd[i] = SCI_readCharBlockingFIFO(SCIA_BASE);//save to the array
            //
            // Echo back the character.
            //
            SCI_writeCharNonBlocking(SCIA_BASE,chararrd[i]);
            i++;
        }while((int)(chararrd[i-1]) != 13);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);
        chararrd[i-1] = '\0';
        dbuff = atof(chararrd);
        break;
    case 't':
        msg = "feed forward torque reference[N-m](-18 to 18): ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 48);
        do{
            chararrt[i] = SCI_readCharBlockingFIFO(SCIA_BASE);//save to the array
            //
            // Echo back the character.
            //
            SCI_writeCharNonBlocking(SCIA_BASE,chararrt[i]);
            i++;
        }while((int)(chararrt[i-1]) != 13);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);
        chararrt[i-1] = '\0';
        tbuff = atof(chararrt);
        break;
    case 's':

        pref = pbuff;
        vref = vbuff;
        kref = kbuff;
        dref = dbuff;
        tref = tbuff;

        msg = "\r\n\r\n\r\nCAN Command\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 20);

        msg = "Position[rad]: ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)mp, 5);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);

        msg = "Velocity[rad/s]: ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 18);
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)mv, 5);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);

        msg = "Kp[N-m/rad]: ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 14);
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)mk, 5);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);

        msg = "Kd[N-m*s/rad]: ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 16);
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)md, 5);
        msg = "\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 2);

        msg = "Torque_feed_forward[N-m]: ";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 27);
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)mt, 5);
        msg = "\r\n\r\n\r\n\r\n\r\n";
        SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 10);




        i = 0;
        //initialize values(Impedance is the same) to Zero(default)
        pbuff = 0;
        vbuff = 0;
        tbuff = 0;
        while(i<20){
            chararrp[i] = '0';
            chararrv[i] = '0';
            //chararrk[i] = '0';
            //chararrd[i] = '0';
            chararrt[i] = '0';
            i++;
        };
        break;
    }
    //
    // Acknowledge this interrupt located in group 9
    //

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


__interrupt void INT_myCPUTIMER0_ISR(void)
{
    pack_command(txMsgData, pref, vref, kref, dref, tref);

    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}


//
// Main
//
void main(void)
{



    initc2000();
    //
    // Send starting message.
    //
    msg = "\n\n\nHello World! Enter a command for position, velocity, impedance(z) and feedforward torque\r\n\n\n";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 96);

    DEVICE_DELAY_US(1000);

    msg = "Type 'm' 'e' 'z' 'p' 'v' 'k' 'd' 't' 's'\r\n\r\n";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 45);
    msg = "'m': Enter a Motor Control mode\r\n'e': Exit the Motor Control mode\r\n'z': Set the current position of the motor to 0\r\n\r\n";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 119);

    msg = "'p': Position reference[radian]\r\n'v': Velocity reference[radian/s]\r\n'k': Kp Impedance[N-m/rad]\r\n'd': Kd Impedance[N-m*s/rad]\r\n't': feed forward torque reference[N-m]\r\n's': Send Command through CAN\r\n";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 199);
    msg = "\n[Default command value is Zero, except gains]\r\n";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 49);
    //msg = "(You should always type 3 numbers, ex) -1.23 = type - 1 2 3)\n\n\n";
    //SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 65);
    msg = "\n1. Type a character command 2. Type a reference value 3. Type an enter key\r\n";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 78);
    msg = "\nIf all command array is set, Type 's' to send!!!\r\n";
    SCI_writeCharArray(SCIA_BASE, (uint16_t*)msg, 52);

    CPUTimer_startTimer(myCPUTIMER0_BASE);

    while(1){

    }
}

//
// End of File
//
