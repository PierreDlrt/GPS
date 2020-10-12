/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//******************************************************************************
//!  EUSCI_A0 External Loopback test using EUSCI_A_UART_init API
//!
//!  Description: This demo connects TX to RX of the MSP430 UART
//!  The example code shows proper initialization of registers
//!  and interrupts to receive and transmit data.
//!
//!  SMCLK = MCLK = BRCLK = DCOCLKDIV = ~1MHz, ACLK = 32.768kHz
//!
//!
//!           MSP430FR2xx_4xx Board
//!             -----------------
//!       RST -|          UCA0TXD|----|
//!            |                 |    |
//!            |                 |    |
//!            |          UCA0RXD|----|
//!            |                 |
//!
//! This example uses the following peripherals and I/O signals. You must
//! review these and change as needed for your own board:
//! - UART peripheral
//! - GPIO Port peripheral (for UART pins)
//! - UCA0TXD
//! - UCA0RXD
//!
//! This example uses the following interrupt handlers. To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - USCI_A0_VECTOR.
//******************************************************************************
#include "driverlib.h"
#include "Board.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define   NBBUFF   2

uint16_t i;
uint8_t RXData = 0, TXData = 0;
uint8_t check = 0, fix = 0;
char buff[NBBUFF][150] = {'\0'};
char temp[5][150] = {'\0'};
uint16_t cpt=0;
uint8_t flag=0, flgsent=0, flgGPRMC=0;
float myACLK;

struct coordinate {
    uint8_t deg;
    uint8_t min;
    uint16_t sec;
    bool sig;
} lon, lat;

void parseData(char* sentence, size_t size) {
    if (sentence[0]=='$'){
        flgsent=1;
        if (!memcmp(strtok(sentence, "$,."), "GPRMC", 5)){
            flgGPRMC=1;
            strtok(NULL, "$,."); //date heure
            strtok(NULL, "$,."); //
            if (*strtok(NULL, "$,.")=='A'){
                char tmp[10], tok[10];
                strcpy(tok, strtok(NULL, "$,."));
                strncpy(tmp,tok,2);

                lat.deg = (uint8_t) atoi(tmp);
                lat.min = (uint8_t) atoi(tok+2);

                strncpy(tok, strtok(NULL, "$,."),3);
                tok[3] = '\0';

                lat.sec = (uint16_t) atoi(tok);

                lat.sig = (*strtok(NULL, "$,.")=='W'); // 0 = +, 1 = -
            }
        }
    }
}

void main(void)
{
    //Stop Watchdog Timer
    WDT_A_hold(WDT_A_BASE);
    FRCTL0 = FRCTLPW | NWAITS_0; //remove printf breakpoint


    //Set ACLK = REFOCLK with clock divider of 1
    CS_initClockSignal(CS_ACLK,CS_REFOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK,CS_DCOCLKDIV_SELECT,CS_CLOCK_DIVIDER_1);
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK,CS_DCOCLKDIV_SELECT,CS_CLOCK_DIVIDER_1);
    myACLK = CS_getACLK();

    //Configure UART pins
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_UCA0TXD,
        GPIO_PIN_UCA0TXD,
        GPIO_FUNCTION_UCA0TXD
    );
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_UCA0RXD,
        GPIO_PIN_UCA0RXD,
        GPIO_FUNCTION_UCA0RXD
    );

    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN7, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7);

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    // Setup Timer (TAR, TACTL)
    //    TimerA1 in Continuous mode using ACLK
    //    Toggle LED2 (Green) on/off every 2 seconds using timer interrupt (TA1IFG)
    Timer_A_initContinuousModeParam initContParam = { 0 };
        initContParam.clockSource =                 TIMER_A_CLOCKSOURCE_ACLK;       // Use ACLK (slower clock)
        initContParam.clockSourceDivider =          TIMER_A_CLOCKSOURCE_DIVIDER_4;  // Input clock = ACLK / 1 = 32KHz
        initContParam.timerInterruptEnable_TAIE =   TIMER_A_TAIE_INTERRUPT_ENABLE;  // Enable TAR -> 0 interrupt
        initContParam.timerClear =                  TIMER_A_DO_CLEAR;               // Clear TAR & clock divider
        initContParam.startTimer =                  false;                          // Don't start the timer, yet
    Timer_A_initContinuousMode( TIMER_A1_BASE, &initContParam );

    //Configure UART
    //SMCLK = 1MHz, Baudrate = 115200
    //UCBRx = 8, UCBRFx = 0, UCBRSx = 0xD6, UCOS16 = 0
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 0x06; // 115200Bd = 8, 9600Bd = 6;
    param.firstModReg = 8; // 115200Bd = 0, 9600Bd = 8;
    param.secondModReg = 0x17; // 115200Bd = 0xD6, 9600Bd = 0x17;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling =  EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION; //EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param)) {
        return;
    }

    // Enable global interrupts
    __enable_interrupt();

    Timer_A_startCounter(
        TIMER_A1_BASE,
        TIMER_A_CONTINUOUS_MODE
    );

    //while(fix != 1);
    Timer_A_disableInterrupt(TIMER_A1_BASE);
    GPIO_disableInterrupt(GPIO_PORT_P2, GPIO_PIN7);

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
        EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
        EUSCI_A_UART_RECEIVE_INTERRUPT);

    //uint8_t temp[150] = {'\0'};


    while (1)
    {
        // Increment TX data
        //TXData = TXData+1;
        // Load data onto buffer
        //EUSCI_A_UART_transmitData(EUSCI_A0_BASE, TXData);

        while(check != 1);

        if (RXData == '\n') {
            cpt++;
            strncat(buff[(cpt-1)%NBBUFF],'\0',1);
            //if (cpt<6) {
                memcpy(temp[(cpt-1)%5], buff[(cpt-1)%NBBUFF], strlen(buff[(cpt-1)%NBBUFF])+1);
            //}
            //memset(buff, '\0', 150*sizeof(uint8_t));
            parseData(buff[(cpt-1)%NBBUFF], strlen(buff[(cpt-1)%NBBUFF]));
            buff[(cpt-1)%NBBUFF][0]='\0';
            flag=1;
        }
        check = 0;
    }
}

//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
/*#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A0_VECTOR)))
#endif
void EUSCI_A0_ISR(void)
{
    printf("interrupt\n");
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            RXData = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
            // Check value
            printf("RXData=%d\n", RXData);
            fflush (stdout);
            if(!(RXData == TXData))
            {
                while(1);
            }
            check =1;
            break;
       case USCI_UART_UCTXIFG: break;
       case USCI_UART_UCSTTIFG: break;
       case USCI_UART_UCTXCPTIFG: break;
    }
}*/

#pragma vector=USCI_A0_VECTOR
__interrupt void EUSCI_A0_ISR(void)
{
    //printf("interrupt\n");
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            RXData = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
            strncat(buff[cpt%NBBUFF], &RXData, 1);
            // Check value
            /*if(!(RXData == TXData))
            {
                while(1);
            }*/
            //printf("int\n");
            check =1;
            break;
       case USCI_UART_UCTXIFG: break;
       case USCI_UART_UCSTTIFG: break;
       case USCI_UART_UCTXCPTIFG: break;
    }
}

#pragma vector=PORT2_VECTOR
__interrupt void fix_ISR (void)
{

    switch( __even_in_range( P2IV, P2IV_P2IFG7 )) {
        case P2IV_NONE:   break;                               // None
        case P2IV_P2IFG0:                                      // Pin 0
             __no_operation();
             break;
       case P2IV_P2IFG1:                                       // Pin 1
            __no_operation();
            break;
       case P2IV_P2IFG2:                                       // Pin 2
           __no_operation();
            break;
       case P2IV_P2IFG3:                                       // Pin 3
            __no_operation();
            break;
       case P2IV_P2IFG4:                                       // Pin 4
            __no_operation();
            break;
       case P2IV_P2IFG5:                                       // Pin 5
            __no_operation();
            break;
       case P2IV_P2IFG6:                                       // Pin 6
            __no_operation();
            break;
       case P2IV_P2IFG7:                                       // Pin 7
            TA1R = 0;
            break;
       default:   _never_executed();
    }
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void timer1_ISR (void)
{

    switch( __even_in_range( TA1IV, TA1IV_TAIFG )) {
     case TA1IV_NONE: break;                 // (0x00) None
     case TA1IV_TACCR1:                      // (0x02) CCR1 IFG
          _no_operation();
           break;
     case TA1IV_TACCR2:                      // (0x04) CCR2 IFG
          _no_operation();
           break;
     case TA1IV_3: break;                    // (0x06) Reserved
     case TA1IV_4: break;                    // (0x08) Reserved
     case TA1IV_5: break;                    // (0x0A) Reserved
     case TA1IV_6: break;                    // (0x0C) Reserved
     case TA1IV_TAIFG:                       // (0x0E) TA1IFG - TAR overflow
          fix=1;
          break;
     default: _never_executed();
    }
}


