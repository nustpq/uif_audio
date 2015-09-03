/*
*********************************************************************************************************
*                                          UIF BOARD APP PACKAGE
*
*                            (c) Copyright 2013 - 2016; Fortemedia Inc.; Nanjing, China
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            APP PACKAGE
*
*                                          Atmel SAM3U4C
*                                               on the
*                                      Unified EVM Interface Board
*
* Filename      : main.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <stdbool.h>
#include <tc/tc.h>
#include <utility/trace.h>
#include <usart/usart.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include "kfifo.h"
#include "led.h"
#include <usb.h>
#include <app.h>
#include <dbgu/dbgu.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <utility/led.h>


int main( void )
{

    TRACE_CONFIGURE( DBGU_STANDARD, 115200, BOARD_MCK );    
    printf("\r\n--------------------------------------------------\r\n");
    printf("--  Unified Interface Board - Audio\r\n");    
    printf("--  CPU [ATSAM3U4C][%dMHz][%X]\r\n", BOARD_MCK/1000000, AT91C_BASE_NVIC->NVIC_CPUID);
    printf("--  HW version: %s\r\n", BOARD_NAME);
    printf("--  SW version: %s\r\n", fw_version);    
    printf("    UsbEpSize = %d B, PlayPreBuffer = %d ms\r\n", USBDATAEPSIZE, PLAY_BUF_DLY_N);
    printf("--  Compiled on %s %s by PQ--\r\n", __DATE__, __TIME__);
    printf("--------------------------------------------------\r\n");
        
    Init_GPIO();
    Disable_SPI_Port();
    Timer0_Init(); 
    Timer1_Init();
    Timer2_Init();
    SysTick_Init();
    UART0_Init();   
    UART1_Init();
    
#ifdef METHOD_BY_RESET_MCU
    USART_Write( AT91C_BASE_US0, 0, 0 ); //Send ACK
    delay_ms(1000);
#endif
    
    USB_Init(); 
    Init_DMA();
    I2S_Init();
 
    while(1) {
      
        //Debug_Info(); 
        Check_UART_CMD();
        Audio_State_Control();
        DBGUART_Service();
        Init_USB_Callback();
        Read_iM501_Voice_Buffer();
      
    }
    
    
    
}

