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
*                                          UART PACKAGE
*
*                                         Atmel  SAM3U4C
*                                             on the
*                                      Unified EVM Interface Board
*
* Filename      : uart.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <pio/pio.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <utility/trace.h>
#include <irq/irq.h>
#include <utility/led.h>
#include <board.h>
#include <stdbool.h>
#include "kfifo.h"
#include "app.h"

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <ssc/ssc.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <stdbool.h>
#include <string.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <dmad/dmad.h>
#include <dma/dma.h>
#include "kfifo.h"
#include "app.h"
#include "usb.h"
#include "uart.h"




volatile unsigned char usartBuffers[2][UART_BUFFER_SIZE] ;
volatile unsigned char usartCurrentBuffer = 0 ;


static const Pin Uart0_Pins[] = {  

    PIN_USART0_RXD,
    PIN_USART0_TXD
      
};

static const Pin Uart1_Pins[] = {  
    
    PIN_USART1_RXD,
    PIN_USART1_TXD,
    PIN_USART1_CTS,
    PIN_USART1_RTS
      
};

unsigned char UART_CMD_Buffer[4];
unsigned char state_mac      = CMD_STAT_SYNC1 ;
unsigned char PcCmdCounter   = 0; 




/*
*********************************************************************************************************
*                                    pcInt()
*
* Description : Process command data byte by byte and decode beased on communication protocol between USB AUDIO MCU and HOST MCU.
* Argument(s) : ch : the one byte data.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void pcInt(  unsigned char ch )
{    
    unsigned char *pChar = UART_CMD_Buffer;
    
    switch( state_mac ) {   
        
        case CMD_STAT_SYNC1 :        
            if(ch == CMD_DATA_SYNC1)  {
                state_mac = CMD_STAT_SYNC2 ;
            }
        break;
        
        case CMD_STAT_SYNC2 :
            if(ch == CMD_DATA_SYNC2)  {           
                 state_mac     =  CMD_STAT_FLAG;
                 PcCmdCounter  =  0 ; 
            } else {              
                state_mac = CMD_STAT_SYNC1;                
            }
        break ;
        
        case CMD_STAT_FLAG :   
       
            switch( ch )  {
                case RULER_CMD_SET_AUDIO_CFG : 
                    state_mac =  CMD_STAT_CMD1 ;
                    break ;
                case RULER_CMD_START_AUDIO :
                    state_mac =  CMD_STAT_CMD2 ;
                    break ;                
                case RULER_CMD_STOP_AUDIO :
                    audio_cmd_index = AUDIO_CMD_STOP ; 
                    state_mac = CMD_STAT_SYNC1;                  
                    break ;  
                case RULER_CMD_RESET_AUDIO :
                    audio_cmd_index = AUDIO_CMD_RESET ; 
                    state_mac = CMD_STAT_SYNC1;                     
                    break ;  
                case RULER_CMD_GET_AUDIO_VERSION  :
                    audio_cmd_index = AUDIO_CMD_VERSION ; 
                    state_mac = CMD_STAT_SYNC1; 
                break ;  
                        
                default :
                    break ;                        
            }
         
        break ;
        
        case CMD_STAT_CMD1 :            
             UART_CMD_Buffer[PcCmdCounter++] = ch;       
             state_mac  = CMD_STAT_DATA ;
          
        break ;
        
        case CMD_STAT_CMD2 :  
             audio_cmd_index = ch;
             PcCmdCounter    = 0;
             state_mac       = CMD_STAT_CMD3 ;
          
        break ;
        
        case CMD_STAT_CMD3 :  
             usb_data_padding = ch;           
             state_mac        = CMD_STAT_SYNC1 ;
          
        break ;
        
        case CMD_STAT_DATA :
            *(pChar+PcCmdCounter) = ch; 
            PcCmdCounter++;
            if( PcCmdCounter > 4 ) { //check overflow
               Audio_Configure[(*pChar)&0x01] = *(AUDIO_CFG *)pChar;                
               audio_cmd_index = AUDIO_CMD_CFG ; 
               PcCmdCounter = 0 ;        
               state_mac = CMD_STAT_SYNC1;                
            } 
        break ;
                
        default :
            state_mac     = CMD_STAT_SYNC1;
            PcCmdCounter  = 0 ;
        break ;
        
    } 

    
}


/*
*********************************************************************************************************
*                                    USART0_IrqHandler()
*
* Description : UART0 interruption service routine. Handles interrupts coming from USART #0.
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : Side-efect:
*               Here we assume the data processing in Check_UART_CMD() is faster than data from 
*               host MCU , or some data will be lost. 
*********************************************************************************************************
*/
static unsigned int data_received = 0;
void USART0_IrqHandler( void )
{
  
    unsigned int    status ;  
    status        = AT91C_BASE_US0->US_CSR;    
    
    // Buffer has been read successfully
    if ( status & AT91C_US_ENDRX ) {  
        data_received = UART_BUFFER_SIZE;
        usartCurrentBuffer = 1 - usartCurrentBuffer; //pingpong buffer index
        USART_ReadBuffer( AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer], UART_BUFFER_SIZE); // Restart read on buffer

    }
    
    if ( status & AT91C_US_TIMEOUT ) {  
        data_received = UART_BUFFER_SIZE - ( AT91C_BASE_US0->US_RCR );
        AT91C_BASE_US0->US_RCR = 0;   
        usartCurrentBuffer = 1 - usartCurrentBuffer;
        USART_ReadBuffer(AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer], UART_BUFFER_SIZE);    // Restart read on buffer
        AT91C_BASE_US0->US_CR   = AT91C_US_STTTO; //restart timeout counter
        AT91C_BASE_US0->US_RTOR = UART_TIMEOUT_BIT;
    
    }
    
    
    if ( status & AT91C_US_ENDTX  )   {  //Transmit INT        
        AT91C_BASE_US0->US_IDR   =  AT91C_US_ENDTX  ; //disable PDC tx INT
        AT91C_BASE_US0->US_PTCR  =   AT91C_PDC_TXTDIS; //stop PDC        
      
    }
    
    
}


/*
*********************************************************************************************************
*                                    USART1_IrqHandler()
*
* Description : UART0 interruption service routine. Handles interrupts coming from USART #0.
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : Side-efect:
*               Here we assume the data processing in Check_UART_CMD() is faster than data from 
*               host MCU , or some data will be lost. 
*********************************************************************************************************
*/

void USART1_IrqHandler( void )
{
    unsigned int status ;
    unsigned int counter ;
    
    status  = AT91C_BASE_US1->US_CSR ;
    status &= AT91C_BASE_US1->US_IMR ;    

    
    if ( status & AT91C_US_ENDTX   )   {  //Transmit INT  
        //printf(" iT.");
        //AT91C_BASE_US1->US_IDR   =  AT91C_US_ENDTX  ; //disable PDC tx INT
        //AT91C_BASE_US1->US_PTCR  =  AT91C_PDC_TXTDIS; //stop PDC
        
        //temp = kfifo_get_data_size(&bulkout_fifo_cmd);  
        counter = kfifo_get( &bulkout_fifo_cmd, (unsigned char *)UARTBuffersOut, UART1_BUFFER_SIZE);
        if( counter ) {
            //USART_WriteBuffer( AT91C_BASE_US1,(void *)UARTBuffersOut, UART1_BUFFER_SIZE);
            AT91C_BASE_US1->US_TPR = (unsigned int)UARTBuffersOut;
            AT91C_BASE_US1->US_TCR = counter;
            AT91C_BASE_US1->US_PTCR= AT91C_PDC_TXTEN;//start PDC
            AT91C_BASE_US1->US_IER = AT91C_US_ENDTX; //PQ
        } else {
            AT91C_BASE_US1->US_IDR  = AT91C_US_ENDTX; 
            uartout_start_cmd = true; 
        }
              
        if (  bulkout_start_cmd && (USBCMDDATAEPSIZE <= kfifo_get_free_space(&bulkout_fifo_cmd)) ) { //               
            bulkout_start_cmd = false ;
            //error_bulkout_full++;
            //printf("[%d] ",kfifo_get_free_space(&bulkout_fifo_cmd));
            CDCDSerialDriver_ReadCMD(   usbCmdBufferBulkOut,
                                        USBCMDDATAEPSIZE,
                                        (TransferCallback) UsbCmdDataReceived,
                                       0);
        }   
    
    }
    
    // Buffer has been read successfully
    if ( status & AT91C_US_ENDRX ) {  
        //printf(" iR.");
        //AT91C_BASE_US1->US_IDR   =  AT91C_US_ENDRX  ; //disable PDC tx INT
        //AT91C_BASE_US1->US_PTCR  =  AT91C_PDC_RXTDIS; //stop PDC
        
        kfifo_put(&bulkin_fifo_cmd, (unsigned char *)UARTBuffersIn, UART1_BUFFER_SIZE) ;
        //dump_buf_debug((unsigned char *)UARTBuffersIn,UART1_BUFFER_SIZE);
        if(  UART1_BUFFER_SIZE < kfifo_get_free_space( &bulkin_fifo_cmd ) ) {
            //USART_ReadBuffer( AT91C_BASE_US1,(void *)UARTBuffersIn, UART1_BUFFER_SIZE); // Restart read on buffer 
            AT91C_BASE_US1->US_RPR  = (unsigned int)UARTBuffersIn;
            AT91C_BASE_US1->US_RCR  = UART1_BUFFER_SIZE;
            AT91C_BASE_US1->US_PTCR = AT91C_PDC_RXTEN;
            AT91C_BASE_US1->US_IER  = AT91C_US_ENDRX | AT91C_US_TIMEOUT; //PQ
        } else {
            AT91C_BASE_US1->US_IDR  = AT91C_US_ENDRX | AT91C_US_TIMEOUT; //PQ
            uartin_start_cmd = true; 
        }
        if ( bulkin_start_cmd  && ( USBCMDDATAEPSIZE <= kfifo_get_data_size(&bulkin_fifo_cmd)) ) {    
            bulkin_start_cmd = false ;
            //error_bulkin_empt++;        
            //printf("[%d] ",kfifo_get_data_size(&bulkin_fifo_cmd));
            kfifo_get(&bulkin_fifo_cmd, usbCmdBufferBulkIn, USBDATAEPSIZE); 
            CDCDSerialDriver_WriteCMD(  usbCmdBufferBulkIn,
                                        USBCMDDATAEPSIZE,
                                        (TransferCallback) UsbCmdDataTransmit,
                                        0);            
       }
       
    }
    
    if ( status & AT91C_US_TIMEOUT ) {  
        //printf(" iRt.");
        kfifo_put(&bulkin_fifo_cmd, (unsigned char *)UARTBuffersIn, UART1_BUFFER_SIZE - ( AT91C_BASE_US1->US_RCR )) ;
        //dump_buf_debug((unsigned char *)UARTBuffersIn,UART1_BUFFER_SIZE- ( AT91C_BASE_US1->US_RCR ));
        if(  UART1_BUFFER_SIZE < kfifo_get_free_space( &bulkin_fifo_cmd ) ) {
            //USART_ReadBuffer( AT91C_BASE_US1,(void *)UARTBuffersIn, UART1_BUFFER_SIZE); // Restart read on buffer 
            AT91C_BASE_US1->US_RPR  = (unsigned int)UARTBuffersIn;
            AT91C_BASE_US1->US_RCR  = UART1_BUFFER_SIZE;
            AT91C_BASE_US1->US_PTCR = AT91C_PDC_RXTEN;
            AT91C_BASE_US1->US_CR   = AT91C_US_STTTO; //restart timeout counter
            AT91C_BASE_US1->US_RTOR = UART_TIMEOUT_BIT;
            AT91C_BASE_US1->US_IER  = AT91C_US_ENDRX | AT91C_US_TIMEOUT; //PQ
        } else {
            AT91C_BASE_US1->US_IDR  = AT91C_US_ENDRX | AT91C_US_TIMEOUT; //PQ
            uartin_start_cmd = true; 
        }
        //AT91C_BASE_US1->US_RCR = 0;
        //USART_ReadBuffer( AT91C_BASE_US1,(void *)UARTBuffersIn, UART1_BUFFER_SIZE); // Restart read on buffer
       
      
        if ( bulkin_start_cmd  && ( USBCMDDATAEPSIZE <= kfifo_get_data_size(&bulkin_fifo_cmd)) ) {    
            bulkin_start_cmd = false ;
            //error_bulkin_empt++;
            //printf("[%d] ",kfifo_get_data_size(&bulkin_fifo_cmd));
            kfifo_get(&bulkin_fifo_cmd, usbCmdBufferBulkIn, USBCMDDATAEPSIZE);            
            CDCDSerialDriver_WriteCMD(  usbCmdBufferBulkIn,
                                        USBCMDDATAEPSIZE,
                                        (TransferCallback) UsbCmdDataTransmit,
                                        0);            
       }       
    
    }
        

    
    
}



/*
*********************************************************************************************************
*                                    UART0_Init()
*
* Description : init UART port : UART0 for cmd from host mcu
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UART0_Init( void )
{
    printf("\r\nInit UART0 ...");  
    
    PIO_Configure(Uart0_Pins, PIO_LISTSIZE(Uart0_Pins));    
    usartCurrentBuffer = 0 ;
    
    // Configure USART0¡¡¡¡//Comm with mcu  
    AT91C_BASE_PMC->PMC_PCER   = 1 << AT91C_ID_US0;
    USART_Configure(AT91C_BASE_US0,USART_MODE_ASYNCHRONOUS,UART0_BAUD, MCK);
   
    AT91C_BASE_US0->US_IDR  = 0xFFFFFFFF;
    AT91C_BASE_US0->US_CR   = AT91C_US_STTTO; //restart timeout counter
    AT91C_BASE_US0->US_RTOR = UART_TIMEOUT_BIT;
    AT91C_BASE_US0->US_TCR  = 0;
    AT91C_BASE_US0->US_RCR  = 0;
    
    USART_ReadBuffer(AT91C_BASE_US0,(void *)usartBuffers[usartCurrentBuffer],UART_BUFFER_SIZE);   
    IRQ_ConfigureIT(AT91C_ID_US0, UART_PRIORITY, USART0_IrqHandler); //priority chaned to max?
    IRQ_EnableIT(AT91C_ID_US0);  
    
    AT91C_BASE_US0->US_IER      =  AT91C_US_ENDRX | AT91C_US_TIMEOUT; 
    
    USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);
    USART_SetReceiverEnabled(AT91C_BASE_US0, 1);       
      
    printf("Done\r\n");
    
}


/*
*********************************************************************************************************
*                                    UART1_Init()
*
* Description : init UART port : UART1 for data between PC and host mcu
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UART1_Init( void )
{
    printf("\r\nInit UART1 ...");  
    
    AT91C_BASE_PMC->PMC_PCER   = 1 << AT91C_ID_US1;
    PIO_Configure(Uart1_Pins, PIO_LISTSIZE(Uart1_Pins));    
    USART_Configure( AT91C_BASE_US1, USART_MODE_ASYNCHRONOUS_HW, UART1_BAUD, MCK );
    
    ///IRQ_DisableIT(AT91C_ID_US1); 
    
    AT91C_BASE_US1->US_IDR  =  0xFFFFFFFF;
    AT91C_BASE_US1->US_CR   = AT91C_US_STTTO; //restart timeout counter
    AT91C_BASE_US1->US_RTOR = UART_TIMEOUT_BIT;
    AT91C_BASE_US1->US_TCR  = 0;
    AT91C_BASE_US1->US_RCR  = 0;
  
    memset((unsigned char *)UARTBuffersOut, 0x55, UART1_BUFFER_SIZE);   
    //USART_ReadBuffer( AT91C_BASE_US1,(void *)UARTBuffersIn[0],  UART1_BUFFER_SIZE);                           
    //USART_ReadBuffer( AT91C_BASE_US1,(void *)UARTBuffersIn[1],  UART1_BUFFER_SIZE);  
    //USART_WriteBuffer( AT91C_BASE_US1,(void *)UARTBuffersOut[0], UART1_BUFFER_SIZE);                             
    //USART_WriteBuffer( AT91C_BASE_US1,(void *)UARTBuffersOut[1], UART1_BUFFER_SIZE); 
    
    IRQ_ConfigureIT(AT91C_ID_US1, UART_PRIORITY, NULL ); 
    IRQ_EnableIT(AT91C_ID_US1);  
    
    
    //AT91C_BASE_US1->US_IER      =  AT91C_US_ENDRX |  AT91C_US_TIMEOUT;     
    //USART_SetTransmitterEnabled(AT91C_BASE_US1, 1);
    //USART_SetReceiverEnabled(AT91C_BASE_US1, 1);       
      
    printf("Done\r\n");
    
}
 

/*
*********************************************************************************************************
*                                    Check_UART_CMD()
*
* Description : Cmd and data parsing from Host MCU
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void Check_UART_CMD( void )
{
    
    unsigned int i ;
    unsigned int counter ;    
    
    counter = data_received ;
    
    if( counter == 0 ) {
        return ;
    }
    
    data_received = 0;
    
    for( i = 0; i < counter; i++)  { //analyze the data          
        pcInt( usartBuffers[1-usartCurrentBuffer][i] ) ;
        
    } 
    
    
}

