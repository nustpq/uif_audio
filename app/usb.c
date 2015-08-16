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
*                                          USB APP PACKAGE
*
*                                         Atmel  SAM3U4C
*                                             on the
*                                      Unified EVM Interface Board
*
* Filename      : usb.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <pio/pio.h>
#include <pio/pio_it.h>
#include <usart/usart.h>
#include <dbgu/dbgu.h>
#include <irq/irq.h>
#include <board.h>
#include <string.h>
#include <stdbool.h>
#include <utility/led.h>
#include <usb/device/core/USBD.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <utility/trace.h>
#include "kfifo.h"
#include <ssc/ssc.h>
#include <app.h>
#include "uart.h"




//------------------------------------------------------------------------------
//      Internal variables
//------------------------------------------------------------------------------
/// State of USB, for suspend and resume
unsigned char USBState  = false ;

unsigned long long total_received = 0 ;
unsigned long long total_transmit = 0 ;
unsigned int error_bulkout_full  = 0 ;
unsigned int error_bulkout_empt  = 0 ;
unsigned int error_bulkin_full   = 0 ;
unsigned int error_bulkin_empt   = 0 ;

unsigned long long total_received_cmd = 0 ;
unsigned long long total_transmit_cmd = 0 ;

//------------------------------------------------------------------------------
//         VBus monitoring (optional)
//------------------------------------------------------------------------------
#if defined(PIN_USB_VBUS)

#define VBUS_CONFIGURE()  VBus_Configure()

/// VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;


/*
*********************************************************************************************************
*                                    ISR_Vbus()
*
* Description :  VBUS interruption service. Check USB attach detection GPIO status and init USB module
* Argument(s) : *pPin: pointer to the USB attach detection GPIO.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void ISR_Vbus(const Pin *pPin)
{
    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        TRACE_INFO("VBUS conn\r\n");
        USBD_Connect();
    }
    else {

        TRACE_INFO("VBUS discon\r\n");
        USBD_Disconnect();
    }
}


/*
*********************************************************************************************************
*                                    VBus_Configure()
*
* Description :  Configures the VBus pin to trigger an interrupt when the level on that pin changes.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static void VBus_Configure( void )
{
    TRACE_INFO("VBus configuration\r\n");

    // Configure PIO
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    // Check current level on VBus
    if (PIO_Get(&pinVbus) == 0 ) { //EVM2.0 reverted transistor

        // if VBUS present, force the connect
        TRACE_INFO("VBUS conn\r\n");
        USBD_Connect();
    }
    else {
        USBD_Disconnect();
    }           
}

#else
    #define VBUS_CONFIGURE()    USBD_Connect()
#endif //#if defined(PIN_USB_VBUS)


/*
//------------------------------------------------------------------------------
//         Callbacks re-implementation
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Invoked when the USB device leaves the Suspended state. By default,
/// configures the LEDs.
//------------------------------------------------------------------------------

void USBDCallbacks_Resumed(void)
{
    // Initialize LEDs
    LED_Configure(USBD_LEDPOWER);
    LED_Set(USBD_LEDPOWER);
    LED_Configure(USBD_LEDDATA);
    LED_Clear(USBD_LEDDATA);
  
}

//------------------------------------------------------------------------------
/// Invoked when the USB device gets suspended. By default, turns off all LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Suspended(void)
{
    // Turn off LEDs
    LED_Clear(USBD_LEDPOWER);
    LED_Clear(USBD_LEDDATA);
    
    if (USBD_GetState() >= USBD_STATE_CONFIGURED);
       
}
*/




////////////////////////////////////////////////////////////////////////////////
/*
const unsigned char test_vec[] = 
{
  '1','2','3','4','5','6','7','8'  
};

static unsigned int check_bulkout_data( unsigned char *pBuf, unsigned int length )
{  
    unsigned int i;    
    unsigned int *pInt = (unsigned int*)pBuf;
    unsigned int *pVec = (unsigned int*)test_vec;
    
    for( i = 0; i < (length>>3); i++ ) {      
      if( *pInt++ != *pVec ) { //1234
        return 1;
      }   
      if( *pInt++ != *(pVec+1) ) {//5678
        return 1;
      }          
    }
    return 0;  
}
*/



void Init_Bulk_FIFO( void )
{   
    kfifo_t *pfifo;
    
    pfifo = &bulkout_fifo;
    kfifo_init_static(pfifo, FIFOBufferBulkOut, USB_OUT_BUFFER_SIZE);
    pfifo = &bulkin_fifo;
    kfifo_init_static(pfifo, FIFOBufferBulkIn, USB_IN_BUFFER_SIZE);
    
    pfifo = &bulkout_fifo_cmd;
    kfifo_init_static(pfifo, FIFOBufferBulkOutCmd, USB_CMD_OUT_BUFFER_SIZE);
    pfifo = &bulkin_fifo_cmd;
    kfifo_init_static(pfifo, FIFOBufferBulkInCmd, USB_CMD_IN_BUFFER_SIZE);

}  




/*
*********************************************************************************************************
*                                    UsbAudioDataReceived()
*
* Description :  USB bulk out process call back subroutine
* Argument(s) :  unused    : not used argument.
*                status    : curren USB data process status, must be SUCCESS to operation
*                received  : received data in buffer
*                remaining : remaining data in buffer
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UsbAudioDataReceived(  unsigned int unused,
                              unsigned char status,
                              unsigned int received,
                              unsigned int remaining )
{
    TRACE_INFO_NEW_WP("\r\n#BO:") ;
    //Play
          
    if ( ! bulkout_enable ) {
        //printf("\r\nstatus %d, bulkout_enable %d\r\n",status, bulkout_enable);
        return ;
    }
    if ( status == USBD_STATUS_SUCCESS ) {     
        //while( received > kfifo_get_free_space( &bulkout_fifo ) ) ; //wait if buf is full   
        
//        // Check 1st data package:
//        if( bulkout_padding_ok ) {
//            kfifo_put(&bulkout_fifo, usbBufferBulkOut, received);            
//        } else {
//            bulkout_padding_ok = First_Pack_Check_BO(received);             
//        } 

         // Check every data package:        
       // LED_CLEAR_DATA;
          bool flag = First_Pack_Check_BO( received );           
          if( flag ) {            
              if( ! bulkout_padding_ok ) {             
                  bulkout_padding_ok = true;
              }              
          } else {              
              if( bulkout_padding_ok ) {
                  kfifo_put(&bulkout_fifo, usbBufferBulkOut, received);  
              }              
          }             
       // LED_SET_DATA;  

        
//// Debug use        
//        if( check_buf_debug(usbBufferBulkOut, received) ) {
//                printf("\r\n Check USB BI buf err : \r\n");
//                dump_buf_debug(usbBufferBulkOut, received );
//                while(1){ DBGUART_Service();};
//         }
        
        
        if ( USBDATAEPSIZE <= kfifo_get_free_space( &bulkout_fifo ) ) { //enouth free buffer                      
            CDCDSerialDriver_Read(    usbBufferBulkOut,
                                      USBDATAEPSIZE,
                                      (TransferCallback) UsbAudioDataReceived,
                                      0);        
        } else { //usb out too fast                     
            bulkout_start  = true ;
            
        }     
        total_received += received ; 
     
    }  else {      
        printf( "\r\nERROR : UsbAudioDataReceived: Transfer error\r\n" ); 
        
    }
    
    
}


/*
*********************************************************************************************************
*                                    UsbAudioDataTransmit()
*
* Description :  USB bulk in process call back subroutine
* Argument(s) :  unused    : not used argument.
*                status    : curren USB data process status, must be SUCCESS to operation
*                received  : received data in buffer
*                remaining : remaining data in buffer
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UsbAudioDataTransmit(  unsigned int unused,
                              unsigned char status,
                              unsigned int transmit,
                              unsigned int remaining )
{
    TRACE_INFO_NEW_WP("\r\n#BI:") ;
            
    //Record    
    if ( ! bulkin_enable ) {
        //printf("\r\nstatus %d, bulkin_enable %d\r\n",status, bulkin_enable);
        return ;
    }
    
    if ( status == USBD_STATUS_SUCCESS  ) {       
       
        if ( USBDATAEPSIZE <= kfifo_get_data_size(  &bulkin_fifo )  ) { //enouth data to send to PC
           
            kfifo_get(&bulkin_fifo, usbBufferBulkIn, USBDATAEPSIZE); 
        
            CDCDSerialDriver_Write( usbBufferBulkIn,
                                    USBDATAEPSIZE,
                                    (TransferCallback) UsbAudioDataTransmit,
                                    0);       
        } else {                    
            bulkin_start  = true ;  
            
        }              
        total_transmit += transmit ; 
     
    }  else {
        CDCDSerialDriver_Write( usbBufferBulkIn,
                                USBDATAEPSIZE,
                                (TransferCallback) UsbAudioDataTransmit,
                                0);  
        TRACE_WARNING( "\r\nERROR : UsbAudioDataTransmit: Rr-transfer hit\r\n" );  
        
    }    
    
}


/*
*********************************************************************************************************
*                                    UsbCmdDataReceived()
*
* Description :  USB bulk out process call back subroutine
* Argument(s) :  unused    : not used argument.
*                status    : curren USB data process status, must be SUCCESS to operation
*                received  : received data in buffer
*                remaining : remaining data in buffer
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UsbCmdDataReceived(      unsigned int unused,
                              unsigned char status,
                              unsigned int received,
                              unsigned int remaining )
{
    unsigned int counter ;
    //printf("\r\n#CMDBO:") ;
    //Play : USB Received,  UART send    

    if ( status == USBD_STATUS_SUCCESS ) {         
        kfifo_put(&bulkout_fifo_cmd, usbCmdBufferBulkOut, received); 
        //printf("[%d]",received) ;
        if ( USBCMDDATAEPSIZE <= kfifo_get_free_space( &bulkout_fifo_cmd ) ) { //enouth free buffer                      
            CDCDSerialDriver_ReadCMD(    usbCmdBufferBulkOut,
                                         USBCMDDATAEPSIZE,
                                        (TransferCallback) UsbCmdDataReceived,
                                        0);        
        } else { //usb out too fast                     
            bulkout_start_cmd  = true ;            
        }  
        
        //start UART send
        if( uartout_start_cmd ) {            
            counter = kfifo_get(&bulkout_fifo_cmd, (unsigned char *)UARTBuffersOut, UART_OUT_BUFFER_SIZE) ;
            //printf("[%d]:",counter) ;
            //dump_buf_debug((unsigned char *)UARTBuffersOut,counter);
            if( counter ) { 
                uartout_start_cmd = false;
                AT91C_BASE_US1->US_TPR  = (unsigned int)UARTBuffersOut;
                AT91C_BASE_US1->US_TCR  = counter;
                AT91C_BASE_US1->US_PTCR = AT91C_PDC_TXTEN;//start PDC 
                AT91C_BASE_US1->US_IER  = AT91C_US_ENDTX; //PQ
            }
        }        
        total_received_cmd += received ; 
     
    }  else {      
        printf( "\r\nERROR : UsbCmdDataReceived: Transfer error\r\n" ); 
        
    }
    
    
}


/*
*********************************************************************************************************
*                                    UsbCmdDataTransmit()
*
* Description :  USB bulk in process call back subroutine
* Argument(s) :  unused    : not used argument.
*                status    : curren USB data process status, must be SUCCESS to operation
*                received  : received data in buffer
*                remaining : remaining data in buffer
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void UsbCmdDataTransmit(      unsigned int unused,
                              unsigned char status,
                              unsigned int transmit,
                              unsigned int remaining )
{
    unsigned int counter ;
    //printf("\r\n#CMDBI:") ;            
    //Record : USB Send,  UART Received       
    
    if ( status == USBD_STATUS_SUCCESS  ) {       
        
        counter =  kfifo_get_data_size(  &bulkin_fifo_cmd );
        //printf("[%d]", counter) ;
        if ( USBCMDDATAEPSIZE <= counter  ) { //enouth data to send to PC
            
            kfifo_get(&bulkin_fifo_cmd, usbCmdBufferBulkIn, USBCMDDATAEPSIZE);         
            CDCDSerialDriver_WriteCMD( usbCmdBufferBulkIn,
                                       USBCMDDATAEPSIZE,
                                       (TransferCallback) UsbCmdDataTransmit,
                                       0);       
        } else if( 0 < counter ){  //need optimize : what if last package data less than EP size
            kfifo_get(&bulkin_fifo_cmd, usbCmdBufferBulkIn, counter); 
        
            CDCDSerialDriver_WriteCMD( usbCmdBufferBulkIn,
                                       counter,
                                       (TransferCallback) UsbCmdDataTransmit,
                                       0);              
        } else{
            bulkin_start_cmd  = true ;             
        }          
        
        //start UART receive
        counter = kfifo_get_free_space(&bulkin_fifo_cmd); 
        
        if( uartin_start_cmd && (UART_IN_BUFFER_SIZE <= counter) ) {           
            uartin_start_cmd = false;            
            //printf("[%d]", counter) ;  
            AT91C_BASE_US1->US_RPR  = (unsigned int)UARTBuffersIn;
            AT91C_BASE_US1->US_RCR  = UART_IN_BUFFER_SIZE;
            AT91C_BASE_US1->US_PTCR = AT91C_PDC_RXTEN; 
            AT91C_BASE_US1->US_IER  = AT91C_US_ENDRX | AT91C_US_TIMEOUT; 
        } 
        total_transmit_cmd += transmit ; 
     
    }  else {
        
//        CDCDSerialDriver_WriteCMD( usbCmdBufferBulkIn,
//                                   USBCMDDATAEPSIZE,
//                                  (TransferCallback) UsbCmdDataTransmit,
//                                  0);  
        TRACE_WARNING( "\r\nERROR : UsbCmdDataTransmit: Re-transfer hit\r\n" );  
        
    }    
    
}

/*
*********************************************************************************************************
*                                    USB_Init()
*
* Description :  Initialize USB function.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void USB_Init(void)
{
    printf("\r\nInit USB ...");
  
   // If there is on board power, switch it off 
#ifdef PIN_USB_POWER_ENB
    const Pin pinUsbPwr = PIN_USB_POWER_ENB;
    PIO_Configure(&pinUsbPwr, 1); 
#endif 
    
  //external pull up seems not working now
#ifdef PIN_USB_PULLUP
    const Pin pinPullUp = PIN_USB_PULLUP;
    PIO_Configure(&pinPullUp,1);
#endif  
      
    // BOT driver initialization
    CDCDSerialDriver_Initialize();
    
    // connect if needed
    VBUS_CONFIGURE();

    // Connect pull-up, wait for configuration
    USBD_Connect();

    Init_Bulk_FIFO();    
 
    //LED_Clear(USBD_LEDDATA); 
    
    printf("Done\r\n");
    
}


void Init_USB_Callback( void )
{
    //delay_ms(1000);    
    if ( (USBD_GetState() < USBD_STATE_CONFIGURED)  ||  USBState ) {
        return;
    }

    USBState = true ;
    printf("\r\nInit USB Callback()\r\n");
    
    // Start receiving data on the USB
    //delay_ms(1000);   
    //AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_CMDDATAIN;
    //AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_CMDDATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;  
         
    AT91C_BASE_US1->US_CR     = AT91C_US_RXEN;    
    CDCDSerialDriver_WriteCMD( usbCmdBufferBulkIn,
                            8,//USBCMDDATAEPSIZE,
                            (TransferCallback) UsbCmdDataTransmit,
                            0);
    
    AT91C_BASE_US1->US_CR     = AT91C_US_TXEN;
    CDCDSerialDriver_ReadCMD(  usbCmdBufferBulkOut,
                            USBCMDDATAEPSIZE,
                            (TransferCallback) UsbCmdDataReceived,
                            0); 
    
      
    //AT91C_BASE_US1->US_IER    = AT91C_US_ENDRX | AT91C_US_TIMEOUT; 
    
    
}
