/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <board.h>
#include "pio/pio.h"
#include "twi/twi.h" 
#include "twi/twid.h" 
#include <irq/irq.h>
#include <tc/tc.h>
#include <stdbool.h>
#include <string.h>
#include "kfifo.h"
#include "app.h"

/// TWI peripheral redefinition if needed
//#if !defined(AT91C_BASE_TWI) && defined(AT91C_BASE_TWI0)
//    #define AT91C_BASE_TWI      AT91C_BASE_TWI0
 //   #define AT91C_ID_TWI        AT91C_ID_TWI0
 //   #define PINS_TWI            PINS_TWI0
//#endif

//use TWI0 as I2C host
#define AT91C_BASE_TWI      AT91C_BASE_TWI0
#define AT91C_ID_TWI        AT91C_ID_TWI0
#define PINS_TWI            PINS_TWI0
   
   
#define TWITIMEOUTMAX         96000  // 1ms timeout @ 96MHz MIP
#define TWITIMEOUTMAX_BURST   96000000  // 1s timeout @ 96MHz MIP
     
//#define TWI_V3XX   // send stop, A3 no need to do this //defined in board.h
#define TWI_ASYNC_MODE // Asynchronous transfer enabled


/// Pio pins to configure.
static const Pin twi_pins[] = {PINS_TWI};

/// TWI driver instance.
Twid  twid;

Async twi_async ;     

//------------------------------------------------------------------------------
//         Local types
//------------------------------------------------------------------------------

/// TWI driver callback function.
typedef void (*TwiCallback)(Async *);

//------------------------------------------------------------------------------
/// TWI asynchronous transfer descriptor.
//------------------------------------------------------------------------------
typedef struct _AsyncTwi {

    /// Asynchronous transfer status.
    volatile unsigned char status;
    // Callback function to invoke when transfer completes or fails.
    TwiCallback callback;
    /// Pointer to the data buffer.
    unsigned char *pData;
    /// Total number of bytes to transfer.
    unsigned int num;
    /// Number of already transferred bytes.
    unsigned int transferred;    
   

} AsyncTwi;


//OS_EVENT *TWI_Sem_lock; //sem for TWI
//OS_EVENT *TWI_Sem_done; //sem for TWI

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initializes a TWI driver instance, using the given TWI peripheral. The
/// peripheral must have been initialized properly before calling this function.
/// \param pTwid  Pointer to the Twid instance to initialize.
/// \param pTwi  Pointer to the TWI peripheral to use.
//------------------------------------------------------------------------------
void TWID_Initialize(Twid *pTwid, AT91S_TWI *pTwi)
{
    //TRACE_DEBUG("TWID_Initialize()\n\r");
    //SANITY_CHECK(pTwid);
    //SANITY_CHECK(pTwi); 
  
    // Initialize driver
    pTwid->pTwi         = pTwi;
    pTwid->pTransfer    = NULL;
   }



//------------------------------------------------------------------------------
/// Interrupt handler for a TWI peripheral. Manages asynchronous transfer
/// occuring on the bus. This function MUST be called by the interrupt service
/// routine of the TWI peripheral if asynchronous read/write are needed.
//------------------------------------------------------------------------------
unsigned char twi_rw_done = 0;

void TWID_Handler( void )
{
    unsigned int status;
    AsyncTwi  *pTransfer;
    AT91S_TWI *pTwi     ;
    
    pTransfer   = (AsyncTwi *) twid.pTransfer;
    pTwi        =              twid.pTwi; 
    status      = TWI_GetMaskedStatus(pTwi);   // Retrieve interrupt status
  
    
        //if NAK detected, end operations
    if( TWI_STATUS_NACK(status) ) { 
      
        //BSP_IntClr(AT91C_ID_TWI ); 
        TWI_DisableIt(pTwi, AT91C_TWI_TXRDY | AT91C_TWI_RXRDY | AT91C_TWI_TXCOMP | AT91C_TWI_NACK );    
        pTransfer->status = 1; //NAK error
        if (pTransfer->callback) {            
           pTransfer->callback((Async *) pTransfer);
            
        }               
        //OSSemPost( TWI_Sem_done );
        twi_rw_done = 1;
    }
    // Byte received
    else if ( TWI_STATUS_RXRDY(status) ) {

        pTransfer->pData[pTransfer->transferred] = TWI_ReadByte(pTwi);
        pTransfer->transferred++;

        // Transfer finished ?
        if (pTransfer->transferred == pTransfer->num) {
            TWI_DisableIt(pTwi, AT91C_TWI_RXRDY);
            TWI_EnableIt(pTwi, AT91C_TWI_TXCOMP);
            
        }
        // Last byte ?
        else if ( pTransfer->transferred == (pTransfer->num - 1) ) {
            TWI_Stop(pTwi);   
            
        }
    // Byte sent       
    } else if (TWI_STATUS_TXRDY(status)) { 
         // Transfer finished ?
        if (pTransfer->transferred == pTransfer->num) {
#ifdef TWI_V3XX
        // Send a STOP condition
        TWI_Stop(pTwi); 
#endif
            TWI_DisableIt(pTwi, AT91C_TWI_TXRDY);
            TWI_EnableIt(pTwi, AT91C_TWI_TXCOMP);
            
        } else  {// Bytes remaining
            TWI_WriteByte(pTwi, pTransfer->pData[pTransfer->transferred]);
            pTransfer->transferred++;     
            
        }        
    // Transfer complete   
    } else if (TWI_STATUS_TXCOMP(status)) { 

        //BSP_IntClr(AT91C_ID_TWI ); 
        TWI_DisableIt(pTwi, AT91C_TWI_TXCOMP | AT91C_TWI_NACK );
        pTransfer->status = 0; //ok
        if (pTransfer->callback) {            
            pTransfer->callback((Async *) pTransfer);
            
        }              
        //OSSemPost( TWI_Sem_done );
        twi_rw_done = 1;
    }    
    

        
}


//-----------------------------------------------------------------------------
/// Asynchronously reads data from a slave on the TWI bus. An optional
/// callback function is triggered when the transfer is complete.
/// Returns 0 if the transfer has been started; otherwise returns a TWI error
/// code.
/// \param address  TWI slave address.
/// \param iaddress  Optional slave internal address.
/// \param isize  Internal address size in bytes.
/// \param pData  Data buffer for storing received bytes.
/// \param num  Number of bytes to read.
/// \param pAsync  Asynchronous transfer descriptor.
//-----------------------------------------------------------------------------
unsigned char TWID_Read      (
        unsigned char  address,
        unsigned int   iaddress,
        unsigned char  isize,
        unsigned char *pData,
        unsigned int   num,
        Async *pAsync         )
{  
  
    AT91S_TWI *pTwi ;
    AsyncTwi *pTransfer;
    unsigned int timeout;
    //unsigned char err;    
    unsigned char state; 
        
    //pAsync    = &twi_async; //force use async
    pTwi      = twid.pTwi; 
    pTransfer = (AsyncTwi *)twid.pTransfer; 
    state     = TWID_NO_ERROR;
    
    if( num == 0 ) {
        return state;
    } 
    
    if (pAsync) {  // Asynchronous transfer
        //OSSemPend( TWI_Sem_lock, 0, &err );  
        if (pTransfer) { // Check that no transfer is already pending
            //TRACE_ERROR("TWID_Read: A transfer is already pending\n\r");   
            state =  TWID_ERROR_BUSY;
            return state ;
        }
        if (num == 1) { // in case of there is only one byte data
            TWI_Stop(pTwi);  //stop must be send before the last byte
            
        }    
        // Update the transfer descriptor
        twid.pTransfer          = pAsync;
        pTransfer               = (AsyncTwi *)pAsync; 
        pTransfer->status       = ASYNC_STATUS_PENDING;
        pTransfer->pData        = pData;
        pTransfer->num          = num;
        pTransfer->transferred  = 0;   
        twi_rw_done = 0;
        // Enable read interrupt and start the transfer      
        //BSP_IntEn(AT91C_ID_TWI);
        TWI_StartRead(pTwi, address, iaddress, isize);
        TWI_EnableIt(pTwi, AT91C_TWI_RXRDY);
        TWI_EnableIt(pTwi, AT91C_TWI_NACK); 
        
        //OSSemPend( TWI_Sem_done, 1000, &err ); 
        timeout = 0;
        while( (twi_rw_done==0) && ( ++timeout < TWITIMEOUTMAX_BURST));
        //if( OS_ERR_NONE != err ) {
            TWI_DisableIt(pTwi, AT91C_TWI_TXRDY | AT91C_TWI_RXRDY | AT91C_TWI_TXCOMP | AT91C_TWI_NACK ); 
        //}
        state          = pTransfer->status ;
        if (timeout == TWITIMEOUTMAX) {
            //TRACE_ERROR("TWID Timeout TC\n\r");           
            state =  TWID_ERROR_TIMEOUT;           
        }      
        twid.pTransfer = NULL;
        //OSSemPost( TWI_Sem_lock );
    }  else {  // Synchronous transfer

        //OSSemPend( TWI_Sem_lock, 0, &err );  
        // Set STOP signal if only one byte is sent
        if (num == 1) {
            TWI_Stop(pTwi);
        }
        // Start read
        TWI_StartRead(pTwi, address, iaddress, isize);
        // Read all bytes, setting STOP before the last byte
        while (num > 0 ) {
            // Last byte
            if (num == 1) {
                TWI_Stop(pTwi);
            }
            // Wait for byte then read and store it
            timeout = 0;
            while( !TWI_ByteReceived(pTwi) && (++timeout<TWITIMEOUTMAX) );
            if (timeout == TWITIMEOUTMAX) {
                //TRACE_ERROR("TWID Timeout BR\n\r");
                 //OSSemPost( TWI_Sem_lock );
                 state =  TWID_ERROR_TIMEOUT;
                 return state;
            }
            *pData++ = TWI_ReadByte(pTwi);
            num--;
        }
        // Wait for transfer to be complete
        timeout = 0;
        while( !TWI_TransferComplete(pTwi) && (++timeout < TWITIMEOUTMAX) );
        if (timeout == TWITIMEOUTMAX) {
            //TRACE_ERROR("TWID Timeout TC\n\r");           
            state =  TWID_ERROR_TIMEOUT2;           
        }
        //OSSemPost( TWI_Sem_lock );
        
    }
       
    return state;
}

//------------------------------------------------------------------------------
/// Asynchronously sends data to a slave on the TWI bus. An optional callback
/// function is invoked whenever the transfer is complete.
/// \param address  Slave address.
/// \param iaddress  Optional slave internal address.
/// \param isize  Number of internal address bytes.
/// \param pData  Data buffer to send.
/// \param num  Number of bytes to send.
/// \param pAsync  Pointer to an Asynchronous transfer descriptor.
//------------------------------------------------------------------------------
unsigned char TWID_Write    (
        unsigned char  address,
        unsigned int   iaddress,
        unsigned char  isize,
        unsigned char *pData,
        unsigned int   num,
        Async *pAsync       )
{
    AT91S_TWI *pTwi;
    AsyncTwi *pTransfer;
    unsigned int timeout;
    //unsigned char err;     
    unsigned char state; 
        
    //pAsync    = &twi_async; //force use async    
    pTwi      = twid.pTwi; 
    pTransfer = (AsyncTwi *)twid.pTransfer;      
    state     = TWID_NO_ERROR;
    
    if( num == 0 ) {
        return state;
    }
    
    if (pAsync) {  // Asynchronous transfer
        //OSSemPend( TWI_Sem_lock, 0, &err );
        if (pTransfer) { // Check that no transfer is already pending
            //TRACE_ERROR("TWID_Read: A transfer is already pending\n\r");   
            state =  TWID_ERROR_BUSY;
            return state ;
        }         
             
        // Update the transfer descriptor
        twid.pTransfer          = pAsync;
        pTransfer               = (AsyncTwi *)pAsync;    
        pTransfer->status       = ASYNC_STATUS_PENDING;
        pTransfer->pData        = pData;
        pTransfer->num          = num;
        pTransfer->transferred  = 1;    
        twi_rw_done = 0;
        // Enable write interrupt and start the transfer
     
        //BSP_IntEn(AT91C_ID_TWI);
        TWI_StartWrite(pTwi, address, iaddress, isize, *pData); 
        TWI_EnableIt(pTwi, AT91C_TWI_TXRDY);         
        TWI_EnableIt(pTwi, AT91C_TWI_NACK);  
        //OSSemPend( TWI_Sem_done, 1000, &err );
        timeout = 0;
        while(  (twi_rw_done==0) && ( ++timeout<TWITIMEOUTMAX_BURST));
        //if( OS_ERR_NONE != err ) {
            TWI_DisableIt(pTwi, AT91C_TWI_TXRDY | AT91C_TWI_RXRDY | AT91C_TWI_TXCOMP | AT91C_TWI_NACK ); 
        //}
        state          =  pTransfer->status ;
        //APP_TRACE_INFO(("\r\nUIF_TYPE_SPI 1388 error: %d\r\n",timeout));
        if (timeout == TWITIMEOUTMAX) {
            //TRACE_ERROR("TWID Timeout TC\n\r");           
            state =  TWID_ERROR_TIMEOUT;           
        }
        twid.pTransfer =  NULL;
        //OSSemPost( TWI_Sem_lock );
    } else {   // Synchronous transfer   

        //OSSemPend( TWI_Sem_lock, 0, &err ); 
        // Start write
        TWI_StartWrite(pTwi, address, iaddress, isize, *pData++);
        num--;
        // Send all bytes
        while (num > 0) {        
            // Wait before sending the next byte
            timeout = 0;
            while( !TWI_ByteSent(pTwi) && (++timeout<TWITIMEOUTMAX) );
            if (timeout == TWITIMEOUTMAX) {
                //TRACE_ERROR("TWID Timeout BS\n\r");             
                 state =  TWID_ERROR_TIMEOUT;
                 return state;
            }
            TWI_WriteByte(pTwi, *pData++);
            num--;
        }

        // Wait for actual end of transfer
        timeout = 0;

        //TWI auto insert a stop in A3 chip
#ifdef TWI_V3XX
        // Send a STOP condition
        TWI_SendSTOPCondition(pTwi);
#endif
        
        while( !TWI_TransferComplete(pTwi) && (++timeout<TWITIMEOUTMAX) );
        if (timeout == TWITIMEOUTMAX) {
            //TRACE_ERROR("TWID Timeout TC2\n\r");   
            state =  TWID_ERROR_TIMEOUT2;         
        }
        //OSSemPost( TWI_Sem_lock );
    }     
    
    return state;
}



//TWI Initial
// Configure TWI
// In IRQ mode: to avoid problems, the priority of the TWI IRQ must be max.
// In polling mode: try to disable all IRQs if possible.
// (in this example it does not matter, there is only the TWI IRQ active)
void TWI_Init( unsigned int twi_clock  )
{
  
//#if (OS_EVENT_NAME_EN  > 0 )
//    CPU_INT08U  err;
//#endif
    
    PIO_Configure(  twi_pins, PIO_LISTSIZE(twi_pins)  );
    
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
    TWI_ConfigureMaster( AT91C_BASE_TWI, twi_clock, BOARD_MCK  );
    TWID_Initialize( &twid, AT91C_BASE_TWI );
    
    //TWI_Sem_lock  = OSSemCreate(1); 
    
//#if (OS_EVENT_NAME_EN > 0)
//   OSEventNameSet(TWI_Sem_lock,         "TWI_Sem_lock", &err);
//#endif
   

   
#ifdef TWI_ASYNC_MODE
        
    //TWI_Sem_done  = OSSemCreate(0); 
    
//#if (OS_EVENT_NAME_EN > 0)
//   OSEventNameSet(TWI_Sem_done,         "TWI_Sem_done", &err);
//#endif
   
    twi_async.callback  = 0 ;
       
    IRQ_ConfigureIT(AT91C_ID_TWI, TWI_PRIORITY, TWID_Handler);
    IRQ_EnableIT(AT91C_ID_TWI);
                                   
//    BSP_IntClr(AT91C_ID_TWI );     
//    BSP_IntEn(AT91C_ID_TWI); 
    
#endif
    
      
}





