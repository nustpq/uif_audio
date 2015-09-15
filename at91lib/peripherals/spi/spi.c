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
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <tc/tc.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <pmc/pmc.h>
#include <irq/nvic.h>
#include "dma/dma.h"
#include "spi.h"
#include "usb.h"




static const Pin spi_pins[]     = { PINS_SPI0, PIN_SPI0_NPCS0 } ;
static const Pin spi_pins_dis[] = { PINS_SPI0_DIS, PIN_SPI0_NPCS0_DIS } ;

AT91PS_SPI spi_if = AT91C_BASE_SPI0 ;

#define SPI_TIME_OUT  2000  //2000ms time out


//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//// Enables a SPI peripheral
//// \param spi  Pointer to an AT91S_SPI instance.
//------------------------------------------------------------------------------
void SPI_Enable(AT91S_SPI *spi)
{
    spi->SPI_CR = AT91C_SPI_SPIEN;
}

//------------------------------------------------------------------------------
//// Disables a SPI peripheral.
//// \param spi  Pointer to an AT91S_SPI instance.
//------------------------------------------------------------------------------
void SPI_Disable(AT91S_SPI *spi)
{
    spi->SPI_CR = AT91C_SPI_SPIDIS;
}

//------------------------------------------------------------------------------
//// Configures a SPI peripheral as specified. The configuration can be computed
//// using several macros (see "SPI configuration macros") and the constants
//// defined in LibV3 (AT91C_SPI_*).
//// \param spi  Pointer to an AT91S_SPI instance.
//// \param id  Peripheral ID of the SPI.
//// \param configuration  Value of the SPI configuration register.
//------------------------------------------------------------------------------
void SPI_Configure(AT91S_SPI *spi,
                          unsigned int id,
                          unsigned int configuration)
{
    AT91C_BASE_PMC->PMC_PCER = 1 << id;  
    spi->SPI_CR = AT91C_SPI_SPIDIS;
    // Execute a software reset of the SPI twice
    spi->SPI_CR = AT91C_SPI_SWRST;
    spi->SPI_CR = AT91C_SPI_SWRST;
    spi->SPI_MR = configuration;
    //APP_TRACE_INFO(("\r\nSPI_MR:0x%0X ",  spi_if->SPI_MR));
}

//------------------------------------------------------------------------------
//// Configures a chip select of a SPI peripheral. The chip select configuration
//// is computed using the definition provided by the LibV3 (AT91C_SPI_*).
//// \param spi  Pointer to an AT91S_SPI instance.
//// \param npcs  Chip select to configure (1, 2, 3 or 4).
//// \param configuration  Desired chip select configuration.
//------------------------------------------------------------------------------
void SPI_ConfigureNPCS(AT91S_SPI *spi,
                              unsigned int npcs,
                              unsigned int configuration)
{
    spi->SPI_CSR[npcs] = configuration;
    //APP_TRACE_INFO(("\r\nSPI_CSR[%d]:0x%0X ", npcs, spi_if->SPI_CSR[npcs]));
}

void Set_AT91C_SPI_CSAAT( AT91S_SPI *spi, unsigned int npcs ) //Force CS assert
{
    unsigned int temp ; 
    temp = spi->SPI_CSR[npcs];
    spi->SPI_CSR[npcs] = temp | AT91C_SPI_CSAAT ;
    
}

void Clear_AT91C_SPI_CSAAT( AT91S_SPI *spi, unsigned int npcs )//De-Force CS assert
{
    unsigned int temp ; 
    temp = spi->SPI_CSR[npcs];
    spi->SPI_CSR[npcs] = temp & (~AT91C_SPI_CSAAT) ;
    
}
//------------------------------------------------------------------------------
//// Sends data through a SPI peripheral. If the SPI is configured to use a fixed
//// peripheral select, the npcs value is meaningless. Otherwise, it identifies
//// the component which shall be addressed.
//// \param spi  Pointer to an AT91S_SPI instance.
//// \param npcs  Chip select of the component to address (1, 2, 3 or 4).
//// \param data  Word of data to send.
//------------------------------------------------------------------------------
void SPI_Write(AT91S_SPI *spi, unsigned int npcs, unsigned short data)
{
    //unsigned char  err;
    
    //OSSemPend( SPI_Sem, 0, &err );
    
    // Discard contents of RDR register
    //volatile unsigned int discard = spi->SPI_RDR;
    // Send data
    while ((spi->SPI_SR & AT91C_SPI_TXEMPTY) == 0);
    spi->SPI_TDR = data | SPI_PCS(npcs);
    while ((spi->SPI_SR & AT91C_SPI_TDRE) == 0);
   
    //OSSemPost( SPI_Sem );
      
}

//------------------------------------------------------------------------------
///// Sends the contents of buffer through a SPI peripheral, using the PDC to
///// take care of the transfer.
///// \param spi  Pointer to an AT91S_SPI instance.
///// \param buffer  Data buffer to send.
///// \param length  Length of the data buffer.
//------------------------------------------------------------------------------
unsigned char SPI_WriteBuffer(AT91S_SPI *spi,
                                     void *buffer,
                                     unsigned int length)
{
  
    if( length > 4096 ) { //single DMA tranfer length
        return 0;
    }
#if !defined(CHIP_SPI_DMA)
    
    // Check if first bank is free
    if (spi->SPI_TCR == 0) {
        spi->SPI_TPR = (unsigned int) buffer;
        spi->SPI_TCR = length;
        spi->SPI_PTCR = AT91C_PDC_TXTEN;
        return 1;
    }
    // Check if second bank is free
    else if (spi->SPI_TNCR == 0) {
        spi->SPI_TNPR = (unsigned int) buffer;
        spi->SPI_TNCR = length;
        return 1;
    }
    // No free bank
    else
    {
        return 0;
    }
    
#else
    
    unsigned char* startSourceAddr;
    unsigned char* startDestAddr;
    startSourceAddr = (unsigned char*)(buffer);
    startDestAddr   = (unsigned char*)(&spi->SPI_TDR);
       
    // Clear any pending interrupts
    DMA_GetStatus();
   
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_SADDR = (unsigned int)startSourceAddr; 
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_DADDR = (unsigned int)startDestAddr;
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_CTRLA = \
                                        (length \
                                        | AT91C_HDMA_SRC_WIDTH_BYTE \
                                        | AT91C_HDMA_DST_WIDTH_BYTE \
                                        | AT91C_HDMA_SCSIZE_1 \
                                        | AT91C_HDMA_DCSIZE_1) ; 
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_CTRLB = \
                                          AT91C_HDMA_SRC_DSCR_FETCH_DISABLE \
                                        | AT91C_HDMA_DST_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_FC_MEM2PER ;  
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_CFG = \
                                         (SPI_OUT_DMA_HW_SRC_REQ_ID \
                                        | SPI_OUT_DMA_HW_DEST_REQ_ID \
                                        | AT91C_HDMA_SRC_H2SEL_SW \
                                        | AT91C_HDMA_DST_H2SEL_HW \
                                        | AT91C_HDMA_SOD_ENABLE \
                                        | AT91C_HDMA_FIFOCFG_LARGESTBURST);        
    
 
    
    DMA_EnableChannel(BOARD_SPI_OUT_DMA_CHANNEL);
    
    return 1;
 
#endif      

}
 
//------------------------------------------------------------------------------
//// Returns 1 if there is no pending write operation on the SPI; otherwise
//// returns 0.
//// \param pSpi  Pointer to an AT91S_SPI instance.
//------------------------------------------------------------------------------
unsigned char SPI_IsWriteFinished(AT91S_SPI *pSpi)
{
#if !defined(CHIP_SPI_DMA)
 
    //   return ((pSpi->SPI_SR & AT91C_SPI_TXEMPTY) != 0);
   return ((pSpi->SPI_SR & AT91C_SPI_ENDTX) != 0);
#else
   return( DMAD_IsFinished(BOARD_SPI_OUT_DMA_CHANNEL) );
#endif
}


//------------------------------------------------------------------------------
//// Returns 1 if there is no pending write operation on the SPI; otherwise
//// returns 0.
//// \param pSpi  Pointer to an AT91S_SPI instance.
//------------------------------------------------------------------------------
unsigned char SPI_IsReadFinished(AT91S_SPI *pSpi)
{
    return ((pSpi->SPI_SR & AT91C_SPI_ENDRX) != 0);
}
//------------------------------------------------------------------------------
//// Reads and returns the last word of data received by a SPI peripheral. This
//// method must be called after a successful SPI_Write call.
//// \param spi  Pointer to an AT91S_SPI instance.
//------------------------------------------------------------------------------
unsigned short SPI_Read(AT91S_SPI *spi)
{
    unsigned char  err;
    unsigned short data;
    
    //OSSemPend( SPI_Sem, 0, &err );
    
    while ((spi->SPI_SR & AT91C_SPI_RDRF) == 0);
    //must add a delay, related to SPI seed  ? 
    delay_us(100);    
    data = spi->SPI_RDR & 0xFFFF;
    
    //OSSemPost( SPI_Sem );
     
    return data;
}

//------------------------------------------------------------------------------
//// Reads data from a SPI peripheral until the provided buffer is filled. This
//// method does NOT need to be called after SPI_Write or SPI_WriteBuffer.
//// \param spi  Pointer to an AT91S_SPI instance.
//// \param buffer  Data buffer to store incoming bytes.
//// \param length  Length in bytes of the data buffer.
//------------------------------------------------------------------------------
unsigned char SPI_ReadBuffer( AT91S_SPI *spi,
                                    void *buffer,
                                    unsigned int length )
{
    
    if( length > 4095 ) { //single DMA tranfer length
        return 0;
    }
    
#if !defined(CHIP_SPI_DMA)
    // Check if the first bank is free
    if (spi->SPI_RCR == 0) {

        spi->SPI_RPR = (unsigned int) buffer;
        spi->SPI_RCR = length ;         
        spi->SPI_TCR = length;
        spi->SPI_PTCR = AT91C_PDC_TXTEN | AT91C_PDC_RXTEN;
        
        return 1;
    }
    // Check if second bank is free
    else if (spi->SPI_RNCR == 0) {

        spi->SPI_RNPR = (unsigned int) buffer;
        spi->SPI_RNCR = length;        
        spi->SPI_TNCR = length;
        
        return 1;
    }    
    // No free bank
    else  
    {
        return 0;
    }
#else
    
    unsigned char* startSourceAddr;
    unsigned char* startDestAddr;
    startSourceAddr = (unsigned char*)(0);
    startDestAddr   = (unsigned char*)(&spi->SPI_TDR);
       
    // Clear any pending interrupts
    DMA_GetStatus();
   
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_SADDR = (unsigned int)startSourceAddr; 
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_DADDR = (unsigned int)startDestAddr;
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_CTRLA = \
                                        (length \
                                        | AT91C_HDMA_SRC_WIDTH_BYTE \
                                        | AT91C_HDMA_DST_WIDTH_BYTE \
                                        | AT91C_HDMA_SCSIZE_1 \
                                        | AT91C_HDMA_DCSIZE_1) ; 
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_CTRLB = \
                                          AT91C_HDMA_SRC_DSCR_FETCH_DISABLE \
                                        | AT91C_HDMA_DST_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_FC_MEM2PER ;  
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_OUT_DMA_CHANNEL].HDMA_CFG = \
                                         (SPI_OUT_DMA_HW_SRC_REQ_ID \
                                        | SPI_OUT_DMA_HW_DEST_REQ_ID \
                                        | AT91C_HDMA_SRC_H2SEL_SW \
                                        | AT91C_HDMA_DST_H2SEL_HW \
                                        | AT91C_HDMA_SOD_ENABLE \
                                        | AT91C_HDMA_FIFOCFG_LARGESTBURST);       
    

    startSourceAddr = (unsigned char*)(&spi->SPI_RDR);
    startDestAddr   = (unsigned char*)(buffer);
       
    // Clear any pending interrupts
    //DMA_GetStatus();
   
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_IN_DMA_CHANNEL].HDMA_SADDR = (unsigned int)startSourceAddr; 
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_IN_DMA_CHANNEL].HDMA_DADDR = (unsigned int)startDestAddr;
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_IN_DMA_CHANNEL].HDMA_CTRLA = \
                                        ((length+1) \
                                        | AT91C_HDMA_SRC_WIDTH_BYTE \
                                        | AT91C_HDMA_DST_WIDTH_BYTE \
                                        | AT91C_HDMA_SCSIZE_1 \
                                        | AT91C_HDMA_DCSIZE_1) ; //Add 1+ byte length to fix last byte lost bug //PQ
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_IN_DMA_CHANNEL].HDMA_CTRLB = \
                                          AT91C_HDMA_SRC_DSCR_FETCH_DISABLE \
                                        | AT91C_HDMA_DST_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_FC_PER2MEM ;       
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SPI_IN_DMA_CHANNEL].HDMA_CFG = \
                                         (SPI_IN_DMA_HW_SRC_REQ_ID \
                                        | SPI_IN_DMA_HW_DEST_REQ_ID \
                                        | AT91C_HDMA_SRC_H2SEL_HW \
                                        | AT91C_HDMA_DST_H2SEL_SW \
                                        | AT91C_HDMA_SOD_ENABLE \
                                        | AT91C_HDMA_FIFOCFG_LARGESTBURST); 
    

    DMA_EnableChannel(BOARD_SPI_IN_DMA_CHANNEL);
    DMA_EnableChannel(BOARD_SPI_OUT_DMA_CHANNEL);
    
    
    return 1;
    
#endif
 
}
              




//return 0 if write succeed
//return 1 if write failed
unsigned char SPI_WriteBuffer_API(  void *buffer,  unsigned int length )
{
    unsigned char state;
    unsigned char err = 0; 
    unsigned int  couter_ms  = 0 ; 
    unsigned int  couter_us  = 0 ; 

    if( length != 0 ) {
        
        //OSSemPend( SPI_Sem, 0, &err );  
        //GPIOPIN_Set_Fast(7,0);
        state = SPI_WriteBuffer( spi_if, buffer, length ); 
        if( state == 1 ) {
            //while( ! SPI_IsWriteFinished( spi_if ) ) {  
            while( !((AT91C_BASE_HDMA->HDMA_EBCISR) & (DMA_BTC<<BOARD_SPI_OUT_DMA_CHANNEL) ) ) {
                //AT91C_BASE_HDMA->HDMA_CHSR
                if( couter_ms >  0 ) {
                    delay_ms(1);  
                    if( couter_ms++ > SPI_TIME_OUT ) { //timeout : 2s
                        err = 2 ;
                        break;
                    }
                } else {  
                  delay_us(5);
                  if( couter_us++ > 200 ) {
                      couter_ms = 1 ;
                  }
                } 
            }
        } else {
            err = 1;
            
        }    
        //GPIOPIN_Set_Fast(7,1);
        //delay_us(50);
        while( !( spi_if->SPI_SR & AT91C_SPI_TXEMPTY ) );
        
        DMA_DisableChannel( BOARD_SPI_OUT_DMA_CHANNEL );
        //OSSemPost( SPI_Sem ); 
        //APP_TRACE_INFO(("\r\nstate:0x%d, couter: %d ",state,couter));
    
    }
    
    return err ;
    
}




//return 0 if read succeed
//return 1 if read failed
unsigned char SPI_ReadBuffer_API(  void *buffer,  unsigned int length )
{
    unsigned char state;
    unsigned char err = 0;  
    unsigned int  couter_ms  = 0 ; 
    unsigned int  couter_us  = 0 ;   
    
    if( length != 0 ) {
        
        //OSSemPend( SPI_Sem, 0, &err ); 
        state = SPI_ReadBuffer( spi_if, buffer,length );     
        if( state == 1 ) {
            //while( ! SPI_IsReadFinished( spi_if ) ) {
            while( !((AT91C_BASE_HDMA->HDMA_EBCISR) & (DMA_BTC<<BOARD_SPI_OUT_DMA_CHANNEL) ) ) {      
                if( couter_ms >  0 ) {
                    delay_ms(1);  
                    if( couter_ms++ > SPI_TIME_OUT ) { //timeout : 2s
                        err = 2 ;
                        break;
                    }
                } else {  
                  delay_us(5);
                  if( couter_us++ > 200 ) {
                      couter_ms = 1 ;
                  }
                }  
            }
            
        } else {        
            err = 1;

        }   
        
        DMA_DisableChannel( BOARD_SPI_IN_DMA_CHANNEL );
        //OSSemPost( SPI_Sem ); 
    }
    
    return err ;
    
}


unsigned char SPI_WriteReadBuffer_API(  void *buffer_r,  void *buffer_w, unsigned int length_r,  unsigned int length_w )
{      
    unsigned char state;
    unsigned char err = 0; 
    unsigned int  couter_ms  = 1 ; 
    unsigned int  couter_us  = 0 ; 
   
    //GPIOPIN_Set_Fast(7,0);
      
    if( length_w != 0 ) {
        if( length_r != 0 ) {
            Set_AT91C_SPI_CSAAT( spi_if, 0 );
        }
        state = SPI_WriteBuffer( spi_if, buffer_w, length_w ); 
        if( state == 1 ) {               
            while( !((AT91C_BASE_HDMA->HDMA_EBCISR) & (DMA_BTC<<BOARD_SPI_OUT_DMA_CHANNEL) ) ) {
                if( couter_ms >  0 ) {
                    delay_ms(1);//OSTimeDly(1);  
                    if( couter_ms++ > SPI_TIME_OUT ) { //timeout : 2s
                        err = 2 ;
                        break;
                    }
                } else {  
                  delay_us(5);
                  if( couter_us++ > 200 ) {
                      couter_ms = 1 ;
                  }
                } 
            }
            
        } else {
            err = 1;
            
        }
        //GPIOPIN_Set_Fast(7,1);
        //delay_us(50);
        while( !( spi_if->SPI_SR & AT91C_SPI_TXEMPTY ) );
        DMA_DisableChannel( BOARD_SPI_OUT_DMA_CHANNEL );    
        if( length_r != 0 ) {
            Clear_AT91C_SPI_CSAAT( spi_if, 0 ); 
        }
                    
        if( err != 0 ) {
            return err;
        }
        
    }

    
    couter_ms  = 1 ; 
    couter_us  = 0 ; 
    //GPIOPIN_Set_Fast(7,0);
    if( length_r != 0 ) { 
       
        state = SPI_ReadBuffer( spi_if, buffer_r, length_r );
         
        if( state == 1 ) {
            //while( ! SPI_IsReadFinished( spi_if ) ) {
            while( !((AT91C_BASE_HDMA->HDMA_EBCISR) & (DMA_BTC<<BOARD_SPI_IN_DMA_CHANNEL) ) ) {      
                if( couter_ms > 0 ) {
                    delay_ms(1);//OSTimeDly(1);  
                    if( couter_ms++ > SPI_TIME_OUT ) { //timeout : 2s
                        err = 2 ;
                        break;
                    }
                } else {  
                  delay_us(5);
                  if( couter_us++ > 200 ) {
                      couter_ms = 1 ;
                  }
                }           
            }             
        } else {        
            err = 1;
        }   
        //GPIOPIN_Set_Fast(7,1);
        //delay_us(50);
        while( !( spi_if->SPI_SR & AT91C_SPI_TXEMPTY ) );
        DMA_DisableChannel( BOARD_SPI_OUT_DMA_CHANNEL );
        DMA_DisableChannel( BOARD_SPI_IN_DMA_CHANNEL );
    }
  
    return err ;    
    
}


void SPI_Initialize( AT91S_SPI *spi, unsigned int npcs, unsigned int spi_clk, unsigned int mclk, unsigned int mode )
{
   
    unsigned char err;
    unsigned int  clk_div ;
     
    clk_div = mclk / spi_clk ;    
    if( clk_div > 255 ){      
        clk_div = 255;
    }    
    //ASSERT(clk>0, "-F- Invalid SPI clock parameters\n\r");
    
    //GPIOPIN_Set_Fast(7,1);      
    PIO_Configure(spi_pins, PIO_LISTSIZE(spi_pins) ); 
   
    SPI_Configure(spi, AT91C_ID_SPI0,  AT91C_SPI_MSTR | AT91C_SPI_MODFDIS |(0x0E << 16))  ;   //0E(1110) for fixed select NPCS0     

    SPI_ConfigureNPCS(spi, npcs,  mode |  (clk_div << 8) | (0<<16) )  ; //delay after NPCS active before send data: MCLK/200, 0 means 1/2 bclk
    //SPI_ConfigureNPCS(spi, npcs,  mode |  (clk_div << 8) | AT91C_SPI_CSNAAT | (0<<16) | ((spi_clk/1000000)<<24))  ; //delay after NPCS active before send data: MCLK/200, 0 means 1/2 bclk, CS delay 1 BCLK cycle
    
    SPI_Enable(spi);   
     
   
} 

void SPI_Init(  unsigned int spi_clk, unsigned char format )  //based on polling method
{

   unsigned int mode;
   
   if( format > 3 ) {
       format = 0;
   }
   
   printf("\r\nSet SPI: Speed=%d kHz, [CPHA..CPOL]=%d",spi_clk/1000, format );
   
   switch( format ) {
              
       case 0 :  //keep SPCK Low, Rising edge latch data   // =0 for iM501   .MCU[NCPHA:CPOL]= 1:0, iM501[CPHA:CPOL]=0:0
            mode = 0 | AT91C_SPI_BITS_8 | AT91C_SPI_NCPHA; 
       break;  
       
       case 1 :  //keep SPCK High, Falling edge latch data // =1 for iM501    .MCU[NCPHA:CPOL]= 1:1, iM501[CPHA:CPOL]=0:1
            mode = 0 | AT91C_SPI_BITS_8 | AT91C_SPI_CPOL | AT91C_SPI_NCPHA; 
       break; 
       
       case 2 :  //keep SPCK Low,  Falling edge latch data // =2 for iM501  .MCU[NCPHA:CPOL]= 0:0, iM501[CPHA:CPOL]=1:0
            mode = 0 | AT91C_SPI_BITS_8 ; 
       break; 
       
       case 3 :  //keep SPCK High, Rising edge latch data  // = 3 for iM501    .MCU[NCPHA:CPOL]= 0:1, iM501[CPHA:CPOL]=1:1
            mode = 0 | AT91C_SPI_BITS_8 | AT91C_SPI_CPOL; 
       break;
       
       default: //keep SPCK Low, Rising edge latch data   // =0 for iM501
             mode = 0 | AT91C_SPI_BITS_8 | AT91C_SPI_NCPHA; 
       break;
       
   }
   
   IRQ_DisableIT( AT91C_ID_SPI0 );
   //IRQ_DisableIT( AT91C_ID_HDMA ); //in case of affect I2S DMA
   
   SPI_Initialize( spi_if, 0, spi_clk, BOARD_MCK, mode ) ;   // Configure SPI   
         
//   // Initialize DMA controller.    
//   DMAD_Initialize(BOARD_SPI_IN_DMA_CHANNEL);    
//   DMAD_Initialize(BOARD_SPI_OUT_DMA_CHANNEL); 
   
//    DMA_DisableIt( 1 << (BOARD_SPI_OUT_DMA_CHANNEL + 0) );
//    DMA_DisableIt( 1 << (BOARD_SPI_IN_DMA_CHANNEL + 0) );
//    DMA_DisableChannel(BOARD_SPI_OUT_DMA_CHANNEL); 
//    DMA_DisableChannel(BOARD_SPI_IN_DMA_CHANNEL);
    
   // Configure and enable the SSC interrupt   
   //BSP_IntVectSet( AT91C_ID_HDMA,                         
   //                (CPU_FNCT_VOID)DMAD_Handler);
   //IRQ_ConfigureIT(AT91C_ID_HDMA, AT91C_AIC_PRIOR_LOWEST-1, NULL);
   //IRQ_EnableIT(AT91C_ID_HDMA);
   //DMA_EnableChannel(BOARD_SPI_DMA_CHANNEL);
}

   
         
void Enable_SPI_Port( unsigned int spi_clk, unsigned char format )
{
    //SPI_Init(spi_clk,format);
    PIO_Configure(spi_pins, PIO_LISTSIZE(spi_pins) ); 
}


void Disable_SPI_Port( void )
{
    PIO_Configure(spi_pins_dis, PIO_LISTSIZE(spi_pins_dis) ); 
}

