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

#include "ssc.h"
#include <utility/trace.h>
#include <tc/tc.h>
#if defined(CHIP_SSC_DMA)
#include <dma/dma.h>
#include <dmad/dmad.h>
#endif


TCMR tcmr ;
TFMR tfmr ;
RCMR rcmr ;
RFMR rfmr ;

unsigned int  DMA_CtrA_Reg_Mode;
unsigned char DMA_CtrA_Len_Shift;


#define DMA_CTRA_MODE_16BIT             ( AT91C_HDMA_SRC_WIDTH_HALFWORD \
                                        | AT91C_HDMA_DST_WIDTH_HALFWORD \
                                        | AT91C_HDMA_SCSIZE_1 \
                                        | AT91C_HDMA_DCSIZE_1);                             

#define DMA_CTRA_MODE_32BIT             ( AT91C_HDMA_SRC_WIDTH_WORD \
                                        | AT91C_HDMA_DST_WIDTH_WORD \
                                        | AT91C_HDMA_SCSIZE_1 \
                                        | AT91C_HDMA_DCSIZE_1); 


//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------
#if defined(CHIP_SSC_DMA)
static DmaLinkList  LLI_CH [MAX_SSC_LLI_SIZE];
static DmaLinkList  LLI_CH2 [MAX_SSC_LLI_SIZE];
#endif

//------------------------------------------------------------------------------
//         Local macros
//------------------------------------------------------------------------------
#if defined(CHIP_SSC_DMA)
#define     LAST_ROW            0x100
#endif

//------------------------------------------------------------------------------
//         Internal Functions
//------------------------------------------------------------------------------
#if defined(CHIP_SSC_DMA)
//void AT91F_Prepare_Multiple_Transfer(unsigned int Channel,
//                                            unsigned int LLI_rownumber,
//                                            unsigned int LLI_Last_Row,
//                                            unsigned int From_add,
//                                            unsigned int To_add,
//                                            unsigned int Ctrla,
//                                            unsigned int Ctrlb)
//{
//    LLI_CH[LLI_rownumber].sourceAddress =  From_add;
//    LLI_CH[LLI_rownumber].destAddress   =  To_add;
//    LLI_CH[LLI_rownumber].controlA =  Ctrla;
//    LLI_CH[LLI_rownumber].controlB =  Ctrlb;
//    if (LLI_Last_Row != LAST_ROW)
//        LLI_CH[LLI_rownumber].descriptor =
//             (unsigned int)&LLI_CH[LLI_rownumber + 1] + 0;
//    else
//        LLI_CH[LLI_rownumber].descriptor = 0;
//}

void AT91F_Prepare_Multiple_Transfer(unsigned int Channel,
                                            unsigned int LLI_rownumber,
                                            unsigned int LLI_Last_Row,
                                            unsigned int From_add,
                                            unsigned int To_add,
                                            unsigned int Ctrla,
                                            unsigned int Ctrlb)
{
    LLI_CH[LLI_rownumber].sourceAddress =  From_add;
    LLI_CH[LLI_rownumber].destAddress   =  To_add;
    LLI_CH[LLI_rownumber].controlA =  Ctrla;
    LLI_CH[LLI_rownumber].controlB =  Ctrlb;
    if (LLI_Last_Row != LAST_ROW)
        LLI_CH[LLI_rownumber].descriptor =
             (unsigned int)&LLI_CH[1-LLI_rownumber] + 0;
    else
        LLI_CH[LLI_rownumber].descriptor = 0;
}


#endif

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Configures a SSC peripheral. If the divided clock is not used, the master
/// clock frequency can be set to 0.
/// \note The emitter and transmitter are disabled by this function.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param id  Peripheral ID of the SSC.
//------------------------------------------------------------------------------
void SSC_Configure(AT91S_SSC *ssc,
                          unsigned int id,
                          unsigned int bitRate,
                          unsigned int masterClock)
{
    // Enable SSC peripheral clock
    AT91C_BASE_PMC->PMC_PCER = 1 << id;

    // Reset, disable receiver & transmitter
    ssc->SSC_CR = AT91C_SSC_RXDIS | AT91C_SSC_TXDIS | AT91C_SSC_SWRST;

    #if !defined( CHIP_SSC_DMA )
    ssc->SSC_PTCR = AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS;
    #endif

    // Configure clock frequency
    if (bitRate != 0) {
    
        ssc->SSC_CMR = masterClock / (2 * bitRate);
    }
    else {

        ssc->SSC_CMR = 0;
    }
}


//------------------------------------------------------------------------------
/// Configures the transmitter of a SSC peripheral. Several macros can be used
/// to compute the values of the Transmit Clock Mode Register (TCMR) and the
/// Transmit Frame Mode Register (TFMR) (see "SSC configuration macros").
/// \param ssc  Pointer to a AT91S_SSC instance.
/// \param tcmr  Transmit Clock Mode Register value.
/// \param tfmr  Transmit Frame Mode Register value.
//------------------------------------------------------------------------------
void SSC_ConfigureTransmitter(AT91S_SSC *ssc,
                                     unsigned int tcmr,
                                     unsigned int tfmr)
{
    ssc->SSC_TCMR = tcmr;
    ssc->SSC_TFMR = tfmr;
}

//------------------------------------------------------------------------------
/// Configures the receiver of a SSC peripheral. Several macros can be used
/// to compute the values of the Receive Clock Mode Register (TCMR) and the
/// Receive Frame Mode Register (TFMR) (see "SSC configuration macros").
/// \param ssc  Pointer to a AT91S_SSC instance.
/// \param rcmr  Receive Clock Mode Register value.
/// \param rfmr  Receive Frame Mode Register value.
//------------------------------------------------------------------------------
void SSC_ConfigureReceiver(AT91S_SSC *ssc,
                                  unsigned int rcmr,
                                  unsigned int rfmr)
{
    ssc->SSC_RCMR = rcmr;
    ssc->SSC_RFMR = rfmr;
}

//------------------------------------------------------------------------------
/// Enables the transmitter of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
void SSC_EnableTransmitter(AT91S_SSC *ssc)
{
    ssc->SSC_CR = AT91C_SSC_TXEN;
}

//------------------------------------------------------------------------------
/// Disables the transmitter of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
void SSC_DisableTransmitter(AT91S_SSC *ssc)
{
    ssc->SSC_CR = AT91C_SSC_TXDIS;
}

//------------------------------------------------------------------------------
/// Enables the receiver of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
void SSC_EnableReceiver(AT91S_SSC *ssc)
{
    ssc->SSC_CR = AT91C_SSC_RXEN;
}

void SSC_EnableBoth( AT91S_SSC *ssc )
{
  ssc->SSC_CR = AT91C_SSC_RXEN | AT91C_SSC_TXEN;
}

//------------------------------------------------------------------------------
/// Disables the receiver of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
void SSC_DisableReceiver(AT91S_SSC *ssc)
{
    ssc->SSC_CR = AT91C_SSC_RXDIS;
}

//------------------------------------------------------------------------------
/// Enables one or more interrupt sources of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param sources  Interrupt sources to enable.
//------------------------------------------------------------------------------
void SSC_EnableInterrupts(AT91S_SSC *ssc, unsigned int sources)
{
    ssc->SSC_IER = sources;
}

//------------------------------------------------------------------------------
/// Disables one or more interrupt sources of a SSC peripheral.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param sources  Interrupt source to disable.
//------------------------------------------------------------------------------
void SSC_DisableInterrupts(AT91S_SSC *ssc, unsigned int sources)
{
    ssc->SSC_IDR = sources;
}

//------------------------------------------------------------------------------
/// Sends one data frame through a SSC peripheral. If another frame is currently
/// being sent, this function waits for the previous transfer to complete.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param frame  Data frame to send.
//------------------------------------------------------------------------------
void SSC_Write(AT91S_SSC *ssc, unsigned int frame)
{
    //while ((ssc->SSC_SR & AT91C_SSC_TXRDY) == 0);
    ssc->SSC_THR = frame;
}


//------------------------------------------------------------------------------
/// Waits until one frame is received on a SSC peripheral, and returns it.
/// \param ssc  Pointer to an AT91S_SSC instance.
//------------------------------------------------------------------------------
unsigned int SSC_Read(AT91S_SSC *ssc)
{
    while ((ssc->SSC_SR & AT91C_SSC_RXRDY) == 0);
    return ssc->SSC_RHR;
}


//------------------------------------------------------------------------------
/// Sends the contents of a data buffer a SSC peripheral, using the PDC. Returns
/// true if the buffer has been queued for transmission; otherwise returns
/// false.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param buffer  Data buffer to send.
/// \param length  Size of the data buffer.
//------------------------------------------------------------------------------
//modified for fast execution
/*
unsigned char SSC_WriteBuffer(  AT91S_SSC *ssc,
                                     void *buffer,
                                     unsigned int length) 
{
    unsigned short* startSourceAddr;
    unsigned short* startDestAddr;
    unsigned int srcAddress;
    unsigned int destAddress;

    startSourceAddr = (unsigned short*)(buffer);
    startDestAddr   = (unsigned short*)(&ssc->SSC_THR);
    srcAddress      = (unsigned int)startSourceAddr;    // Set the data start address
    destAddress     = (unsigned int)startDestAddr;
 

    // Clear any pending interrupts
    AT91C_BASE_HDMA->HDMA_EBCISR;//read EBCISR
        
    // Set DMA channel config
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_CFG =  \
                                         (BOARD_SSC_OUT_DMA_HW_SRC_REQ_ID \
                                        | BOARD_SSC_OUT_DMA_HW_DEST_REQ_ID \
                                        | AT91C_HDMA_SRC_H2SEL_SW \
                                        | AT91C_HDMA_DST_H2SEL_HW \
                                        | AT91C_HDMA_SOD_DISABLE \
                                        | AT91C_HDMA_FIFOCFG_LARGESTBURST); 
   
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_SADDR = srcAddress;
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_DADDR = destAddress;    
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_CTRLA = \
                                        ((length>>1) \
                                        | AT91C_HDMA_SRC_WIDTH_HALFWORD \
                                        | AT91C_HDMA_DST_WIDTH_HALFWORD \
                                        | AT91C_HDMA_SCSIZE_1 \
                                        | AT91C_HDMA_DCSIZE_1);
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_CTRLB =  \
                                         (AT91C_HDMA_DST_DSCR_FETCH_DISABLE \
                                        | AT91C_HDMA_DST_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_SRC_DSCR_FETCH_DISABLE \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_FC_MEM2PER );

  
    return 0 ;
  
}
*/

 


unsigned char SSC_WriteBuffer_Start(  AT91S_SSC *ssc,
                                     void *buffer,
                                     void *buffer_next,
                                     unsigned int length)
{
    
    //LLI DMA method
    unsigned short* startSourceAddr;
    unsigned short* startSourceAddr_next;
    unsigned short* startDestAddr;
    unsigned int srcAddress;
    unsigned int srcAddress_next;
    unsigned int destAddress;
    //unsigned int buffSize;

    startSourceAddr      = (unsigned short*)(buffer);
    startSourceAddr_next = (unsigned short*)(buffer_next);
    startDestAddr        = (unsigned short*)(&ssc->SSC_THR);
    srcAddress           = (unsigned int)startSourceAddr;    // Set the data start address
    srcAddress_next      = (unsigned int)startSourceAddr_next;    // Set the data start address
    destAddress          = (unsigned int)startDestAddr;
    //buffSize             = length;
    
//    if(buffSize > 0x8000){
//        TRACE_WARNING("SSC DMA, size too big %d\n\r", buffSize);
//        buffSize = 0x8000;
//    }

    // Set DMA channel DSCR
    //DMA_SetDescriptorAddr(BOARD_SSC_OUT_DMA_CHANNEL, (unsigned int)&LLI_CH[0]);
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_DSCR = (unsigned int)&LLI_CH[0] ;
    
    // Clear any pending interrupts
    //DMA_GetStatus();

   
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_CTRLA = \
                                         (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_CTRLB = \
                                          AT91C_HDMA_DST_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_FC_MEM2PER ;
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_OUT_DMA_CHANNEL].HDMA_CFG =  \
                                         (BOARD_SSC_OUT_DMA_HW_SRC_REQ_ID \
                                        | BOARD_SSC_OUT_DMA_HW_DEST_REQ_ID \
                                        | AT91C_HDMA_SRC_H2SEL_SW \
                                        | AT91C_HDMA_DST_H2SEL_HW \
                                        | AT91C_HDMA_SOD_DISABLE \
                                        | AT91C_HDMA_FIFOCFG_LARGESTBURST); 
        
    LLI_CH[0].sourceAddress =  srcAddress;
    LLI_CH[0].destAddress   =  destAddress;
    LLI_CH[0].controlA      =  (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;
    LLI_CH[0].controlB      =   \
                               AT91C_HDMA_DST_ADDRESS_MODE_FIXED \
                             | AT91C_HDMA_SRC_ADDRESS_MODE_INCR \
                             | AT91C_HDMA_FC_MEM2PER ;
    LLI_CH[0].descriptor    =  (unsigned int)&LLI_CH[1] + 0;
  
    LLI_CH[1].sourceAddress =  srcAddress_next;
    LLI_CH[1].destAddress   =  destAddress;
    LLI_CH[1].controlA      =  (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;
    LLI_CH[1].controlB      =   \
                               AT91C_HDMA_DST_ADDRESS_MODE_FIXED \
                             | AT91C_HDMA_SRC_ADDRESS_MODE_INCR \
                             | AT91C_HDMA_FC_MEM2PER ;
    LLI_CH[1].descriptor    =  (unsigned int)&LLI_CH[0] + 0;    
        
    return 0;
    
}


unsigned char SSC_WriteBuffer(  AT91S_SSC *ssc,
                                     void *buffer,
                                     unsigned char buffer_index,
                                     unsigned int length)
{
   
    //LLI DMA method
    unsigned short* startSourceAddr;
    unsigned short* startDestAddr;
    unsigned int srcAddress;
    unsigned int destAddress;
    //unsigned int buffSize;

    startSourceAddr = (unsigned short*)(buffer);
    startDestAddr   = (unsigned short*)(&ssc->SSC_THR);
    
    srcAddress      = (unsigned int)startSourceAddr;   
    destAddress     = (unsigned int)startDestAddr;
    //buffSize        = length;
    
//    if(buffSize > 0x8000){
//        TRACE_WARNING("SSC DMA, size too big %d\n\r", buffSize);
//        buffSize = 0x8000;
//    }
           
    LLI_CH[buffer_index].sourceAddress =  srcAddress;
    LLI_CH[buffer_index].destAddress   =  destAddress;
    LLI_CH[buffer_index].controlA      =  (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;
    LLI_CH[buffer_index].controlB      = \
                                          AT91C_HDMA_DST_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_FC_MEM2PER ;
    LLI_CH[buffer_index].descriptor    = length == 0 ? 0 : ((unsigned int)&LLI_CH[1-buffer_index] + 0);
    
    return 0;
    
}


//------------------------------------------------------------------------------
/// Reads data coming from a SSC peripheral receiver and stores it into the
/// provided buffer. Returns true if the buffer has been queued for reception;
/// otherwise returns false.
/// \param ssc  Pointer to an AT91S_SSC instance.
/// \param buffer  Data buffer used for reception.
/// \param length  Size in bytes of the data buffer.
//------------------------------------------------------------------------------
//modified for fast execution
/*
unsigned char SSC_ReadBuffer(  AT91S_SSC *ssc,
                                    void *buffer,
                                    unsigned int length)
{
    unsigned short* startSourceAddr;
    unsigned short* startDestAddr;
    unsigned int srcAddress;
    unsigned int destAddress;
 
    startSourceAddr = (unsigned short*)(&ssc->SSC_RHR);
    startDestAddr   = (unsigned short*)(buffer);
    srcAddress      = (unsigned int)startSourceAddr;    // Set the data start address
    destAddress     = (unsigned int)startDestAddr;
  
    // Clear any pending interrupts
    AT91C_BASE_HDMA->HDMA_EBCISR; //read EBCISR ????
    
    
    // Set DMA channel config
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_CTRLA = \
                                        ((length>>1) \
                                        | AT91C_HDMA_SRC_WIDTH_HALFWORD \
                                        | AT91C_HDMA_DST_WIDTH_HALFWORD \
                                        | AT91C_HDMA_SCSIZE_1 \
                                        | AT91C_HDMA_DCSIZE_1);  
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_SADDR = srcAddress;
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_DADDR = destAddress;
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_CTRLB = \
                                         (AT91C_HDMA_DST_DSCR_FETCH_DISABLE \
                                        | AT91C_HDMA_DST_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_SRC_DSCR_FETCH_DISABLE \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_FC_PER2MEM );  
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_CFG = \
                                         (BOARD_SSC_IN_DMA_HW_SRC_REQ_ID \
                                        | BOARD_SSC_IN_DMA_HW_DEST_REQ_ID \
                                        | AT91C_HDMA_SRC_H2SEL_HW \
                                        | AT91C_HDMA_DST_H2SEL_SW \
                                        | AT91C_HDMA_SOD_DISABLE \
                                        | AT91C_HDMA_FIFOCFG_LARGESTBURST);   
    
    return 0;
  
  
}
*/

unsigned char SSC_ReadBuffer_Start(  AT91S_SSC *ssc,
                                     void *buffer,
                                     void *buffer_next,
                                     unsigned int length)
{
    
    //LLI DMA method
    unsigned short* startSourceAddr;
    unsigned short* startDestAddr;
    unsigned short* startDestAddr_next;
    unsigned int srcAddress;
    unsigned int destAddress_next;
    unsigned int destAddress;
    //unsigned int buffSize;

    startSourceAddr    = (unsigned short*)(&ssc->SSC_RHR);
    startDestAddr      = (unsigned short*)(buffer);
    startDestAddr_next = (unsigned short*)(buffer_next);
    
    srcAddress        = (unsigned int)startSourceAddr;    // Set the data start address
    destAddress_next  = (unsigned int)startDestAddr_next;    
    destAddress       = (unsigned int)startDestAddr;
    //buffSize          = length;
    
//    if(buffSize > 0x8000){
//        TRACE_WARNING("SSC DMA, size too big %d\n\r", buffSize);
//        buffSize = 0x8000;
//    }

    // Set DMA channel DSCR    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_DSCR = (unsigned int)&LLI_CH2[0] ;
    
    // Clear any pending interrupts
    //DMA_GetStatus();
   
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_CTRLA = \
                                        (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_CTRLB = \
                                          AT91C_HDMA_DST_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_FC_PER2MEM ;  
    
    AT91C_BASE_HDMA->HDMA_CH[BOARD_SSC_IN_DMA_CHANNEL].HDMA_CFG = \
                                         (BOARD_SSC_IN_DMA_HW_SRC_REQ_ID \
                                        | BOARD_SSC_IN_DMA_HW_DEST_REQ_ID \
                                        | AT91C_HDMA_SRC_H2SEL_HW \
                                        | AT91C_HDMA_DST_H2SEL_SW \
                                        | AT91C_HDMA_SOD_DISABLE \
                                        | AT91C_HDMA_FIFOCFG_LARGESTBURST);       
    
    LLI_CH2[0].sourceAddress =  srcAddress;
    LLI_CH2[0].destAddress   =  destAddress;
    LLI_CH2[0].controlA      = (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;    
    LLI_CH2[0].controlB      =    \
                                 AT91C_HDMA_DST_ADDRESS_MODE_INCR \
                               | AT91C_HDMA_SRC_ADDRESS_MODE_FIXED \
                               | AT91C_HDMA_FC_PER2MEM ; 
    LLI_CH2[0].descriptor    =  (unsigned int)&LLI_CH2[1] + 0;
    
    LLI_CH2[1].sourceAddress =  srcAddress;
    LLI_CH2[1].destAddress   =  destAddress_next;
    LLI_CH2[1].controlA      = (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;
    LLI_CH2[1].controlB      =    \
                                AT91C_HDMA_DST_ADDRESS_MODE_INCR \
                              | AT91C_HDMA_SRC_ADDRESS_MODE_FIXED \
                              | AT91C_HDMA_FC_PER2MEM ; 
    LLI_CH2[1].descriptor    =  (unsigned int)&LLI_CH2[0] + 0;    
        
    return 0;
    
}


unsigned char SSC_ReadBuffer(  AT91S_SSC *ssc,
                                     void *buffer,
                                     unsigned char buffer_index,
                                     unsigned int length)
{
    
    //LLI DMA method
    unsigned short* startSourceAddr;
    unsigned short* startDestAddr;
    unsigned int srcAddress;
    unsigned int destAddress;
    //unsigned int buffSize;

    startDestAddr   = (unsigned short*)(buffer);
    startSourceAddr = (unsigned short*)(&ssc->SSC_RHR);
    
    srcAddress      = (unsigned int)startSourceAddr;    // Set the data start address
    destAddress     = (unsigned int)startDestAddr;
    //buffSize        = length;
    
//    if(buffSize > 0x8000){
//        TRACE_WARNING("SSC DMA, size too big %d\n\r", buffSize);
//        buffSize = 0x8000;
//    }
           
    LLI_CH2[buffer_index].sourceAddress =  srcAddress;
    LLI_CH2[buffer_index].destAddress   =  destAddress;
    LLI_CH2[buffer_index].controlA      = (length>>DMA_CtrA_Len_Shift) | DMA_CtrA_Reg_Mode;
    LLI_CH2[buffer_index].controlB      =  \
                                          AT91C_HDMA_DST_ADDRESS_MODE_INCR \
                                        | AT91C_HDMA_SRC_ADDRESS_MODE_FIXED \
                                        | AT91C_HDMA_FC_PER2MEM ; 
    LLI_CH2[buffer_index].descriptor    =  length == 0 ? 0 : ((unsigned int)&LLI_CH2[1-buffer_index] + 0);
    
    return 0;
    
}

 
 
//void SSC_Channel_Set( unsigned char tx_ch_num, unsigned char rx_ch_num )
//{   
//    
//    if( tx_ch_num > 0 ) {
//        tfmr.datnb  = tx_ch_num - 1 ; //5 ; //6 slot TDM
//    }
//    
//    if( rx_ch_num > 0 ) {
//        rfmr.datnb  = rx_ch_num - 1 ; //5 ; 
//    }
//    
//    SSC_ConfigureTransmitter( BOARD_AT73C213_SSC,  tcmr.value,  tfmr.value   );
//    SSC_ConfigureReceiver(  BOARD_AT73C213_SSC,  rcmr.value , rfmr.value   );
//    
//}

void SSC_Channel_Set_Tx( unsigned char tx_ch_num, unsigned char slot_len, unsigned char cki, unsigned char delay, unsigned char start )
{       
    
    if( slot_len == 16 ) {
        DMA_CtrA_Reg_Mode  = DMA_CTRA_MODE_16BIT ;
        DMA_CtrA_Len_Shift = 1;    
    } else { //32bit
        DMA_CtrA_Reg_Mode  = DMA_CTRA_MODE_32BIT ;
        DMA_CtrA_Len_Shift = 2;  
    }
    
    if( tx_ch_num > 0 ) {        
        tfmr.datnb  = tx_ch_num - 1 ; //5 ; //6 slot TDM
        tfmr.datlen = slot_len-1;//31 ; //32bits
        tcmr.cki=cki;
        tcmr.sttdly=delay;
        tcmr.start=start;
        SSC_ConfigureTransmitter( BOARD_AT73C213_SSC,  tcmr.value,  tfmr.value   );
        
    }       
         
}


void SSC_Channel_Set_Rx( unsigned char rx_ch_num, unsigned char slot_len, unsigned char cki, unsigned char delay, unsigned char start )
{     
   
    if( slot_len == 16 ) {
        DMA_CtrA_Reg_Mode  = DMA_CTRA_MODE_16BIT ;
        DMA_CtrA_Len_Shift = 1;    
    } else { //32bit
        DMA_CtrA_Reg_Mode  = DMA_CTRA_MODE_32BIT ;
        DMA_CtrA_Len_Shift = 2;  
    }
    
    if( rx_ch_num > 0 ) {
        
        rfmr.datnb  = rx_ch_num - 1 ; //5 ; 
        rfmr.datlen = slot_len-1;//31 ;
        rcmr.cki=cki;
        rcmr.sttdly=delay;
        rcmr.start=start;
        SSC_ConfigureReceiver(  BOARD_AT73C213_SSC,  rcmr.value , rfmr.value   );
        
    }
    
}


void SSC_Init( unsigned int mclk, unsigned char slot_num, unsigned char slot_len )
{
    
    SSC_Configure(  
                    BOARD_AT73C213_SSC,
                    BOARD_AT73C213_SSC_ID,
                    0,  //slave not gen clk
                    mclk 
                 );    
      
    tcmr.cks    = 1 ;   // tx use TK pin
    rcmr.cks    = 1 ;   // rx use TK pin, as RK pin for sync
    tcmr.cko    = 0 ;   // input only
    rcmr.cko    = 0 ;   // input only  
    
    tcmr.cki    = 0;  // 0: falling egde send
    rcmr.cki    = 1;  // 1: rising edge lock  
    tcmr.start  = 4;  // 4: falling edge trigger for low left, 5: rising edge trigger for high left,
    rcmr.start  = 4; 
    
    tcmr.sttdly = 1;
    rcmr.sttdly = 1;   
    tcmr.period = 0;  // period ;  slave not use
    rcmr.period = 0;  // period ;  slave not use
    
    tcmr.ckg    = 0 ; //slave not use
    rcmr.ckg    = 0 ; //slave not use
       
    tfmr.fsos   = 0 ; //input only
    rfmr.fsos   = 0 ; //input only
    
    tfmr.datnb  = slot_num-1;//5 ; //6 slot TDM
    rfmr.datnb  = slot_num-1;//5 ;   
    tfmr.datlen = slot_len-1;//31 ; //32bits
    rfmr.datlen = slot_len-1;//31 ;
    
    tfmr.fslen  = 0 ; //frame sync is not used
    rfmr.fslen  = 0 ; //frame sync is not used
       
    tfmr.fsedge = 1 ;
    rfmr.fsedge = 1 ;
          
    tfmr.msbf   = 1 ;
    rfmr.msbf   = 1 ;   

    tfmr.datdef = 0 ;
    tfmr.fsden  = 0 ;
    
    rfmr.loop   = 0 ;    
    
    SSC_ConfigureTransmitter( BOARD_AT73C213_SSC,  tcmr.value,  tfmr.value   );
    SSC_ConfigureReceiver(  BOARD_AT73C213_SSC,  rcmr.value , rfmr.value   );
    
    //SSC_DisableTransmitter( BOARD_AT73C213_SSC ) ; 
    //SSC_DisableReceiver(  BOARD_AT73C213_SSC ) ; 
    //SSC_EnableTransmitter( BOARD_AT73C213_SSC ) ; 
    //SSC_EnableReceiver(  BOARD_AT73C213_SSC ) ; 
    
}


void SSC_Reset( void )
{ 
    
    //BOARD_AT73C213_SSC->SSC_SR;
    //BOARD_AT73C213_SSC->SSC_IDR = 0xFFFF;
    BOARD_AT73C213_SSC->SSC_CR  = AT91C_SSC_RXDIS | AT91C_SSC_TXDIS | AT91C_SSC_SWRST;
    //BOARD_AT73C213_SSC->SSC_CMR = 0;   
        
    SSC_ConfigureTransmitter( BOARD_AT73C213_SSC,  tcmr.value,  tfmr.value   );
    SSC_ConfigureReceiver(    BOARD_AT73C213_SSC,  rcmr.value , rfmr.value   ); 
 
    //Reset_DMAC_Reg();     
    DMAD_Initialize(BOARD_SSC_IN_DMA_CHANNEL);
    DMAD_Initialize(BOARD_SSC_OUT_DMA_CHANNEL);
    
}

                              

