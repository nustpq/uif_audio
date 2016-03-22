/*
*********************************************************************************************************
*                               UIF BOARD APP PACKAGE
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
*                                        FM1388 Communication Related
*
*                                          Atmel AT91SAM3U4C
*                                               on the
*                                      Unified EVM Interface Board
*
* Filename      : fm1388_comm.c
* Version       : V1.1.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <stdbool.h>
#include <intrinsics.h>
#include <utility/trace.h>
#include <tc/tc.h>
#include <pio/pio.h>
#include <utility/led.h>
#include "fm1388_comm.h"
#include "spi_rec.h"
#include "kfifo.h"
#include "app.h"
#include "gpio.h"


unsigned int  FM1388_Rec_Data_Addr[5];


/*
*********************************************************************************************************
*                                           fm1388_single_write_dram_spi()
*
* Description :  FM1388 write DRAM via SPI, just limited to 2 bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 4 bytes length and need 2 bytes alignment
*
*                *pdata         is point to where data to be written is stored 
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char fm1388_single_write_dram_spi( unsigned int mem_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[8];
    //unsigned char *pbuf;
    
    err   =  NO_ERR;
    //pbuf  = (unsigned char *)SPI_Data_Buffer; //global usage
      
    buf[0] =  FM1388_SPI_S_16BIT_WR; //0x01
    buf[4] =  mem_addr & 0xFF;
    buf[3] =  (mem_addr>>8) & 0xFF;
    buf[2] =  (mem_addr>>16) & 0xFF;
    buf[1] =  (mem_addr>>24) & 0xFF;
    buf[6] =  *pdata++;
    buf[5] =  *pdata;    
    buf[7] =  0; //dummy data byte
  
    state =  SPI_WriteBuffer_API( buf, sizeof(buf) );

    if (state != SUCCESS) {
        err = SPI_BUS_ERR;
        //APP_TRACE_INFO(("\r\nSPI_ReadBuffer_API err = %d",state));
    }   
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           fm1388_single_read_dram_spi()
*
* Description :  FM1388 read DRAM via SPI, just limited to 2 bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 4 bytes length and need 2 bytes alignment
*
*                *pdata         is point to where read back data will be stored 
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char fm1388_single_read_dram_spi( unsigned int mem_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[9];
    unsigned char *pbuf;
    
    err   =  NO_ERR;
    pbuf  = (unsigned char *)SPI_Data_Buffer; //global usage
  
    
    buf[0] =  FM1388_SPI_S_16BIT_RD; //0x00
    buf[4] =  mem_addr & 0xFF;
    buf[3] =  (mem_addr>>8) & 0xFF;
    buf[2] =  (mem_addr>>16) & 0xFF;
    buf[1] =  (mem_addr>>24) & 0xFF;
    buf[5] =  0;
    buf[6] =  0;    
    buf[7] =  0; 
    buf[8] =  0; 

    state =  SPI_WriteReadBuffer_API(  pbuf, 
                                       buf, 
                                       2 , 
                                       sizeof(buf) );

    if (state != SUCCESS) {
        err = SPI_BUS_ERR;
        //APP_TRACE_INFO(("\r\nSPI_ReadBuffer_API err = %d",state));
        return err;
    }   
        
    pbuf = pbuf + 1; //fix bug
    *pdata++ = *(pbuf+1); //LSB
    *pdata++ = *(pbuf);   //MSB
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           fm1388_single_read_dram_spi_32bit()
*
* Description :  FM1388 read DRAM via SPI, just limited to 4 bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 4 bytes length and need 4 bytes alignment
*
*                *pdata         is point to where read back data will be stored 
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char fm1388_single_read_dram_spi_32bit( unsigned int mem_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[9];
    unsigned char *pbuf;
    
    err   =  NO_ERR;
    pbuf  = (unsigned char *)SPI_Data_Buffer; //global usage
  
    
    buf[0] =  FM1388_SPI_S_32BIT_RD; //0x00
    buf[4] =  mem_addr & 0xFF;
    buf[3] =  (mem_addr>>8) & 0xFF;
    buf[2] =  (mem_addr>>16) & 0xFF;
    buf[1] =  (mem_addr>>24) & 0xFF;
    buf[5] =  0;
    buf[6] =  0;    
    buf[7] =  0; 
    buf[8] =  0; 

    state =  SPI_WriteReadBuffer_API(  pbuf, 
                                       buf, 
                                       4 , 
                                       sizeof(buf) );

    if (state != SUCCESS) {
        err = SPI_BUS_ERR;
        //APP_TRACE_INFO(("\r\nSPI_ReadBuffer_API err = %d",state));
        return err;
    }   
        
    pbuf = pbuf + 1; //fix bug
    *pdata++ = *(pbuf+3); //LSB
    *pdata++ = *(pbuf+2);  
    *pdata++ = *(pbuf+1);
    *pdata++ = *(pbuf);   //MSB
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           fm1388_burst_read_dram_spi()
*
* Description :  burst read iM501 DRAM via SPI, just limited to 4095 bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 3 bytes length and need 4bytes alignment
*
*                **pdata        is point to where the pointer pointing to read back data will be stored 
*
*                data_len       is data length to read in bytes, maxium 4095 bytes  
*
* Return(s)   :  error number.           
*
* Note(s)     :  Non-Reentrant function.   Reg_RW_Data is used.
*                Be care full this function use fixed buffer:    SPI_Data_Buffer
*                and return a **pointer to.
*********************************************************************************************************
*/
unsigned char fm1388_burst_read_dram_spi( unsigned int mem_addr, unsigned char **pdata, unsigned int data_len )
{
    unsigned char  err, state;
    unsigned char  buf[9];
    unsigned char *pbuf;
    
    err   =  NO_ERR;
    pbuf = (unsigned char *)SPI_Data_Buffer; //global usage    
    
    buf[0] =  FM1388_SPI_B_RD; //0x04
    buf[4] =  mem_addr & 0xFF;
    buf[3] =  (mem_addr>>8) & 0xFF;
    buf[2] =  (mem_addr>>16) & 0xFF;
    buf[1] =  (mem_addr>>24) & 0xFF;
    buf[5] =  0;
    buf[6] =  0;    
    buf[7] =  0; 
    buf[8] =  0;    

    state =  SPI_WriteReadBuffer_API(  pbuf, 
                                       buf, 
                                       data_len , 
                                       sizeof(buf) );
            
    if (state != SUCCESS) {
        err = SPI_BUS_ERR;
        //APP_TRACE_INFO(("\r\nSPI_ReadBuffer_API err = %d",state));
        return err;
    }           
    *pdata =  pbuf + 1; 
       
    return err;
    
}


/*
*********************************************************************************************************
*                                           data_revert_burst_mode()
*
* Description :  revert endian-mode from[7..0] to[0..7] of the burst read data .
*
* Argument(s) :  *pDest          is point to where the reverted data will be stored
*
*                *pSour          is point to where the source from  
*
*                data_length     is data length to revert in bytes, must be X 8   
*
* Return(s)   :  error number.           
*
* Note(s)     : None.
*********************************************************************************************************
*/
void data_revert_burst_mode( unsigned char *pDest, unsigned char *pSour,unsigned int data_length )
{
    unsigned int i,j;  
    
    for( i=0; i < (data_length>>3); i++ ) {
        for( j=0; j<8; j++ ) {
            *(pDest+j) = *(pSour+7-j);
        }
        pDest += 8;
        pSour += 8;
    }
    
}

/*
*********************************************************************************************************
*                                           fetch_voice_data()
*
* Description :  read voice data from iM501 DRAM via SPI 
*
* Argument(s) :  start_addr     is DRAM data address
*
*                data_length    is data size(bytes) to read
*
* Return(s)   :  error number.           
*
* Note(s)     :  
*********************************************************************************************************
*/
/*
unsigned char fetch_voice_data( void )
{
    unsigned char   err;
    unsigned int    i,j;
    unsigned char  *pbuf;
    unsigned short *pShortDest, *pShortSource;
    unsigned int    data_length;
    unsigned int    start_addr;
     
    data_length = FM1388_Rec_Data_Addr[4];
    pbuf        = (unsigned char *)&SPI_Data_Buffer2;
    //AEC Ref
    start_addr  = FM1388_Rec_Data_Addr[0];
    err = fm1388_burst_read_dram_spi( start_addr,  &pbuf,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    //MIC0
    start_addr   = FM1388_Rec_Data_Addr[1];
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pbuf,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    //MIC1
    start_addr  += data_length;
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pbuf,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    //MIC2
    start_addr  += data_length;
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pbuf,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    //Lout
    start_addr  = FM1388_Rec_Data_Addr[2];
    pbuf       += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pbuf,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    } 
    //Lin
    start_addr  = FM1388_Rec_Data_Addr[3];
    pbuf       += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pbuf,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    } 
    
    pShortDest    = (unsigned short *)&SPI_Data_Buffer;
    pShortSource  = (unsigned short *)&SPI_Data_Buffer2;
    unsigned int sr_num = data_length/2 ;
    unsigned int ch_num = 6; 
    for(i = 0 ; i<sr_num ; i++ ) { 
        pShortSource += sr_num * ch_num * i ;
        for( j = 0; j< ch_num ; j++ ){
          *pShortDest++ = *(pShortSource + sr_num*j );        
        }        
    }
    
    pbuf = (unsigned char *)&SPI_Data_Buffer;    
    kfifo_put(&spi_rec_fifo, pbuf, data_length*6);          
       
    return err;
    
}
*/

//due to ram limitation in MCU, changed to fetch buffer data twice
static unsigned char fetch_voice_data0( void )
{
    unsigned char   err;
    unsigned int    i,j;
    unsigned char  *pbuf, *pdata, *phalf;
    unsigned short *pShortDest, *pShortSource;
    unsigned int    data_length;
    unsigned int    start_addr;
    unsigned int    sr_num ;
    unsigned int    ch_num ; 
      
    
    data_length = FM1388_Rec_Data_Addr[4] >> 1;
    phalf       = (unsigned char *)&SPI_Data_Buffer + (FM1388_BUF_SIZE_HALF + 2) ; //use bottom half of SPI_Data_Buffer to save temp data
    
    ////////////////////////////     top half       //////////////////////////
    pbuf        = phalf;
    //AEC Ref
    start_addr  = FM1388_Rec_Data_Addr[0];
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //MIC0
    start_addr   = FM1388_Rec_Data_Addr[1];
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //MIC1
    start_addr  += data_length<<1;
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //MIC2
    start_addr  += data_length<<1;
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //Lout
    start_addr  = FM1388_Rec_Data_Addr[2];
    pbuf       += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    } 
    data_revert_burst_mode( pbuf, pdata, data_length);
    //Lin
    start_addr  = FM1388_Rec_Data_Addr[3];
    pbuf       += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    } 
    data_revert_burst_mode( pbuf, pdata, data_length);
    
    pShortDest    = (unsigned short *)&SPI_Data_Buffer;
    pShortSource  = (unsigned short *)phalf;
    sr_num        = data_length>>1 ;//bytes to word
    ch_num        = 6; //6 channel data
    for(i = 0 ; i < sr_num ; i++ ) {     
        for( j = 0; j < ch_num ; j++ ){
          *pShortDest++ = *(pShortSource + i + j*(data_length>>1) );     
        }        
    }
    
    while(1){
      i= kfifo_get_free_space( &spi_rec_fifo );
      if( i>data_length*6 ) {
          break;
      }else{
          printf("\r\nfifo full");
      }
    }    
    pbuf = (unsigned char *)&SPI_Data_Buffer;     
    kfifo_put(&spi_rec_fifo, pbuf, data_length*6); //total 6 channel data          
    
    /////////////////////////   bottom half data   ////////////////////////////  
    pbuf        = phalf;;
    //AEC Ref
    start_addr  = FM1388_Rec_Data_Addr[0] + data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //MIC0
    start_addr   = FM1388_Rec_Data_Addr[1] + data_length;
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //MIC1
    start_addr  += data_length<<1;
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //MIC2
    start_addr  += data_length<<1;
    pbuf        += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    //Lout
    start_addr  = FM1388_Rec_Data_Addr[2] + data_length;
    pbuf       += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    } 
    data_revert_burst_mode( pbuf, pdata, data_length);
    //Lin
    start_addr  = FM1388_Rec_Data_Addr[3] + data_length;
    pbuf       += data_length;
    err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
    if( err != NO_ERR ){ 
        return err;
    }
    data_revert_burst_mode( pbuf, pdata, data_length);
    
    pShortDest    = (unsigned short *)&SPI_Data_Buffer;
    pShortSource  = (unsigned short *)phalf;
    sr_num        = data_length>>1 ;//bytes to word
    ch_num        = 6; 
    for(i = 0 ; i < sr_num ; i++ ) {      
        for( j = 0; j < ch_num ; j++ ){
           *pShortDest++ = *(pShortSource + i + j*(data_length>>1) );     
        }        
    }
    
    while(1){
      i= kfifo_get_free_space( &spi_rec_fifo );
      if( i>data_length*6 ) {
          break;
      }else{
          printf("\r\nfifo full");
      }
    } 
    pbuf = (unsigned char *)&SPI_Data_Buffer;    
    kfifo_put(&spi_rec_fifo, pbuf, data_length*6); //total 6 channel data 
    
    //////////////////////////////////////////////////////////
    return err;
    
}


//due to ram limitation in MCU, changed to fetch buffer data twice( top/bottom half, k for loop)
//sampe data is 16 bit length only

static unsigned char fetch_voice_data( void )
{
    unsigned char   err;
    unsigned int    i,j,k;
    unsigned char  *pbuf, *pdata, *phalf;
    unsigned short *pShortDest, *pShortSource;
    unsigned int    data_length;
    unsigned int    start_addr;
    unsigned int    sr_num ;
    unsigned int    ch_num ; 
          
    data_length = FM1388_Rec_Data_Addr[4] >> 1; // data_length /2 as we fetch data twice for top half and bottom half 
    phalf       = (unsigned char *)&SPI_Data_Buffer + (FM1388_BUF_SIZE_HALF + 2) ; //use bottom half of SPI_Data_Buffer to save temp data
       
    for(  k = 0; k <2; k++ ) {
      
        pbuf        = phalf;        
        if( (global_rec_spi_mask>>0) & 0x01 ) {
            //AEC Ref  
            start_addr  = FM1388_Rec_Data_Addr[0] + data_length * k ;
            err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
            if( err != NO_ERR ){ 
                return err;
            }
            data_revert_burst_mode( pbuf, pdata, data_length);
            pbuf  += data_length;
        }
        if( (global_rec_spi_mask>>1) & 0x01 ) {    
            //MIC0
            start_addr   = FM1388_Rec_Data_Addr[1] + data_length * k ;;        
            err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
            if( err != NO_ERR ){ 
                return err;
            }
            data_revert_burst_mode( pbuf, pdata, data_length);
            pbuf  += data_length;
        }
        if( (global_rec_spi_mask>>2) & 0x01 ) { 
            //MIC1
            start_addr  = FM1388_Rec_Data_Addr[1] + (data_length<<1) + data_length * k ;;        
            err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
            if( err != NO_ERR ){ 
                return err;
            }
            data_revert_burst_mode( pbuf, pdata, data_length);
            pbuf  += data_length;
        }
        if( (global_rec_spi_mask>>3) & 0x01 ) { 
            //MIC2
            start_addr  = FM1388_Rec_Data_Addr[1] + (data_length<<2) + data_length * k ;;        
            err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
            if( err != NO_ERR ){ 
                return err;
            }
            data_revert_burst_mode( pbuf, pdata, data_length);
            pbuf  += data_length;
        }
        if( (global_rec_spi_mask>>4) & 0x01 ) {    
            //Lout
            start_addr  = FM1388_Rec_Data_Addr[2] + data_length * k ;;       
            err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
            if( err != NO_ERR ){ 
                return err;
            } 
            data_revert_burst_mode( pbuf, pdata, data_length);
            pbuf  += data_length;
        }
        if( (global_rec_spi_mask>>5) & 0x01 ) { 
            //Lin
            start_addr  = FM1388_Rec_Data_Addr[3] + data_length * k ;;        
            err = fm1388_burst_read_dram_spi( start_addr,  &pdata,  data_length ); 
            if( err != NO_ERR ){ 
                return err;
            } 
            data_revert_burst_mode( pbuf, pdata, data_length);
        }
        
        pShortDest    = (unsigned short *)&SPI_Data_Buffer;
        pShortSource  = (unsigned short *)phalf;
        sr_num        = data_length>>1 ;//length to samples for 16bit data
        ch_num        = global_rec_spi_num; //6 channel data ?
        for(i = 0 ; i < sr_num ; i++ ) {     
            for( j = 0; j < ch_num ; j++ ){
              *pShortDest++ = *(pShortSource + i + j*(data_length>>1) );     
            }        
        }
        
        while(1){
          i= kfifo_get_free_space( &spi_rec_fifo );
          if( i>data_length * ch_num ) {
              break;
          }else{
              printf("\r\nfifo full");
          }
        }    
        pbuf = (unsigned char *)&SPI_Data_Buffer;     
        kfifo_put(&spi_rec_fifo, pbuf, data_length * ch_num); //total 6 channel data    
        
    }
        
    return err;
    
}
/*
*********************************************************************************************************
*                                           spi_rec_start_cmd()
*
* Description :  send CMD to FM1388 for starting SPI record 
*
* Argument(s) :  None.
*
* Return(s)   :  error number.           
*
* Note(s)     :  
*********************************************************************************************************
*/
unsigned char spi_rec_start_cmd( void )
{
    unsigned char err;
    unsigned short data;
    
    data = DSP_INIT_CMD ;
    err = fm1388_single_write_dram_spi( DSP_CMD_ADDR, (unsigned char *)&data );
    if( err != 0 ){ 
        return err;
    }
    return err;
    
}


/*
*********************************************************************************************************
*                                           spi_rec_stop_cmd()
*
* Description :  send CMD to FM1388 for stopping SPI record 
*
* Argument(s) :  None.
*
* Return(s)   :  error number.           
*
* Note(s)     :  
*********************************************************************************************************
*/
unsigned char spi_rec_stop_cmd( void )
{
    unsigned char err;
    unsigned short data;
    
    data = DSP_STOP_CMD ;    
    err = fm1388_single_write_dram_spi( DSP_CMD_ADDR, (unsigned char *)&data );
    if( err != 0 ){ 
        return err;
    }
    return err;
    
}





/*
*********************************************************************************************************
*                                    spi_rec_get_addr()
*
* Description :  get voive buffer address from predefined pointer  
*                use global varies 
*
* Argument(s) :  None
* 
* Return(s)   :  Error number.
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char spi_rec_get_addr( void )
{
    
    unsigned char err;
    unsigned int  i;
    
    unsigned int  addr[] = {
        DSP_BUFFER_ADDR0,
        DSP_BUFFER_ADDR1,
        DSP_BUFFER_ADDR2,
        DSP_BUFFER_ADDR3,
        DSP_SPI_FRAMESIZE_ADDR 
    };
    
    for( i = 0; i<5; i++ ) {      
      err = fm1388_single_read_dram_spi_32bit( addr[i],(unsigned char*)&FM1388_Rec_Data_Addr[i]);
      if( err != 0 ){
          return err;
      }
    }
    FM1388_Rec_Data_Addr[4] &= 0x0000FFFF;
    if( FM1388_Rec_Data_Addr[4] != global_rec_spi_buffer_size ) {
        err = 1;
    }
    
//    unsigned char *pdata;
//    err = fm1388_burst_read_dram_spi( addr[0],  &pdata,  16 );
//    memcpy( (unsigned char*)&FM1388_Rec_Data_Addr[0], pdata, 16);
        
    return err;
    
}


/*
*********************************************************************************************************
*                                           spi_rec_check_ready()
*
* Description :  send CMD to FM1388 for stopping SPI record 
*
* Argument(s) :  None.
*
* Return(s)   : 0 - data ready
*               100 - not ready
*               others - error humber
*
* Note(s)     :  
*********************************************************************************************************
*/
unsigned char spi_rec_check_ready( void )
{
    unsigned char   err;
    unsigned short  state;
    
    err = fm1388_single_read_dram_spi( DSP_CMD_ADDR, (unsigned char*)&state );
    if( err != 0 ){ 
        return err;
    }
    
    if( DSP_READY_CMD != state ) {
        err = 100;
    }
    
    //printf("\r\nCheckReady  : %04X",  state);
    
    return err;
    
}


/*
*********************************************************************************************************
*                                    Service_To_FM1388_Poll()
*
* Description :  Service to iM501 IRQ interruption 
*                Should be in Main Loop, inquiring the data ready flag
*
* Argument(s) :  None.
*
* Return(s)   :  None.
*
* Note(s)     : 
     ----------------------------------------------------------------------
     SR  | data_fetch_time per frame |  frame_time  |   MIPS cost @ 300MIPS
     ----|---------------------------|--------------|----------------------
     8k  |      1.0ms                |      10ms    |       30 MIPS
     16k |      1.9ms                |      10ms    |       57 MIPS
     24k |      2.4ms                |       8ms    |       90 MIPS
     ----------------------------------------------------------------------
*********************************************************************************************************
*/
void Service_To_FM1388_Poll( void )
{  
    unsigned char err;
    
    if( spi_rec_check_ready() == 0 ) { 
        //GPIOPIN_Set(7,1);
        LED_Set(USBD_LEDPOWER);   //on LED
        fetch_voice_data();  
        spi_rec_start_cmd(); 
        //GPIOPIN_Set(7,0);
        LED_Clear(USBD_LEDPOWER);  //off LED
    }    
}

/*
*********************************************************************************************************
*                                    FM1388_SPI_Rec_Start()
*
* Description :  Start SPI data recording procedure.
*
* Argument(s) :  None.
*
* Return(s)   :  error number. 
*
* Note(s)     : None.
*********************************************************************************************************
*/
unsigned char FM1388_SPI_Rec_Start( void )
{
    unsigned char err = 0;
    
    err = spi_rec_get_addr();
    if( err != 0 ) {
        return err; 
    }
    err = spi_rec_start_cmd();         
    LED_Clear(USBD_LEDPOWER); //off LED
    
    return err;
}


/*
*********************************************************************************************************
*                                    FM1388_SPI_Rec_Stop()
*
* Description :  Stop SPI data recordin.
*
* Argument(s) :  None.
*
* Return(s)   :  error number. 
*
* Note(s)     : None.
*********************************************************************************************************
*/
unsigned char FM1388_SPI_Rec_Stop( void )
{
    
    unsigned char err = 0 ;   

    err = spi_rec_stop_cmd(); 
    
    LED_Set(USBD_LEDPOWER); //on LED
    
    return err;
    
}






