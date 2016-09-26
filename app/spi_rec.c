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
*                                        SPI Record Related
*
*                                          Atmel AT91SAM3U4C
*                                               on the
*                                      Unified EVM Interface Board
*
* Filename      : spi_rec.c
* Version       : V1.0.0
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
#include <spi/spi.h>
#include "fm1388_comm.h"
#include "im501_comm.h"
#include "spi_rec.h"
#include "kfifo.h"
#include "app.h"
#include "gpio.h"




//unsigned char SPI_FIFO_Buffer[ SPI_FIFO_SIZE ] ;
//unsigned char SPI_PLAY_FIFO_Buffer[ SPI_FIFO_SIZE ];
//unsigned char SPI_PLAY_FIFO_Buffer2[ SPI_FIFO_SIZE ];

//unsigned char SPI_Data_Buffer[ SPI_BUF_SIZE + 2];  //+1 fix spi bug +1 for short aligment
//unsigned char SPI_Data_Buffer2[ SPI_BUF_SIZE +1 ]; 



unsigned char* pSPI_FIFO_Buffer  =  (unsigned char*)I2SBuffersIn[0] ; 
unsigned char* pSPI_Data_Buffer  =  (unsigned char*)I2SBuffersOut[0];
unsigned char* pSPI_Data_Buffer2 =  (unsigned char*)I2SBuffersIn[1] ;


SPI_PLAY_REC_CFG spi_rec_cfg;

unsigned char global_rec_spi_en  = 0 ;
unsigned char global_play_spi_en = 0 ;


/*
*********************************************************************************************************
*                                           Init_SPI_FIFO()
*
* Description :  Init kfifo for SPI recording.
*
* Argument(s) :  None.
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
void Init_SPI_FIFO( void )
{   
    kfifo_t *pfifo;
    
    pfifo = &spi_rec_fifo;;
    kfifo_init_static(pfifo, pSPI_FIFO_Buffer, SPI_FIFO_SIZE);
    
}



/*
*********************************************************************************************************
*                                    SPI_Service()
*
* Description :  Service to iM501 IRQ interruption/ FM1388 status polling 
*                Should be in Main Loop, inquiring the data ready flag
*
* Argument(s) :  None.
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void SPI_Service( void )
{      
      
        switch( spi_rec_cfg.chip_id ) {
            case ATTRI_DUT_ID_IM501 :
                Service_To_iM501_IRQ();
            break;
            
            case ATTRI_DUT_ID_FM1388 :
                Service_To_FM1388_Poll();  
                
            break;
            
            default:
                Service_To_FM1388_Poll();
            break;
        }    

}


/*
*********************************************************************************************************
*                                    SPI_Rec_Start()
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
unsigned char SPI_Rec_Start( void )
{
    
    unsigned char err = 0; 
    
    if( global_rec_spi_en == 1 ) { //in case SPI Rec already started 
        return 0;
    }
    
    init_ch_num_and_ch_index();
    get_ch_num_and_ch_index(spi_rec_cfg.rec_ch_mask);
    
    if( ch_num == 0 ) {
        return 0;
    }
//    if( global_rec_spi_num > 6 ) {
//        return 1;
//    }
    
    bulkin_enable  = true ;          
    Init_SPI_FIFO();
    First_Pack_Padding_BI();  
    
    Enable_SPI_Port(spi_rec_cfg.spi_speed, spi_rec_cfg.spi_mode);   
        
    switch( spi_rec_cfg.chip_id ) {
        case ATTRI_DUT_ID_IM501 :
            err = iM501_SPI_Rec_Start( spi_rec_cfg.gpio_irq );
        break;
        
        case ATTRI_DUT_ID_FM1388 :
            Rec_Voice_Buf_Start();            
        break;
        
        default:
            Rec_Voice_Buf_Start();                
        break;
    } 
    if( err == 0 ) {
        global_rec_spi_en = 1;
    }
    
    return err;
    
}

/*
*********************************************************************************************************
*                                    SPI_Play_Start()
*
* Description :  Start SPI data playing  procedure.
*
* Argument(s) :  None.
*
* Return(s)   :  error number. 
*
* Note(s)     : Must be called after SPI_Rec_Start()
*********************************************************************************************************
*/
unsigned char SPI_Play_Start( void )
{
    
    unsigned char err = 0; 
    
    if( global_play_spi_en == 1 ) { //in case SPI Rec already started 
        return 0;
    }
    
    init_play_ch_num_and_play_ch_index();               
    get_play_ch_num_and_play_ch_index(spi_rec_cfg.play_ch_mask);
    
          if(spi_rec_cfg.time_dly==1){
                  spi_play_timertick=4;
            
              }
              else if(spi_rec_cfg.time_dly==2){
                  //spi_play_timertick=8;
                  spi_play_timertick=7;
              }
              else if(spi_rec_cfg.time_dly==3){
                  spi_play_timertick=10;
              }
              else{
                  spi_play_timertick=10;
              }
    
    if( play_ch_num == 0 ) {
        return 0;
    }
    printf("\r\nSPI play start, %d Hz SR, Timetick: %d ms",Audio_Configure[0].sample_rate,spi_play_timertick)  ;
//    if( global_rec_spi_num > 6 ) {
//        return 1;
//    }
 
     bulkout_enable  = true ;
     
     unsigned int data_len = FM1388_Rec_Data_Addr[4] * play_ch_num ;
     memset((unsigned char *)usbBufferBulkOut, 0, data_len);
     kfifo_put(&bulkout_fifo, usbBufferBulkOut, data_len);     
    
//    Init_SPI_FIFO();
 //   Enable_SPI_Port(spi_rec_cfg.spi_speed, spi_rec_cfg.spi_mode);   
        
    switch( spi_rec_cfg.chip_id ) {  
        
        case ATTRI_DUT_ID_FM1388 :         
            Play_Voice_Buf_Start();
        break;
        
        default:           
            Play_Voice_Buf_Start();
        break;
    } 
    if( err == 0 ) {
        global_play_spi_en = 1;
    }
    
    return err;
    
}


/*
*********************************************************************************************************
*                                    SPI_Rec_Stop()
*
* Description :  Stop SPI data recordin.
*
* Argument(s) :  None.
*
* Return(s)   :  error number. .
*
* Note(s)     : None.
*********************************************************************************************************
*/
unsigned char SPI_Rec_Stop( void )
{    
    
    unsigned char err = 0;  
    
    if( global_rec_spi_en == 0 ) { //in case SPI Rec already Stopped  
        return 0;
    }
    global_rec_spi_en  = 0;
    global_play_spi_en = 0;
    
    switch( spi_rec_cfg.chip_id ) {
        
        case ATTRI_DUT_ID_IM501 :
            err = iM501_SPI_Rec_Stop();
        break;
        
        case ATTRI_DUT_ID_FM1388 :
             Rec_Voice_Buf_Stop();
        break;
        
        default:
             Rec_Voice_Buf_Stop();
        break;
    }        
    
    Disable_SPI_Port();
    
    return err;
    
}
                          
       
        
     



