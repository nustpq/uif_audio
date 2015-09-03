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
*                                          APP PACKAGE
*
*                                         Atmel  SAM3U4C
*                                               on the
*                                      Unified EVM Interface Board
*
* Filename      : app.c
* Version       : V2.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <board.h>
#include <stdbool.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <tc/tc.h>
#include <usart/usart.h>
#include <ssc/ssc.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <pmc/pmc.h>
#include "im501_comm.h"
#include "kfifo.h"
#include "usb.h"
#include "app.h"


#ifndef AT91C_ID_TC0
    #define AT91C_ID_TC0 AT91C_ID_TC
#endif

////////////////////////////////////////////////////////////////////////////////
char fw_version[] = "[FW:A:V2.43]";
////////////////////////////////////////////////////////////////////////////////

//Buffer Level 1:  USB data stream buffer : 64 B
unsigned char usbBufferBulkOut[USBDATAEPSIZE] ;//@0x20100A80;
unsigned char usbBufferBulkIn[USBDATAEPSIZE] ; //@0x20100AC0;
//#pragma location
//Buffer Level 2:  FIFO Loop Data Buffer : 16384 B
unsigned char FIFOBufferBulkOut[USB_OUT_BUFFER_SIZE] ;
unsigned char FIFOBufferBulkIn[USB_IN_BUFFER_SIZE] ; 

//Buffer Level 3:  Double-buffer for I2S data : MAX 48*2*8*2*2 = 3072 B
unsigned char I2SBuffersOut[2][I2S_OUT_BUFFER_SIZE];  // Play //put this 3072B buffer in NAND FLASH Reserved 4224 SRAM(start from0x20100000)
unsigned char I2SBuffersIn[2][I2S_IN_BUFFER_SIZE] ;   // Record
// Current I2S buffer index.
volatile unsigned char i2s_buffer_out_index = 0;
volatile unsigned char i2s_buffer_in_index  = 0;

AUDIO_CFG  Audio_Configure[2]; //[0]: rec config. [1]: play config.
VOICE_BUF_CFG Voice_Buf_Cfg;
unsigned char audio_cmd_index     = AUDIO_CMD_IDLE ; 
unsigned char usb_data_padding    = 0; //add for usb BI/BO padding for first package

kfifo_t bulkout_fifo;
kfifo_t bulkin_fifo;
kfifo_t spi_rec_fifo;

//////////////////////////////////////////
//Buffer Level 1:  USB Cmd data stream buffer : 64 B
unsigned char usbCmdBufferBulkOut[USBCMDDATAEPSIZE] ;//@0x20100A00;
unsigned char usbCmdBufferBulkIn[USBCMDDATAEPSIZE]  ;//@0x20100A40;

//#pragma location = 0x20100000
//Buffer Level 2:  FIFO Loop Data Buffer : 1024 B
unsigned char FIFOBufferBulkOutCmd[USB_CMD_OUT_BUFFER_SIZE] ;// @0x20100000;
unsigned char FIFOBufferBulkInCmd[USB_CMD_IN_BUFFER_SIZE]  ;//  @0x20100500;  


//Buffer Level 3:  Double-buffer for UART data : 256 B
unsigned char UARTBuffersOut[UART_OUT_BUFFER_SIZE] ;// @0x20100A00;  // Play
unsigned char UARTBuffersIn[UART_IN_BUFFER_SIZE]  ;//  @0x20100C00;  // Record
// Current I2S buffer index.
volatile unsigned char uart_buffer_out_index = 0;
volatile unsigned char uart_buffer_in_index  = 0;

kfifo_t  bulkout_fifo_cmd;
kfifo_t  bulkin_fifo_cmd;


///////////////////////////////////////

volatile unsigned int i2s_play_buffer_size ; //real i2s paly buffer
volatile unsigned int i2s_rec_buffer_size ;  //real i2s record buffer


volatile bool bulkout_enable   = false ;
volatile bool bulkin_enable    = false ;
volatile bool bulkin_start     = true;
volatile bool bulkout_start    = true;

volatile bool bulkin_start_cmd     = true;
volatile bool bulkout_start_cmd    = true;

volatile bool uartin_start_cmd     = true;
volatile bool uartout_start_cmd    = true;

volatile bool bulkout_trigger  = false ;
volatile bool flag_stop        = false ;
volatile bool bulkout_padding_ok  = false ;
volatile unsigned char Toggle_PID_BI    = 0;

volatile unsigned int bulkout_empt = 0;
volatile bool         flag_bulkout_empt = false;
volatile unsigned int debug_trans_counter1 = 0 ;
volatile unsigned int debug_trans_counter2 = 0 ;  
volatile unsigned int debug_trans_counter3 = 0 ;
volatile unsigned int debug_trans_counter4 = 0 ;  
volatile unsigned int debug_usb_dma_IN = 0;
volatile unsigned int debug_usb_dma_OUT = 0;

extern unsigned int debug_stall_counter;
//extern unsigned int debug_trans_counter1,debug_trans_counter2;
extern kfifo_t dbguart_fifo;
extern const Pin SSC_Sync_Pin;

static unsigned char audio_state_check    = 0; //avoid re-start issue in case of not stop previous start 
static unsigned int BO_free_size_max      = 0;
static unsigned int BI_free_size_min      = 100; 
static unsigned int DBGUART_free_size_min = 100; 
static unsigned int Stop_CMD_Miss_Counter = 0;


static unsigned char global_rec_num               ;  //total TDM channels
static unsigned char global_rec_samples           ;  //samples per package of one interruption 

static unsigned char global_rec_gpio_mask         ; // gpio   to record 
static unsigned char global_rec_gpio_num          ; //total gpio num to record
static unsigned char global_rec_gpio_index        ;  //index gpio start at which TDM channel 



unsigned int counter_play    = 0;
unsigned int counter_rec     = 0;
unsigned int test_dump       = 0;
unsigned int time_start_test = 0;

extern unsigned char Check_Toggle_State( void );


void Init_Bus_Matix( void )
{
     *(unsigned int*)0x400E03E4 = 0x4D415400 ; 
     *(unsigned int*)0x400E0240 = 0x011200FF ; 
     *(unsigned int*)0x400E0244 = 0x011200FF ; 
     *(unsigned int*)0x400E0280 = 0x30000 ; 
     *(unsigned int*)0x400E0288 = 0x30000 ;  
     printf("\r\nBUS PROTECT: %08X",  *(unsigned int*)0x400E03E8);
     printf("\r\nBUS  : %08X",  *(unsigned int*)0x400E0240);
     printf("\r\nBUS  : %08X",  *(unsigned int*)0x400E0244);
}


/*
*********************************************************************************************************
*                                    Merge_GPIO_Data()
*
* Description :  merge GPIO data to audio data package.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void __ramfunc Merge_GPIO_Data( unsigned short *pdata )
{
    unsigned int   i, j, n;
    unsigned char  temp;
    unsigned short gpio_data[8];
    
    if( global_rec_gpio_num == 0 ) {
        return;
    }
    
    GPIOPort_Get_Fast( &temp );
    
    for( i = 0, n = 0 ; i < 8 ; i++ ) {
     
        if( global_rec_gpio_mask &(1<<i) ) {
            if( temp & (1<<i) ) {
                gpio_data[n++] = 0x3FFF;  //high level
            } else {
                gpio_data[n++] = 0;       //low level
            }
        }
    }
    
    for( i = 0; i < global_rec_samples ; i++ ) { //2ms buffer
        
        for( j = 0; j < global_rec_gpio_num ; j ++ ) {            
         *( pdata + global_rec_gpio_index + j ) = gpio_data[j];
        }
        
        pdata += global_rec_num;
    }
            
}


//in test, try to merge SPI mono data to the override the last channel data
void Merge_SPI_Data( unsigned short *pdata )
{
    unsigned int    i, j, n;
    unsigned char   temp;
    unsigned short *pshort;
    unsigned int    data_size;
    
    if( global_rec_spi_en == 0 ) {
        return;
    }
    
    pshort = (unsigned short *)(SPI_Data_Buffer2); 
    
    if( global_rec_spi_fast == 1 ) { //fast read
        if(  SPI_BUF_SIZE <= kfifo_get_data_size( &spi_rec_fifo) ) {
            kfifo_get(&spi_rec_fifo, SPI_Data_Buffer2, SPI_BUF_SIZE);
        } else {
            return ;
        }
        
        for( i = 0; i < SPI_BUF_SIZE/2 ; i++ ) {
            *( pdata + global_rec_gpio_index + global_rec_gpio_num -1 ) = *(pshort+i);               
            pdata += global_rec_num;
        } 
        
    } else {//real time rec
        data_size = i2s_rec_buffer_size/Audio_Configure[0].channel_num;
        if(  data_size <= kfifo_get_data_size( &spi_rec_fifo) ) {
            kfifo_get(&spi_rec_fifo, SPI_Data_Buffer2, data_size);  
        } else {
            return ;
        }
        
        for( i = 0; i < global_rec_samples ; i++ ) { //2ms buffer        
            *( pdata + global_rec_gpio_index + global_rec_gpio_num -1 ) = *(pshort+i);               
            pdata += global_rec_num;
        }
    }
        
            
}

/*
*********************************************************************************************************
*                                  First_Pack_Check_BO()
*
* Description :  Check if first USB bulk out package is same as padding data.
* Argument(s) :  None.
* Return(s)   :  true -- check ok.
*                false -- check failed.
*
* Note(s)     :  None.
*********************************************************************************************************
*/
static bool bo_check_sync = false;
__ramfunc bool First_Pack_Check_BO( unsigned int size )
{    
    
    unsigned int i;
    
    for( i = 0; i < size ; i++ )   {
        if( usb_data_padding != usbBufferBulkOut[i]) {
            return false;
        }
    }
    bo_check_sync = true;
    //printf("\r\nSync\r\n");
    return true; 

}

/*
*********************************************************************************************************
*                                First_Pack_Padding_BI()
*
* Description :  Padding the first USB bulk in package.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     :  Must be called after reset FIFO and before start audio.
*********************************************************************************************************
*/
static void First_Pack_Padding_BI( void )
{
    memset( (unsigned char *)I2SBuffersIn[0], usb_data_padding, USBDATAEPSIZE );
    kfifo_put(&bulkin_fifo, (unsigned char *)I2SBuffersIn[0], USBDATAEPSIZE) ; 
    kfifo_put(&bulkin_fifo, (unsigned char *)I2SBuffersIn[0], USBDATAEPSIZE) ;//2 package incase of PID error
}


/*
*********************************************************************************************************
*                                    Init_Play_Setting()
*
* Description :  Initialize USB bulk out (play) settings.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static unsigned char Init_Play_Setting( void )
{
    unsigned char err;
    unsigned short sample_rate ; //not support 44.1khz now
    unsigned char  channels_play;
    unsigned char  bit_length;
    
    
    err  = NULL;
    channels_play = Audio_Configure[1].channel_num ; 
    sample_rate   = Audio_Configure[1].sample_rate ;
    bit_length    = Audio_Configure[1].bit_length ; 
    printf( "\r\nStart [%dth]Play[%dCH - %dHz - %dBit] ...\r\n",counter_play++,channels_play,sample_rate,bit_length);  
    
    if( (channels_play == 0) ||  (channels_play > 8) ) {        
        err = ERR_TDM_FORMAT ; 
    }  
    if( (bit_length != 16) && (bit_length != 32)  ) {        
        err = ERR_TDM_FORMAT ; 
    }  
    if( NULL != err ) {
        printf("Init Play Setting Error !\r\n");
        return err;
    }
    
    if( bit_length == 16 ) {
        i2s_play_buffer_size = sample_rate / 1000 * channels_play * 2 * 2;
    } else { //32
        i2s_play_buffer_size = sample_rate / 1000 * channels_play * 2 * 4;        
    }
    SSC_Channel_Set_Tx( channels_play, bit_length );  
    
    return err;
}


/*
*********************************************************************************************************
*                                    Init_Rec_Setting()
*
* Description :  Initialize USB bulk in (record) settings.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static unsigned char Init_Rec_Setting( void )
{
    unsigned char  err;
    unsigned short sample_rate ; //not support 44.1kHz now
    unsigned char  channels_rec;
    unsigned char  bit_length;
    
    err  = NULL;    
    channels_rec = Audio_Configure[0].channel_num ;
    sample_rate  = Audio_Configure[0].sample_rate ;
    bit_length   = Audio_Configure[0].bit_length ;    
   
    global_rec_samples    =  sample_rate / 1000 * 2;    
    global_rec_num        =  Audio_Configure[0].channel_num ;
    global_rec_gpio_mask  =  Audio_Configure[0].gpio_rec_bit_mask ;
    global_rec_gpio_num   =  Audio_Configure[0].gpio_rec_num ;
    global_rec_gpio_index =  Audio_Configure[0].gpio_rec_start_index ;
    
    printf( "\r\nStart [%dth]Rec [%dCH - %dHz - %dBit]...\r\n",counter_rec++,channels_rec,sample_rate,bit_length);     
    
    if( (channels_rec == 0) || (channels_rec > 8) ) {  
        err = ERR_TDM_FORMAT ; 
    }  
    if( (bit_length != 16)  && (bit_length != 32)  ) {     
        err = ERR_TDM_FORMAT ; 
    }  
    if( NULL != err ) {
        printf("Init Rec Setting Error !\r\n");
        return err;
    }
    
    if( bit_length == 16 ) {
        i2s_rec_buffer_size  = sample_rate / 1000 * channels_rec  * 2 * 2; 
    } else { //32
        i2s_rec_buffer_size  = sample_rate / 1000 * channels_rec  * 2 * 4;        
    }
    SSC_Channel_Set_Rx( channels_rec, bit_length );
          
    First_Pack_Padding_BI();    
    
    return 0;
}


/*
*********************************************************************************************************
*                                    Audio_Start_Rec()
*
* Description :  Start USB data transfer for recording.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static unsigned char Audio_Start_Rec( void )
{  
    unsigned char err;  
    
//   if( Toggle_PID_BI ) { //send padding package if PC Driver expect DATA1 Token          
//         kfifo_put(&bulkin_fifo, (unsigned char *)I2SBuffersIn[0], USBDATAEPSIZE) ;
//    }  
    err = Init_Rec_Setting();
    if( err != 0 ) {
        return err;
    }
    SSC_Record_Start(); 
    bulkin_enable  = true ;
    
    while( !PIO_Get(&SSC_Sync_Pin) ) ;
    while(  PIO_Get(&SSC_Sync_Pin) ) ;     
    SSC_EnableReceiver(AT91C_BASE_SSC0);    //enable AT91C_SSC_RXEN
    
    return 0;  
}


/*
*********************************************************************************************************
*                                    Audio_Start_Play()
*
* Description :  Start USB data transfer for playing.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
static unsigned char Audio_Start_Play( void )
{  
    unsigned char err;  
    Init_I2S_Buffer();
    err = Init_Play_Setting(); 
    if( err != 0 ) {
        return err;
    }
    SSC_Play_Start(); 
    bulkout_enable  = true ;
    
    while( !PIO_Get(&SSC_Sync_Pin) ) ;
    while(  PIO_Get(&SSC_Sync_Pin) ) ;    
    SSC_EnableTransmitter(AT91C_BASE_SSC0); //enable AT91C_SSC_TXEN  
  
    return 0;
}


/*
*********************************************************************************************************
*                                    Audio_Stop()
*
* Description :  Stop USB data transfer.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

static void Audio_Stop( void )
{  
    
#ifdef METHOD_BY_RESET_MCU             
    printf("\r\n Got command to reset MCU...MCM_RESET_CMD");                                   
    while(1) {
        AT91C_BASE_RSTC->RSTC_RCR = 0xA5000005 ; //reset MCU     
    }       
#endif  
     
    printf( "\r\nStop Play & Rec..."); 
    flag_stop        = true ;     
    delay_ms(20); //wait until DMA interruption done. 
    bulkin_enable    = false ;
    bulkout_enable   = false ;    
    delay_ms(10);             
    SSC_Play_Stop();  
    SSC_Record_Stop();     
    delay_ms(10);  
    
    printf( "\r\nEnd Audio Transfer..."); 
    End_Audio_Transfer(); 
    delay_ms(10); 
    
    printf("\r\nReset USB EP...");
//    if( audio_state_check != 0 ) { //in case of error from repeat Stop CMD 
//        Toggle_PID_BI =  Check_Toggle_State();
//    }
    //Reset Endpoint Fifos
    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_AUDIODATAOUT;
    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_AUDIODATAIN; 
    delay_ms(50);
    //AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAOUT].UDPHS_EPTCLRSTA = 0xFFFF; //AT91C_UDPHS_NAK_OUT | AT91C_UDPHS_FRCESTALL;                  
    //AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAIN].UDPHS_EPTCLRSTA  = 0xFFFF; //AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;
    //AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;
    //delay_ms(50);
    
//    for( unsigned char i = 1; i<=4; i++ ) {
//      Get_EP_State(i);
//    }
    
    //Idle_EP_State(CDCDSerialDriverDescriptors_AUDIODATAOUT);
    //Idle_EP_State(CDCDSerialDriverDescriptors_AUDIODATAIN);
    
////////////////////////////////////////////////////////////////////////////////    
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;  
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAIN].UDPHS_EPTCLRSTA  = AT91C_UDPHS_TOGGLESQ ;
//    delay_ms(50);
//    AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_AUDIODATAIN ;
//    delay_ms(50); 
    //Reset_USBHS_HDMA( CDCDSerialDriverDescriptors_AUDIODATAIN );   
    //AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAOUT].UDPHS_EPTCLRSTA  = AT91C_UDPHS_TOGGLESQ ;
//   
//    AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAOUT].UDPHS_EPTCLRSTA  = AT91C_UDPHS_NAK_OUT ;
//    delay_ms(50);    
//    AT91C_BASE_UDPHS->UDPHS_EPTRST =  1<<CDCDSerialDriverDescriptors_AUDIODATAOUT;          
    //Reset_USBHS_HDMA( CDCDSerialDriverDescriptors_AUDIODATAOUT);   
////////////////////////////////////////////////////////////////////////////////
    
    SSC_Reset(); //I2S_Init();    
    
    Init_Bulk_FIFO();    
    LED_Clear( USBD_LEDDATA );
    
    bulkin_start      = true ; 
    bulkout_start     = true ;    
    bulkout_trigger   = false ;     
    flag_stop         = false ;
    flag_bulkout_empt = false;
    bulkout_empt      = 0;  
    bulkout_padding_ok  = false ;
    
    //reset debug counters
    BO_free_size_max      = 0 ;
    BI_free_size_min      = 100 ; 
    total_received        = 0 ;
    total_transmit        = 0 ;
    error_bulkout_full    = 0 ;
    error_bulkout_empt    = 0 ;
    error_bulkin_full     = 0 ;
    error_bulkin_empt     = 0 ;    
    debug_trans_counter1  = 0 ;
    debug_trans_counter2  = 0 ;
    debug_trans_counter3  = 0 ;
    debug_trans_counter4  = 0 ; 
    debug_usb_dma_IN      = 0 ;
    debug_usb_dma_OUT     = 0 ;    
    test_dump             = 0 ;

}

void Rec_Voice_Buf_Start( void )
{
    im501_irq_counter = 0;
    Enable_SPI_Port(Voice_Buf_Cfg.spi_speed, Voice_Buf_Cfg.spi_mode);
    Config_GPIO_Interrupt( Voice_Buf_Cfg.gpio_irq, ISR_iM501_IRQ );
    Init_SPI_FIFO();
    global_rec_spi_en = 1;
    
}


void Rec_Voice_Buf_Stop( void )
{
    Disable_SPI_Port();
    Disable_GPIO_Interrupt( Voice_Buf_Cfg.gpio_irq );
    global_rec_spi_en = 0;
}
/*
*********************************************************************************************************
*                                    Audio_State_Control()
*
* Description : Process command from Host MCU via UART.
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void Audio_State_Control( void )
{    
    unsigned char err ;
    unsigned int  temp ;    
    
    if( audio_cmd_index == AUDIO_CMD_IDLE ) {
        return;
    }
    
    err = 0 ;
    if( USBD_GetState() < USBD_STATE_CONFIGURED && 
        audio_cmd_index != AUDIO_CMD_VERSION ) {
        err = ERR_USB_STATE;
        
    } else {
   
        switch( audio_cmd_index ) {
            
            case AUDIO_CMD_START_REC :                
                if( audio_state_check != 0 ) {
                    Audio_Stop(); 
                    Rec_Voice_Buf_Stop(); 
                    Stop_CMD_Miss_Counter++;
                }                 
                bulkout_trigger = true; //trigger paly&rec sync
                err = Audio_Start_Rec();  
                time_start_test = second_counter ;
                audio_state_check = 1;
            break;

            case AUDIO_CMD_START_PLAY :                
                if( audio_state_check != 0 ) {
                    Audio_Stop(); 
                    Rec_Voice_Buf_Stop(); 
                    Stop_CMD_Miss_Counter++;
                }                     
                err = Audio_Start_Play();  
                time_start_test = second_counter ;
                audio_state_check = 2; 
            break;
            
            case AUDIO_CMD_START_PALYREC :                
                if( audio_state_check != 0 ) {
                    Audio_Stop();
                    Rec_Voice_Buf_Stop(); 
                    Stop_CMD_Miss_Counter++;
                }                                         
                err = Audio_Start_Play();
                if( err == 0 ) {                    
                  delay_ms(1);  //make sure play and rec enter interruption in turns 2ms              
                  err = Audio_Start_Rec(); 
                }
                time_start_test = second_counter ;
                audio_state_check = 3; 
            break;

            case AUDIO_CMD_STOP : 
                if( audio_state_check != 0 ) {
                  Audio_Stop(); 
                  Rec_Voice_Buf_Stop(); 
                  printf("\r\nThis cycle test time cost: ");
                  Get_Run_Time(second_counter - time_start_test);   
                  printf("\r\n\r\n");
                  time_start_test = 0 ;
                  audio_state_check = 0; 
                }
            break;   
        
            case AUDIO_CMD_CFG: 
                if( Audio_Configure[1].bit_length == 16 ) {
                    temp = Audio_Configure[1].sample_rate / 1000 *  Audio_Configure[1].channel_num * 2 * 2;
                } else { //32
                    temp = Audio_Configure[1].sample_rate / 1000 *  Audio_Configure[1].channel_num * 2 * 4;        
                }            
                if( (temp * PLAY_BUF_DLY_N) > USB_OUT_BUFFER_SIZE ) { //play pre-buffer must not exceed whole play buffer
                    err = ERR_AUD_CFG;
                }              
            break;
            
            case AUDIO_CMD_VERSION: 
                USART_WriteBuffer( AT91C_BASE_US0,(void *)fw_version, sizeof(fw_version) );  //Version string, no ACK 
            break;         
            
//            case AUDIO_CMD_RESET:                 
//                printf("\r\nReset USB EP...");   
//                if( audio_state_check != 0 ) { //in case of error from repeat Stop CMD 
//                    Toggle_PID_BI =  Check_Toggle_State();
//                }
//                //Reset Endpoint Fifos
//                AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_AUDIODATAOUT;
//                AT91C_BASE_UDPHS->UDPHS_EPTRST = 1<<CDCDSerialDriverDescriptors_AUDIODATAIN; 
//                delay_ms(10);
//                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAOUT].UDPHS_EPTCLRSTA = 0xFFFF; //AT91C_UDPHS_NAK_OUT | AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;                  
//                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAIN].UDPHS_EPTCLRSTA  = 0xFFFF;//AT91C_UDPHS_TOGGLESQ | AT91C_UDPHS_FRCESTALL;
//                AT91C_BASE_UDPHS->UDPHS_EPT[CDCDSerialDriverDescriptors_AUDIODATAIN].UDPHS_EPTSETSTA  = AT91C_UDPHS_KILL_BANK ;
//                printf("Done.\r\n");
//                delay_ms(10); 
//                printf("\r\nReset USB EP...");
    
            break;  
            
            case AUDIO_CMD_READ_VOICE_BUF_START :
                Rec_Voice_Buf_Start();
   
            break; 
            
            case AUDIO_CMD_READ_VOICE_BUF_STOP :
                Rec_Voice_Buf_Stop(); 
                
            break;             
            
            default:         
                err = ERR_CMD_TYPE;
            break;
        
        }
        
     }   
    
     if( audio_cmd_index != AUDIO_CMD_VERSION ) {       
         USART_Write( AT91C_BASE_US0, err, 0 ); //ACK
            
     }    
     audio_cmd_index = AUDIO_CMD_IDLE ;      
    
}

/*
*********************************************************************************************************
*                                    Debug_Info()
*
* Description : Print debug infomation via UART port.
* Argument(s) : None.
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void Debug_Info( void )
{
  
    unsigned int BO_free_size ;
    unsigned int BI_free_size ;
    unsigned int DBGUART_free_size ;
    
    static unsigned int counter;
    static unsigned int counter2;
  
    
    if( !(bulkout_enable || bulkin_enable) ) { 
        if( Check_SysTick_State() == 0 ) { 
              return;
        }
        if( counter++ % 20 == 0 ) { //100ms * 20 = 2s              
            counter2++;
            printf("\r\n");
        } 
        printf("\r");
        Get_Run_Time(second_counter);
        
        if( counter2 & 0x01 ) { //            
            BO_free_size = kfifo_get_free_space(&bulkout_fifo_cmd) ;
            BO_free_size = BO_free_size * 100 / USB_CMD_OUT_BUFFER_SIZE;    
            BI_free_size = kfifo_get_free_space(&bulkin_fifo_cmd) ;  
            BI_free_size = BI_free_size * 100 / USB_CMD_IN_BUFFER_SIZE;            
            printf(" CMD:   IN[%6.6f MB, Free:%3u%]  OUT[%6.6f MB, Free:%3u%]. ",  
                   total_transmit_cmd/1000000.0,  BI_free_size,  total_received_cmd/1000000.0,  BO_free_size ); 
            return;   
        } 
        
        printf(" AUDIO: [LostStop: %d][LastPadding: 0x%X]  Wait for starting...",Stop_CMD_Miss_Counter,usb_data_padding);
        return ; 
        
    }
    

    //start print debug_after USB trans started
    if( (total_received < 102400) && (total_transmit < 102400) ){  
        return;
    } 
         
    BO_free_size = kfifo_get_free_space(&bulkout_fifo) ;
    BO_free_size = BO_free_size * 100 / USB_OUT_BUFFER_SIZE;
    
    BI_free_size = kfifo_get_free_space(&bulkin_fifo) ;  
    BI_free_size = BI_free_size * 100 / USB_IN_BUFFER_SIZE; 
    
    BO_free_size_max = BO_free_size > BO_free_size_max ? BO_free_size : BO_free_size_max ;
    BI_free_size_min = BI_free_size < BI_free_size_min ? BI_free_size : BI_free_size_min ;
    
    if( Check_SysTick_State() == 0 ){ 
        return;
    }     

    if( counter++ % 20 == 0 ) { //100ms * 20 = 2s 
        printf("\r\n");        
    } 
            
//    if( (total_received>>1) > total_transmit ) {
//        printf( "\rbulkin_start = %d , bulkin_start = %d, bulkin_fifo data size = %d ",                
//                            bulkin_enable,
//                            bulkin_start,
//                            kfifo_get_data_size(&bulkin_fifo));
//        return;
//    }
    //if(total_transmit >5000000 ) {  error_bulkin_full++; } //simulate bulkin fifo full error 
    //printf("\r\nPLAY %d, REC %d",counter_play++,counter_rec++); 
    //printf("\rIN[Size:%6.6f MB, Full:%u, Empty:%u, FreeSize:%3u%>%3u%]  OUT[Size:%6.6f MB, Full:%u, Empty:%u, FreeSize:%3u%<%3u%]",
      
    if( bo_check_sync ) {
        bo_check_sync = false;
        printf("\r\nReceived USB Sync package.\r\n");
    }
    
    printf("\rAUDIO: IN[%6.6f MB, Full:%u, Empty:%u, Free:%3u%>%3u%]  OUT[%6.6f MB, Full:%u, Empty:%u, Free:%3u%<%3u%]. ",
                         
               total_transmit/1000000.0,               
               error_bulkin_full,
               error_bulkin_empt,
               BI_free_size,
               BI_free_size_min,               
               
               total_received/1000000.0,
               error_bulkout_full,
               error_bulkout_empt,
               BO_free_size,           
               BO_free_size_max 
                   
            
             ); 
     
    Get_Run_Time(second_counter - time_start_test);
     
//     DBGUART_free_size = kfifo_get_free_space(&dbguart_fifo) ;
//     DBGUART_free_size = DBGUART_free_size * 100 / DBGUART_FIFO_SIZE;  
//     DBGUART_free_size_min = DBGUART_free_size < DBGUART_free_size_min ? DBGUART_free_size : DBGUART_free_size_min ;
//     printf( " [DBGUART:%3u%>%3u%]", DBGUART_free_size, DBGUART_free_size_min );                         
     
}


void Get_Run_Time( unsigned int time )
{
    unsigned char  msec, sec, min, hour;
    unsigned int   day;

    msec = time % 10;
    sec  = time /10 % 60 ;
    min  = time / 600 %60 ;
    hour = time / 36000 %24 ; 
    day  = time / 36000 /24 ;
    printf("[%d:%02d:%02d:%02d.%d]", day, hour, min, sec, msec ); 
   
    
}

