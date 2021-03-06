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

#ifndef _APP_INC_
#define _APP_INC_

//#define METHOD_BY_RESET_MCU

//#define DEBUG_VOICE_BUFFER

//Softpack Version
#define MCK                             BOARD_MCK
#define I2S_IN_BUFFER_SIZE              4096//3072//1536   //audio data transfered per frame, Max 48 kHz:   48k*8Slot*2ms*4B=3072
#define I2S_OUT_BUFFER_SIZE             4096//1536  
#define USBDATAEPSIZE                   BOARD_USB_ENDPOINTS_MAXPACKETSIZE( CDCDSerialDriverDescriptors_AUDIODATAIN ) //64//512
#define USB_OUT_BUFFER_SIZE             16384//32768//16384  //2^14=16384  //USB audio data, size MUST be 2^n .
#define USB_IN_BUFFER_SIZE              (8192)//16384  //2^14=16384  //USB audio data, size MUST be 2^n .

#define PLAY_BUF_DLY_N                  5//  delay 5 * 2ms = 10 ms // 3072 * 5 = 15360 < 16384

//#define SPI_CLK                         (1000000*10)      //define in spi.h
////////////////////////////////////////////////////////////////////////////////
#define UART_IN_BUFFER_SIZE             128   
#define UART_OUT_BUFFER_SIZE            128   
#define USBCMDDATAEPSIZE                BOARD_USB_ENDPOINTS_MAXPACKETSIZE( CDCDSerialDriverDescriptors_CMDDATAIN ) //64
#define USB_CMD_OUT_BUFFER_SIZE         1024//USB audio data, size MUST be 2^n .
#define USB_CMD_IN_BUFFER_SIZE          1024//USB audio data, size MUST be 2^n .



// A programmable priority level of 0-15 for each interrupt. A higher level corresponds to a lower 
// priority, so level 0 is the highest interrupt priority

//#define PIO_PRIORITY        7

#define TIMER_PRIORITY      6
#define UART_PRIORITY       4
#define GPIO_PRIORITY       3
#define USB_PRIORITY        2
#define HDMA_PRIORITY       2 //SSC must have highest priority, but now


#define  AUDIO_CMD_IDLE                 0x00
#define  AUDIO_CMD_START_REC            0x01
#define  AUDIO_CMD_START_PLAY           0x02
#define  AUDIO_CMD_START_PALYREC        0x03
#define  AUDIO_CMD_STOP                 0x04
#define  AUDIO_CMD_CFG_REC              0x05
#define  AUDIO_CMD_CFG_PLAY             0x06
#define  AUDIO_CMD_VERSION              0x07
#define  AUDIO_CMD_RESET                0x08
#define  AUDIO_CMD_READ_VOICE_BUF       0x09


#define  AUDIO_STATE_STOP               0x00
#define  AUDIO_STATE_PLAY               0x01
#define  AUDIO_STATE_REC                0x02
#define  AUDIO_STATE_PLAYREC            0x03

//Error
#define  ERR_USB_STATE                  250u
#define  ERR_AUD_CFG                    251u
#define  ERR_CMD_TYPE                   252u
#define  ERR_TDM_FORMAT                 253u
#define  ERR_SPI_REC                    254u

#define  ATTRI_DUT_ID_IM501             501u
#define  ATTRI_DUT_ID_FM1388            1388u

//ERROR CODE from 0 ~ 244 reserved for Audio MCU


typedef struct { 
  unsigned char  type;//Rec =0, Play =1
  unsigned char  channel_num; //valid data cahnnel num 1~8  
  unsigned short sample_rate; //8000, 16000, 24000, 32000, 48000 

  unsigned char  bit_length; // 16, 24, 32
  unsigned char  lin_ch_mask;
  
  unsigned char  gpio_rec_num;
  unsigned char  gpio_rec_start_index;
  unsigned char  gpio_rec_bit_mask;
  
  unsigned char  spi_rec_num;
  unsigned char  spi_rec_start_index;
  unsigned char  spi_rec_bit_mask;
  
  unsigned char  format;  //1:I2S  2:PDM  3:PCM/TDM  
  unsigned char  slot_num;  //bus BCLK slot num
  unsigned char  ssc_cki;
  unsigned char  ssc_delay;
  
  unsigned char  ssc_start;
  unsigned char  master_slave;
  unsigned char  bclk_polarity;  //0 or 1
  
  unsigned char  reserved[1];
}AUDIO_CFG;



typedef void  (*CPU_FNCT_VOID)(void);



extern AUDIO_CFG  Audio_Configure[];
extern unsigned char audio_cmd_index;
extern unsigned char usb_data_padding;
extern unsigned char audio_cmd_ack;

extern unsigned char usbBufferBulkOut[];
extern unsigned char usbBufferBulkIn[];
extern unsigned char FIFOBufferBulkOut[];
extern unsigned char FIFOBufferBulkIn[]; 
extern unsigned char I2SBuffersOut[2][I2S_OUT_BUFFER_SIZE]; 
extern unsigned char I2SBuffersIn[2][I2S_IN_BUFFER_SIZE]; 
extern volatile unsigned char i2s_buffer_out_index;
extern volatile unsigned char i2s_buffer_in_index;


extern unsigned char usbCmdBufferBulkOut[];
extern unsigned char usbCmdBufferBulkIn[]; 
extern unsigned char FIFOBufferBulkOutCmd[];
extern unsigned char FIFOBufferBulkInCmd[];  
extern unsigned char UARTBuffersOut[UART_OUT_BUFFER_SIZE]; 
extern unsigned char UARTBuffersIn[UART_IN_BUFFER_SIZE];
extern volatile unsigned char uart_buffer_out_index ;
extern volatile unsigned char uart_buffer_in_index ;

extern kfifo_t bulkout_fifo_cmd;
extern kfifo_t bulkin_fifo_cmd;


extern volatile unsigned int i2s_play_buffer_size ;
extern volatile unsigned int i2s_rec_buffer_size ;

//extern unsigned char audio_state_check;
extern unsigned char  sample_index;


extern volatile bool bulkin_start;
extern volatile bool bulkout_start;
extern volatile bool bulkin_start_cmd;
extern volatile bool bulkout_start_cmd;
extern volatile bool uartin_start_cmd;
extern volatile bool uartout_start_cmd;

extern volatile bool bulkout_enable;
extern volatile bool bulkin_enable;
extern volatile bool bulkout_trigger;
extern volatile bool flag_stop;
extern volatile bool bulkout_padding_ok;
extern kfifo_t bulkout_fifo;
extern kfifo_t bulkin_fifo;
extern kfifo_t spi_rec_fifo;

extern volatile unsigned char Toggle_PID_BI ;
extern volatile unsigned int bulkout_empt;
extern volatile bool         flag_bulkout_empt;
extern void Debug_Info( void );
extern void Usb_Init(void);
extern void Init_Buffer( void );
extern void Audio_State_Control( void );

void Get_Run_Time( unsigned int time );
void UART0_Init( void );
void UART1_Init( void );
void Check_UART_CMD( void );

void USB_Init( void );
void I2S_Init( void );
void Init_DMA( void );
void I2S_ReInit( void );
void SSC_Play_Start(void);
void SSC_Record_Start(void);
void SSC_Play_Stop(void);
void SSC_Record_Stop(void);
void Init_I2S_Buffer( void );
bool First_Pack_Check_BO( unsigned int size  );
unsigned char SPI_Rec_Start( void );
  
extern void Init_Bus_Matix();

extern char fw_version[];

extern unsigned int counter_play ;
extern unsigned int counter_rec ;

extern unsigned char global_rec_spi_mask         ;  //spi   to record 
extern unsigned char global_rec_spi_num           ;  //total spi channel num to record
extern unsigned char global_rec_spi_index         ;  //index spi start at which TDM channel 
extern unsigned char global_rec_spi_buffer_size   ; 

extern unsigned int test_dump;
extern volatile unsigned int debug_trans_counter1 ;
extern volatile unsigned int debug_trans_counter2 ;  
extern volatile unsigned int debug_trans_counter3 ;
extern volatile unsigned int debug_trans_counter4 ;  
extern volatile unsigned int debug_usb_dma_IN ;
extern volatile unsigned int debug_usb_dma_OUT;
#endif //#ifndef APP_H
