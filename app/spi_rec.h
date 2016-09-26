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
#ifndef __SPI_REC_H__
#define __SPI_REC_H__

#define  DSP_CMD_ADDR                    (0x5FFDFF7E)//(0x5FFDFFCC)//
#define  DSP_STOP_CMD                    (0)
#define  DSP_INIT_CMD                    (0x8B2F)
#define  DSP_READY_CMD                   (0x8A2F)

#define  DSP_BUFFER_ADDR0                (0x5FFDFF80)
#define  DSP_BUFFER_ADDR1                (0x5FFDFF84)
#define  DSP_BUFFER_ADDR2                (0x5FFDFF88)
#define  DSP_BUFFER_ADDR3                (0x5FFDFF8C)
#define  DSP_SPI_FRAMESIZE_ADDR          (0x5FFDFF90)
       
#define SUCCESS                          0u
#define NO_ERR                           0u


#define SPI_BUS_ERR                      179u
#define I2C_BUS_ERR                      180u
#define TO_501_CMD_ERR                   181u 


//#define SPI_FIFO_SIZE                    (3072)//2048 < 160*2*8=2560 < 3072
#define FM1388_BUF_SIZE                  (2304)// = 24k*2B*8ms*6CH      //(1920) = 16k*2B*10ms*6CH  
#define FM1388_BUF_SIZE_HALF             (FM1388_BUF_SIZE / 2)  //(1152)// (top + bottom half)
#define IM501_BUF_SIZE                   (2048)//iM501 voice buffer size
//#define SPI_BUF_SIZE                     (FM1388_BUF_SIZE_HALF > IM501_BUF_SIZE ? FM1388_BUF_SIZE_HALF : IM501_BUF_SIZE)
 
#define SPI_FIFO_SIZE                     4096   
#define SPI_BUF_SIZE                      4096



#if(  SPI_BUF_SIZE  > SPI_FIFO_SIZE ) 
#error("SPI_BUF_SIZE must be less than SPI_FIFO_SIZE ")
#endif



typedef struct {
  unsigned int    spi_speed; 
  
  unsigned short  rec_ch_mask;
  unsigned short  play_ch_mask;
  
  unsigned short  chip_id;
  unsigned char   spi_mode;  
  unsigned char   gpio_irq; 
  
  unsigned char   time_dly;  
  unsigned char   reserved[3];

}SPI_PLAY_REC_CFG;

extern unsigned char  global_rec_spi_en;
extern unsigned char  global_play_spi_en;
extern SPI_PLAY_REC_CFG spi_rec_cfg;
extern unsigned char im501_irq_counter;
//extern unsigned char SPI_Data_Buffer[];
//extern unsigned char SPI_Data_Buffer2[]; 


unsigned char SPI_Rec_Start( void );
unsigned char SPI_Play_Start( void );
unsigned char SPI_Rec_Stop( void );
void  SPI_Service( void );

extern unsigned char* pSPI_FIFO_Buffer ; 
extern unsigned char* pSPI_Data_Buffer ;
extern unsigned char* pSPI_Data_Buffer2  ; 

#endif
