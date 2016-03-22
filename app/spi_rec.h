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


#define SPI_FIFO_SIZE                    (3072)//2048 < 160*2*8=2560 < 3072
#define FM1388_BUF_SIZE                  (2304)// = 24k*2B*8ms*6CH      //(1920) = 16k*2B*10ms*6CH  
#define FM1388_BUF_SIZE_HALF             (FM1388_BUF_SIZE / 2)  //(1152)// (top + bottom half)
#define IM501_BUF_SIZE                   (2048)//iM501 voice buffer size
#define SPI_BUF_SIZE                     (FM1388_BUF_SIZE_HALF > IM501_BUF_SIZE ? FM1388_BUF_SIZE_HALF : IM501_BUF_SIZE)
     
#if(  SPI_BUF_SIZE  > SPI_FIFO_SIZE ) 
#error("SPI_BUF_SIZE must be less than SPI_FIFO_SIZE ")
#endif


#define     FM1388_SPI_S_16BIT_RD      0x00
#define     FM1388_SPI_S_16BIT_WR      0x01
#define     FM1388_SPI_S_32BIT_RD      0x02
#define     FM1388_SPI_B_RD            0x04

typedef struct {
  unsigned int   spi_speed;
  unsigned char  spi_mode;  
  unsigned char  gpio_irq;
  unsigned char  chip_id;
  unsigned char  reserved[1];
}SPI_REC_CFG;

extern unsigned char  global_rec_spi_en;
extern SPI_REC_CFG spi_rec_cfg;
extern unsigned char im501_irq_counter;
extern unsigned char SPI_Data_Buffer[];
//extern unsigned char SPI_Data_Buffer2[]; 


unsigned char SPI_Rec_Start( void );
unsigned char SPI_Rec_Stop( void );
void  SPI_Rec_Service( void );

 

#endif
