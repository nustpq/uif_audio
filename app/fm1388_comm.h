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
#ifndef __FM1388_COMM_H__
#define __FM1388_COMM_H__

//#define  DSP_CMD_ADDR                    (0x5FFDFF7E)//(0x5FFDFFCC)//
//#define  DSP_STOP_CMD                    (0)
//#define  DSP_INIT_CMD                    (0x8B2F)
//#define  DSP_READY_CMD                   (0x8A2F)
//
//#define  DSP_BUFFER_ADDR0                (0x5FFDFF80)
//#define  DSP_BUFFER_ADDR1                (0x5FFDFF84)
//#define  DSP_BUFFER_ADDR2                (0x5FFDFF88)
//#define  DSP_BUFFER_ADDR3                (0x5FFDFF8C)
//#define  DSP_SPI_FRAMESIZE_ADDR          (0x5FFDFF90)
//       
//#define SUCCESS                          0u
//#define NO_ERR                           0u
//#define NULL                             0u

//#define SPI_BUS_ERR                      179u
//#define TO_501_CMD_ERR                   181u 

//
//#define SPI_FIFO_SIZE                  4096
//
//#define SPI_BUF_SIZE                     4096


#define     FM1388_SPI_S_16BIT_RD      0x00
#define     FM1388_SPI_S_16BIT_WR      0x01
#define     FM1388_SPI_S_32BIT_RD      0x02
#define     FM1388_SPI_B_RD            0x04

#define     SYNC_WORD_ADDR    0x5FFDFF7E

#define     P_RDY_1             (1<<0)
#define     P_RDY_0             (0<<0)
#define     R_RDY_1             (1<<1)
#define     R_RDY_0             (0<<1)
#define     P_ERR_1             (1<<2)
#define     P_ERR_0             (0<<2)
#define     R_ERR_1             (1<<3)
#define     R_ERR_0             (0<<3)
#define     P_EN_1              (1<<14)
#define     P_EN_0              (0<<14)
#define     R_EN_1              (1<<15)
#define     R_EN_0              (0<<15)
#define     KEY_PHRASE          (0x25A<<4)

//extern unsigned int  global_rec_spi_en;
//extern unsigned int  global_play_spi_en;
////extern unsigned char global_rec_spi_fast;

extern unsigned char im501_irq_counter;
extern unsigned char SPI_Data_Buffer[];
extern unsigned char SPI_Data_Buffer2[]; 
extern unsigned char SPI_DATA[];

extern unsigned int  FM1388_Rec_Data_Addr[];

void Service_To_FM1388_Poll( void );


unsigned char Request_Start_Voice_Buf_Trans( void );
unsigned char Request_Stop_Voice_Buf_Trans( void );
unsigned char Request_Enter_PSM( void );

unsigned char spi_rec_start_cmd( void );
unsigned char spi_rec_stop_cmd( void );
unsigned char spi_rec_get_addr( void );
unsigned char spi_rec_enable(void);
unsigned char SPI_Rec_Over( void );
unsigned char get_ch_num_and_ch_index(unsigned short mask);
unsigned char init_ch_num_and_ch_index(void);
unsigned char get_play_ch_num_and_play_ch_index(unsigned short mask);
unsigned char init_play_ch_num_and_play_ch_index(void);

void Play_Voice_Buf_Start( void );
void Rec_Voice_Buf_Start( void );
void Rec_Voice_Buf_Stop( void );

extern unsigned char ch_num;
extern unsigned char play_ch_num;
extern unsigned char timedelay_flag;
extern unsigned char spi_play_timertick;
#endif
