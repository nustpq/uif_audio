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
#define NULL                             0u

#define SPI_BUS_ERR                      179u
#define TO_501_CMD_ERR                   181u 

#define SPI_FIFO_SIZE                    (3072)//2048 < 160*2*8=2560 < 3072
#define SPI_BUF_SIZE                     (1280)//(160*2*8)/2 

#define     FM1388_SPI_S_16BIT_RD      0x00
#define     FM1388_SPI_S_16BIT_WR      0x01
#define     FM1388_SPI_S_32BIT_RD      0x02
#define     FM1388_SPI_B_RD            0x04


extern unsigned int  global_rec_spi_en;
//extern unsigned char global_rec_spi_fast;

extern unsigned char im501_irq_counter;
extern unsigned char SPI_Data_Buffer[];
extern unsigned char SPI_Data_Buffer2[]; 


//typedef struct {
//    unsigned short attri;
//    unsigned char  cmd_byte;
//    unsigned char  status;
//}To_Host_CMD ;
//
//typedef struct {
//    unsigned short attri;
//    unsigned char  cmd_byte_ext;
//    unsigned char  status;
//    unsigned char  cmd_byte;
//}To_501_CMD ;


typedef struct {
    unsigned int   attri    : 24;
    unsigned int   cmd_byte : 7;
    unsigned int   status   : 1;
}To_Host_CMD ;

typedef struct {
    unsigned int  attri        : 24 ;
    unsigned int  cmd_byte_ext : 7 ;
    unsigned int  status       : 1 ;    
    unsigned char cmd_byte;
}To_501_CMD ;

typedef struct {
    unsigned int   length;
    unsigned int   index;
    unsigned char *pdata;
    unsigned char  done;
}VOICE_BUF ;




//extern unsigned char iM401_Bypass( void );
//extern unsigned char iM401_Standby( void );



unsigned char Request_Start_Voice_Buf_Trans( void );
unsigned char Request_Stop_Voice_Buf_Trans( void );
unsigned char Request_Enter_PSM( void );

unsigned char spi_rec_start_cmd( void );
unsigned char spi_rec_stop_cmd( void );
unsigned char spi_rec_get_addr( void );

#endif
