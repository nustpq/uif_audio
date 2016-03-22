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
       



#define     FM1388_SPI_S_16BIT_RD      0x00
#define     FM1388_SPI_S_16BIT_WR      0x01
#define     FM1388_SPI_S_32BIT_RD      0x02
#define     FM1388_SPI_B_RD            0x04


void Service_To_FM1388_Poll( void );
unsigned char FM1388_SPI_Rec_Start( void ) ;                   
unsigned char FM1388_SPI_Rec_Stop( void ) ;




#endif
