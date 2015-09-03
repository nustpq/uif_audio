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

#ifndef _UART_H_
#define _UART_H_

#define RULER_CMD_SET_AUDIO_CFG        0x01
#define RULER_CMD_START_AUDIO          0x02
#define RULER_CMD_STOP_AUDIO           0x03
#define RULER_CMD_RESET_AUDIO          0x10
#define RULER_CMD_GET_AUDIO_VERSION    0x0B
#define RULER_CMD_START_RD_VOICE_BUF   0x11
#define RULER_CMD_STOP_RD_VOICE_BUF    0x12

#define CMD_STAT_SYNC1     0
#define CMD_STAT_SYNC2     1
#define CMD_STAT_FLAG      2
#define CMD_STAT_CMD1      3
#define CMD_STAT_CMD2      4
#define CMD_STAT_CMD3      5
#define CMD_STAT_DATA      6
#define CMD_STAT_CMD4      7

#define CMD_DATA_SYNC1     0xEB
#define CMD_DATA_SYNC2     0x90



#define UART_TIMEOUT_BIT     ( 50 * 10  ) // 50=50*10  timeout in 5 Bytes' time  
#define UART_BUFFER_SIZE     ( 128     ) // uart buf size

#define UART0_BAUD           (  115200 )
#define UART1_BAUD           ( 3000000 )



#endif
