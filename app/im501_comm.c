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
*                                        iM501 Communication Related
*
*                                          Atmel AT91SAM7A3
*                                               on the
*                                      Unified EVM Interface Board
*
* Filename      : im501_comm.c
* Version       : V1.0.0
* Programmer(s) : PQ
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

#include <stdbool.h>
#include <intrinsics.h>
#include <tc/tc.h>
#include "im501_comm.h"
#include "kfifo.h"
#include "app.h"
#include "gpio.h"

VOICE_BUF  voice_buf_data;

unsigned char SPI_FIFO_Buffer[ SPI_FIFO_SIZE ] ;
unsigned char SPI_Data_Buffer[ SPI_BUF_SIZE +1 ];  //+1 fix spi bug
unsigned char SPI_Data_Buffer2[ SPI_BUF_SIZE ]; 
unsigned int im501_irq_counter;

unsigned int global_rec_spi_en = 0 ;
unsigned char global_rec_spi_fast = 0; //true: fast read, false: real time rec


void Init_SPI_FIFO( void )
{   
    kfifo_t *pfifo;
    
    pfifo = &spi_rec_fifo;;
    kfifo_init_static(pfifo, SPI_FIFO_Buffer, SPI_FIFO_SIZE);
    
}


/*
*********************************************************************************************************
*                                           im501_read_reg_i2c()
*
* Description :  read iM501 Reg via I2C, just read one byte.
*
* Argument(s) :  reg_addr      is the iM501 register adress(one byte length)
*
*                *pdata        is point to where read back data will be stored  
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char im501_read_reg_i2c( unsigned char reg_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[2];
    
    err    =  0;
    buf[0] =  IM501_I2C_CMD_REG_RD;
    buf[1] =  reg_addr;
    state  =  TWID_Write( iM501_I2C_ADDR>>1,
                         0, 
                         0, 
                         buf, 
                         sizeof(buf), 
                         NULL );     
    if (state != SUCCESS) {
        err = I2C_BUS_ERR;
        return err;
    } 
   
    state =  TWID_Read( iM501_I2C_ADDR>>1,
                        0, 
                        0, 
                        pdata, 
                        1, 
                        NULL );     
    if (state != SUCCESS) {
        err = I2C_BUS_ERR;        
    } 
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           im501_read_reg_spi()
*
* Description :  read iM501 Reg via SPI, just read one byte.
*
* Argument(s) :  reg_addr      is the iM501 register adress(one byte length)
*
*                *pdata        is point to where read back data will be stored
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/


unsigned char im501_read_reg_spi( unsigned char reg_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[2];
    unsigned char *pbuf;
    
    err  = NO_ERR;
    pbuf = (unsigned char *)SPI_Data_Buffer; //global usage
  
    buf[0] =  IM501_SPI_CMD_REG_RD;
    buf[1] =  reg_addr;
    
    state =  SPI_WriteReadBuffer_API(  pbuf, 
                                       buf, 
                                       1 , 
                                       sizeof(buf));
             
    if (state != SUCCESS) {
        err = SPI_BUS_ERR;
        //APP_TRACE_INFO(("\r\nSPI_ReadBuffer_API err = %d",state));
        return err;
    }              
    pbuf = pbuf + 1; //fix bug
    *pdata = *pbuf;
    
    return err;
    
}




/*
*********************************************************************************************************
*                                           im501_write_reg_i2c()
*
* Description :  write iM501 Reg via I2C, just write one byte.
*
* Argument(s) :  reg_addr      is the iM501 register adress(one byte length)
*
*                data          is the data will be write to im501
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char im501_write_reg_i2c( unsigned char reg_addr, unsigned char data )
{
    unsigned char err, state;
    unsigned char buf[3];
    
    err    =  NO_ERR;
    buf[0] =  IM501_I2C_CMD_REG_WR_1;
    buf[1] =  reg_addr;
    buf[2] =  data;
    state  =  TWID_Write( iM501_I2C_ADDR>>1,
                         0, 
                         0, 
                         buf, 
                         sizeof(buf), 
                         NULL );     
    if (state != SUCCESS) {
        err = I2C_BUS_ERR;
        return err;
    } 
    
    return err;
    
}
/*
*********************************************************************************************************
*                                           im501_write_reg_spi()
*
* Description :  write iM501 Reg via SPI, just write one byte.
*
* Argument(s) :  reg_addr      is the iM501 register adress(one byte length)
*
*                data          is the data will be write to im501
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char im501_write_reg_spi( unsigned char reg_addr, unsigned char data )
{
    unsigned char err, state;
    unsigned char buf[3];
    
    err    =  NO_ERR;
    buf[0] =  IM501_SPI_CMD_REG_WR;
    buf[1] =  reg_addr;
    buf[2] =  data;
      
    state =  SPI_WriteBuffer_API( buf, sizeof(buf) ); 
    if (state != SUCCESS) {
        err = I2C_BUS_ERR;
        return err;
    } 
    
    return err;
    
}



/*
*********************************************************************************************************
*                                           im501_read_dram_i2c()
*
* Description :  read iM501 DRAM via I2C, just read 4 bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 3 bytes length and need 4bytes alignment
*
*                *pdata         is point to where read back data will be stored   
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char im501_read_dram_i2c( unsigned int mem_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[4];
    
    err   =  NO_ERR;
    buf[0] =  IM501_I2C_CMD_DM_RD;
    buf[1] =  mem_addr & 0xFF;
    buf[2] =  (mem_addr>>8) & 0xFF;
    buf[3] =  (mem_addr>>16) & 0xFF;
    state =  TWID_Write( iM501_I2C_ADDR>>1,
                         0, 
                         0, 
                         buf, 
                         sizeof(buf),  
                         NULL );     
    if (state != SUCCESS) {
        err = I2C_BUS_ERR;
        return err;
    }   
    err = im501_read_reg_i2c(0x0A, pdata);
    if( err != NO_ERR ) {
        return err;
    }
    err = im501_read_reg_i2c(0x0B, pdata+1);
    if( err != NO_ERR ) {
        return err;
    }  
    err = im501_read_reg_i2c(0x0C, pdata+2);
    if( err != NO_ERR ) {
        return err;
    } 
    err = im501_read_reg_i2c(0x0D, pdata+3);
    if( err != NO_ERR ) {
        return err;
    }
    
    return err;
    
}

/*
*********************************************************************************************************
*                                           im501_read_dram_spi()
*
* Description :  burst read iM501 DRAM via SPI, just limited to 4 bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 3 bytes length and need 4bytes alignment
*
*                *pdata         is point to where read back data will be stored 
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char im501_read_dram_spi( unsigned int mem_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[6];
    unsigned char *pbuf;
    
    err   =  NO_ERR;
    pbuf  = (unsigned char *)SPI_Data_Buffer; //global usage
  
    
    buf[0] =  IM501_SPI_CMD_DM_RD;
    buf[1] =  mem_addr & 0xFF;
    buf[2] =  (mem_addr>>8) & 0xFF;
    buf[3] =  (mem_addr>>16) & 0xFF;
    buf[4] =  2;
    buf[5] =  0;    

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
    for (unsigned char i = 0; i<4; i++ ) {
      *(pdata+i) = *(pbuf+i);
    }
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           im501_burst_read_dram_spi()
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
*                Be care full this function use fixed buffer, and return a **pointer to.
*********************************************************************************************************
*/
unsigned char im501_burst_read_dram_spi( unsigned int mem_addr, unsigned char **pdata, unsigned int data_len )
{
    unsigned char  err, state;
    unsigned char  buf[6];
    unsigned char *pbuf;
    
    err   =  NO_ERR;
    pbuf = (unsigned char *)SPI_Data_Buffer; //global usage    

#ifndef DEBUG_VOICE_BUFFER
    
    buf[0] =  IM501_SPI_CMD_DM_RD;
    buf[1] =  mem_addr     & 0xFF;
    buf[2] =  (mem_addr>>8) & 0xFF;
    buf[3] =  (mem_addr>>16) & 0xFF;
    buf[4] =  (data_len>>1)   & 0xFF;
    buf[5] =  (data_len>>(1+8))& 0xFF;    

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
    
#else 
    
    *pdata =  pbuf;
    Demo_Sine_Gen( pbuf, data_len, 16000, 1 );
//    for(unsigned int i = 0 ; i<(data_len>>1); i++ ) {
//        *((unsigned short *)pbuf +i) = i; 
//    }

#endif
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           im501_write_dram_i2c()
*
* Description :  write iM501 DRAM via I2C, just write two bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 3 bytes length and need 4bytes alignment
*
*                *pdata         is pointer to where to be written data stores 
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char im501_write_dram_i2c( unsigned int mem_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[6];
    
    err   =  NO_ERR;
    buf[0] =  IM501_I2C_CMD_DM_WR;
    buf[1] =  mem_addr & 0xFF;
    buf[2] =  (mem_addr>>8) & 0xFF;
    buf[3] =  (mem_addr>>16) & 0xFF;
    buf[4] =  *pdata;
    buf[5] =  *(pdata+1);
    
    state =  TWID_Write( iM501_I2C_ADDR>>1,
                         0, 
                         0, 
                         buf, 
                         sizeof(buf),  
                         NULL );  
    
    if (state != SUCCESS) {
        err = I2C_BUS_ERR;
        return err;
    } 
       
    return err;
    
}


/*
*********************************************************************************************************
*                                           im501_write_dram_spi()
*
* Description :  write iM501 DRAM via SPI, just write 4 bytes.
*
* Argument(s) :  mem_addr       is dram adress, total 3 bytes length and need 4bytes alignment
*
*                *pdata         is pointer to where to be written data stores 
*
* Return(s)   :  error number.           
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char im501_write_dram_spi( unsigned int mem_addr, unsigned char *pdata )
{
    unsigned char err, state;
    unsigned char buf[10];
    
    err    =  0;
    
    buf[0] =  IM501_SPI_CMD_DM_WR;
    buf[1] =  mem_addr & 0xFF;
    buf[2] =  (mem_addr>>8) & 0xFF;
    buf[3] =  (mem_addr>>16) & 0xFF;
    buf[4] =  2; //2 words
    buf[5] =  0;
    buf[6] =  *pdata;
    buf[7] =  *(pdata+1);
    buf[8] =  *(pdata+2);
    buf[9] =  *(pdata+3);
      
    state =  SPI_WriteBuffer_API( buf, sizeof(buf) ); 
    if (state != SUCCESS) {
        err = I2C_BUS_ERR;
        return err;
    } 
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           im501_switch_i2c_spi()
*
* Description :  change iM501 actived interface type 
*
* Argument(s) :  if_type     is for indicating which interface will be actived, SPI(if_type = 2) or  I2C(if_type = 1)
*
*                spi_mode    is for indicating  SPI format( 0~3 )
*
* Return(s)   :  error number.           
*
* Note(s)     :  
*********************************************************************************************************
*/
unsigned char im501_switch_i2c_spi( unsigned char if_type, unsigned char spi_mode )
{
    unsigned char err;
    unsigned char data;
    
    err = im501_read_reg_i2c( iM501_I2C_SPI_REG, &data );
    if( err != 0 ) {
        return err;
    }
    if(( data & 0x04 ) && (if_type == 1) ) { // I2C mode, need switch to SPI
        err = im501_write_reg_spi(iM501_I2C_SPI_REG, 0x80+(spi_mode&0x03));
        if( err != 0 ) {
            return err;
        }
        err = im501_write_reg_spi(iM501_I2C_SPI_REG, spi_mode&0x03);
        if( err != 0 ) {
            return err;
        }
    } else if( (!(data & 0x04 )) && (if_type == 2) ) { // SPI mode, need switch to I2C
        err = im501_write_reg_i2c(iM501_I2C_SPI_REG, 0x04);
        if( err != 0 ) {
            return err;
        }
    }
    
    return err;
    
}





unsigned char fetch_voice_data( unsigned int start_addr, unsigned int data_length )
{
    unsigned char err;
    unsigned char *pbuf;
    unsigned int  i,a,b;
     
    a = data_length / SPI_BUF_SIZE ;
    b = data_length % SPI_BUF_SIZE ;
    
    for( i=0; i<a; i++ ) {
        err = im501_burst_read_dram_spi( start_addr,  &pbuf,  SPI_BUF_SIZE ); 
        if( err != NO_ERR ){ 
            return err;
        }    
        start_addr +=  SPI_BUF_SIZE;               
        while ( SPI_BUF_SIZE > kfifo_get_free_space( &spi_rec_fifo ) ) ; //atom operation?
        kfifo_put(&spi_rec_fifo, pbuf, SPI_BUF_SIZE);        
    }
    
    if( b > 0 ) {
        err = im501_burst_read_dram_spi( start_addr,  &pbuf,  b ); 
        if( err != NO_ERR ){ 
            return err;
        }        
        while ( b > kfifo_get_free_space( &spi_rec_fifo ) ); //atom operation?
        kfifo_put(&spi_rec_fifo, pbuf, SPI_BUF_SIZE);        
    }
    
    return err;
    
}

unsigned char parse_to_host_command( To_Host_CMD cmd )
{
    unsigned char err; 
    unsigned int address;
       
    switch( cmd.cmd_byte ) {
        
        case 0x40 : //Infom host Keywords detected

        break;
        
        case 0x41 : //Reuest host to read To-Host Buffer-Fast
            voice_buf_data.length   = (cmd.attri & 0xFFFF ) << 1;  //sample to bytes
            address = HW_VOICE_BUF_START;
            global_rec_spi_fast = 1;
            err = fetch_voice_data( address, voice_buf_data.length ); 
            if( err != NO_ERR ){
                return err;
            }
        break;
        
        case 0x42 : //Reuest host to read To-Host Buffer-RealTime            
            voice_buf_data.length   = ( (cmd.attri & 0xFF0000 )>>16 ) << 1;  //sample to bytes
            address = HW_VOICE_BUF_START + (cmd.attri & 0xFFFF);
            global_rec_spi_fast = 0;  
            err = fetch_voice_data( address,  voice_buf_data.length );
            if( err != NO_ERR ){ 
                return err;
            }
        break;
                       
        default:
            err = 2;           
        break;
        
    }
            
    return err;
    
}



unsigned char send_to_dsp_command( To_501_CMD cmd )
{
    unsigned char err;
    unsigned int  i;
    
    err = im501_write_dram_spi( TO_DSP_CMD_ADDR, (unsigned char *)&cmd );
    if( err != 0 ){ 
        return err;
    }
    err = im501_write_reg_spi( 0x01, cmd.cmd_byte ); //generate interrupt to DSP
    if( err != 0 ){ 
        return err;
    }
    
    for( i = 0; i< 50; i++ ) {   //wait for (50*100us = 5ms) to check if DSP finished 
        err = im501_read_dram_spi( TO_DSP_CMD_ADDR, (unsigned char *)&cmd );
        if( err != 0 ){ 
            return err;
        }
        if( cmd.status != 0 ) {
            err = TO_501_CMD_ERR;
        } else {
            err = 0;
        }
        delay_us(100); //??
    }
    
    return err;
    
}



unsigned char resp_to_host_command( void )
{
    unsigned char err;
    To_Host_CMD   cmd;
    
#ifndef DEBUG_VOICE_BUFFER    
    err = im501_read_dram_spi( TO_HOST_CMD_ADDR, (unsigned char *)&cmd );
    if( err != 0 ){
        return err;
    }
#else
    cmd.cmd_byte = 0x41;
    cmd.attri    = 32768/2; //32k bytes, 16k samples
#endif    
    
    err = parse_to_host_command( cmd );
    if( err != 0 ){ 
        return err;
    }
    
#ifndef DEBUG_VOICE_BUFFER      
    cmd.status = 0;
    err = im501_write_dram_spi( TO_HOST_CMD_ADDR, (unsigned char *)&cmd );
    if( err != 0 ){ 
        return err;
    }
#endif
    
    return err;
    
}



void ISR_iM501_IRQ( void )
{
    if( Check_GPIO_Intrrupt( Voice_Buf_Cfg.gpio_irq ) ) {                       
        im501_irq_counter++;        
    }
}



unsigned char Write_CMD_To_iM501( unsigned char cmd_index, unsigned short para )
{
    
    unsigned char err;
    To_501_CMD    cmd;
    
    cmd.cmd_byte = cmd_index;//((cmd_index & 0x3F) << 2) | 0x01; //D[1] : "1", interrupt DSP. This bit generates NMI (non-mask-able interrupt), D[0]: "1" generate mask-able interrupt
    cmd.attri    = para & 0xFFFF ;
    cmd.status   = 1;
    err = send_to_dsp_command( cmd );
    
    return err;
    
}



unsigned char Request_Start_Voice_Buf_Trans( void )
{
    
    unsigned char err;
        
    err = Write_CMD_To_iM501( TO_DSP_CMD_REQ_START_BUF_TRANS, 0 );
 
    return err;
    
}

unsigned char Request_Stop_Voice_Buf_Trans( void )
{
    
    unsigned char err;
        
    err = Write_CMD_To_iM501( TO_DSP_CMD_REQ_STOP_BUF_TRANS, 0 );
 
    return err;
    
}

unsigned char Request_Enter_PSM( void )
{
    
    unsigned char err;
        
    err = Write_CMD_To_iM501( TO_DSP_CMD_REQ_ENTER_PSM, 0 );
 
    return err;
    
}

void Read_iM501_Voice_Buffer( void )
{  
    
    unsigned char err;
      
    if( global_rec_spi_en == 0 ) {
        return;
    }
        
#ifdef DEBUG_VOICE_BUFFER  
        im501_irq_counter = 1;   
#endif
    
    if ( im501_irq_counter-- ) {            
        //APP_TRACE_INFO(("::ISR_iM501_IRQ : %d\r\n",im501_irq_counter));   //for test       
        err = resp_to_host_command( );            
        if( err != NO_ERR ){ 
            return ;
        }           
            
    } 
       
}




