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
*                                          Atmel AT91SAM3U4C
*                                               on the
*                                      Unified EVM Interface Board
*
* Filename      : im501_comm.c
* Version       : V2.0.0
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
#include "im501_comm.h"
#include "spi_rec.h"
#include "kfifo.h"
#include "app.h"
#include "gpio.h"


VOICE_BUF  voice_buf_data;
unsigned char im501_irq_counter;

unsigned char MCU_Load_Vec( unsigned char firsttime );

unsigned char im501_read_reg_i2c( unsigned char reg_addr, unsigned char *pdata );
unsigned char im501_read_reg_spi( unsigned char reg_addr, unsigned char *pdata );

unsigned char im501_write_reg_i2c( unsigned char reg_addr, unsigned char data );
unsigned char im501_write_reg_spi( unsigned char reg_addr, unsigned char data );

unsigned char im501_read_dram_i2c( unsigned int mem_addr, unsigned char *pdata );
unsigned char im501_read_dram_spi( unsigned int mem_addr, unsigned char *pdata );

unsigned char im501_burst_read_dram_spi( unsigned int mem_addr, unsigned char **pdata, unsigned int data_len );

unsigned char im501_write_dram_i2c( unsigned int mem_addr, unsigned char *pdata );
unsigned char im501_write_dram_spi( unsigned int mem_addr, unsigned char *pdata );

unsigned char im501_switch_i2c_spi( unsigned char if_type, unsigned char spi_mode );

unsigned char Request_Start_Voice_Buf_Trans( void );
unsigned char Request_Stop_Voice_Buf_Trans( void );
unsigned char Request_Enter_PSM( void );

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
    pbuf = pSPI_Data_Buffer;//(unsigned char *)SPI_Data_Buffer; //global usage
    
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
    pbuf  = pSPI_Data_Buffer;//(unsigned char *)SPI_Data_Buffer; //global usage
  
    
    buf[0] =  IM501_SPI_CMD_DM_RD;
    buf[1] =  mem_addr & 0xFC;
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
    pbuf = pSPI_Data_Buffer;//(unsigned char *)SPI_Data_Buffer; //global usage    
    
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
* Argument(s) :  if_type     is for indicating which interface will be actived :
*                            I2C : if_type = 1
*                            SPI : if_type = 2
*
*                spi_mode    is for indicating SPI format( 0~3 ), used only when if_type=2
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
    if(( data & 0x04 ) && (if_type == 2) ) { // I2C mode, need switch to SPI
        err = im501_write_reg_spi(iM501_I2C_SPI_REG, 0x80+(spi_mode&0x03));
        if( err != 0 ) {
            return err;
        }
        err = im501_write_reg_spi(iM501_I2C_SPI_REG, spi_mode&0x03);
        if( err != 0 ) {
            return err;
        }
    } else if( (!(data & 0x04 )) && (if_type == 1) ) { // SPI mode, need switch to I2C
        err = im501_write_reg_i2c(iM501_I2C_SPI_REG, 0x04);
        if( err != 0 ) {
            return err;
        }
    }
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           fetch_voice_data()
*
* Description :  read voice data from iM501 DRAM via SPI 
*
* Argument(s) :  start_addr     is DRAM data address
*
*                data_length    is data size(bytes) to read
*
* Return(s)   :  error number.           
*
* Note(s)     :  
*********************************************************************************************************
*/
static unsigned char fetch_voice_data( unsigned int start_addr, unsigned int data_length )
{
    unsigned char err;
    unsigned char *pbuf;
    unsigned int  i,a,b;
    
#if( 0 )   /////////debug    
    unsigned int  test;
    test = 0;
#endif
    
    a = data_length / IM501_BUF_SIZE ;
    b = data_length % IM501_BUF_SIZE ;    
    
    for( i=0; i<a; i++ ) {   
        err = im501_burst_read_dram_spi( start_addr,  &pbuf,  IM501_BUF_SIZE ); 
        if( err != NO_ERR ){ 
            return err;
        }    
        start_addr +=  IM501_BUF_SIZE;               
        while ( IM501_BUF_SIZE > kfifo_get_free_space( &spi_rec_fifo ) ) ; //atom operation?
#if( 0 )   /////////debug
        unsigned short *ps = (unsigned short *)pbuf;
        for( unsigned int k = 0; k < (IM501_BUF_SIZE>>1); k++) {
            *(ps++) = test++;    
        }
#endif  ////////////////
        kfifo_put(&spi_rec_fifo, pbuf, IM501_BUF_SIZE);          
    }
    
    if( b > 0 ) {
        err = im501_burst_read_dram_spi( start_addr,  &pbuf,  b ); 
        if( err != NO_ERR ){ 
            return err;
        }        
        while ( b > kfifo_get_free_space( &spi_rec_fifo ) ); //atom operation?
#if( 0 )    /////////debug
        unsigned short *ps = (unsigned short *)pbuf;
        for( unsigned int k = 0; k < (b>>1); k++) {
            *(ps++) = test++;    
        }
#endif   ///////////////
        kfifo_put(&spi_rec_fifo, pbuf, b);        
    }
    
    return err;
    
}


/*
*********************************************************************************************************
*                                           parse_to_host_command()
*
* Description :  parse cmd from iM501 
*
* Argument(s) :  cmd     is To_Host_CMD type data
*
* Return(s)   :  error number.           
*
* Note(s)     :  
*********************************************************************************************************
*/
unsigned char parse_to_host_command( To_Host_CMD cmd )
{
    unsigned char err; 
    unsigned int address;
       
    switch( cmd.cmd_byte ) {
        
        case TO_HOST_CMD_KEYWORD_DET : //Info host Keywords detected
              //this is implemeted in HOST MCU only
        break;
        
        case TO_HOST_CMD_DATA_BUF_RDY : //Reuest host to read To-Host Buffer-Fast         
            voice_buf_data.index   = (cmd.attri >>8) & 0xFFFF;  //package index
            printf("Pack No.[%5d], Bank[%0X]\r\n",voice_buf_data.index, cmd.attri & 0xFF );            
            if( (cmd.attri & 0xFF) == 0xF0 ) {  
               address = HW_VOICE_BUF_START; //BANK0 address 
            } else {
               address = HW_VOICE_BUF_START + HW_VOICE_BUF_BANK_SIZE ; //BANK1 address
            }
            err = fetch_voice_data( address, HW_VOICE_BUF_BANK_SIZE ); 
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


/*
*********************************************************************************************************
*                                           send_to_dsp_command()
*
* Description :  parse cmd from iM501 
*
* Argument(s) :  cmd     is To_501_CMD type data
*
* Return(s)   :  error number.           
*
* Note(s)     :  
*********************************************************************************************************
*/
unsigned char send_to_dsp_command( To_501_CMD cmd )
{
    unsigned char err;
    unsigned int  i;
    
    err = im501_write_dram_spi( TO_DSP_CMD_ADDR, (unsigned char *)&cmd );
    if( err != 0 ){ 
        return err;
    }
    err = im501_write_reg_spi( 0x01,  cmd.cmd_byte ); //generate interrupt to DSP
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


/*
*********************************************************************************************************
*                                    Write_CMD_To_iM501()
*
* Description :  Send command data to iM501 
*
* Argument(s) :  cmd_index : command index number
*                para      : parameters
* 
* Return(s)   :  Error number.
*
* Note(s)     :  None.
*********************************************************************************************************
*/
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


/*
*********************************************************************************************************
*                                    Request_Start_Voice_Buf_Trans()
*
* Description :  Send command(0x19) to iM501 to request voice buf data fetch 
*
* Argument(s) :  None.
*
* Return(s)   :  Error number.
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char Request_Start_Voice_Buf_Trans( void )
{
    
    unsigned char err; 
    
    err = Write_CMD_To_iM501( TO_DSP_CMD_REQ_START_BUF_TRANS, 0 );
 
    return err;
    
}


/*
*********************************************************************************************************
*                                    Request_Stop_Voice_Buf_Trans()
*
* Description :  Send command(0x1D) to iM501 to stop voice buf data fetch 
*
* Argument(s) :  None.
*
* Return(s)   :  error number.
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char Request_Stop_Voice_Buf_Trans( void )
{
    
    unsigned char err;
        
    err = Write_CMD_To_iM501( TO_DSP_CMD_REQ_STOP_BUF_TRANS, 0 );
 
    return err;
    
}


/*
*********************************************************************************************************
*                                    Request_Enter_PSM()
*
* Description :  Send command(0x0D) to iM501 to prepare for entering power saving mode 
*
* Argument(s) :  None.
*
* Return(s)   :  error number.
*
* Note(s)     :  None.
*********************************************************************************************************
*/
unsigned char Request_Enter_PSM( void )
{
    
    unsigned char err;
        
    err = Write_CMD_To_iM501( TO_DSP_CMD_REQ_ENTER_PSM, 0 );
 
    return err;
    
}


/*
*********************************************************************************************************
*                                    Service_To_iM501_IRQ()
*
* Description :  Service to iM501 IRQ interruption 
*                Should be in Main Loop, inquiring the IRQ signal by checking im501_irq_counter
*
* Argument(s) :  None.
*
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/
void Service_To_iM501_IRQ( void )
{  
    
    unsigned char err;
    To_Host_CMD   cmd;
    
    if( global_rec_spi_en == 0 ) {
        return ;
    }
    
#if( 0 ) //debug     
    global_rec_spi_fast = 1;
    fetch_voice_data( HW_VOICE_BUF_START, 32768);            
#else   
    
    if ( im501_irq_counter ) {
        
        im501_irq_counter--; //
        //APP_TRACE_INFO(("::ISR_iM501_IRQ : %d\r\n",im501_irq_counter));   //for test 
          
        err = im501_read_dram_spi( TO_HOST_CMD_ADDR, (unsigned char *)&cmd );
        if( err != 0 ){
            return ;
        }
        
        err = parse_to_host_command( cmd );
        if( err != 0 ){ 
            return ;
        }
        
        cmd.status = 0;
        err = im501_write_dram_spi( TO_HOST_CMD_ADDR, (unsigned char *)&cmd );
        if( err != 0 ){ 
            return ;
        }           
    }       
   
#endif   
    
}

                          
/*
*********************************************************************************************************
*                                           ISR_iM501_IRQ()
*
* Description :  Interruption service routine for iM501 IRQ
*
* Argument(s) :  pPin     : useless
*
* Return(s)   :  None.           
*
* Note(s)     :  The ISR register is read in PioInterruptHandler().
*********************************************************************************************************
*/
void ISR_iM501_IRQ( const Pin *pPin )
{
   im501_irq_counter++;        
   
}


/*
*********************************************************************************************************
*                                    iM501_SPI_Rec_Start()
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
unsigned char iM501_SPI_Rec_Start( unsigned char gpio_irq )
{
    unsigned char err = 0;
    
    im501_irq_counter = 0;     
    Config_GPIO_Interrupt( gpio_irq, ISR_iM501_IRQ );
    err = Request_Start_Voice_Buf_Trans();        
   
    return err;
}


/*
*********************************************************************************************************
*                                    iM501_SPI_Rec_Stop()
*
* Description :  Stop SPI data recordin.
*
* Argument(s) :  None.
*
* Return(s)   :  error number. 
*
* Note(s)     : None.
*********************************************************************************************************
*/
unsigned char iM501_SPI_Rec_Stop( void )
{
    
    unsigned char err = 0 ;   

    err = Request_Stop_Voice_Buf_Trans(); 

    
    return err;
    
}

