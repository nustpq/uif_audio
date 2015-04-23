#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <utility/led.h>
#include  "gpio.h"

//inner use
static Pin pinsGpios[]  = {          
    
    GPIO_0,     GPIO_1,      GPIO_2,     GPIO_3,     GPIO_4,     
    GPIO_5,     GPIO_6,      GPIO_7
        
};



/*
*********************************************************************************************************
*                                    Init_GPIO()
*
* Description :  Initialize LED GPIOs.
* Argument(s) :  None.
* Return(s)   :  None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

void Init_GPIO( void )
{     
    //PIO_InitializeInterrupts( PIO_PRIORITY ); 
    
    PIO_Configure( pinsGpios, PIO_LISTSIZE(pinsGpios) ); 
     
    LED_Configure(USBD_LEDPOWER);
    LED_Configure(USBD_LEDDATA);   
    //LED_Set(USBD_LEDPOWER); 
    //LED_Set(USBD_LEDDATA); 
  
}




//ranfunc for a faster execution 
unsigned char  GPIOPIN_Set(unsigned int pin , unsigned int dat)
{  


    if( pin >= PIO_LISTSIZE(pinsGpios) ) {
        return 1;
        
    }
    
    //APP_TRACE_INFO(("\r\nSet GPIO[%d]=%d ", pin, dat));

    switch ( dat ) {
        
        case 0: 
           pinsGpios[pin].attribute  = PIO_PULLUP ;
           pinsGpios[pin].type       = PIO_OUTPUT_0   ;  
           //PIO_Clear(&pinsGpios[pin]);
           PIO_Configure(&pinsGpios[pin], 1);
        break;
                
        case 1: 
           pinsGpios[pin].attribute  = PIO_PULLUP ;
           pinsGpios[pin].type       = PIO_OUTPUT_1   ;  
           //PIO_Set(&pinsGpios[pin]);
           PIO_Configure(&pinsGpios[pin], 1);
        break;
        
        case 2: 
            pinsGpios[pin].attribute  = PIO_DEFAULT ; 
            pinsGpios[pin].type       = PIO_INPUT   ;            
            PIO_Configure(&pinsGpios[pin], 1);
        break;
        
        default:
            return 2;
        break;      

    }
    
    return 0;
    
    
}


unsigned char  GPIOPIN_Get(unsigned int pin , unsigned char *pdat)
{  


    if( pin >= PIO_LISTSIZE(pinsGpios) ) {
        return 1;
        
    }
       
    *pdat = PIO_Get(&pinsGpios[pin]);       

    
    return 0;
    
    
}


void  __ramfunc GPIOPIN_Set_Fast( unsigned char pin , unsigned char data )
{    
     
    switch( data ) {
        
        case 0 : //output 0
            
            pinsGpios[pin].pio->PIO_CODR = pinsGpios[pin].mask; 
            pinsGpios[pin].pio->PIO_OER  = pinsGpios[pin].mask;
        break;
        
        case 1 : //output 1
            pinsGpios[pin].pio->PIO_SODR = pinsGpios[pin].mask;
            pinsGpios[pin].pio->PIO_OER  = pinsGpios[pin].mask;
        break; 
        
        case 2 : //input        
            pinsGpios[pin].pio->PIO_ODR  = pinsGpios[pin].mask;
        break;
        
        default:
        break;             
   } 
   
}


void  __ramfunc GPIOPIN_Get_Fast( unsigned char pin, unsigned char * pdata )
{
    unsigned int reg ;
    
    reg = pinsGpios[pin].pio->PIO_PDSR;    

    if ((reg & pinsGpios[pin].mask) == 0) {

        *pdata = 0 ;
    }
    else {

        *pdata = 1 ;
    }
    
}


void  __ramfunc GPIOPort_Get_Fast( unsigned char * pdata )
{
    
    *pdata = (unsigned char) pinsGpios[0].pio->PIO_PDSR;    

    
}




void GPIOPIN_Init_Fast( unsigned int pin )
{
    
    pinsGpios[pin].pio->PIO_IDR = pinsGpios[pin].mask;
    
    //pull up
    pinsGpios[pin].pio->PIO_PPUER = pinsGpios[pin].mask;  //enable
    //pinsGpios[pin].pio->PIO_PPUDR = pinsGpios[pin].mask;
    
    //multi-drive OP
    //pinsGpios[pin].pio->PIO_MDER = pinsGpios[pin].mask;  //enable
    pinsGpios[pin].pio->PIO_MDDR = pinsGpios[pin].mask;
    
    // Enable filter(s)
    pinsGpios[pin].pio->PIO_IFER = pinsGpios[pin].mask;  //enable
    //pinsGpios[pin].pio->PIO_IFDR = pinsGpios[pin].mask;
    
    pinsGpios[pin].pio->PIO_PER = pinsGpios[pin].mask;  
    
}






//// additional time delay :  +10us
//// so, the critical time delay is 11us
void  __ramfunc GPIOPIN_Set_Session( unsigned int pin , unsigned int dat )
{    
    unsigned int i; 
    
    for( i=0; i < 28; i++ ) {  //here 28 is used instead of numGpios for speed up !
      
        if( pin & 0x01<<i ) { 
                            
            if( dat  & 0x01<<i ) {
                //PIO_Set( &pinsGpios[i]);
                pinsGpios[i].pio->PIO_SODR = pinsGpios[i].mask;
            } else {
                //PIO_Clear( &pinsGpios[i]);
                pinsGpios[i].pio->PIO_CODR = pinsGpios[i].mask;              
            }              
        }         
     }     
}



