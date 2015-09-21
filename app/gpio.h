#ifndef __GPIO_H__
#define __GPIO_H__


typedef struct __PORTS
{                          
    unsigned char porta ;
    unsigned char portc ;
    unsigned char portg ;
    unsigned char portj ;
}PORTS ;

typedef union __GPIOPINSTATE
{
    PORTS   portStt ;     // pin
    unsigned int pinStt ;
}GPIOPIN ;
typedef union __GPIODIRSTATE
{
    PORTS  portDir ;     // dir
    unsigned int pinDir ;
}GPIODIR ;
typedef union __GPIOPORT
{                           
    PORTS  portDat ;     // port
    unsigned int pinDat ; 
}GPIOPORT ;


extern void GPIO_Init(void);
extern void GPIODIR_FLOAT( unsigned int pin  ) ;

extern void GPIODIR_Set(unsigned int pin ) ;
extern unsigned char GPIOPIN_Set(unsigned int pin , unsigned int dat) ;
extern unsigned char  GPIOPIN_Get(unsigned int pin , unsigned char *pdat);
extern void GPIOPIN_Set_Session(unsigned int pin , unsigned int dat) ;

extern void GPIOPIN_Init_Fast( unsigned int pin );
extern void GPIOPIN_Get_Fast( unsigned char pin, unsigned char * pdata );
extern void GPIOPIN_Set_Fast( unsigned char pin , unsigned char data );
extern void GPIOPort_Get_Fast( unsigned char * pdata );

//extern void Config_GPIO_Interrupt( unsigned char gpio_index, CPU_FNCT_VOID isr_handler );
extern void Config_GPIO_Interrupt( unsigned char gpio_index, void (*isr_handler)(const Pin *) );
extern void Disable_GPIO_Interrupt( unsigned char gpio_index );
extern unsigned char Check_GPIO_Intrrupt( unsigned char gpio_index );
    
#endif





