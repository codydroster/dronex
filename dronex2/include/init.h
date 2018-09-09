#include "stm32l496xx.h"













void system_init(void);







//initialize port A
GPIO_TypeDef *pGPIOA;

//port B
GPIO_TypeDef *pGPIOB;

//port C
GPIO_TypeDef *pGPIOC;

//port D
GPIO_TypeDef *pGPIOD;

//initialize clock
RCC_TypeDef *pRCC;

//UART init - FC
USART_TypeDef *pUART2;

//UART2 init - Xbee
USART_TypeDef *pUART5;

//UART3 init - Lidar
USART_TypeDef *pUARTLID;

//TIM2 typedef
TIM_TypeDef *pTIM2;

//SPI typedef
SPI_TypeDef *pSPI2;



//DMA
DMA_TypeDef *pDMA1;
DMA_TypeDef *pDMA2;


//DMA1
DMA_Channel_TypeDef *pDMA1C3;

DMA_Channel_TypeDef *pDMASPIRX;
DMA_Channel_TypeDef *pDMASPITX;

DMA_Request_TypeDef *pDMA1SEL;




//DMA2
DMA_Channel_TypeDef *pDMA2C2;
DMA_Channel_TypeDef *pDMA2C1;

DMA_Request_TypeDef *pDMA2SEL;
