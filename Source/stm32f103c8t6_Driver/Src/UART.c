//==================================================================================================
//
// 	File Name		Drv_UART.c
//
//	CPU Type		STM32F103C8T6
//	Builder			STM32CUBE_IDE
//					
//	Coding			V.VU	ASTI DANANG
//
//	Outline			
//					
//
//	History			Ver.0.01	2025.02.18 V.Vu	New
//					
//
//==================================================================================================
//==================================================================================================
//	Compile Option
//==================================================================================================

//==================================================================================================
//	#pragma section
//==================================================================================================

//==================================================================================================
//	Local Compile Option
//==================================================================================================

//==================================================================================================
//	Header File
//==================================================================================================
#include <stdint.h>
#include <string.h>
#include "Drv_GPIO.h"
#include "Drv_UART.h"

//==================================================================================================
//	Local define
//==================================================================================================

//==================================================================================================
//	Local define I/O
//==================================================================================================

//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM 
//==================================================================================================
char str[] = "Hello World\r\n";
uint8_t Rxbuf[13];

UART_Handle_t UART1Handle;
GPIO_Handle_t UARTPins;
//==================================================================================================
//	Local ROM
//==================================================================================================

//==================================================================================================
//	Local Function Prototype
//==================================================================================================
static void UART_Config(void);
static void UARTpin_Config(void);
//==================================================================================================
//	Source Code
//==================================================================================================
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			UART1_IRQHandler
//	Name:			-
//	Function:		UART peripheral interrupt handler
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void USART1_IRQHandler(void)
{
	// Call the UART IRQ handling function
	UART_IRQHandling(&UART1Handle);
}
__attribute__((weak)) void UART_ApplicationEventCallback(UART_Handle_t *pUARTHandle)
{
	if (strcmp((char *)Rxbuf, str) == 0)
	{
		// Clear the RX buffer
		memset(Rxbuf, 0, sizeof(Rxbuf));
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13); // Toggle the LED pin
		UART_ReceiveDataIT(&UART1Handle, Rxbuf, 13);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			UART_Config
//	Name:			-
//	Function:		UART peripheral initialization
//
//	Argument:		pUARTx - Pointer to UART peripheral
//					EnOrDi - Enable or Disable (1 or 0)
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void UART_Config(void)
{
	// Confige UART1
	UART1Handle.pUARTx = USART1;
	UART1Handle.UART_Config.BaudRate = UART_BAUDRATE_9600;
	UART1Handle.UART_Config.DataFormat = UART_DATA_LEN_8BIT;
	UART1Handle.UART_Config.ParityControl = UART_PARITY_DISABLE;
	UART1Handle.UART_Config.StopbitLen = UART_STOP_BIT_1;
	UART1Handle.UART_Config.UARTMode = UART_MODE_TXRX;
	UART1Handle.UART_Config.HardwareFlowCtrl = UART_HARDWARE_FLOW_CTRL_DISABLE;
	UART_Init(&UART1Handle);

	// Configure the IRQ priority 
	UART_IRQInterruptConfig(UART_IRQ_INDEX_USART1, ENABLE);
	// Confige the interrupt for UART peripheral at the NVIC
	UART_IRQPriorityConfig(UART_IRQ_INDEX_USART1, 2);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			UARTpin_Config
//	Name:			-
//	Function:		UART peripheral pin initialization
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void UARTpin_Config(void)
{
	// UART remap to config alternative funtion
	AFIO->MAPR &= ~(1U << 2);
	// Congif UART1 with Tx: A9 and Rx: A10
	UARTPins.pGPIOx = GPIOA;
	UARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
	UARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO_PULL;
	UARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// UART1_TX
	UARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&UARTPins);

	// UART1_RX
	UARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Init(&UARTPins);

	// Confige Led pin 
	UARTPins.pGPIOx = GPIOC;
	UARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	UARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	UARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO_PULL;
	UARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&UARTPins);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			main
//	Name:			-
//	Function:		Main function
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	UARTpin_Config();
	UART_Config();
	UART_ReceiveDataIT(&UART1Handle, Rxbuf, 13);
	while (1)
	{
		for (uint32_t timerCnt = 0; timerCnt < 1000000; timerCnt++)
		{
			
		}
		//UART_SendData(&UART1Handle, (uint8_t *)str, strlen(str));
	}
}
