//==================================================================================================
//
// 	File Name		Drv_SPI.c
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
#include "Drv_SPI.h"

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
char str[] = "Hello";
uint8_t Rxbuf[sizeof(str)];
uint32_t Reg;
volatile uint32_t timerCnt = 0;

SPI_Handle_t SPI1Handle;
SPI_Handle_t SPI2Handle;
GPIO_Handle_t SPIPins;
//==================================================================================================
//	Local ROM
//==================================================================================================

//==================================================================================================
//	Local Function Prototype
//==================================================================================================
static void SPI_Config(void);
static void SPIpin_Config(void);
//==================================================================================================
//	Source Code
//==================================================================================================
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			SPI1_IRQHandler
//	Name:			-
//	Function:		SPI peripheral interrupt handler
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&SPI1Handle);
}
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle)
{
	if (strcmp(str, (char*)Rxbuf) == 0)
	{
		GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_2);
		memset(Rxbuf, 0, strlen(str));
		SPI_ReceiveDataIT(&SPI1Handle, Rxbuf, strlen(str));
	}
	
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			SPI_Config
//	Name:			-
//	Function:		SPI peripheral initialization
//
//	Argument:		pSPIx - Pointer to SPI peripheral
//					EnOrDi - Enable or Disable (1 or 0)
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void SPI_Config(void)
{
	// Confige SPI1
	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_HARDWARE;
	SPI1Handle.SPIConfig.SPI_FirstBit = SPI_FIRSTBIT_MSB;
	SPI_Init(&SPI1Handle);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			SPIpin_Config
//	Name:			-
//	Function:		SPI peripheral pin initialization
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void SPIpin_Config(void)
{
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO_PULL;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// SPI1_NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
	GPIO_Init(&SPIPins);

	// SPI1_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_Init(&SPIPins);

	// SPI1_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&SPIPins);

	// SPI1_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT_PULL;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&SPIPins);

	// Configure the IRQ priority
	SPI_IRQPriorityConfig(IRQ_NO_SPI1, 2);
	// unmask the interrupt for SPI peripheral at the NVIC
	SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);

	// Confige Led pin 
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NO_PULL;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&SPIPins);


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
	SPI_Config();
	SPIpin_Config();
	//SPI_ReceiveDataIT(&SPI1Handle, Rxbuf, strlen(str));
	while (1)
	{
		SPI_SendData(&SPI1Handle, (uint8_t *)str, strlen(str));
		for (uint32_t timerCnt = 0; timerCnt < 1000000; timerCnt++)
		{
			
		}
		
	}
}
