//==================================================================================================
//
// 	File Name		DS1307_RTC_LCD.c
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
#include "Drv_I2C.h"
#include "DS1307_RTC.h"
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
I2C_Handle_t I2C1Handle;
GPIO_Handle_t I2CPins;
ST_RTC_Handle_t ds1307_Rtc;
ST_RTC_TIME ds1307_Time;
ST_RTC_DATE ds1307_Date;
//==================================================================================================
//	Local ROM
//==================================================================================================

//==================================================================================================
//	Local Function Prototype
//==================================================================================================

//==================================================================================================
//	Source Code
//==================================================================================================
void delay()
{
	for (uint32_t i = 0; i < 1000000; i++)
	{
		/* code */
	}
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pi2cHandle, uint8_t Event)
{

}

void Ds1307_Rtc_Config(void)
{
	ds1307_Rtc.pI2CHandle = &I2C1Handle;
	ds1307_Rtc.Config.HourMode = U1_DS1307_HOUR_24;
	ds1307_Rtc.Config.AM_PM = U1_DS1307_HOUR_AM;
	DS1307_Init();
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			I2C1_EV_IRQHandler
//	Name:			-
//	Function:		Handles the I2C1 event interrupt
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C1_EV_IRQHandler(void)
{
	// Call the event interrupt handling function for I2C1
	I2C_EV_IRQHandling(&I2C1Handle);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			I2C1_ER_IRQHandler
//	Name:			-
//	Function:		Handles the I2C1 error interrupt
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C1_ER_IRQHandler(void)
{
	// Call the error interrupt handling function for I2C1
	I2C_ER_IRQHandling(&I2C1Handle);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			I2C_Config
//	Name:			-
//	Function:		I2C peripheral initialization
//
//	Argument:		pI2Cx - Pointer to I2C peripheral
//					EnOrDi - Enable or Disable (1 or 0)
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void I2C_Config(void)
{
	// Confige I2C1
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.ClockSpeed = I2C_SCL_SPEED_SM;
	//I2C1Handle.I2C_Config.DutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.Address = 0x68;
	I2C_Init(&I2C1Handle);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			I2Cpin_Config
//	Name:			-
//	Function:		I2C peripheral pin initialization
//
//	Argument:		-
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void I2Cpin_Config(void)
{
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PULL_UP;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// I2C1_SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);
	// I2C1_SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
	// Config Interrupt for I2C
	I2C_IRQInterruptConfig(NVIC_IRQ_POS_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(NVIC_IRQ_POS_I2C1_ER, ENABLE);
	I2C_IRQPriorityConfig(NVIC_IRQ_POS_I2C1_EV, 3);
	I2C_IRQPriorityConfig(NVIC_IRQ_POS_I2C1_EV, 4);
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
	I2Cpin_Config();
	I2C_Config();
	Ds1307_Rtc_Config();
	// Set time to 12:30:00
	ds1307_Rtc.Data.Time.Hour = 12;
	ds1307_Rtc.Data.Time.Minute = 30;
	ds1307_Rtc.Data.Time.Second = 0;
	DS1307_SetTime(&ds1307_Rtc.Data.Time); // Set time to 12:30:00
	// Set date to 06/04/2025
	ds1307_Rtc.Data.Date.Day = U1_DS1307_SUNDAY;
	ds1307_Rtc.Data.Date.Date = 6;
	ds1307_Rtc.Data.Date.Month = 4;
	ds1307_Rtc.Data.Date.Year = 23;
	DS1307_SetDate(&ds1307_Rtc.Data.Date); // Set date to 06/04/2025
	while (1)
	{
		delay();		
		// Get time
		DS1307_GetTime(&ds1307_Time); // Get time from DS1307
		// Get date
		DS1307_GetDate(&ds1307_Date); // Get date from DS1307
		delay();		
	}
}
