//==================================================================================================
//
// 	File Name		Drv_I2C.c
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

//==================================================================================================
//	Local define
//==================================================================================================

#define SSD1306_I2C_ADDR  (0x3C) 
#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT    32

#define SLAVE_ADDR			(0x33U)
#define CMD_READ_LEN	(0x55)
#define CMD_GET_DATA	(0x44)

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
//==================================================================================================
//	Local ROM
//==================================================================================================
const char TxBuff[] = "Hello world";
uint8_t Rxlen;
 uint8_t RxBuff[20]; 
 uint8_t len;
volatile uint8_t flag;
uint8_t cmd_readlen = 0x55;
uint8_t cmd_getdata = 0x44;
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
void SSD1306_WriteCommand(uint8_t command) {
	uint8_t data[2] = {0x00, command};  // 0x00 ?? ch? ??nh byte l?nh
	I2C_MasterSendData(&I2C1Handle, data, 2, SSD1306_I2C_ADDR, ENABLE);
  }
  
  void SSD1306_Init() {
//	HAL_Delay(100);  // ??i m?n h?nh ?n ??nh
  
	// G?i chu?i l?nh kh?i t?o
	  SSD1306_WriteCommand(0xAE);  // Display OFF
	  SSD1306_WriteCommand(0x8D);  // Charge pump enable
	  SSD1306_WriteCommand(0x14);
	  SSD1306_WriteCommand(0xAF);  // Display ON
	  SSD1306_WriteCommand(0xA6);  // Normal display mode
	  SSD1306_WriteCommand(0x20);  // Addressing mode
	  SSD1306_WriteCommand(0x00);  // Horizontal addressing mode
	  SSD1306_WriteCommand(0xC0);  // No remapping
	  SSD1306_WriteCommand(0xD3);  // Display offset
	  SSD1306_WriteCommand(0x00);
	  SSD1306_WriteCommand(0x21);  // Column range
	  SSD1306_WriteCommand(0x00);
	  SSD1306_WriteCommand(0x7F);  // 127
	  SSD1306_WriteCommand(0x22);  // Page range
	  SSD1306_WriteCommand(0x00);
	  SSD1306_WriteCommand(0x03);
	  SSD1306_WriteCommand(0xA8);  // Multiplex ratio
	  SSD1306_WriteCommand(0x1F);  // 31
	  SSD1306_WriteCommand(0xDA);  // COM pins config
	  SSD1306_WriteCommand(0x02);
  }
  void SSD1306_Clear(void) {
	uint8_t buf[1025]; // 1 byte command + 1024 bytes (128x32 pixels / 8 pixels per byte)
	buf[0] = 0x40; // Data mode
	
	// Ghi t?t c? c?c byte th?nh 0 ?? x?a m?n h?nh
	memset(&buf[1], 0x00, 1024);
  
	// Ch?n v?ng hi?n th? to?n m?n h?nh
	SSD1306_WriteCommand(0x21); // Column range
	SSD1306_WriteCommand(0x00);
	SSD1306_WriteCommand(127);
	
	SSD1306_WriteCommand(0x22); // Page range
	SSD1306_WriteCommand(0x00);
	SSD1306_WriteCommand(0x03);
	
	// G?i d? li?u ?? x?a m?n h?nh
	for (uint8_t i = 0; i < 8; i++) {
		I2C_MasterSendData(&I2C1Handle, buf, 1025, SSD1306_I2C_ADDR, ENABLE);
	}
  }
  
  void SSD1306_SetPixel(uint8_t x, uint8_t y) {
	uint8_t page = y / 8;
	uint8_t bit_position = y % 8;
	
	// ??t v? tr? hi?n th?
	SSD1306_WriteCommand(0xB0 + page);        // Ch?n trang
	SSD1306_WriteCommand(0x00 + (x & 0x0F));  // Lower column address
	SSD1306_WriteCommand(0x10 + (x >> 4));    // Higher column address
  
	// G?i d? li?u hi?n th? ?i?m ?nh (bit mask)
	uint8_t data[2] = {0x40, (1 << bit_position)};
	I2C_MasterSendData(&I2C1Handle, data, 2, SSD1306_I2C_ADDR, ENABLE);
  }

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pi2cHandle, uint8_t Event)
{

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
	while (1)
	{
		I2C_MasterSendDataIT(&I2C1Handle, &cmd_readlen, 1, SLAVE_ADDR, ENABLE);
		while (I2C1Handle.State != I2C_STATE_READY){}
		
		I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, ENABLE);
		while (I2C1Handle.State != I2C_STATE_READY){}
		
		I2C_MasterSendDataIT(&I2C1Handle, &cmd_getdata, 1, SLAVE_ADDR, ENABLE);
		while (I2C1Handle.State != I2C_STATE_READY){}
		
		I2C_MasterReceiveDataIT(&I2C1Handle, RxBuff, len, SLAVE_ADDR, ENABLE);
		while (I2C1Handle.State != I2C_STATE_READY){}
		
		delay();		
	}
}
