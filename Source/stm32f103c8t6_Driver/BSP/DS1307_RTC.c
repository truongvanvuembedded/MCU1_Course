//==================================================================================================
//
// 	File Name		DS1307_RTC.c
//
//	CPU Type		STM32F103C8T6
//	Builder			STM32CUBE_IDE
//					
//	Coding			V.VU    ASTI DANANG
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
#include <stdio.h>
#include "DS1307_RTC.h"
//==================================================================================================
//	Local define
//==================================================================================================
extern ST_RTC_Handle_t ds1307_Rtc;
ST_RTC_Handle_t *pDs1307_Rtc = &ds1307_Rtc;
#define DS1307_RTC_HANDLE	pDs1307_Rtc
#define DS1307_SEND_DATA(pu1_TxData, u1_Len, u1_StopF)		I2C_MasterSendData(DS1307_RTC_HANDLE->pI2CHandle, pu1_TxData, u1_Len, U1_DS1307_I2C_ADDRESS, u1_StopF)
#define DS1307_RECEIVE_DATA(pu1_RxData, u1_Len, u1_StopF)	I2C_MasterReceiveData(DS1307_RTC_HANDLE->pI2CHandle, pu1_RxData, u1_Len, U1_DS1307_I2C_ADDRESS, u1_StopF)
//==================================================================================================
//	Local define I/O
//==================================================================================================

//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM 
//==================================================================================================
static uint8_t u1_Data[2];	// Data buffer for I2C communication
//==================================================================================================
//	Local ROM
//==================================================================================================

//==================================================================================================
//	Local Function Prototype
//==================================================================================================
static uint8_t u1_RTC_Read_Register(uint8_t u1_Register);
static void RTC_Write_Register(uint8_t u1_Register, uint8_t u1_TxData);
static uint8_t u1_DS1307_FormatBeForeSend(uint8_t u1_Data);
static uint8_t u1_DS1307_DecodeFormat(uint8_t u1_Data);
//==================================================================================================
//	Source Code
//==================================================================================================
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			DS1307_Init
//	Name:			DS1307_Init
//	Function:		Init DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DS1307_Init(void)
{
	uint8_t u1_Temp;
	// Init variables
	DS1307_RTC_HANDLE->Data.Time.Second = 0;
	DS1307_RTC_HANDLE->Data.Time.Minute = 0;
	DS1307_RTC_HANDLE->Data.Time.Hour = 0;
	DS1307_RTC_HANDLE->Data.Date.Day = 0;
	DS1307_RTC_HANDLE->Data.Date.Date = 0;
	DS1307_RTC_HANDLE->Data.Date.Month = 0;
	DS1307_RTC_HANDLE->Data.Date.Year = 0;
	// Init RTC configuration
	// 1. Set oscillator enable
	u1_Temp = u1_RTC_Read_Register(U1_DS1307_SECOND_ADD);	// Read current value of the second register
	if (u1_Temp & 0x80U)	// Check if oscillator is enabled or disabled
	{
		RTC_Write_Register(U1_DS1307_SECOND_ADD, u1_Temp & 0x7FU);	// Enable oscillator
	}
	// 2. Set hour mode (24-hour or 12-hour) and AM/PM mode
	DS1307_SetHourMode(DS1307_RTC_HANDLE->Config.HourMode);	// Set hour mode (24-hour or 12-hour)
	DS1307_SetAmPmMode(DS1307_RTC_HANDLE->Config.AM_PM);	// Set AmPm mode 
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			DS1307_SetTime
//	Name:			DS1307_SetTime
//	Function:		Set time for DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DS1307_SetTime(ST_RTC_TIME *pTime)
{
	uint8_t au1_Data;
	au1_Data = u1_DS1307_FormatBeForeSend(pTime->Second);	// Format seconds before sending
	RTC_Write_Register(U1_DS1307_SECOND_ADD, au1_Data);	// Set seconds
	au1_Data = u1_DS1307_FormatBeForeSend(pTime->Minute);	// Format minutes before sending
	RTC_Write_Register(U1_DS1307_MINUTE_ADD, au1_Data);	// Set minutes
	au1_Data = DS1307_RTC_HANDLE->Config.HourMode | DS1307_RTC_HANDLE->Config.AM_PM;	// Format hours before sending
	au1_Data |= u1_DS1307_FormatBeForeSend(pTime->Hour);	// Format hours before sending
	RTC_Write_Register(U1_DS1307_HOUR_ADD, au1_Data);	// Set hours
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			DS1307_GetTime
//	Name:			DS1307_GetTime
//	Function:		Get time from DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DS1307_GetTime(ST_RTC_TIME *pTime)
{
	uint8_t au1_Tmp;
	au1_Tmp = u1_RTC_Read_Register(U1_DS1307_SECOND_ADD);	// Get seconds
	pTime->Second = u1_DS1307_DecodeFormat(au1_Tmp);	// Decode seconds
	au1_Tmp = u1_RTC_Read_Register(U1_DS1307_MINUTE_ADD);	// Get minutes
	pTime->Minute = u1_DS1307_DecodeFormat(au1_Tmp);	// Decode minutes
	au1_Tmp = u1_RTC_Read_Register(U1_DS1307_HOUR_ADD);	// Get hours
	if (au1_Tmp & 0x40U)
	{
		pTime->Hour = u1_DS1307_DecodeFormat(au1_Tmp & 0x0FU);	// Get hours in 12-hour mode
	}
	else
	{
		pTime->Hour = u1_DS1307_DecodeFormat(au1_Tmp & 0x3FU);	// Get hours in 24-hour mode
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			DS1307_SetDate
//	Name:			DS1307_SetDate
//	Function:		Set date for DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DS1307_SetDate(ST_RTC_DATE *pDate)
{
	uint8_t au1_Data;
	// Set day
	RTC_Write_Register(U1_DS1307_DAY_ADD, pDate->Day);	// Set day
	// Set date
	au1_Data = u1_DS1307_FormatBeForeSend(pDate->Date);	// Format date before sending
	RTC_Write_Register(U1_DS1307_DATE_ADD, au1_Data);	// Set date
	// Set month
	au1_Data = u1_DS1307_FormatBeForeSend(pDate->Month);	// Format date before sending
	RTC_Write_Register(U1_DS1307_MONTH_ADD, au1_Data);	// Set month
	// Set year
	au1_Data = u1_DS1307_FormatBeForeSend(pDate->Year);	// Format date before sending
	RTC_Write_Register(U1_DS1307_YEAR_ADD, au1_Data);	// Set year
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			DS1307_GetDate
//	Name:			DS1307_GetDate
//	Function:		Get date from DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DS1307_GetDate(ST_RTC_DATE *pDate)
{
	uint8_t au1_Tmp;
	au1_Tmp = u1_RTC_Read_Register(U1_DS1307_DAY_ADD);	// Get day
	pDate->Day = u1_DS1307_DecodeFormat(au1_Tmp);		// Decode day
	au1_Tmp = u1_RTC_Read_Register(U1_DS1307_DATE_ADD);	// Get date
	pDate->Date = u1_DS1307_DecodeFormat(au1_Tmp);		// Decode date
	au1_Tmp = u1_RTC_Read_Register(U1_DS1307_MONTH_ADD);	// Get month
	pDate->Month = u1_DS1307_DecodeFormat(au1_Tmp);		// Decode month
	au1_Tmp = u1_RTC_Read_Register(U1_DS1307_YEAR_ADD);	// Get year
	pDate->Year= u1_DS1307_DecodeFormat(au1_Tmp);		// Decode year
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			DS1307_SetHourMode
//	Name:			DS1307_SetHourMode
//	Function:		Set hour mode for RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DS1307_SetHourMode(uint8_t u1_HourMode)
{
	uint8_t u1_Temp;
	u1_Temp = u1_RTC_Read_Register(U1_DS1307_HOUR_ADD);	// Read current value of the second register
	u1_Temp &= 0x3FU;	// Clear hour mode bit
	u1_Temp |= u1_HourMode;	// Set hour mode (24-hour or 12-hour)
	RTC_Write_Register(U1_DS1307_HOUR_ADD, u1_Temp);	// Send data to RTC
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			DS1307_SetAmPmMode
//	Name:			DS1307_SetAmPmMode
//	Function:		Set AM/PM mode
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DS1307_SetAmPmMode(uint8_t u1_AmPmMode)
{
	uint8_t u1_Temp;
	u1_Temp = u1_RTC_Read_Register(U1_DS1307_HOUR_ADD);	// Read current value of the second register
	u1_Temp &= 0x5FU;	// Clear hour mode bit
	u1_Temp |= u1_AmPmMode;	// Set hour mode (24-hour or 12-hour)
	RTC_Write_Register(U1_DS1307_HOUR_ADD, u1_Temp);	// Send data to RTC
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			RTC_Read_Register
//	Name:			RTC_Read_Register
//	Function:		Read register from DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t u1_RTC_Read_Register(uint8_t u1_Register)
{
	u1_Data[0] = u1_Register;	// Register address
	u1_Data[1] = (uint8_t)0;	// Register address
	DS1307_SEND_DATA(&u1_Data[0], 1, 0);	// Send register address to DS1307
	DS1307_RECEIVE_DATA(&u1_Data[1], 1, 1);	// Send register address to DS1307
	return u1_Data[1];	// Return register data
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			RTC_Write_Register
//	Name:			RTC_Write_Register
//	Function:		Write register to DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void RTC_Write_Register(uint8_t u1_Register, uint8_t u1_TxData)
{
	u1_Data[0] = u1_Register;	// Register address
	u1_Data[1] = u1_TxData;		// Data to write
	DS1307_SEND_DATA(u1_Data, 2, 1);	// Send register address to DS1307
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			u1_DS1307_FormatBeForeSend
//	Name:			u1_DS1307_FormatBeForeSend
//	Function:		Format data before sending to DS1307 RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t u1_DS1307_FormatBeForeSend(uint8_t u1_Data)
{
	uint8_t u1_tmp1;
	uint8_t u1_tmp2;
	uint8_t u1_Ret;
	
    if (u1_Data > 10)
    {
        u1_tmp1 = (int)(u1_Data / 10);
    }
    else
    {
        u1_tmp1 = 0;
    }
    u1_tmp2 = u1_Data - (u1_tmp1 * 10);

    u1_Ret = (u1_tmp1 << 4) | u1_tmp2;

    return u1_Ret;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Tag:			u1_DS1307_DecodeFormat
//	Name:			u1_DS1307_DecodeFormat
//	Function:		Decode format receive from RTC
//
//	Argument:		
//
//	Return value:	-
//	Create:			2025.02.18 V.Vu  New
//	Change:			-
//	Remarks:		-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t u1_DS1307_DecodeFormat(uint8_t u1_Data)
{
	uint8_t u1_tmp1;
	uint8_t u1_tmp2;
	uint8_t u1_Ret;
	
    u1_tmp1 = u1_Data & 0x0F;	// Get lower 4 bits
	u1_tmp2 = (u1_Data >> 4) & 0x0F;	// Get upper 4 bits
	u1_Ret = (u1_tmp2 * 10) + u1_tmp1;	// Combine upper and lower 4 bits to get the original value

    return u1_Ret;
}