//==================================================================================================
//
// 	File Name		DS1307_RTC.h
//
//	CPU Type		STM32F103C8T6
//	Builder			
//						
//	Coding			
//
//	Outline			-
//
//	History			
//==================================================================================================
//	Local Compile Option
//==================================================================================================
#ifndef INC_DRS1307_RTC_H_
#define INC_DRS1307_RTC_H_

//==================================================================================================
//	Header File
//==================================================================================================
#include "stm32f103c8t6.h"
#include "Drv_I2C.h"
//==================================================================================================
//	Local define
//==================================================================================================
// Define DS1307 I2C address
#define U1_DS1307_I2C_ADDRESS	((uint8_t)0x68U)	// DS1307 I2C address (7-bit address)
// Define day of week
#define U1_DS1307_MONDAY		((uint8_t)0x00U)	// Monday
#define U1_DS1307_TUEDAY		((uint8_t)0x01U)	// Tuesday
#define U1_DS1307_WEDDAY		((uint8_t)0x02U)	// Wednesday
#define U1_DS1307_THUDAY		((uint8_t)0x03U)	// Thursday
#define U1_DS1307_FRIDAY		((uint8_t)0x04U)	// Friday
#define U1_DS1307_SATDAY		((uint8_t)0x05U)	// Saturday
#define U1_DS1307_SUNDAY		((uint8_t)0x06U)	// Sunday

// Define hour mode
#define U1_DS1307_HOUR_24		((uint8_t)0x00U)	// 24-hour mode
#define U1_DS1307_HOUR_12		((uint8_t)0x40U)	// 12-hour mode
// Define AM/PM mode
#define U1_DS1307_HOUR_AM		((uint8_t)0x00U)	// AM mode
#define U1_DS1307_HOUR_PM		((uint8_t)0x20U)	// PM mode
// Define RTC address
#define U1_DS1307_SECOND_ADD	((uint8_t)0x00U)	// Time register address
#define U1_DS1307_MINUTE_ADD	((uint8_t)0x01U)	// Time register address
#define U1_DS1307_HOUR_ADD		((uint8_t)0x02U)	// Time register address
#define U1_DS1307_DAY_ADD		((uint8_t)0x03U)	// Day register address
#define U1_DS1307_DATE_ADD		((uint8_t)0x04U)	// Date register address
#define U1_DS1307_MONTH_ADD		((uint8_t)0x05U)	// Month register address
#define U1_DS1307_YEAR_ADD		((uint8_t)0x06U)	// Year register address
#define U1_DS1307_CONTROL_ADD	((uint8_t)0x07U)	// Control register address
//==================================================================================================
//	Struct Template
//==================================================================================================
// Structure for DS1307 RTC time
typedef struct
{
	uint8_t Second;		// Seconds (0-59)
	uint8_t Minute;		// Minutes (0-59)
	uint8_t Hour;		// Hours (0-23)
} ST_RTC_TIME;
// Structure for DS1307 RTC day
typedef struct
{
	uint8_t Day;		// Day of the week (1-7, 1 = Sunday)
	uint8_t Date;		// Date of the month (1-31)
	uint8_t Month;		// Month (1-12)
	uint8_t Year;		// Year (0-99)
} ST_RTC_DATE;
// Variable for DS1307 RTC time and date
typedef struct
{
	ST_RTC_TIME Time;	// Time structure
	ST_RTC_DATE Date;	// Date structure
} ST_RTC_DATA;
// Structure for DS1307 RTC configuration
typedef struct
{
	uint8_t HourMode;	// Hour mode (24-hour or 12-hour)
	uint8_t AM_PM;		// AM/PM mode (for 12-hour mode only)
} ST_RTC_CONFIG;
// Structure for DS1307 RTC handle
typedef struct
{
	I2C_Handle_t *pI2CHandle;	// Pointer to I2C handle
	ST_RTC_CONFIG Config;		// RTC configuration
	ST_RTC_DATA Data;			// RTC data (time and date)
} ST_RTC_Handle_t;

//==================================================================================================
//	Function prototype declaration
//==================================================================================================
void DS1307_Init(void);
// Funtion for setting and get the time and date
void DS1307_SetTime(ST_RTC_TIME *pTime);
void DS1307_GetTime(ST_RTC_TIME *pTime);
void DS1307_SetDate(ST_RTC_DATE *pDate);
void DS1307_GetDate(ST_RTC_DATE *pDate);
void DS1307_SetHourMode(uint8_t u1_HourMode);
void DS1307_SetAmPmMode(uint8_t u1_AM_PM);
#endif /* INC_DRS1307_RTC_H_ */