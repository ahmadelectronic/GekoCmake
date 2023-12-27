/**
 * \file htu21d.h
 *
 * \brief HTU21 Temperature & Humidity sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

typedef bool (*I2cWriteBuffer_t)( uint8_t deviceAddr, uint8_t *buffer, uint16_t size);
typedef bool (*I2cReadBuffer_t)( uint8_t deviceAddr, uint8_t *buffer, uint16_t size);
typedef void (*DelayMs_t)(uint32_t ms);

I2cWriteBuffer_t SensorI2cWriteBuffer;
I2cReadBuffer_t SensorI2cReadBuffer;
DelayMs_t SensorDelayMs;

// Default I2C address for the HTU21D
#define HTU21_ADDR											(uint8_t)0x80 //0b1000000

// HTU21 device commands
#define HTU21_RESET_COMMAND									0xFE
#define HTU21_READ_TEMPERATURE_W_HOLD_COMMAND				0xE3
#define HTU21_READ_TEMPERATURE_WO_HOLD_COMMAND				0xF3
#define HTU21_READ_HUMIDITY_W_HOLD_COMMAND					0xE5
#define HTU21_READ_HUMIDITY_WO_HOLD_COMMAND					0xF5
#define HTU21_READ_SERIAL_FIRST_8BYTES_COMMAND				0xFA0F
#define HTU21_READ_SERIAL_LAST_6BYTES_COMMAND				0xFCC9
#define HTU21_WRITE_USER_REG_COMMAND						0xE6
#define HTU21_READ_USER_REG_COMMAND							0xE7

#define RESET_TIME											15			// ms value

// Processing constants
#define HTU21_TEMPERATURE_COEFFICIENT						(float)(-0.15)
#define HTU21_CONSTANT_A									(float)(8.1332)
#define HTU21_CONSTANT_B									(float)(1762.39)
#define HTU21_CONSTANT_C									(float)(235.66)

// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL								(175.72)
#define TEMPERATURE_COEFF_ADD								(-46.85)

// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL									(125)
#define HUMIDITY_COEFF_ADD									(-6)

// Conversion timings
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_14b_RH_12b		50000
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_13b_RH_10b		25000
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_12b_RH_8b		13000
#define HTU21_TEMPERATURE_CONVERSION_TIME_T_11b_RH_11b		7000
#define HTU21_HUMIDITY_CONVERSION_TIME_T_14b_RH_12b			16000
#define HTU21_HUMIDITY_CONVERSION_TIME_T_13b_RH_10b			5000
#define HTU21_HUMIDITY_CONVERSION_TIME_T_12b_RH_8b			3000
#define HTU21_HUMIDITY_CONVERSION_TIME_T_11b_RH_11b			8000

// HTU21 User Register masks and bit position
#define HTU21_USER_REG_RESOLUTION_MASK						0x81
#define HTU21_USER_REG_END_OF_BATTERY_MASK					0x40
#define HTU21_USER_REG_ENABLE_ONCHIP_HEATER_MASK			0x4
#define HTU21_USER_REG_DISABLE_OTP_RELOAD_MASK				0x2

// HTU User Register values
// Resolution
#define HTU21_USER_REG_RESOLUTION_T_14b_RH_12b				0x00
#define HTU21_USER_REG_RESOLUTION_T_13b_RH_10b				0x80
#define HTU21_USER_REG_RESOLUTION_T_12b_RH_8b				0x01
#define HTU21_USER_REG_RESOLUTION_T_11b_RH_11b				0x81

// End of battery status
#define HTU21_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V		0x00
#define HTU21_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V		0x40
// Enable on chip heater
#define HTU21_USER_REG_ONCHIP_HEATER_ENABLE					0x04
#define HTU21_USER_REG_OTP_RELOAD_DISABLE					0x02

// Functions

/**
 * \brief Configures the I2C master to be used with the HTU21 device.
 */
void Htu21Init(I2cWriteBuffer_t I2cWriteBufferPtr,I2cReadBuffer_t I2cReadBufferPtr,DelayMs_t SensorDelayMsPtr);

/**
 * \brief Reset the HTU21 device
 *
 * \return void
 */
void  Htu21Reset(void);

/**
 * \brief Writes the HTU21 8-bits command with the value passed
 *
 * \param[in] void
 *
 * \return float : Humidity
 */
float Htu21ReadHumidity(void);

/**
 * \brief Writes the HTU21 8-bits command with the value passed
 *        Do not send the STOP bit in the I2C transfer
 *
 * \param[in] void
 *
 * \return float : Temperature
 */
float Htu21ReadTemperature(void);