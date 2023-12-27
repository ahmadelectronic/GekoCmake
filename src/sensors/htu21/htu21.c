/**
 * \file htu21.c
 *
 * \brief HTU21 Temperature & Humidity sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to htu21 datasheet :
 * http://www.meas-spec.com/downloads/HTU21D.pdf
 *
 */
#include "htu21.h"

/**
 * \brief Configures the I2C master to be used with the htu21 device.
 *
 * \return void
 */
void Htu21Init(I2cWriteBuffer_t I2cWriteBufferPtr,I2cReadBuffer_t I2cReadBufferPtr,DelayMs_t SensorDelayMsPtr )
{
	SensorI2cWriteBuffer = I2cWriteBufferPtr;
	SensorI2cReadBuffer = I2cReadBufferPtr;
	SensorDelayMs = SensorDelayMsPtr;
	
	Htu21Reset();
}

/**
 * \brief Sends a 'reset' request to the HTU21DF, followed by a 15ms delay.
 *
 * \return void
 */
void  Htu21Reset(void)
{
	uint8_t cmd[1];
	cmd[0]=HTU21_RESET_COMMAND;
	SensorI2cWriteBuffer(HTU21_ADDR , cmd, 1);
	SensorDelayMs(RESET_TIME);
}

/**
 * \brief Writes the HTU21 8-bits command with the value passed
 *
 * \param[in] void
 *
 * \return float : Humidity
 */
float Htu21ReadHumidity(void)
{
	uint8_t cmd[1];
	uint8_t buf[3];

	/* Prepare the I2C request. */
	cmd[0]=HTU21_READ_HUMIDITY_W_HOLD_COMMAND;
	if (!SensorI2cWriteBuffer( HTU21_ADDR,cmd , 1)) {
		return NAN;
	}
	
	if (!SensorI2cReadBuffer( HTU21_ADDR, buf, 3)) {
		return NAN;
	}

	/* Read 16 bits of data, dropping the last two status bits. */
	uint16_t hum = buf[0] << 8;
	// remove Status bits and add LSB byte
	hum |= buf[1] & 0xFC; 
	// 3rd byte is the CRC

	float humidity = hum;
	humidity *= 125.0f;
	humidity /= 65536.0f;
	humidity -= 6.0f;

	return humidity;
}

/**
 * \brief Writes the HTU21 8-bits command with the value passed
 *        Do not send the STOP bit in the I2C transfer
 *
 * \param[in] void
 *
 * \return float : Temperature
 */
float Htu21ReadTemperature(void)
{
	uint8_t cmd[1];
	uint8_t buf[3];

	cmd[0]=HTU21_READ_TEMPERATURE_W_HOLD_COMMAND;
	if (!SensorI2cWriteBuffer( HTU21_ADDR, cmd, 1)) {
		return NAN;
	}

	if (!SensorI2cReadBuffer( HTU21_ADDR, buf, 3)) {
		return NAN;
	}
    /* Read 16 bits of data, dropping the last two status bits. */
	uint16_t temp = buf[0] << 8;
	// remove Status bits and add LSB byte
	temp |= buf[1] & 0xFC; 
	// 3rd byte is the CRC

	float temperature = temp;
	temperature *= 175.72f;
	temperature /= 65536.0f;
	temperature -= 46.85f;

	return temperature;
}