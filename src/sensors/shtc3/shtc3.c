/**
 * \file shtc3.c
 *
 * \brief shtc3 Temperature & Humidity sensor driver source file
 *
 */
#include "shtc3.h"

/**
 * \brief Configures the I2C master to be used with the shtc3 device.
 */
void Shtc3Init(I2cWriteBuffer_t I2cWriteBufferPtr,I2cReadBuffer_t I2cReadBufferPtr,DelayMs_t SensorDelayMsPtr )
{
	SensorI2cWriteBuffer = I2cWriteBufferPtr;
	SensorI2cReadBuffer = I2cReadBufferPtr;
	SensorDelayMs = SensorDelayMsPtr;
	
	Shtc3Reset();
}

/**
 * \brief Sends a 'reset' request to the Shtc3, followed by a 1ms delay.
 *
 * \return void
 */
void Shtc3Reset(void)
{
	uint8_t cmd[2];
	cmd[0]=SHTC3_SOFTRESET >> 8;
	cmd[1]=SHTC3_SOFTRESET & 0x00FF;
	SensorI2cWriteBuffer(SHTC3_DEFAULT_ADDR , cmd, 2);
	SensorDelayMs(RESET_TIME);
}

/**
 * \brief Writes the Shtc3 8-bits command with the value passed
 *
 * \param[in] void
 *
 * \return float : Humidity
 */
float Shtc3ReadHumidity(void)
{
	uint8_t cmd[2];
	uint8_t buf[6];

	/* Prepare the I2C request. */
	cmd[0]=SHTC3_NORMAL_MEAS_HFIRST_STRETCH >> 8;
	cmd[1]=SHTC3_NORMAL_MEAS_HFIRST_STRETCH & 0x00FF;
	if (!SensorI2cWriteBuffer(SHTC3_DEFAULT_ADDR , cmd, 2)) {
		return NAN;
	}
	
	if (!SensorI2cReadBuffer( SHTC3_DEFAULT_ADDR, buf, 6)) {
		return NAN;
	}

	/* Read 16 bits of data, dropping the last two status bits. */
	uint16_t hum = buf[0] << 8;	
	//Add LSB byte
	hum |= buf[1];
	// 3rd byte is the CRC

	float humidity = hum;
	humidity *= 125.0f;
	humidity /= 65536.0f;
	humidity -= 6.0f;

	return humidity;
}

/**
 * \brief Writes the Shtc3 8-bits command with the value passed
 *        Do not send the STOP bit in the I2C transfer
 *
 * \param[in] void
 *
 * \return float : Temperature
 */
float Shtc3ReadTemperature(void)
{
	uint8_t cmd[2];
	uint8_t buf[6];
	
	/* Prepare the I2C request. */
	cmd[0]=SHTC3_NORMAL_MEAS_HFIRST_STRETCH >> 8;
	cmd[1]=SHTC3_NORMAL_MEAS_HFIRST_STRETCH & 0x00FF;
	if (!SensorI2cWriteBuffer(SHTC3_DEFAULT_ADDR , cmd, 2)) {
		return NAN;
	}
	
	if (!SensorI2cReadBuffer( SHTC3_DEFAULT_ADDR, buf, 6)) {
		return NAN;
	}

    /* Read 16 bits of data, dropping the last two status bits. */
	uint16_t temp = buf[3] << 8;
	//Add LSB byte
	temp |= buf[4];
	// 3rd byte is the CRC

	float temperature = temp;
	temperature *= 175.72f;
	temperature /= 65536.0f;
	temperature -= 46.85f;

	return temperature;
}