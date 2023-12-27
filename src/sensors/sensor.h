/**
 * \file sensors.h
 *
 * \brief sensors Temperature & Humidity sensor function
 *
 */

// typedef
#include <stdint.h>
typedef bool (*I2cWriteBuffer_t)( uint8_t deviceAddr, uint8_t *buffer, uint16_t size);
typedef bool (*I2cReadBuffer_t)( uint8_t deviceAddr, uint8_t *buffer, uint16_t size);
typedef void (*DelayMs_t)(uint32_t ms);

// Combine strings
#define CONCAT_HELPER(X, Y) X##Y
#define CONCAT(X, Y) CONCAT_HELPER(X, Y)
#define CALL_FUNCTION(FUNCTION_NAME) CONCAT(SENSOR, FUNCTION_NAME)

#define SensorInit  CALL_FUNCTION(Init)
#define SensorReadTemperature  CALL_FUNCTION(ReadTemperature)
#define SensorReadHumidity  CALL_FUNCTION(ReadHumidity)
/**
 * \brief Configures the I2C master to be used with the HTU21 device.
 * 
 * \param [IN] -                pointer : I2cWriteBuffer
 * \param [IN] -                pointer : I2cReadBuffer
 * \param [OUT] -               void
 */
void SensorInit(I2cWriteBuffer_t I2cWriteBufferPtr,I2cReadBuffer_t I2cReadBufferPtr , DelayMs_t SensorDelayMsPtr );

/**
 * \brief Reads the temperature value
 *
 * \param[in] void
 *
 * \return float : Temperature
 */
float SensorReadTemperature(void);

/**
 * \brief Reads the relative humidity value
 *
 * \param[in] void
 *
 * \return float : Humidity
 */
float SensorReadHumidity(void);