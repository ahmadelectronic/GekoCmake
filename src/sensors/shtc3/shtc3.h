#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sensor.h"

I2cWriteBuffer_t SensorI2cWriteBuffer;
I2cReadBuffer_t SensorI2cReadBuffer;
DelayMs_t SensorDelayMs;

#define SHTC3_DEFAULT_ADDR                    (uint8_t)0xE0 /**< SHTC3 I2C Address 0x70 7-bit*/

#define SHTC3_NORMAL_MEAS_TFIRST_STRETCH      0x7CA2 /**< Normal measurement, temp first with Clock Stretch Enabled */
#define SHTC3_LOWPOW_MEAS_TFIRST_STRETCH      0x6458 /**< Low power measurement, temp first with Clock Stretch Enabled */
#define SHTC3_NORMAL_MEAS_HFIRST_STRETCH      0x5C24 /**< Normal measurement, hum first with Clock Stretch Enabled */
#define SHTC3_LOWPOW_MEAS_HFIRST_STRETCH      0x44DE /**< Low power measurement, hum first with Clock Stretch Enabled */
#define SHTC3_NORMAL_MEAS_TFIRST              0x7866 /**< Normal measurement, temp first with Clock Stretch disabled */
#define SHTC3_LOWPOW_MEAS_TFIRST              0x609C /**< Low power measurement, temp first with Clock Stretch disabled */
#define SHTC3_NORMAL_MEAS_HFIRST              0x58E0 /**< Normal measurement, hum first with Clock Stretch disabled */
#define SHTC3_LOWPOW_MEAS_HFIRST              0x401A /**< Low power measurement, hum first with Clock Stretch disabled */
#define SHTC3_READID                          0xEFC8 /**< Read Out of ID Register */
#define SHTC3_SOFTRESET                       0x805D /**< Soft Reset */
#define SHTC3_SLEEP                           0xB098 /**< Enter sleep mode */
#define SHTC3_WAKEUP                          0x3517 /**< Wakeup mode */
#define SHTC3_ID                              0xEFC8 /**<Read-out of ID Register */

#define RESET_TIME											      15		 // ms value

// Functions

/**
 * \brief Configures the I2C master to be used with the SHTC3 device.
 */
void Shtc3Init(I2cWriteBuffer_t I2cWriteBufferPtr,I2cReadBuffer_t I2cReadBufferPtr,DelayMs_t SensorDelayMsPtr);

/**
 * \brief Reset the SHTC3 device
 *
 * \return void
 */
void Shtc3Reset(void);

/**
 * \brief Writes the SHTC3 8-bits command with the value passed
 *
 * \param[in] void
 *
 * \return float : Humidity
 */
float Shtc3ReadHumidity(void);

/**
 * \brief Writes the SHTC3 8-bits command with the value passed
 *        Do not send the STOP bit in the I2C transfer
 *
 * \param[in] void
 *
 * \return float : Temperature
 */
float Shtc3ReadTemperature(void);