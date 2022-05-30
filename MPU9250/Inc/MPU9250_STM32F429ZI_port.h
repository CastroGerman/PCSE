/*
 * Board: STM32F429ZI Nucleo
 */

#ifndef MPU9250_STM32F429ZI_PORT_H_
#define MPU9250_STM32F429ZI_PORT_H_

#include <stdint.h>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_i2c.h>
#include <stm32f4xx_hal_def.h>

extern I2C_HandleTypeDef hi2c1;

int portI2C_Master_WriteBytes(uint8_t devAddress, uint8_t devRegister, uint8_t *txData, uint16_t size);
int portI2C_Master_ReadBytes(uint8_t devAddress, uint8_t devRegister, uint8_t *rxData, uint16_t size);

#endif /* MPU9250_STM32F429ZI_PORT_H_ */
