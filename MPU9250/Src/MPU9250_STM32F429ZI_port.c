/*
 * Board: STM32F429ZI Nucleo
 */

#include <MPU9250_STM32F429ZI_port.h>

int portI2C_Master_WriteBytes(uint8_t devAddress, uint8_t devRegister, uint8_t *txData, uint16_t size) {
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddress<<1, &devRegister, sizeof(devRegister), HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddress<<1, txData, size, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
	return HAL_OK;
}

int portI2C_Master_ReadBytes(uint8_t devAddress, uint8_t devRegister, uint8_t *rxData, uint16_t size) {
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddress<<1, &devRegister, sizeof(devRegister), HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
	if (HAL_I2C_Master_Receive(&hi2c1, devAddress<<1, rxData, size, HAL_MAX_DELAY) != HAL_OK) return HAL_ERROR;
	return HAL_OK;
}
