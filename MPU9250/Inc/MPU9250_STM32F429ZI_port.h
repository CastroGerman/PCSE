/*
 * Board: STM32F429ZI Nucleo
 */

#ifndef MPU9250_PORT_H_
#define MPU9250_PORT_H_

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_i2c.h>
#include <stm32f4xx_hal_def.h>

/*-------------------- I2C Port Facilities --------------------*/

typedef enum I2CPortSignal {
	I2CPORT_OK_SIG = HAL_OK,
	I2CPORT_ERR_SIG = HAL_ERROR,
	I2CPORT_INVALID_PARAM_SIG = HAL_ERROR
}I2CPortSignal;

typedef struct I2CPort I2CPort;

typedef struct I2CPortVT {
	I2CPortSignal (*masterWriteBytes)(I2CPort * const me, uint8_t devAddress, uint8_t devRegister, uint8_t *txData, uint16_t size);
	I2CPortSignal (*masterReadBytes)(I2CPort * const me, uint8_t devAddress, uint8_t devRegister, uint8_t *rxData, uint16_t size);
	I2CPortSignal (*setInterface)(I2CPort * const me, I2C_HandleTypeDef * const interface);
	I2CPortSignal (*getInterface)(I2CPort * const me, I2C_HandleTypeDef * interface);
}I2CPortVT;

struct I2CPort {
	const struct I2CPortVT *vptr;
	I2C_HandleTypeDef *interface;
};

I2CPortSignal I2CPort_masterWriteBytes (I2CPort * const me, uint8_t devAddress, uint8_t devRegister, uint8_t *txData, uint16_t size);
I2CPortSignal I2CPort_masterReadBytes (I2CPort * const me, uint8_t devAddress, uint8_t devRegister, uint8_t *rxData, uint16_t size);
I2CPortSignal I2CPort_setInterface (I2CPort * const me, I2C_HandleTypeDef * const interface);
I2CPortSignal I2CPort_getInterface (I2CPort * const me, I2C_HandleTypeDef * interface);
void I2CPort_ctor (I2CPort * const me);

#endif /* MPU9250_PORT_H_ */
