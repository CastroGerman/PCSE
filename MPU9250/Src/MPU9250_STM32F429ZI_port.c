/*
 * Board: STM32F429ZI Nucleo
 */

#include <MPU9250_STM32F429ZI_port.h>

/*-------------------- I2C Port Facilities --------------------*/

I2CPortSignal I2CPort_masterWriteBytes (I2CPort * const me, uint8_t devAddress, uint8_t devRegister, uint8_t *txData, uint16_t size) {
	if (HAL_I2C_Master_Transmit(me->interface, devAddress<<1, &devRegister, sizeof(devRegister), HAL_MAX_DELAY) != HAL_OK) return I2CPORT_ERR_SIG;
	if (HAL_I2C_Master_Transmit(me->interface, devAddress<<1, txData, size, HAL_MAX_DELAY) != HAL_OK) return I2CPORT_ERR_SIG;
	return I2CPORT_OK_SIG;
}

I2CPortSignal I2CPort_masterReadBytes (I2CPort * const me, uint8_t devAddress, uint8_t devRegister, uint8_t *rxData, uint16_t size) {
	if (HAL_I2C_Master_Transmit(me->interface, devAddress<<1, &devRegister, sizeof(devRegister), HAL_MAX_DELAY) != HAL_OK) return I2CPORT_ERR_SIG;
	if (HAL_I2C_Master_Receive(me->interface, devAddress<<1, rxData, size, HAL_MAX_DELAY) != HAL_OK) return I2CPORT_ERR_SIG;
	return I2CPORT_OK_SIG;
}

I2CPortSignal I2CPort_setInterface (I2CPort * const me, I2C_HandleTypeDef * const interface) {
	if (interface == NULL) return I2CPORT_INVALID_PARAM_SIG;
	me->interface = interface;
	return I2CPORT_OK_SIG;
}

I2CPortSignal I2CPort_getInterface (I2CPort * const me, I2C_HandleTypeDef * interface) {
	interface = me->interface;
	return I2CPORT_OK_SIG;
}

void I2CPort_ctor (I2CPort * const me) {
	static const I2CPortVT vTable = {
			.masterReadBytes = &I2CPort_masterReadBytes,
			.masterWriteBytes = &I2CPort_masterWriteBytes,
			.setInterface = &I2CPort_setInterface,
			.getInterface = &I2CPort_getInterface
	};
	me->vptr = &vTable;
	me->interface = NULL;
}
