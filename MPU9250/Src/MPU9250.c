#include <MPU9250.h>
#include <MPU9250_STM32F429ZI_port.h>
#include <stddef.h>

/*-------------------- Vector Facilities --------------------*/

VectorSignal Vector_set (Vector * const me, uint16_t x, uint16_t y, uint16_t z) {
	me->x = x;
	me->y = y;
	me->z = z;
	return VECTOR_OK_SIG;
}

VectorSignal Vector_get (Vector * const me, uint16_t * x, uint16_t * y, uint16_t * z) {
	if (x != NULL) *x = me->x;
	if (y != NULL) *y = me->y;
	if (z != NULL) *z = me->z;
	return VECTOR_OK_SIG;
}

void Vector_ctor (Vector * const me) {
	static const VectorVT vTable = {
			.get = &Vector_get,
			.set = &Vector_set
	};
	me->vptr = &vTable;
	me->x = 0;
	me->y = 0;
	me->z = 0;
}

/*-------------------- MPU9250 Facilities --------------------*/

MPU9250Signal MPU9250_setDeviceAddress (MPU9250 * const me, uint8_t devAddress) {
	me->devAddress = devAddress;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_getRawMeasurements (MPU9250 * const me, uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz, uint16_t *mx, uint16_t *my, uint16_t *mz) {
	if (ax != NULL) *ax = me->accel.x;
	if (ay != NULL) *ay = me->accel.y;
	if (az != NULL) *az = me->accel.z;
	if (gx != NULL) *gx = me->gyro.x;
	if (gy != NULL) *gy = me->gyro.y;
	if (gz != NULL) *gz = me->gyro.z;
	if (mx != NULL) *mx = me->magn.x;
	if (my != NULL) *my = me->magn.y;
	if (mz != NULL) *mz = me->magn.z;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_takeGyroFS (MPU9250 * const me) {
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	me->gyroFs = (data>>3) & 0x3;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_takeAccelFS (MPU9250 * const me) {
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	me->accelFs = (data>>3) & 0x3;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_setGyroFS (MPU9250 * const me, uint8_t gyroFs) {
	if (gyroFs > 3) return MPU9250_INVALID_PARAM_SIG;
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	data &=~(0x3 << 3);
	data |= (gyroFs << 3);
	if ((int)I2CPort_masterWriteBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_CONFIG, &data, 1) != MPU9250_OK_SIG) {
		return MPU9250_ERR_SIG;
	} else {
		me->gyroFs = gyroFs;
	}
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_setAccelFS (MPU9250 * const me, uint8_t accelFs) {
	if (accelFs > 3) return MPU9250_INVALID_PARAM_SIG;
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	data &=~(0x3 << 3);
	data |= (accelFs << 3);
	if ((int)I2CPort_masterWriteBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_CONFIG, &data, 1) != MPU9250_OK_SIG) {
		return MPU9250_ERR_SIG;
	} else {
		me->accelFs = accelFs;
	}
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_takeFullScaleRanges (MPU9250 * const me) {
	if (MPU9250_takeAccelFS(me) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	if (MPU9250_takeGyroFS(me) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_takeMeasurements (MPU9250 * const me) {
	uint8_t rxBuf[6];

	if ((int)I2CPort_masterReadBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_XOUT_H, rxBuf, 6) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	Vector_set(&me->accel, (uint16_t)(rxBuf[0]<<8 | rxBuf[1]), (uint16_t)(rxBuf[2]<<8 | rxBuf[3]), (uint16_t)(rxBuf[4]<<8 | rxBuf[5]));

	if ((int)I2CPort_masterReadBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_XOUT_H, rxBuf, 6) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	Vector_set(&me->gyro, (uint16_t)(rxBuf[0]<<8 | rxBuf[1]), (uint16_t)(rxBuf[2]<<8 | rxBuf[3]), (uint16_t)(rxBuf[4]<<8 | rxBuf[5]));

	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_setDefaultSettings (MPU9250 * const me) {
	uint8_t data = 0x80;
	if ((int)I2CPort_masterWriteBytes(&me->i2c, me->devAddress, (uint8_t)MPU9250_PWR_MGMT_1, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	if (MPU9250_takeFullScaleRanges(me) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	return MPU9250_OK_SIG;
}

void MPU9250_ctor (MPU9250 * const me) {
	static const MPU9250VT vTable = {
			.setDeviceAddress = &MPU9250_setDeviceAddress,
			.getRawMeasurements = &MPU9250_getRawMeasurements,
			.takeGyroFS = &MPU9250_takeGyroFS,
			.takeAccelFS = &MPU9250_takeAccelFS,
			.setGyroFS = &MPU9250_setGyroFS,
			.setAccelFS = &MPU9250_setAccelFS,
			.takeFullScaleRanges = &MPU9250_takeFullScaleRanges,
			.takeMeasurements = &MPU9250_takeMeasurements,
			.setDefaultSettings = &MPU9250_setDefaultSettings
	};
	me->vptr = &vTable;
	me->devAddress = 0;
	I2CPort_ctor(&me->i2c);
	Vector_ctor(&me->accel);
	Vector_ctor(&me->gyro);
	Vector_ctor(&me->magn);
	me->accelFs = 0;
	me->gyroFs = 0;
}
