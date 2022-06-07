#include <MPU9250.h>
#include <MPU9250_STM32F429ZI_port.h>
#include <stddef.h>

/*-------------------- Vector3D Facilities --------------------*/

Vector3DSignal Vector3D_set (Vector3D * const me, uint16_t x, uint16_t y, uint16_t z) {
	me->x = x;
	me->y = y;
	me->z = z;
	return VECTOR3D_OK_SIG;
}

Vector3DSignal Vector3D_get (Vector3D * const me, uint16_t * x, uint16_t * y, uint16_t * z) {
	if (x != NULL) *x = me->x;
	if (y != NULL) *y = me->y;
	if (z != NULL) *z = me->z;
	return VECTOR3D_OK_SIG;
}

void Vector3D_ctor (Vector3D * const me, uint16_t x, uint16_t y, uint16_t z) {
	static const Vector3DVT vTable = {
			.get = &Vector3D_get,
			.set = &Vector3D_set
	};
	me->vptr = &vTable;
	me->x = x;
	me->y = y;
	me->z = z;
}
/*-------------------- MPU9250 Facilities --------------------*/

MPU9250Signal MPU9250_setDeviceAddress (MPU9250 * const me, uint8_t devAddress) {
	me->devAddress = devAddress;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_getRawMeasurements (MPU9250 * const me, uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz, uint16_t *mx, uint16_t *my, uint16_t *mz) {
	(*me->vptr->takeMeasurements)(me);
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

MPU9250Signal MPU9250_getMeasurements (MPU9250 * const me, float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
	(*me->vptr->takeMeasurements)(me);
	float accelScale = (1/(16384.0f/(1<<(me->accelFs)))); /* Measured in: G force */
	float gyroScale = (1/(131.072f/(1<<(me->gyroFs))));	/* Measured in: degrees/second */
	float magnScale = (0.15f); /* Measured in: T */
	if (ax != NULL) *ax = (float)((int16_t)me->accel.x - (int16_t)me->accelOffset.x) * accelScale;
	if (ay != NULL) *ay = (float)((int16_t)me->accel.y - (int16_t)me->accelOffset.y) * accelScale;
	if (az != NULL) *az = (float)((int16_t)me->accel.z - (int16_t)me->accelOffset.z) * accelScale;
	if (gx != NULL) *gx = (float)((int16_t)me->gyro.x - (int16_t)me->gyroOffset.x) * gyroScale;
	if (gy != NULL) *gy = (float)((int16_t)me->gyro.y - (int16_t)me->gyroOffset.y) * gyroScale;
	if (gz != NULL) *gz = (float)((int16_t)me->gyro.z - (int16_t)me->gyroOffset.z) * gyroScale;
	if (mx != NULL) *mx = (float)((int16_t)me->magn.x - (int16_t)me->magnOffset.x) * magnScale;
	if (my != NULL) *my = (float)((int16_t)me->magn.y - (int16_t)me->magnOffset.y) * magnScale;
	if (mz != NULL) *mz = (float)((int16_t)me->magn.z - (int16_t)me->magnOffset.z) * magnScale;

	//    // Read gyro values and convert to Radians per second
	//    _data->gx.cooked = ((double)_data->gx.raw - _data->gx.offset) * DEG_TO_RAD * GYRO_SCALE;
	//    _data->gy.cooked = ((double)_data->gy.raw - _data->gy.offset) * DEG_TO_RAD * GYRO_SCALE;
	//    _data->gz.cooked = ((double)_data->gz.raw - _data->gz.offset) * DEG_TO_RAD * GYRO_SCALE;
	//    // As datasheet says:
	//    // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
	//    // As I say:
	//    _data->temp.cooked = ((double)_data->temp.raw - _data->temp.offset) * TEMP_SCALE;
	//    // Read accel values and convert to G force.
	//    _data->ax.cooked = ((double)_data->ax.raw - _data->ax.offset) * ACCEL_SCALE;
	//    _data->ay.cooked = ((double)_data->ay.raw - _data->ay.offset) * ACCEL_SCALE;
	//    _data->az.cooked = ((double)_data->az.raw - _data->az.offset) * ACCEL_SCALE;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_takeGyroFS (MPU9250 * const me) {
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	me->gyroFs = (data>>3) & 0x3;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_takeAccelFS (MPU9250 * const me) {
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	me->accelFs = (data>>3) & 0x3;
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_setGyroFS (MPU9250 * const me, uint8_t gyroFs) {
	if (gyroFs > 3) return MPU9250_INVALID_PARAM_SIG;
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	data &=~(0x3 << 3);
	data |= (gyroFs << 3);
	if ((int)I2CPort_masterWriteBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_CONFIG, &data, 1) != MPU9250_OK_SIG) {
		return MPU9250_ERR_SIG;
	} else {
		me->gyroFs = gyroFs;
	}
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_setAccelFS (MPU9250 * const me, uint8_t accelFs) {
	if (accelFs > 3) return MPU9250_INVALID_PARAM_SIG;
	uint8_t data;
	if ((int)I2CPort_masterReadBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_CONFIG, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	data &=~(0x3 << 3);
	data |= (accelFs << 3);
	if ((int)I2CPort_masterWriteBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_CONFIG, &data, 1) != MPU9250_OK_SIG) {
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

	if ((int)I2CPort_masterReadBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_ACCEL_XOUT_H, rxBuf, 6) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	Vector3D_set(&me->accel, (uint16_t)(rxBuf[0]<<8 | rxBuf[1]), (uint16_t)(rxBuf[2]<<8 | rxBuf[3]), (uint16_t)(rxBuf[4]<<8 | rxBuf[5]));

	if ((int)I2CPort_masterReadBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_GYRO_XOUT_H, rxBuf, 6) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	Vector3D_set(&me->gyro, (uint16_t)(rxBuf[0]<<8 | rxBuf[1]), (uint16_t)(rxBuf[2]<<8 | rxBuf[3]), (uint16_t)(rxBuf[4]<<8 | rxBuf[5]));

	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_takeOffsets (MPU9250 * const me, uint16_t iterations) {
	int32_t accelAccX = 0, accelAccY = 0, accelAccZ = 0;
	int32_t gyroAccX = 0, gyroAccY = 0, gyroAccZ = 0;
	int32_t magnAccX = 0, magnAccY = 0, magnAccZ = 0;
	for(uint16_t i = 0; i < iterations; ++i){
		(*me->vptr->takeMeasurements)(me);
		accelAccX += (int16_t)me->accel.x;
		accelAccY += (int16_t)me->accel.y;
		accelAccZ += (int16_t)me->accel.z;
		gyroAccX += (int16_t)me->gyro.x;
		gyroAccY += (int16_t)me->gyro.y;
		gyroAccZ += (int16_t)me->gyro.z;
		magnAccX += (int16_t)me->magn.x;
		magnAccY += (int16_t)me->magn.y;
		magnAccZ += (int16_t)me->magn.z;
	}
	accelAccX /= iterations;
	accelAccY /= iterations;
	accelAccZ /= iterations;
	gyroAccX /= iterations;
	gyroAccY /= iterations;
	gyroAccZ /= iterations;
	magnAccX /= iterations;
	magnAccY /= iterations;
	magnAccZ /= iterations;
	(*me->accelOffset.vptr->set)(&me->accelOffset, (uint16_t)accelAccX, (uint16_t)accelAccY, (uint16_t)accelAccZ);
	(*me->gyroOffset.vptr->set)(&me->gyroOffset, (uint16_t)gyroAccX, (uint16_t)gyroAccY, (uint16_t)gyroAccZ);
	(*me->magnOffset.vptr->set)(&me->magnOffset, (uint16_t)magnAccX, (uint16_t)magnAccY, (uint16_t)magnAccZ);
	/* takeGForceOff(); */
	return MPU9250_OK_SIG;
}

MPU9250Signal MPU9250_setDefaultSettings (MPU9250 * const me) {
	uint8_t data = 0x80;
	if ((int)I2CPort_masterWriteBytes(me->i2c, me->devAddress, (uint8_t)MPU9250_PWR_MGMT_1, &data, 1) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	if (MPU9250_takeFullScaleRanges(me) != MPU9250_OK_SIG) return MPU9250_ERR_SIG;
	return MPU9250_OK_SIG;
}

void MPU9250_ctor (MPU9250 * const me, uint8_t devAddress, I2CPort * i2c) {
	static const MPU9250VT vTable = {
			.setDeviceAddress = &MPU9250_setDeviceAddress,
			.getRawMeasurements = &MPU9250_getRawMeasurements,
			.getMeasurements = &MPU9250_getMeasurements,
			.takeGyroFS = &MPU9250_takeGyroFS,
			.takeAccelFS = &MPU9250_takeAccelFS,
			.setGyroFS = &MPU9250_setGyroFS,
			.setAccelFS = &MPU9250_setAccelFS,
			.takeFullScaleRanges = &MPU9250_takeFullScaleRanges,
			.takeMeasurements = &MPU9250_takeMeasurements,
			.takeOffset = &MPU9250_takeOffsets,
			.setDefaultSettings = &MPU9250_setDefaultSettings
	};
	me->vptr = &vTable;
	me->devAddress = devAddress;
	me->accelFs = 0;
	me->gyroFs = 0;
	Vector3D_ctor(&me->accel, 0, 0, 0);
	Vector3D_ctor(&me->gyro, 0, 0, 0);
	Vector3D_ctor(&me->magn, 0, 0, 0);
	Vector3D_ctor(&me->accelOffset, 0, 0, 0);
	Vector3D_ctor(&me->gyroOffset, 0, 0, 0);
	Vector3D_ctor(&me->magnOffset, 0, 0, 0);
	me->i2c = i2c;
}
