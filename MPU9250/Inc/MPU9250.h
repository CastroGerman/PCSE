#ifndef MPU9250_H_
#define MPU9250_H_

#include <MPU9250_STM32F429ZI_port.h>
#include <stdint.h>


/*-------------------- MPU9250 Registers Defines --------------------*/

#define MPU9250_ADDRESS_AD0_LOW 	(0x68)
#define MPU9250_ADDRESS_AD0_HIGH 	(0x69)
#define AK8963_ADDRESS	(0x0C)
#define MPU9250_ADDRESS_AD0_LOW_READ 	((MPU9250_ADDRESS_AD0_LOW << 1) | 1)
#define MPU9250_ADDRESS_AD0_LOW_WRITE 	((MPU9250_ADDRESS_AD0_LOW << 1))
#define MPU9250_ADDRESS_AD0_HIGH_READ 	((MPU9250_ADDRESS_AD0_HIGH << 1) | 1)
#define MPU9250_ADDRESS_AD0_HIGH_WRITE 	((MPU9250_ADDRESS_AD0_HIGH << 1))
#define AK8963_ADDRESS_READ		((AK8963_ADDRESS << 1) | 1)
#define AK8963_ADDRESS_WRITE 	((AK8963_ADDRESS << 1))

#define MPU9250_GYRO_CONFIG		(27)
/*
*   FS_SEL     Full Scale Range        LSB Sensitivity
*   0           +/- 250 º/s             131 LSB/(º/s)
*   1           +/- 500 º/s             65.5 LSB/(º/s)
*   2           +/- 1000 º/s            32.8 LSB/(º/s)
*   3           +/- 2000 º/s            16.4 LSB/(º/s)
*/
#define MPU9250_GYRO_FS_SEL_250DPS	(0)
#define MPU9250_GYRO_FS_SEL_500DPS	(1)
#define MPU9250_GYRO_FS_SEL_1000DPS	(2)
#define MPU9250_GYRO_FS_SEL_2000DPS	(3)

#define MPU9250_ACCEL_CONFIG	(28)
/*
*   AFS_SEL     Full Scale Range        LSB Sensitivity
*   0               +/- 2 g                 16384 LSB/g
*   1               +/- 4 g                 8192 LSB/g
*   2               +/- 8 g                 4096 LSB/g
*   3               +/- 16 g                2048 LSB/g
*/
#define MPU9250_ACCEL_FS_SEL_2G	(0)
#define MPU9250_ACCEL_FS_SEL_4G	(1)
#define MPU9250_ACCEL_FS_SEL_8G	(2)
#define MPU9250_ACCEL_FS_SEL_16G	(3)

#define MPU9250_ACCEL_CONFIG2	(29)
#define MPU9250_FIFO_EN			(35)
#define MPU9250_I2C_MASTER_CONTROL	(36)
#define MPU9250_I2C_MASTER_STATUS	(54)
#define MPU9250_INT_PIN_EN_CONFIG	(55)
#define MPU9250_INT_EN	(56)
#define MPU9250_INT_STATUS	(58)

#define MPU9250_ACCEL_XOUT_H	(59)
#define MPU9250_ACCEL_XOUT_L	(60)
#define MPU9250_ACCEL_YOUT_H	(61)
#define MPU9250_ACCEL_YOUT_L	(62)
#define MPU9250_ACCEL_ZOUT_H	(63)
#define MPU9250_ACCEL_ZOUT_L	(64)
#define MPU9250_TEMP_OUT_H	(65)
#define MPU9250_TEMP_OUT_L	(66)
#define MPU9250_GYRO_XOUT_H	(67)
#define MPU9250_GYRO_XOUT_L	(68)
#define MPU9250_GYRO_YOUT_H	(69)
#define MPU9250_GYRO_YOUT_L	(70)
#define MPU9250_GYRO_ZOUT_H	(71)
#define MPU9250_GYRO_ZOUT_L	(72)

#define MPU9250_ACCEL_INTEL_CTRL	(105)
#define MPU9250_USER_CTRL	(106)
#define MPU9250_PWR_MGMT_1	(107)
#define MPU9250_PWR_MGMT_2	(108)
#define MPU9250_FIFO_COUNTH	(114)
#define MPU9250_FIFO_COUNTL	(115)
#define MPU9250_FIFO_R_W	(116)
#define MPU9250_WHOAMI	(117)

#define MPU9250_MAGN_HXL	(3)
#define MPU9250_MAGN_HXH	(4)
#define MPU9250_MAGN_HYL	(5)
#define MPU9250_MAGN_HYH	(6)
#define MPU9250_MAGN_HZL	(7)
#define MPU9250_MAGN_HZH	(8)

/*-------------------- Vector3D Facilities --------------------*/

typedef enum Vector3DSignal {
	VECTOR3D_OK_SIG,
	VECTOR3D_ERR_SIG
}Vector3DSignal;

typedef struct Vector3D Vector3D;

typedef struct Vector3DVT {
	Vector3DSignal (*set)(Vector3D * const me, uint16_t x, uint16_t y, uint16_t z);
	Vector3DSignal (*get)(Vector3D * const me, uint16_t * x, uint16_t * y, uint16_t * z);
}Vector3DVT;

struct Vector3D {
	const struct Vector3DVT *vptr; /* virtual pointer */
	uint16_t x, y, z;
};

Vector3DSignal Vector3D_set (Vector3D * const me, uint16_t x, uint16_t y, uint16_t z);
Vector3DSignal Vector3D_get (Vector3D * const me, uint16_t * x, uint16_t * y, uint16_t * z);
void Vector3D_ctor (Vector3D * const me, uint16_t x, uint16_t y, uint16_t z);

/*-------------------- MPU9250 Facilities --------------------*/

typedef enum MPU9250Signal {
	MPU9250_OK_SIG,
	MPU9250_ERR_SIG,
	MPU9250_INVALID_PARAM_SIG
}MPU9250Signal;

typedef struct MPU9250 MPU9250;

typedef struct MPU9250VT {
	MPU9250Signal (*setDeviceAddress)(MPU9250 * const me, uint8_t devAddress);
	MPU9250Signal (*getRawMeasurements)(MPU9250 * const me, uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz, uint16_t *mx, uint16_t *my, uint16_t *mz);
	MPU9250Signal (*takeGyroFS)(MPU9250 * const me);
	MPU9250Signal (*takeAccelFS)(MPU9250 * const me);
	MPU9250Signal (*setGyroFS)(MPU9250 * const me, uint8_t gyroFs);
	MPU9250Signal (*setAccelFS)(MPU9250 * const me, uint8_t accelFs);
	MPU9250Signal (*takeFullScaleRanges)(MPU9250 * const me);
	MPU9250Signal (*takeMeasurements)(MPU9250 * const me);
	MPU9250Signal (*setDefaultSettings)(MPU9250 * const me);
}MPU9250VT;

struct MPU9250 {
	const struct MPU9250VT *vptr; /* virtual pointer */
	uint8_t devAddress;
	uint8_t accelFs, gyroFs;
	Vector3D accel, gyro, magn; /* composition relationship */
	I2CPort * i2c;		/* aggregation relationship */
};

MPU9250Signal MPU9250_setDeviceAddress (MPU9250 * const me, uint8_t devAddress);
MPU9250Signal MPU9250_getRawMeasurements (MPU9250 * const me, uint16_t *ax, uint16_t *ay, uint16_t *az, uint16_t *gx, uint16_t *gy, uint16_t *gz, uint16_t *mx, uint16_t *my, uint16_t *mz);
MPU9250Signal MPU9250_takeGyroFS (MPU9250 * const me);
MPU9250Signal MPU9250_takeAccelFS (MPU9250 * const me);
MPU9250Signal MPU9250_setGyroFS (MPU9250 * const me, uint8_t gyroFs);
MPU9250Signal MPU9250_setAccelFS (MPU9250 * const me, uint8_t accelFs);
MPU9250Signal MPU9250_takeFullScaleRanges (MPU9250 * const me);
MPU9250Signal MPU9250_takeMeasurements (MPU9250 * const me);
MPU9250Signal MPU9250_setDefaultSettings (MPU9250 * const me);
void MPU9250_ctor (MPU9250 * const me, uint8_t devAddress, I2CPort * i2c);


/*
#define MAGN_SCALE		(0.15f)
#define TEMP_SCALE      (1/340.00f)
#define ACCEL_SCALE     (1/(16384.0f/(1<<(AFS_SEL))))
#define GYRO_SCALE      (1/(131.072f/(1<<(FS_SEL))))
#define DEG_TO_RAD      (3.14159265f/180)
#define RAD_TO_DEG      (1/DEG_TO_RAD)
*/

#endif /* MPU9250_H_ */
