#include "ICM20948.h"
#include "AK09918.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

HAL_StatusTypeDef ret; // to store return status
int16_t val; // data from IMU

HAL_StatusTypeDef IMU_WriteOneByte(ICM20948 *dev, uint8_t reg, uint8_t data);
HAL_StatusTypeDef IMU_ReadOneByte(ICM20948 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef Gyro_calibrate(ICM20948 *dev);

int16_t gyro_offset[3] = { 0 }; // gyro_offset value calibrated by Gyro_calibrate()

/*
 * INITIALISATION
 */
uint8_t* IMU_Initialise(ICM20948 *dev, I2C_HandleTypeDef *i2cHandle) {

	char hex[2];
	uint8_t regData;

	dev->i2cHandle = i2cHandle;


	//select bank 0
	/*	  //buf[0] = REG_BANK_SEL;  // register number
	 buf[0] = REG_ADD_REG_BANK_SEL;  // bank select register
	 buf[1] = REG_VAL_REG_BANK_0;           // bank 0
	 ret = HAL_I2C_Master_Transmit(i2cHandle, IMU_ADDR, buf, I2C_MEMADD_SIZE_16BIT, 10);

	 */
	ret = IMU_WriteOneByte(dev, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);


	//check ID
	/*	  buf[0] = REG_WHO_AM_I;  //(Should return ID =  0xEA)
	 ret = HAL_I2C_Mem_Read(i2cHandle, IMU_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, buf, I2C_MEMADD_SIZE_8BIT, 10);  */
	ret = IMU_ReadOneByte(dev, REG_WHO_AM_I, &regData);

	// Initialize
	// Bank 0 - Reset the device and then auto selects the best available clock source
	ret = IMU_WriteOneByte(dev, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

	//ret = IMU_WriteOneByte(dev, REG_ADD_PWR_MGMT_1, REG_VAL_ALL_RGE_RESET); // reset device - check header file value should be 0xF1

	HAL_Delay(10);
	ret = IMU_WriteOneByte(dev, REG_ADD_PWR_MGMT_1, REG_VAL_RUN_MODE); // auto selects the best available clock source for device

	// Turn off and on Accelator and Gyro - page 28
	HAL_Delay(10);
	ret = IMU_WriteOneByte(dev, REG_ADD_PWR_MGMT_2, REG_VAL_ACCEL_GYROLL_OFF); // OFF

	HAL_Delay(10);
	ret = IMU_WriteOneByte(dev, REG_ADD_PWR_MGMT_2, REG_VAL_ACCEL_GYROLL_ON); // ON

	ret = IMU_WriteOneByte(dev, REG_ADD_INT_ENABLE_1, REG_VAL_INT_ENABLED); // Turn on int on pin INT1

	uint8_t tmp = 0;
	IMU_ReadOneByte(dev, REG_ADD_INT_PIN_CFG, &tmp);
	IMU_WriteOneByte(dev, REG_ADD_INT_PIN_CFG, tmp | 0x02);

	// Bank 2 - Gyro and Acce and start running
	/* user bank 2 register */
	ret = IMU_WriteOneByte(dev, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);

	ret = IMU_WriteOneByte(dev, REG_ADD_GYRO_SMPLRT_DIV, 0x00); // changed to 0 from 0x16 **NEW**

	ret = IMU_WriteOneByte(dev, REG_ADD_GYRO_CONFIG_1,
			REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_2000DPS
					| REG_VAL_BIT_GYRO_DLPF); // enable low pass filter and set Gyro FS

	ret = IMU_WriteOneByte(dev, REG_ADD_GYRO_CONFIG_2, 0x06); // 8x average **NEW**
	ret = IMU_WriteOneByte(dev, REG_ADD_ACCEL_SMPLRT_DIV_2, 0x10); //
	ret = IMU_WriteOneByte(dev, REG_ADD_ACCEL_SMPLRT_DIV_1, 0x00); // upper 3 bit of sample rate = 0

	// enable LPF and set accel full scale to +/-2G, sensitivity scale factor = 16384 LSB/g
	ret = IMU_WriteOneByte(dev, REG_ADD_ACCEL_CONFIG,
			REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g
					| REG_VAL_BIT_ACCEL_DLPF);

	IMU_WriteOneByte(dev, REG_ADD_TEMP_CONFIG, REG_VAL_TEMP_CONFIG); // Temp configuration pg 67

	IMU_WriteOneByte(dev, 0x15, 0x03);
	// back to bank 0
	ret = IMU_WriteOneByte(dev, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);

	HAL_Delay(100);

	/* offset */
	//Gyro_calibrate(dev);  // calibrate the offset of the gyroscope

	// everthing OK
	//strcpy((char*)buf, "Initialize OK\r\n");
	//return &buf;
	return 0; // 0 means 0 error

}

HAL_StatusTypeDef IMU_WriteOneByte(ICM20948 *dev, uint8_t reg, uint8_t data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, IMU_ADDR, reg, 1, &data, 1, 100);

}

HAL_StatusTypeDef IMU_ReadOneByte(ICM20948 *dev, uint8_t reg, uint8_t *data) {

	return HAL_I2C_Mem_Read(dev->i2cHandle, IMU_ADDR, reg, 1, data, 1, 100);
}

HAL_StatusTypeDef Mag_WriteOneByte(ICM20948 *dev, uint8_t reg, uint8_t data) {

	return HAL_I2C_Mem_Write(dev->i2cHandle, MAG_ADDR, reg, 1, &data, 1, 100);
}

HAL_StatusTypeDef Mag_ReadOneByte(ICM20948 *dev, uint8_t reg, uint8_t *data) {

	//HAL_I2C_Master_Transmit_IT(dev->i2cHandle, MAG_ADDR, &reg, 1);
	//ret = HAL_I2C_Master_Receive_IT(dev->i2cHandle, MAG_ADDR, data, 1);

	return HAL_I2C_Mem_Read(dev->i2cHandle, MAG_ADDR, reg, 1, data, 1, 100);

}

HAL_StatusTypeDef IMU_TempRead(ICM20948 *dev) {
	uint8_t u8Buf[2];
	int16_t tempRaw;
	int32_t tempC; // temperature in deg C, with sensitivity 333.87.   Offset value 0 = 21 deg C

	ret = IMU_ReadOneByte(dev, REG_ADD_TEMP_OUT_L, &u8Buf[0]);
	ret = IMU_ReadOneByte(dev, REG_ADD_TEMP_OUT_H, &u8Buf[1]);
	tempRaw = (u8Buf[1] << 8) | u8Buf[0];
	dev->temp_C = (tempRaw) / 333.81 + 21; // assuming no further offset apart from 21 dec C
}

HAL_StatusTypeDef IMU_AccelRead(ICM20948 *dev) {
	uint8_t u8Buf[2] = { 0 }; // reset to zero
	int16_t accRaw[3] = { 0 };  // reset to zero

	ret = IMU_ReadOneByte(dev, REG_ADD_ACCEL_XOUT_L, &u8Buf[0]);
	ret = IMU_ReadOneByte(dev, REG_ADD_ACCEL_XOUT_H, &u8Buf[1]);
	accRaw[0] = (u8Buf[1] << 8) | u8Buf[0];

	ret = IMU_ReadOneByte(dev, REG_ADD_ACCEL_YOUT_L, &u8Buf[0]);
	ret = IMU_ReadOneByte(dev, REG_ADD_ACCEL_YOUT_H, &u8Buf[1]);
	accRaw[1] = (u8Buf[1] << 8) | u8Buf[0];

	ret = IMU_ReadOneByte(dev, REG_ADD_ACCEL_ZOUT_L, &u8Buf[0]);
	ret = IMU_ReadOneByte(dev, REG_ADD_ACCEL_ZOUT_H, &u8Buf[1]);
	accRaw[2] = (u8Buf[1] << 8) | u8Buf[0];

	/* Convert to SIGNED integers (two's complement) */
	int32_t accRawSigned[3];

	if ((accRaw[0] & 0x00080000) == 0x00080000)
		accRawSigned[0] = accRaw[0] | 0xFFF00000;
	else
		accRawSigned[0] = accRaw[0];

	if ((accRaw[1] & 0x00080000) == 0x00080000)
		accRawSigned[1] = accRaw[1] | 0xFFF00000;
	else
		accRawSigned[1] = accRaw[1];

	if ((accRaw[2] & 0x00080000) == 0x000080000)
		accRawSigned[2] = accRaw[2] | 0xFFF00000;
	else
		accRawSigned[2] = accRaw[2];

	// accel full scale set to +/-2G, sensitivity scale factor = 16384 LSB/g
	dev->acc[0] = 0.00006103515625f * accRawSigned[0] * 9.81f;
	dev->acc[1] = 0.00006103515625f * accRawSigned[1] * 9.81f;
	dev->acc[2] = 0.00006103515625f * accRawSigned[2] * 9.81f; // * 9.81f

	return ret;

}

HAL_StatusTypeDef Gyro_calibrate(ICM20948 *dev) // calibrate the offset of the gyro
// store the offset in int16_t gyro_offset[3]
{
	uint8_t u8Buf[2] = { 0 }; // reset to zero upon entry
	int16_t gyroRaw[3] = { 0 }; // reset to zero upon entry
	int8_t i;
	int16_t temp;

	for (i = 0; i < 64; i++) {
		IMU_ReadOneByte(dev, REG_ADD_GYRO_XOUT_L, &u8Buf[0]);
		IMU_ReadOneByte(dev, REG_ADD_GYRO_XOUT_H, &u8Buf[1]);
		temp = (u8Buf[1] << 8) | u8Buf[0]; // for debugging
		gyroRaw[0] = temp + gyroRaw[0];
		//gyroRaw[0] = (u8Buf[1]<<8)|u8Buf[0] + gyroRaw[0];
		HAL_Delay(1);
		IMU_ReadOneByte(dev, REG_ADD_GYRO_YOUT_L, &u8Buf[0]);
		IMU_ReadOneByte(dev, REG_ADD_GYRO_YOUT_H, &u8Buf[1]);
		gyroRaw[1] = ((u8Buf[1] << 8) | u8Buf[0]) + gyroRaw[1];
		HAL_Delay(1);
		IMU_ReadOneByte(dev, REG_ADD_GYRO_ZOUT_L, &u8Buf[0]);
		ret = IMU_ReadOneByte(dev, REG_ADD_GYRO_ZOUT_H, &u8Buf[1]);
		gyroRaw[2] = ((u8Buf[1] << 8) | u8Buf[0]) + gyroRaw[2];

		osDelay(25); // wait for 100msec
	}

	dev->gyro_bias[0] = (float)(gyroRaw[0] >> 6);  // average of 32 reads
	dev->gyro_bias[1] = (float)(gyroRaw[1] >> 6);
	dev->gyro_bias[2] = (float)(gyroRaw[2] >> 6);


	return ret;
}

HAL_StatusTypeDef IMU_GyroRead(ICM20948 *dev) { // return the change in value instead of current value
	uint8_t u8Buf[2] = { 0 }; // reset to zero
	int16_t gyroRaw[3] = { 0 };  // reset to zero
	int16_t gyroDiff[3];
	int16_t temp;

	ret = IMU_ReadOneByte(dev, REG_ADD_GYRO_YOUT_L, &u8Buf[0]);
	ret = IMU_ReadOneByte(dev, REG_ADD_GYRO_YOUT_H, &u8Buf[1]);
	gyroRaw[1] = (u8Buf[1] << 8) | u8Buf[0];


	ret = IMU_ReadOneByte(dev, REG_ADD_GYRO_ZOUT_L, &u8Buf[0]);
	ret = IMU_ReadOneByte(dev, REG_ADD_GYRO_ZOUT_H, &u8Buf[1]);
	gyroRaw[2] = (u8Buf[1] << 8) | u8Buf[0];


	ret = IMU_ReadOneByte(dev, REG_ADD_GYRO_XOUT_L, &u8Buf[0]);
	ret = IMU_ReadOneByte(dev, REG_ADD_GYRO_XOUT_H, &u8Buf[1]);
	temp = (u8Buf[1] << 8) | u8Buf[0]; // for debugging
	gyroRaw[0] = (u8Buf[1] << 8) | u8Buf[0];

	float gyroRawFloat[3] = {0};
	gyroRawFloat[0] = gyroRaw[0] - dev->gyro_bias[0];
	gyroRawFloat[1] = gyroRaw[1] - dev->gyro_bias[1];
	gyroRawFloat[2] = gyroRaw[2] - dev->gyro_bias[2];

	// gyro full scale set to +/-500 dps, sensitivity scale factor = 65.5 LSB/dps
	// degree per second = value/65.5



	dev->gyro[0] = 0.06106870229f * gyroRawFloat[0];
	dev->gyro[1] = 0.06106870229f * gyroRawFloat[1];
	dev->gyro[2] = 0.06106870229f * gyroRawFloat[2];

	return ret;

}

HAL_StatusTypeDef Mag_init(ICM20948 *dev) {
	uint8_t tmp = 0;

	Mag_WriteOneByte(dev, AK09916__CNTL2__REGISTER, REG_VAL_MAG_MODE_10HZ);

}

HAL_StatusTypeDef Mag_read(ICM20948 *dev) {

	uint8_t buf[8] = { 0 };
	uint8_t tmp = 0;
	Mag_ReadOneByte(dev, AK09916__ST1__REGISTER, &tmp);

	if (tmp & 0x1) {
		for (uint32_t i = 0; i < 8; i++)
			Mag_ReadOneByte(dev, AK09916__XOUT_L__REGISTER + i, &buf[i]);

		dev->mag[0] = (buf[1] << 8) | buf[0];
		dev->mag[1] = (buf[3] << 8) | buf[2];
		dev->mag[2] = (buf[5] << 8) | buf[4];
	}
	Mag_ReadOneByte(dev, AK09916__ST2__REGISTER, &tmp);
	return 0;

}

void magCalICM20948(ICM20948 *dev, float *bias_dest, float *scale_dest) {
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { 0x8000, 0x8000, 0x8000 }, mag_min[3] = { 0x7FFF,
			0x7FFF, 0x7FFF };

	float mRes = 10.0f * 4912.0f / 32760.0f;
	sample_count = 15;

	for (ii = 0; ii < sample_count; ii++) {
		Mag_read(dev);  // Read the mag data

		for (int jj = 0; jj < 3; jj++) {
			if (dev->mag[jj] > mag_max[jj]) {
				mag_max[jj] = dev->mag[jj];
			}
			if (dev->mag[jj] < mag_min[jj]) {
				mag_min[jj] = dev->mag[jj];
			}
		}
		HAL_Delay(110); // At 8 Hz ODR, new mag data is available every 125 ms

	}

	// Get hard iron correction
	// Get 'average' x mag bias in counts
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
	// Get 'average' y mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
	// Get 'average' z mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

	// Save mag biases in G for main program
	bias_dest[0] = (float) mag_bias[0] * mRes;	  // * factoryMagCalibration[0];
	bias_dest[1] = (float) mag_bias[1] * mRes;	  // * factoryMagCalibration[1];
	bias_dest[2] = (float) mag_bias[2] * mRes;	  // * factoryMagCalibration[2];

	// Get soft iron correction estimate
	// Get average x axis max chord length in counts
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
	// Get average y axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
	// Get average z axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	scale_dest[0] = avg_rad / ((float) mag_scale[0]);
	scale_dest[1] = avg_rad / ((float) mag_scale[1]);
	scale_dest[2] = avg_rad / ((float) mag_scale[2]);

}

// These are the free parameters in the Mahony filter and fusion scheme, Kp
// for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f
#define Ki 0.0f

static const float GyroMeasError = 3.1415 * (40.0f / 180.0f);
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static const float GyroMeasDrift = 3.1415 * (0.0f / 180.0f);
// There is a tradeoff in the beta parameter between accuracy and response
// speed. In the original Madgwick study, beta of 0.041 (corresponding to
// GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds
// to a stable initial quaternion. Subsequent changes also require a
// longish lag time to a stable output, not fast enough for a quadcopter or
// robot car! By increasing beta (GyroMeasError) by about a factor of
// fifteen, the response time constant is reduced to ~2 sec. I haven't
// noticed any reduction in solution accuracy. This is essentially the I
// coefficient in a PID control sense; the bigger the feedback coefficient,
// the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and
// fusion scheme.
static float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // Compute beta
// Compute zeta, the other free parameter in the Madgwick scheme usually
// set to a small or zero value
static float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

// Vector to hold integral error for Mahony method
static float eInt[3] = { 0.0f, 0.0f, 0.0f };
// Vector to hold quaternion
static float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

void MahonyQuaternionUpdate(ICM20948 *dev, float ax, float ay, float az,
		float gx, float gy, float gz, float mx, float my, float mz,
		float deltat) {
	// short name local variable for readability
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // Handle NaN
	norm = 1.0f / norm;       // Use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // Handle NaN
	norm = 1.0f / norm;       // Use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4)
			+ 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4)
			+ 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2)
			+ 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f) {
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	} else {
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	if (norm == 0.0f)
		return; // Handle NaN
	norm = 1.0f / (norm);
	dev->q[0] = q1 * norm;
	dev->q[1] = q2 * norm;
	dev->q[2] = q3 * norm;
	dev->q[3] = q4 * norm;
}
const float* getQ(ICM20948 *dev) {
	return dev->q;
}
