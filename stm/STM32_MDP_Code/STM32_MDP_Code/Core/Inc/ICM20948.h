#ifndef __ICM_20948_H
#define __ICM_20948_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> 		/* need for data type declarations */
#include "stm32f4xx_hal.h"  /* Needed for I2C */


/* =====================================================
   Application-Facing Register Definitions (Primary Set)
   These are the registers used directly in application code.
   Do NOT remove or rename these.
   ===================================================== */

/* ====== I2C Addresses ========== */
static const uint8_t IMU_ADDR     = 0x68<<1;   //8-bit I2C address for IMU, 110 1000b (the upper seven bit)
static const uint8_t MAG_ADDR     = 0x0C<< 1;

/* === REGISTERS Commonly Used === */
static const uint8_t REG_BANK_SEL = 0x7f; //Register to select different bank
static const uint8_t REG_WHO_AM_I = 0x00; // address of  WHO_AM_I register (@Bank 0)
static const uint8_t REG_TEMP_H   = 0x39; // address of temperature register (@Bank 0)
static const uint8_t REG_TEMP_L   = 0x3A; // address of temperature register (@Bank 0)

/* ============================================
   [APPLICATION] ICM-20948 User Bank 0 Registers
   ============================================ */
#define REG_ADD_WIA             	0x00
#define REG_VAL_WIA             	0xEA
#define REG_ADD_USER_CTRL       	0x03
#define REG_VAL_BIT_DMP_EN          0x80
#define REG_VAL_BIT_FIFO_EN         0x40
#define REG_VAL_BIT_I2C_MST_EN      0x20
#define REG_VAL_BIT_I2C_IF_DIS      0x10
#define REG_VAL_BIT_DMP_RST         0x08
#define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MGMT_1     		0x06
//#define REG_VAL_ALL_RGE_RESET   	0xF1    // Should be 0xF1 if want to reset - pg 27 of datasheet
#define REG_VAL_ALL_RGE_RESET   	0xF1    // Reset device registers - pg 27 of datasheet
#define REG_VAL_RUN_MODE        	0x01    // Device run with auto selects the best available clock source - pg 37 of datasheeet
#define REG_ADD_LP_CONFIG      	 	0x05
//#define REG_ADD_PWR_MIGMT_1      	0x06
#define REG_ADD_PWR_MGMT_2      	0x07
#define REG_VAL_ACCEL_GYROLL_OFF   (0x38 | 0x07)  // see page 28
#define REG_VAL_ACCEL_GYROLL_ON    (0x00 | 0x00)
#define REG_ADD_INT_PIN_CFG      	0x0F  // Reference page 38
#define REG_ADD_INT_ENABLE_1     	0x10
#define REG_VAL_INT_ENABLED      	0x01  // Reference page 39  - Enable INT1 pin
#define REG_ADD_ACCEL_XOUT_H     	0x2D
#define REG_ADD_ACCEL_XOUT_L     	0x2E
#define REG_ADD_ACCEL_YOUT_H     	0x2F
#define REG_ADD_ACCEL_YOUT_L     	0x30
#define REG_ADD_ACCEL_ZOUT_H     	0x31
#define REG_ADD_ACCEL_ZOUT_L     	0x32
#define REG_ADD_GYRO_XOUT_H      	0x33
#define REG_ADD_GYRO_XOUT_L      	0x34
#define REG_ADD_GYRO_YOUT_H      	0x35
#define REG_ADD_GYRO_YOUT_L      	0x36
#define REG_ADD_GYRO_ZOUT_H      	0x37
#define REG_ADD_GYRO_ZOUT_L      	0x38
#define REG_ADD_TEMP_OUT_H       	0x39
#define REG_ADD_TEMP_OUT_L       	0x3A
#define REG_ADD_EXT_SENS_DATA_00 	0x3B
#define REG_ADD_REG_BANK_SEL    	0x7F
#define REG_VAL_REG_BANK_0  	 	0x00
#define REG_VAL_REG_BANK_1  	 	0x10
#define REG_VAL_REG_BANK_2  	 	0x20
#define REG_VAL_REG_BANK_3  	 	0x30

/* ============================================
   [APPLICATION] ICM-20948 user bank 1 & 2 Registers
   ============================================ */
#define REG_ADD_GYRO_SMPLRT_DIV 	0x00
#define REG_ADD_GYRO_CONFIG_1   	0x01
#define REG_ADD_GYRO_CONFIG_2   	0x02
#define REG_VAL_BIT_GYRO_DLPCFG_2   0x10 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4   0x20 /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6   0x30 /* bit[5:3]  LPF = 5.7 Hz pg 59 */
#define REG_VAL_BIT_GYRO_FS_250DPS  0x00 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS  0x02 /* bit[2:1] Gyro Full Scale = +/- 500 degree per second */
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF       0x01 /* bit[0]    enable LPF */
#define REG_ADD_ACCEL_SMPLRT_DIV_1  0x10
#define REG_ADD_ACCEL_SMPLRT_DIV_2  0x11
#define REG_ADD_ACCEL_CONFIG        0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2  0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4  0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6  0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g     0x00 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g     0x02 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g     0x04 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g    0x06 /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF      0x01 /* bit[0]   */
#define REG_ADD_TEMP_CONFIG         0x53
#define REG_VAL_TEMP_CONFIG         0x00 // page 67 , set temp sensor bandwidth to 17Hz
/* define ICM-20948 Register  end */

/* ============================================
   [APPLICATION] ICM-20948 user bank 3 Registers
   ============================================ */
#define REG_ADD_I2C_SLV0_ADDR   0x03
#define REG_ADD_I2C_SLV0_REG    0x04
#define REG_ADD_I2C_SLV0_CTRL   0x05
#define REG_VAL_BIT_SLV0_EN     0x80
#define REG_VAL_BIT_MASK_LEN    0x07
#define REG_ADD_I2C_SLV0_DO     0x06
#define REG_ADD_I2C_SLV1_ADDR   0x07
#define REG_ADD_I2C_SLV1_REG    0x08
#define REG_ADD_I2C_SLV1_CTRL   0x09
#define REG_ADD_I2C_SLV1_DO     0x0A
/* define ICM-20948 Register  end */

/* ============================================
   [APPLICATION] ICM-20948 Magnetometer Registers
   ============================================*/
#define MAG_DATA_LEN 6
#define REG_ADD_MAG_WIA1    0x00
#define REG_VAL_MAG_WIA1    0x48
#define REG_ADD_MAG_WIA2    0x01
#define REG_VAL_MAG_WIA2    0x09
#define REG_ADD_MAG_ST2     0x10
#define REG_ADD_MAG_DATA    0x11
#define REG_ADD_MAG_CNTL2   0x31
#define REG_VAL_MAG_MODE_PD     0x00
#define REG_VAL_MAG_MODE_SM     0x01
#define REG_VAL_MAG_MODE_10HZ   0x02
#define REG_VAL_MAG_MODE_20HZ   0x04
#define REG_VAL_MAG_MODE_50HZ   0x05
#define REG_VAL_MAG_MODE_100HZ  0x08
#define REG_VAL_MAG_MODE_ST     0x10
/* define ICM-20948 MAG Register  end */

/* =====================================================
   ICM-20948 REGISTER MAP (For Documentation / Reference)
   Organized by USER BANK for clarity. Some of these may
   duplicate the application-facing macros defined earlier.
   ===================================================== */

/* define ICM-20948 User Banks*/
#define USER_BANK_0		(0x00)
#define USER_BANK_1		(0x10)
#define USER_BANK_2		(0x20)
#define USER_BANK_3		(0x30)

/* define ICM-20948 Device I2C address*/
#define I2C_ADD_ICM20948               0xD0
#define I2C_ADD_ICM20948_AK09916       0x0C
#define I2C_ADD_ICM20948_AK09916_READ  0x80
#define I2C_ADD_ICM20948_AK09916_WRITE 0x00

/* ===========================
   User Bank 0 Register Map
   =========================== */
#define WHO_AM_I  		   0x00 // [ICM20948] Should return 0xEA
#define USER_CTRL          0x03 // Bit 7 enable DMP, bit 3 reset DMP
#define LP_CONFIG		   0x05
#define PWR_MGMT_1         0x06 // Device defaults to the SLEEP mode
#define PWR_MGMT_2         0x07
#define INT_PIN_CFG        0x0F
#define INT_ENABLE         0x10
#define INT_ENABLE_1	   0x11
#define INT_ENABLE_2	   0x12
#define INT_ENABLE_3	   0x13
#define I2C_MST_STATUS     0x17
#define INT_STATUS         0x19
#define INT_STATUS_1	   0x1A
#define INT_STATUS_2	   0x1B
#define INT_STATUS_3	   0x1C
#define DELAY_TIMEH		   0x28
#define DELAY_TIMEL		   0x29
#define ACCEL_XOUT_H       0x2D
#define ACCEL_XOUT_L       0x2E
#define ACCEL_YOUT_H       0x2F
#define ACCEL_YOUT_L       0x30
#define ACCEL_ZOUT_H       0x31
#define ACCEL_ZOUT_L       0x32
#define GYRO_XOUT_H        0x33
#define GYRO_XOUT_L        0x34
#define GYRO_YOUT_H        0x35
#define GYRO_YOUT_L        0x36
#define GYRO_ZOUT_H        0x37
#define GYRO_ZOUT_L        0x38
#define TEMP_OUT_H         0x39
#define TEMP_OUT_L         0x3A
#define EXT_SENS_DATA_00   0x3B
#define EXT_SENS_DATA_01   0x3C
#define EXT_SENS_DATA_02   0x3D
#define EXT_SENS_DATA_03   0x3E
#define EXT_SENS_DATA_04   0x3F
#define EXT_SENS_DATA_05   0x40
#define EXT_SENS_DATA_06   0x41
#define EXT_SENS_DATA_07   0x42
#define EXT_SENS_DATA_08   0x43
#define EXT_SENS_DATA_09   0x44
#define EXT_SENS_DATA_10   0x45
#define EXT_SENS_DATA_11   0x46
#define EXT_SENS_DATA_12   0x47
#define EXT_SENS_DATA_13   0x48
#define EXT_SENS_DATA_14   0x49
#define EXT_SENS_DATA_15   0x4A
#define EXT_SENS_DATA_16   0x4B
#define EXT_SENS_DATA_17   0x4C
#define EXT_SENS_DATA_18   0x4D
#define EXT_SENS_DATA_19   0x4E
#define EXT_SENS_DATA_20   0x4F
#define EXT_SENS_DATA_21   0x50
#define EXT_SENS_DATA_22   0x51
#define EXT_SENS_DATA_23   0x52
#define FIFO_EN_1          0x66
#define FIFO_EN_2          0x67
#define FIFO_RST		   0x68
#define FIFO_MODE		   0x69
#define FIFO_COUNTH        0x70
#define FIFO_COUNTL        0x71
#define FIFO_R_W           0x72
#define DATA_RDY_STATUS	   0x74
#define FIFO_CFG		   0x76
#define REG_BANK_SEL	   0x7F
/* USER Bank 0 Map End */


/* ===========================
   User Bank 1 Register Map
   =========================== */
#define SELF_TEST_X_GYRO  			0x02
#define SELF_TEST_Y_GYRO  			0x03
#define SELF_TEST_Z_GYRO  			0x04
#define SELF_TEST_X_ACCEL 			0x0E
#define SELF_TEST_Y_ACCEL 			0x0F
#define SELF_TEST_Z_ACCEL 			0x10
#define XA_OFFSET_H       			0x14
#define XA_OFFSET_L       			0x15
#define YA_OFFSET_H       			0x17
#define YA_OFFSET_L       			0x18
#define ZA_OFFSET_H       			0x1A
#define ZA_OFFSET_L       			0x1B
#define TIMEBASE_CORRECTION_PLL		0x28
/* USER Bank 1 Map End */


/* ===========================
   User Bank 2 Register Map
   =========================== */
#define GYRO_SMPLRT_DIV        	0x00
#define GYRO_CONFIG_1      		0x01
#define GYRO_CONFIG_2      		0x02
#define XG_OFFSET_H       		0x03  // User-defined trim values for gyroscope
#define XG_OFFSET_L       		0x04
#define YG_OFFSET_H       		0x05
#define YG_OFFSET_L       		0x06
#define ZG_OFFSET_H       		0x07
#define ZG_OFFSET_L       		0x08
#define ODR_ALIGN_EN			0x09
#define ACCEL_SMPLRT_DIV_1     	0x10
#define ACCEL_SMPLRT_DIV_2     	0x11
#define ACCEL_INTEL_CTRL		0x12
#define ACCEL_WOM_THR			0x13 // (could be WOM_THR)
#define ACCEL_CONFIG      		0x14
#define ACCEL_CONFIG_2     		0x15 // (could be ACCEL_CONFIG2)
#define FSYNC_CONFIG			0x52
#define TEMP_CONFIG				0x53
#define MOD_CTRL_USR			0x54
/* USER Bank 2 Map End */


/* ===========================
   User Bank 3 Register Map
   =========================== */
#define I2C_MST_ODR_CONFIG		0x00
#define I2C_MST_CTRL       		0x01
#define I2C_MST_DELAY_CTRL 		0x02
#define I2C_SLV0_ADDR      		0x03
#define I2C_SLV0_REG       		0x04
#define I2C_SLV0_CTRL      		0x05
#define I2C_SLV0_DO        		0x06
#define I2C_SLV1_ADDR      		0x07
#define I2C_SLV1_REG       		0x08
#define I2C_SLV1_CTRL      		0x09
#define I2C_SLV1_DO        		0x0A
#define I2C_SLV2_ADDR      		0x0B
#define I2C_SLV2_REG       		0x0C
#define I2C_SLV2_CTRL      		0x0D
#define I2C_SLV2_DO        		0x0E
#define I2C_SLV3_ADDR      		0x0F
#define I2C_SLV3_REG       		0x10
#define I2C_SLV3_CTRL      		0x11
#define I2C_SLV3_DO        		0x12
#define I2C_SLV4_ADDR      		0x13
#define I2C_SLV4_REG       		0x14
#define I2C_SLV4_CTRL      		0x15
#define I2C_SLV4_DO        		0x16
#define I2C_SLV4_DI        		0x17
/* USER Bank 3 Map End */

/* ===========================
   Task and Stack size
   =========================== */
#define ICM20948_TASK_PRIO   3
#define ICM20948_STK_SIZE    256

/* ===========================
   Data Structure
   =========================== */
/**
 * @brief Structure representing the ICM20948 sensor device and its data.
 */
typedef struct {
    I2C_HandleTypeDef *i2cHandle;  /**< Pointer to the HAL I2C handle used for communication */
    float acc[3];                  /**< Latest accelerometer readings [X, Y, Z] in m/s² */
    float gyro[3];                 /**< Latest gyroscope readings [X, Y, Z] in degrees per second (°/s) */
    int16_t mag[3];                /**< Latest raw magnetometer readings [X, Y, Z] */
    float temp_C;                  /**< Latest temperature reading in degrees Celsius */
    float acc_bias[3];             /**< Accelerometer bias offsets for calibration [X, Y, Z] */
    float gyro_bias[3];            /**< Gyroscope bias offsets for calibration [X, Y, Z] */
    float mag_bias[3];             /**< Magnetometer bias offsets for calibration [X, Y, Z] */
    float mag_scale[3];            /**< Magnetometer scaling factors for calibration [X, Y, Z] */
    float q[4];                   /**< Orientation quaternion [w, x, y, z] representing sensor's orientation */
} ICM20948;

/* ===========================
   Function Prototypes
   =========================== */

uint8_t* IMU_Initialise(ICM20948 *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef IMU_WriteOneByte(ICM20948 *dev, uint8_t reg, uint8_t data);
HAL_StatusTypeDef IMU_ReadOneByte(ICM20948 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef Gyro_calibrate(ICM20948 *dev);

HAL_StatusTypeDef IMU_TempRead(ICM20948 *dev);
HAL_StatusTypeDef IMU_AccelRead(ICM20948 *dev);
HAL_StatusTypeDef IMU_GyroRead(ICM20948 *dev);

HAL_StatusTypeDef Mag_init(ICM20948 *dev);
HAL_StatusTypeDef Mag_read(ICM20948 *dev);

void magCalICM20948(ICM20948 *dev, float *bias_dest, float *scale_dest);
void MahonyQuaternionUpdate(ICM20948 *dev, float ax, float ay, float az,
                           float gx, float gy, float gz,
                           float mx, float my, float mz,
                           float deltat);

const float* getQ(ICM20948 *dev);

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif //__ICM_20948_H
