#ifndef INC_APP_MAIN_H_
#define INC_APP_MAIN_H_

// Ensure that C++ compilers don't mangle the function names, so C code can link properly
#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "ICM20948.h"

// =====================
// Data Valid Bitmasks
// These constants are used as flags in sensorData_t.d_valid to indicate
// which sensor data is valid. Each bit represents one sensor.
const uint32_t AC_VALID_MASK      = 0x1;    // Accelerometer data valid
const uint32_t GY_VALID_MASK      = 0x2;    // Gyroscope data valid
const uint32_t MG_VALID_MASK      = 0x4;    // Magnetometer data valid
const uint32_t TM_VALID_MASK      = 0x8;    // Temperature data valid
const uint32_t IR_L_VALID_MASK    = 0x10;   // Left IR sensor valid
const uint32_t IR_R_VALID_MASK    = 0x20;   // Right IR sensor valid
const uint32_t USONIC_VALID_MASK  = 0x40;   // Ultrasonic sensor valid

// =====================
// Function prototypes
void sensorIRTask(void *pv);                 // Task to handle IR sensor
void sensorUSTask(void *pv);				 // Task to handle ultra sonic sensor
void sensorIMUTask(void *pv);                // Task to handle IMU sensor


void UARTReceiveTask(void const * argument); // Task to handle UART reception
void initializeCPPconstructs(void);          // Function to initialize C++ objects if needed
void _ext_sig_halt(void);
void processorTask(void const *);
void quaternionUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float deltat);

// =====================
// Sensor Data Structure
typedef struct {
    ICM20948 *imu;                     // Pointer to IMU object
    float     ir_distL;                // Left IR distance measurement
    float     ir_distR;                // Right IR distance measurement
    float     usonic_dist;             // Ultrasonic distance measurement
    float     yaw_abs;                 // Absolute yaw angle
    float     yaw_abs_prev;            // Previous yaw angle (for delta/sgn calculations)
    uint32_t  yaw_abs_time;            // Timestamp of yaw measurement
    float     yaw_cur_dbg;             // Debug value for current yaw
    uint32_t  d_valid;                 // Bitmask indicating which sensor data is valid
    uint32_t  ql;                      // Length of UART incoming queue
    bool      is_allow_motor_override; // Allows motor commands to be overridden if IR < threshold
    float     ir_dist_th_L;            // Threshold for left IR sensor
    float     ir_dist_th_R;            // Threshold for right IR sensor
    uint32_t  last_halt_val;           // Stores magnitude of last motion command executed
    float     cur_left;
    float     cur_right;
    float     target;
} sensorData_t;

// =====================
// Task Alive Structure
typedef struct {
    bool proc;   // Processing task alive
    bool lsnr;   // Listener task alive
    bool senr;   // Sensor task alive
    bool motn;   // Motion task alive
} isTaskAlive_t;

// =====================
// External global variables
extern sensorData_t sensor_data;           // Global instance of sensor data
extern isTaskAlive_t is_task_alive_struct; // Global task alive flags
extern bool test_run;                      // Flag for test mode

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INC_APP_MAIN_H_ */
