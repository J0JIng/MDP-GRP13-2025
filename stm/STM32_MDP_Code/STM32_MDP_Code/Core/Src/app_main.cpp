#include <app_parser.h>
#include <app_motion.h>
#include "app_main.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "ICM20948.h"
#include "app_display.h"
#include "oled.h"

#include <cmath>
#include <cstdio>

// Variables
sensorData_t sensor_data;
isTaskAlive_t is_task_alive_struct = { 0 };
bool test_run = false;

volatile uint32_t us_rise = 0, us_fall = 0;
#define US_EVT_DONE  (1U << 0)
osEventFlagsId_t us_evt;

// Function declaration
void sensorIRTask(void *pv);
void sensorUSTask(void *pv);
void sensorIMUTask(void *pv);

// ---- µs delay using DWT cycle counter (Cortex-M4) ----
static inline void DWT_Delay_Init(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline void delay_us(uint32_t us){
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock/1000000U) * us;
    while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
}

static inline float ticks_to_us_TIM8(uint32_t ticks) {
  // adjust if you’re using a different timer/prescaler
  uint32_t tim_clk = HAL_RCC_GetPCLK2Freq();        // TIM8 on APB2
  if ((RCC->CFGR & RCC_CFGR_PPRE2) && (RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) tim_clk *= 2U;
  float tick_us = ((float)(htim8.Init.Prescaler + 1U) * 1e6f) / (float)tim_clk;
  return ticks * tick_us;
}

// External handles provided by CubeMX-generated main.c
extern ADC_HandleTypeDef hadc1;   // ADC1 for 2 IR channels (scan mode, Rank1/Rank2)
extern TIM_HandleTypeDef htim8;   // TIM8 CH2 input capture for US echo


osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attr = {
		.name = "oledTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t irTaskHandle;
const osThreadAttr_t irTask_attr = {
		.name = "irTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t usTaskHandle;
const osThreadAttr_t usTask_attr = {
		.name = "usTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityBelowNormal,
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attr = {
		.name = "imuTask",
		.stack_size = 4096,
		.priority = (osPriority_t) osPriorityNormal,
};


const osThreadAttr_t procTask_attr = {
		.name = "procTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t ctrlTaskHandle;
const osThreadAttr_t ctrlTask_attr = {
		.name = "ctrlTask",
		.stack_size = 2048,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t procTaskHandle;
static u_ctx procCtx = {
		.runner = procTaskHandle,
		.attr = procTask_attr,
		.mailbox = { .queue = NULL }
};

osMessageQueueId_t ctrlQueue = osMessageQueueNew(
		10,                                  // max number of messages in queue
		sizeof(AppParser::MOTION_PKT_t),      // size of each message
		NULL
);

u_ctx ctrlCtx = {
		.runner = ctrlTaskHandle,
		.attr = ctrlTask_attr,
		.mailbox = {
				.queue = ctrlQueue
		}
};

AppMotion::MotionController controller(&ctrlCtx);
AppParser::Processor processor(&procCtx, &ctrlCtx);
AppParser::Listener listener(&procCtx);
/*****************************************************************************************/

/*
 * This function initializes the C++ stuff, called from within main() context.
 */
void initializeCPPconstructs(void) {

	sensor_data.is_allow_motor_override = true;
	sensor_data.ir_dist_th_L = 10.0f;
	sensor_data.ir_dist_th_R = 10.0f;

	us_evt = osEventFlagsNew(NULL);

	// create instance of the Task
	// 1. Processor related task
	processor.start();

	// 2. Motor related task
	controller.start();


	// 3. Display related task
	oledTaskHandle = osThreadNew(Display::oledTask, NULL, &oledTask_attr);

	// 4. Sensor related task
	imuTaskHandle = osThreadNew(sensorIMUTask, NULL, &imuTask_attr);
	irTaskHandle = osThreadNew(sensorIRTask, NULL, &irTask_attr);
	usTaskHandle = osThreadNew(sensorUSTask, NULL, &usTask_attr);

}

#define BUFFER_SIZE 4  // Buffer size for 10 samples

float irBufferL[BUFFER_SIZE]; // Buffer for left IR sensor
float irBufferR[BUFFER_SIZE]; // Buffer for right IR sensor
int bufferIndex = 0;          // Current index in the buffer
float ir_distL_Avg = 0;       // Average distance for left IR sensor
float ir_distR_Avg = 0;       // Average distance for right IR sensor


void sensorIRTask(void *pv) {

	// init sensorIRTask
    const int NSAMPLES = 16;
    const uint32_t poll_timeout = 2;  // ms per conversion

    for (;;) {
        uint32_t acc1 = 0, acc2 = 0;

        for (int i = 0; i < NSAMPLES; ++i) {
            HAL_ADC_Start(&hadc1);

            // Rank 1 -> IR Left (ADC channel configured as Rank 1 in CubeMX)
            HAL_ADC_PollForConversion(&hadc1, poll_timeout);
            uint16_t raw1 = HAL_ADC_GetValue(&hadc1);
            acc1 += raw1;

            // Rank 2 -> IR Right (ADC channel configured as Rank 2 in CubeMX)
            HAL_ADC_PollForConversion(&hadc1, poll_timeout);
            uint16_t raw2 = HAL_ADC_GetValue(&hadc1);
            acc2 += raw2;

            HAL_ADC_Stop(&hadc1);
        }

        uint16_t raw1 = acc1 / NSAMPLES;
        uint16_t raw2 = acc2 / NSAMPLES;

        // ADC -> volts (12-bit, 3.3 V ref)
        float v1 = (raw1 * 3.3f) / 4095.0f;
        float v2 = (raw2 * 3.3f) / 4095.0f;

        // Simple inverse-voltage distance fit (same as your previous main.c)
        float d1 = (v1 > 0.1f) ? (13.0f / v1 - 0.42f) : -1.0f;
        float d2 = (v2 > 0.1f) ? (13.0f / v2 - 0.42f) : -1.0f;

        // Clamp to a reasonable range (10–80 cm) to avoid spikes
        if (d1 > 80.0f) d1 = 80.0f; if (d1 > 0 && d1 < 10.0f) d1 = 10.0f;
        if (d2 > 80.0f) d2 = 80.0f; if (d2 > 0 && d2 < 10.0f) d2 = 10.0f;

        // Optional moving average using your ring buffer
        irBufferL[bufferIndex] = d1;
        irBufferR[bufferIndex] = d2;
        bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

        float sumL = 0, sumR = 0;
        for (int i = 0; i < BUFFER_SIZE; ++i) { sumL += irBufferL[i]; sumR += irBufferR[i]; }
        ir_distL_Avg = sumL / BUFFER_SIZE;
        ir_distR_Avg = sumR / BUFFER_SIZE;

        // Publish to shared sensor struct
        sensor_data.ir_distL = ir_distL_Avg;   // left IR in cm
        sensor_data.ir_distR = ir_distR_Avg;   // right IR in cm

        is_task_alive_struct.senr = true;
        osDelay(100);     // ~10 Hz
        osThreadYield();
    }
	//	for(;;){
	// add logic ...
	//	sensor_data.ir_distL = 10.0;
	//	sensor_data.ir_distR = 10.0;
	// sensor_data.usonic_dist = 10.0;
	//}
}

void sensorUSTask(void *pv) {

	// init sensorUSTask
    DWT_Delay_Init();  // precise µs timing for 10 µs TRIG pulse

    for (;;) {
      // Arm timer for rising edge + start capture IRQ
      __HAL_TIM_SET_COUNTER(&htim8, 0);
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);

      // 10–12 µs trigger pulse
      HAL_GPIO_WritePin(asy_US_TRIG_GPIO_Port, asy_US_TRIG_Pin, GPIO_PIN_SET);
      delay_us(12);
      HAL_GPIO_WritePin(asy_US_TRIG_GPIO_Port, asy_US_TRIG_Pin, GPIO_PIN_RESET);

      // Wait for ISR to flag completion (timeout ~30 ms)
      uint32_t flags = osEventFlagsWait(us_evt, US_EVT_DONE, osFlagsWaitAny, 30);
      if ((int32_t)flags >= 0) {
        // snapshot and compute
        uint32_t rise = us_rise, fall = us_fall;
        uint32_t period = htim8.Init.Period + 1U;
        uint32_t ticks  = (fall >= rise) ? (fall - rise) : (period - rise + fall);

        float pulse_us = ticks_to_us_TIM8(ticks);
        float cm = (pulse_us * 0.0343f) * 0.5f;   // speed of sound, round-trip

        // publish (writer)
        sensor_data.usonic_dist = cm;             // <-- now the display can show it
      } else {
        // timeout
        sensor_data.usonic_dist = -1.0f;
      }

      osDelay(60);  // ~16 Hz
    }
	//	for(;;){
	// add logic ...
	//
	// sensor_data.usonic_dist = 10.0;
	//}
}


// estimated orientation quaternion elements with initial conditions
float SEq_1 = 1.0f;
float SEq_2 = 0.0f;
float SEq_3 = 0.0f;
float SEq_4 = 0.0f;

void sensorIMUTask(void *pv) {

	/**Init IMU**/
	IMU_Initialise(&imu, &hi2c2);
	osDelay(400);
	Gyro_calibrate(&imu);
	Mag_init(&imu);
	sensor_data.imu = &imu;
	char sbuf[100] = { 0 };
	HAL_StatusTypeDef result;

	/**I2C scanner for debug purposes **/
	printf("Scanning I2C bus:\r\n");
	for (uint8_t addr = 1; addr < 127; addr++) {
		result = HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 10);
		if (result == HAL_OK) {
			uint16_t len = sprintf(&sbuf[0], "I2C device found at 0x%02X\r\n", addr);
			HAL_UART_Transmit(&huart3, (uint8_t*)sbuf, len, HAL_MAX_DELAY);
		}
	}
	uint32_t timeNow = HAL_GetTick();
	float DEG2RAD = 0.017453292519943295769236907684886f;

	for (;;) {
		osDelay(80); // 281hz gyro
		osThreadYield();

		IMU_AccelRead(&imu);
		IMU_GyroRead(&imu);

		quaternionUpdate(
				imu.gyro[0] * DEG2RAD,
				imu.gyro[1] * DEG2RAD,
				imu.gyro[2] * DEG2RAD,
				imu.acc[0],
				imu.acc[1],
				imu.acc[2],
				(HAL_GetTick() - timeNow) * 0.001f
				);

		timeNow = HAL_GetTick();

		imu.q[0] = SEq_1;
		imu.q[1] = SEq_2;
		imu.q[2] = SEq_3;
		imu.q[3] = SEq_4;

		sensor_data.yaw_abs_prev = sensor_data.yaw_abs;

		// yaw = atan2(2(q1​q2​+q0​q3​),q02​+q12​−q22​−q32​)
		sensor_data.yaw_abs = atan2(
				2.0f * (imu.q[1] * imu.q[2] + imu.q[0] * imu.q[3]),
				imu.q[0] * imu.q[0] + imu.q[1] * imu.q[1] - imu.q[2] * imu.q[2] - imu.q[3] * imu.q[3])
				* 57.295779513082320876798154814105f;

		sensor_data.yaw_abs_time = timeNow;
//		uint16_t len = sprintf(
//				&sbuf[0],
//				"%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\r\n",
//				imu.acc[0],imu.acc[1],imu.acc[2],
//				imu.gyro[0], imu.gyro[1],imu.gyro[2],
//				imu.q[0],
//				sensor_data.yaw_abs,
//				sensor_data.ir_distL
//				);
//
//		HAL_UART_Transmit(&huart3, (uint8_t*) sbuf, len, 10);
		is_task_alive_struct.senr = true;

	}
}

#define gyroMeasError 3.14159265358979f * (1.0f / 180.0f)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError

void quaternionUpdate(float w_x, float w_y, float w_z, float a_x, float a_y,
		float a_z, float deltat) {

	float norm;                                                   		  // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derivative from gyroscopes elements
	float f_1, f_2, f_3;                          						  // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; 			  // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; 			  // estimated direction of the gyro error

	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;

	// Normalize the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3;
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1;
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

	// Normalize the gradient
	norm = sqrt(
			SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2
					+ SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;

	// Compute the quaternion derivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

	// Compute then integrate the estimated quaternion derivative
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

	// Normalize quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	test_run = true;
//}

void _ext_sig_halt(void) {
	controller.emergencyStop();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart3) {
		listener.invoke();
	}
}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//	static int t1, t2, first=0,echo=0;
//	char buffer[15];
//
//	if(htim==&htim8){
//		if (first == 0){
//			t1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//			first=1;
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
//		} else if (first == 1){
//			t2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//			__HAL_TIM_SET_COUNTER(htim, 0);
//
//			if(t2 >= t1){
//				echo = t2 - t1;
//			} else{
//				echo = (0xffff - t1) + t2;
//			}
//			sensor_data.usonic_dist = echo * 0.0343f / 2;
//			first=0;
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
//			HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_2);
//		}
//	}
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uint32_t c = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    static uint8_t stage = 0;          // 0=wait rising, 1=wait falling

    if (stage == 0) {                   // rising
      us_rise = c;
      stage = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {                            // falling
      us_fall = c;
      stage = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      osEventFlagsSet(us_evt, US_EVT_DONE);  // notify task
    }
  }
}



