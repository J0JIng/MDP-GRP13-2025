#include "app_motion.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "main.h"
#include "app_main.h"
#include "pid.h"
#include "oled.h"
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>

namespace AppMotion {

	#define ALPHA 0.8
	#define REAR_WHEEL_RADIUS_CM 6.5/2
	#define REAR_WHEEL_ROTATION_DISTANCE (2 * 3.142 * REAR_WHEEL_RADIUS_CM)
	#define ENCODER_PULSES_PER_ROTATION 1560 * 1.045
	#define DISTANCE_PER_ENCODER_PULSE (REAR_WHEEL_ROTATION_DISTANCE / ENCODER_PULSES_PER_ROTATION)

	MotionController::MotionController(u_ctx *ctx) {
		this->ctx = ctx;
		/* Instantiate the physical devices */

	}

	void MotionController::start(void) {
		this->servo = new Servo(
			&htim12,
			TIM_CHANNEL_1,
			CENTER_POS_PWM - LEFT_DELTA,
			CENTER_POS_PWM + RIGHT_DELTA,
			CENTER_POS_PWM
		);

		//
//		this->lencoder = new Encoder(
//			&htim2,
//			TIM_CHANNEL_ALL
//		);
//
//		this->rencoder = new Encoder(
//			&htim3,
//			TIM_CHANNEL_ALL
//		);

		this->lmotor = new Motor(
			&htim4,
			TIM_CHANNEL_3,
			TIM_CHANNEL_4,
			2000
		);

		this->rmotor = new Motor(
			&htim9,
			TIM_CHANNEL_1,
			TIM_CHANNEL_2,
			2000
		);

		float pid_param_right[3] = {
			3.1,
			0.0,
			0.1
		};

		float pid_param_sync[3] = {
			12,
			3,
			1
		};

		PID_init(&this->left_pid, PID_POSITION, pid_param_right, 7500, 7500);
		PID_init(&this->right_pid, PID_POSITION, pid_param_right, 7500, 7500);
		PID_init(&this->sync_left_pid, 0, pid_param_sync, 1000, 1000);
		PID_init(&this->sync_right_pid, 0, pid_param_sync, 1000, 1000);
		emergency = false;

		instance_wrapper *wrapper_instance = new instance_wrapper();
		wrapper_instance->ctx = ctx;
		wrapper_instance->i = this;
		this->ctx->runner = osThreadNew(
				(osThreadFunc_t) MotionController::motionTask, wrapper_instance,
				&(ctx->attr));
		return;
	}

	/* MOTIONCONTROLLER LOGIC */
	void MotionController::motionTask(void *pv) {

		// workaround section START
		instance_wrapper *wrapper = static_cast<instance_wrapper*>(pv);
		u_ctx *ctx = wrapper->ctx;
		MotionController *self = wrapper->i;

		Motor *lmotor = self->lmotor;
		Motor *rmotor = self->rmotor;
		Servo *servo = self->servo;
		osDelay(4500);
		servo->turnLeft();
		servo->turnRight();
		servo->turnFront();

		/* workaround section END. henceforth refer to any "this" as "self" */


		for (;;) {
			osDelay(50);
			is_task_alive_struct.motn = true;
			servo->turnFront();
			self->move(true, 1000, 35, true, true);


			//osThreadYield();
//			if (osMessageQueueGetCount(ctx->mailbox.queue) > 0) {
//				HAL_GPIO_WritePin(Movement_Ind_Port, Movement_Ind_Pin, GPIO_PIN_SET);
//				AppParser::MOTION_PKT_t pkt;
//				osMessageQueueGet(ctx->mailbox.queue, &pkt, 0, 5);
//				char buffer[20] = { 0 };
//				sprintf((char*) &buffer, "cmd:%ld, arg:%ld\r\n", (uint32_t) pkt.cmd,
//						pkt.arg);
//				//HAL_UART_Transmit(&huart3, (uint8_t*) buffer, sizeof(buffer), 10);
//				if (pkt.cmd == AppParser::MOTION_CMD::MOVE_FWD) {
//					servo->turnFront();
//
//					self->move(true, pkt.arg, 35, pkt.is_crawl, pkt.linear);
//
//				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_BWD) {
//					servo->turnFront();
//
//					self->move(false, pkt.arg, 35, pkt.is_crawl, pkt.linear);
//
//				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_LEFT_FWD) {
//					self->turn(false, true, pkt.linear, pkt.arg);
//
//				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_RIGHT_FWD)
//					self->turn(true, true, pkt.linear, pkt.arg);
//
//				else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_LEFT_BWD) {
//					self->turn(false, false, pkt.linear, pkt.arg);
//
//				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_RIGHT_BWD)
//					self->turn(true, false, pkt.linear, pkt.arg);
//
//				else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_T2_S180R)
//								self->task2ScanAndRot(pkt.turn_opt);
//				else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_T2_S90R)
//								self->task2ScanAndReturn(pkt.turn_opt);
//				else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_T2_O1)
//					self->task2ScanAndReturn(pkt.turn_opt);
//
//			}
//			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
//			HAL_GPIO_WritePin(Movement_Ind_Port, Movement_Ind_Pin, GPIO_PIN_RESET);

		}
	}


	void MotionController::move(bool isFwd, uint32_t arg, uint32_t speed, bool isCrawl, bool nostop) {
		emergency = false;
		servo->turnFront();
		isFwd ? lmotor->setForward() : lmotor->setBackward();
		isFwd ? rmotor->setForward() : rmotor->setBackward();
		lmotor->setSpeed(speed);
		rmotor->setSpeed(speed);

		if (isCrawl)
		{
			lmotor->setSpeed(35);
			rmotor->setSpeed(35);
		}

		uint32_t timeStart = HAL_GetTick();
		uint32_t l_encoder_count = lencoder->getCount();
		uint32_t r_encoder_count = rencoder->getCount();
		double target = (double) arg / DISTANCE_PER_ENCODER_PULSE;

		double cur_left = 0, cur_right = 0;
		float count_left = 0, count_right = 0;
		double speed_error = 0;
		do {

			count_left = (double) lencoder->getDelta(l_encoder_count, lencoder->getCount());
			count_right = (double) rencoder->getDelta(r_encoder_count, rencoder->getCount());

			cur_left += count_left;
			cur_right += count_right;
			speed_error += (count_left - count_right);

			if (!isCrawl && !nostop) {

				// Close to target check
				if (cur_left > target - 2000 || cur_right > target - 2000) {
					lmotor->setSpeed(map(target - cur_left, 2000, 330, 35, 15));
					rmotor->setSpeed(map(target - cur_right, 2000, 330, 35, 15));
				}

				// Use PID
				else {
					float pid_left = PID_calc(&this->left_pid, target - cur_left, target);
					float pid_right = PID_calc(&this->right_pid, target - cur_right, target);
					float pid_left_d = PID_calc(&this->sync_left_pid, speed_error, 0);
					float pid_right_d = PID_calc(&this->sync_right_pid, -speed_error, 0);

					// Update the speed
					lmotor->_setDutyCycleVal((uint32_t) ((pid_left + pid_left_d) > 1000 ?(pid_left + pid_left_d) : 1000));
					rmotor->_setDutyCycleVal((uint32_t) ((pid_right + pid_right_d) > 1000 ?(pid_right + pid_right_d) : 1000));
				}
			}

			l_encoder_count = lencoder->getCount();
			r_encoder_count = rencoder->getCount();

			if ((cur_left > target && cur_right > target) || emergency)
			{
				sensor_data.last_halt_val = (uint32_t) (cur_left>cur_right?cur_right:cur_left) * DISTANCE_PER_ENCODER_PULSE;
				break;
			}


			osDelay(10);
			sensor_data.last_halt_val = arg;
			osThreadYield();

		} while (1);

		uint8_t buf[10] = { 0 };
		snprintf((char*) buf, sizeof(buf), "%4.0f", cur_left - cur_right);
		OLED_ShowString(85, 48, (uint8_t*) &buf);
		OLED_Refresh_Gram();
		emergency = false;
		lmotor->halt();
		rmotor->halt();
	}

	void MotionController::turn(bool isRight, bool isFwd, bool arc, uint32_t arg) {
		emergency = false;
		isRight ? servo->turnRight() : servo->turnLeft();

		isFwd ? lmotor->setForward() : lmotor->setBackward();
		isFwd ? rmotor->setForward() : rmotor->setBackward();
		isRight ? lmotor->setSpeed(51) : lmotor->setSpeed(11);
		isRight ? rmotor->setSpeed(11) : rmotor->setSpeed(51);
		if(arc) // arc increases turn radius
		{
			isRight ? lmotor->setSpeed(55) : lmotor->setSpeed(20);
			isRight ? rmotor->setSpeed(20) : rmotor->setSpeed(55);
		}
		uint32_t timeNow = HAL_GetTick();
		uint32_t timeStart = timeNow;
		uint8_t buf[30] = { 0 };
		float target_yaw = 0;
		float req = ((float) arg) ;
		float cur = sensor_data.yaw_abs; //[-179,180]
		float prev_yaw = cur;
		float last_target_dist = 99999.0f; // overshoot protection
		float bwd_diffn_delta = 0;

		if((!isRight && isFwd) || (isRight && !isFwd) ) //increase
		{
			if((req + cur) > 179) target_yaw = -180 + (req - (180 - cur));
			else target_yaw = req + cur;
		}
		else
		{
			if((cur - req) < -179) target_yaw = 180 - (req + (-180 - cur));
			else target_yaw = cur - req;
		}

		do{
			if (abs(target_yaw - cur) < 45 ) {
				if(isRight) lmotor->setSpeed((uint32_t)map(abs(target_yaw - cur), 45, 0, 30, 15));

				else rmotor->setSpeed((uint32_t)map(abs(target_yaw - cur), 45, 0, 30, 15));
			}
			else if(fmod(abs(abs(target_yaw) - abs(cur)), 180) < 45 )
			{
				if(isRight) lmotor->setSpeed((uint32_t)map(fmod(abs(abs(target_yaw) - abs(cur)), 180), 45, 0, 30, 15));

				else rmotor->setSpeed((uint32_t)map(fmod(abs(abs(target_yaw) - abs(cur)), 180), 45, 0, 30, 15));
			}

			timeNow = HAL_GetTick();
			/* Use backward differentiation algorithm here to estimate the current yaw based on time
			 * elapsed since last sample.
			 * Attempting to increase the gyro sample rate is worse because the drift errors pile up.
			 * Since we dont want to measure changes in sgn(cur - prev yaw) anyway, this method seems fine.
			 *
			 * abs(sensor_data.yaw_abs - sensor_data.yaw_abs_prev) is STEP SIZE
			 * 50 is TIME PER STEP
			 * sgn(sensor_data.yaw_abs - sensor_data.yaw_abs_prev) is DIRECTION
			 *
			 * */
			if(timeNow != sensor_data.yaw_abs_time)
				bwd_diffn_delta = abs(sensor_data.yaw_abs - sensor_data.yaw_abs_prev) * (float)(abs(timeNow - sensor_data.yaw_abs_time)/80);
			else
				bwd_diffn_delta = 0;
			cur = sensor_data.yaw_abs +  (bwd_diffn_delta * sgn(sensor_data.yaw_abs - sensor_data.yaw_abs_prev)); // already dlpf and qtn filtered
			sensor_data.yaw_cur_dbg = cur;
			prev_yaw = cur;
			//break off immediately if overshoot
			if (last_target_dist < abs(target_yaw - cur)
					&& abs(target_yaw - cur) < 15)
				break;
			else last_target_dist = abs(target_yaw - cur);

			if (abs(target_yaw - cur) <= 0.375 || (abs(target_yaw - cur) <= 1.5 && arc)
					|| (HAL_GetTick() - timeStart) > 10000)
			{
				sensor_data.last_halt_val = ((uint32_t)abs(target_yaw - cur)) %180;
				break;
			}

			sensor_data.last_halt_val = arg;
			osDelay(2);
			osThreadYield(); // need to ensure yield for the sensortask

		} while (1);

		emergency = false;
		lmotor->halt();
		rmotor->halt();
	}

	void MotionController::emergencyStop() {
		emergency = true;
	}

	/* SERVO LOGIC */
	Servo::Servo(TIM_HandleTypeDef *ctrl, uint32_t channel, uint32_t min,
			uint32_t max, uint32_t center) {
		this->htimer = ctrl;
		this->channel = channel;
		this->MIN_PWM = min;
		this->MAX_PWM = max;
		this->CTR_PWM = center;
		HAL_TIM_PWM_Start(ctrl, channel);
	}

	void Servo::turnLeft() {
		this->htimer->Instance->CCR1 = MIN_PWM;
		osDelay(TURN_DELAY_MS);

	}
	void Servo::turnRight() {
		this->htimer->Instance->CCR1 = MAX_PWM;
		osDelay(TURN_DELAY_MS);
	}

	void Servo::turnFront() {
		this->htimer->Instance->CCR1 = CTR_PWM;
		osDelay(TURN_DELAY_MS);
	}


	/* MOTOR LOGIC */
	Motor::Motor(TIM_HandleTypeDef *ctrl,
			uint32_t channelA,
			uint32_t channelB,
			uint32_t max_period
	) {

		this->htimer = ctrl;
		this->channelA = channelA;
		this->channelB = channelB;
		this->max_period = max_period;
		this->cur_value = max_period;

		HAL_TIM_PWM_Start(ctrl, channelA);
		HAL_TIM_PWM_Start(ctrl, channelB);
	}

	bool Motor::setSpeed(uint32_t percent) {
		if (percent > 100){
			return false;
		}
		uint32_t value = this->max_period / 100 * percent;
		this->cur_value = value;
		return true;
	}

	bool Motor::_setDutyCycleVal(uint32_t dc) {
		if (dc > this->max_period){
			return false;
		}
		this->cur_value = dc;
		return true;
	}

	void Motor::halt() {
		__HAL_TIM_SET_COMPARE(this->htimer, this->channelA, 1);
		__HAL_TIM_SET_COMPARE(this->htimer, this->channelB, 1);
	}

	void Motor::setForward() {
	__HAL_TIM_SetCompare(this->htimer, this->channelA, 1);
	__HAL_TIM_SetCompare(this->htimer, this->channelB, this->cur_value);
	}

	void Motor::setBackward() {
	__HAL_TIM_SetCompare(this->htimer, this->channelA, this->cur_value);
	__HAL_TIM_SetCompare(this->htimer, this->channelB, 1);
	}


	/* ENCODER LOGIC */
	Encoder::Encoder(TIM_HandleTypeDef *ctrl, uint32_t channel) {

		this->htimer = ctrl;
		this->channel = channel;
		HAL_TIM_Encoder_Start(ctrl, channel);

	}

	uint32_t Encoder::getCount(void) {
		return (uint32_t) __HAL_TIM_GET_COUNTER(this->htimer);
	}

	uint32_t Encoder::getDelta(uint32_t ct1, uint32_t ct2) {
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(this->htimer)) {
			if (ct2 <= ct1) {
				return ct1 - ct2;
			} else {
				return (65535 - ct2) + ct1;
			}
		} else {
			if (ct2 >= ct1) {
				return ct2 - ct1;
			} else {
				return (65535 - ct1) + ct2;
			}
		}
	}

}
