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

	#define LEFT_ENCODER_SCALE 1.0
	#define RIGHT_ENCODER_SCALE 0.996  // adjust based on reference choice
	#define REAR_WHEEL_ROTATION_DISTANCE (3.142 * 6.5)
	#define ENCODER_PULSES_PER_WHEEL_ROTATION 1560
	#define DISTANCE_PER_ENCODER_PULSE (REAR_WHEEL_ROTATION_DISTANCE / ENCODER_PULSES_PER_WHEEL_ROTATION)

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


		this->lencoder = new Encoder(
			&htim2,
			TIM_CHANNEL_ALL
		);

		this->rencoder = new Encoder(
			&htim3,
			TIM_CHANNEL_ALL
		);

		this->lmotor = new Motor(
			&htim4,
			TIM_CHANNEL_3,
			TIM_CHANNEL_4,
			4500
		);

		this->rmotor = new Motor(
			&htim9,
			TIM_CHANNEL_1,
			TIM_CHANNEL_2,
			4500
		);

		// Edit the following to control the DC motor's PID.

		float pid_param_left[3] = {
			2.1,  //3.1 0.25
			0.0,
			0.1
		};


		float pid_param_right[3] = {
			3.1,  //3.1 0.35
			0.0,
			0.1
		};

		float pid_param_sync[3] = {
			0,
			0,
			0
		};

		PID_init(&this->left_pid, PID_POSITION, pid_param_left, 7000, 7000);
		PID_init(&this->right_pid, PID_POSITION, pid_param_right, 7000, 7000);

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
//		servo->turnLeft();
//		servo->turnRight();
		servo->turnFront();

		/* workaround section END. henceforth refer to any "this" as "self" */


		for (;;) {
			is_task_alive_struct.motn = true;

//			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);

			osDelay(50);
			osThreadYield();

//          Uncomment the following code to make the robot move without the RPI
//          WARNING: This is just to test the robots movement, do not uncomment in production env.

			// Move straight 100 m
//			self->move(true, 100, 35, false, false);

//			osDelay(1000);

			// Move straight 100 m
//			self->move(false, 100, 35, false, false);

			// Turn right
//			self->move(true, 10, 35, false, false);
//			self->turn(true, true, false, 90);
//			self->turn(true, false, false, 90);
//			self->move(true, 10, 35, false, false);

			// Turn left
//			self->move(true, 10, 35, false, false);
//			self->turn(false, true, false, 90); // fwd left
//		    self->turn(false, false, false, 90);
//			self->move(true, 10, 35, false, false);

			//while(1){} // uncomment this code if you are using any of the test code above.


			if (osMessageQueueGetCount(ctx->mailbox.queue) > 0) {
				AppParser::MOTION_PKT_t pkt;
				osMessageQueueGet(ctx->mailbox.queue, &pkt, 0, 5);
				char buffer[20] = { 0 };
				sprintf((char*) &buffer, "cmd:%ld, arg:%ld\r\n", (uint32_t) pkt.cmd, pkt.arg);
				//HAL_UART_Transmit(&huart3, (uint8_t*) buffer, sizeof(buffer), 10);

				if (pkt.cmd == AppParser::MOTION_CMD::MOVE_FWD) {
					servo->turnFront();
					self->move(true, pkt.arg, 35, pkt.is_crawl, pkt.linear);

				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_BWD) {
					servo->turnFront();
					self->move(false, pkt.arg, 35, pkt.is_crawl, pkt.linear);

				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_LEFT_FWD) {
					self->turn(false, true, pkt.linear, pkt.arg);

				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_RIGHT_FWD) {
					self->turn(true, true, pkt.linear, pkt.arg);

				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_LEFT_BWD) {
					self->turn(false, false, pkt.linear, pkt.arg);

				} else if (pkt.cmd == AppParser::MOTION_CMD::MOVE_RIGHT_BWD){
					self->turn(true, false, pkt.linear, pkt.arg);
				}
			}
		}
	}

	void MotionController::moveConstantPWM(bool isFwd, uint32_t speed, uint32_t duration_ms) {
	    emergency = false;
	    servo->turnFront();

	    // Apply constant PWM
	    lmotor->setSpeed(speed, isFwd);
	    rmotor->setSpeed(speed, isFwd);

	    // Run for fixed time
	    uint32_t timeStart = HAL_GetTick();
	    while (HAL_GetTick() - timeStart < duration_ms) {
	        if (emergency) break;
	        osDelay(10);
	        osThreadYield();
	    }

	    // Stop motors
	    lmotor->halt();
	    rmotor->halt();
	}

	void MotionController::move(bool isFwd, uint32_t arg, uint32_t speed, bool isCrawl, bool nostop) {
		emergency = false;
		sensor_data.is_moving = true;
		servo->turnFront();

		lmotor->setSpeed(speed, isFwd);
		rmotor->setSpeed(speed, isFwd);

		if (isCrawl)
		{
			lmotor->setSpeed(35, isFwd);
			rmotor->setSpeed(35, isFwd);
		}

		uint32_t l_encoder_count = lencoder->getCount();
		uint32_t r_encoder_count = rencoder->getCount();
		double target = (double) arg / DISTANCE_PER_ENCODER_PULSE;

		double cur_left = 0, cur_right = 0;
		float count_left = 0, count_right = 0;

		sensor_data.target = target;
		sensor_data.cur_left = cur_left;
		sensor_data.cur_right = cur_right;

//		 OLED_ShowString(0, 10, (uint8_t*)"Entered move()");
//		 OLED_Refresh_Gram();


		double speed_error = 0;
		do {

//    		OLED_ShowString(0, 20, (uint8_t*)"looping...");
//			OLED_Refresh_Gram();

			count_left = (double) lencoder->getDelta(l_encoder_count, lencoder->getCount());
			count_right = (double) rencoder->getDelta(r_encoder_count, rencoder->getCount());


			cur_left += count_left * LEFT_ENCODER_SCALE;
			cur_right += count_right * RIGHT_ENCODER_SCALE;
			speed_error += (count_left - count_right);

			if (!isCrawl && !nostop) {

				// Close to target check
				if (cur_left > target - 2000 || cur_right > target - 2000) {


					float pid_left = map(target - cur_left, 2000, 330, 35, 15);
					float pid_right = map(target - cur_left, 2000, 330, 35, 15);

					lmotor->setSpeed(pid_left, isFwd);
					rmotor->setSpeed(pid_right, isFwd);

				}

				// Use PID
				else {

					float pid_left = PID_calc(&this->left_pid, target - cur_left, target);
					float pid_right = PID_calc(&this->right_pid, target - cur_right, target);
					float pid_left_d = PID_calc(&this->sync_left_pid, speed_error, 0);
					float pid_right_d = PID_calc(&this->sync_right_pid, -speed_error, 0);

					// Update the speed
					lmotor->_setDutyCycleVal((uint32_t) ((pid_left + pid_left_d) > 1000 ?(pid_left + pid_left_d) : 1000), isFwd);
					rmotor->_setDutyCycleVal((uint32_t) ((pid_right + pid_right_d) > 1000 ?(pid_right + pid_right_d) : 1000), isFwd);
				}
			}

			l_encoder_count = lencoder->getCount();
			r_encoder_count = rencoder->getCount();

			if ((cur_left > target && cur_right > target) || emergency)
			{
				sensor_data.last_halt_val = (uint32_t) (cur_left>cur_right?cur_right:cur_left) * DISTANCE_PER_ENCODER_PULSE;
				sensor_data.cur_left = cur_left;
				sensor_data.cur_right = cur_right;
				lmotor->halt();
				rmotor->halt();
				break;
			}

			osDelay(10);
			sensor_data.cur_left = cur_left;
			sensor_data.cur_right = cur_right;
//			char buf[50];
//			snprintf(buf, sizeof(buf), "L:%3.2f R:%3.2f T:%3.2f", cur_left, cur_right, target);
//			OLED_ShowString(0, 30, (uint8_t*)buf);
//			OLED_Refresh_Gram();
			osThreadYield();

		} while (1);


		emergency = false;
		lmotor->halt();
		rmotor->halt();
		sensor_data.is_moving = false;
	}

	void MotionController::turn(bool isRight, bool isFwd, bool arc, uint32_t arg) {
		sensor_data.is_moving = true;
		emergency = false;
		isRight ? servo->turnRight() : servo->turnLeft();

		isRight ? lmotor->setSpeed(51, isFwd) : lmotor->setSpeed(20, isFwd);
		isRight ? rmotor->setSpeed(20, isFwd) : rmotor->setSpeed(51, isFwd);

		if(arc) // arc increases turn radius
		{
			isRight ? lmotor->setSpeed(55, isFwd) : lmotor->setSpeed(20, isFwd);
			isRight ? rmotor->setSpeed(20, isFwd) : rmotor->setSpeed(55, isFwd);
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
				if(isRight) lmotor->setSpeed((uint32_t)map(abs(target_yaw - cur), 45, 0, 30, 15), isFwd);

				else rmotor->setSpeed((uint32_t)map(abs(target_yaw - cur), 45, 0, 30, 15), isFwd);
			}
			else if(fmod(abs(abs(target_yaw) - abs(cur)), 180) < 45 )
			{
				if(isRight) lmotor->setSpeed((uint32_t)map(fmod(abs(abs(target_yaw) - abs(cur)), 180), 45, 0, 30, 15), isFwd);

				else rmotor->setSpeed((uint32_t)map(fmod(abs(abs(target_yaw) - abs(cur)), 180), 45, 0, 30, 15), isFwd);
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
			if (last_target_dist < abs(target_yaw - cur) && abs(target_yaw - cur) < 15) {
				lmotor->halt();
				rmotor->halt();
				break;
			}
			else last_target_dist = abs(target_yaw - cur);

			if (abs(target_yaw - cur) <= 0.375 || (abs(target_yaw - cur) <= 1.5 && arc) || (HAL_GetTick() - timeStart) > 10000)
			{
				sensor_data.last_halt_val = ((uint32_t)abs(target_yaw - cur)) %180;
				lmotor->halt();
				rmotor->halt();
				break;
			}

			sensor_data.last_halt_val = arg;
			osDelay(2);
			osThreadYield(); // need to ensure yield for the sensortask

		} while (1);

		emergency = false;
		lmotor->halt();
		rmotor->halt();
		sensor_data.is_moving = false;
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

	bool Motor::setSpeed(uint32_t percent, bool isFwd) {
		if (percent > 100){
			return false;
		}
		uint32_t value = this->max_period / 100 * percent;
		this->cur_value = value;

		if (isFwd){
			__HAL_TIM_SetCompare(this->htimer, this->channelA, value);
			__HAL_TIM_SetCompare(this->htimer, this->channelB, 1);
		} else {
			__HAL_TIM_SetCompare(this->htimer, this->channelA, 1);
			__HAL_TIM_SetCompare(this->htimer, this->channelB, value);
		}

		return true;
	}

	bool Motor::_setDutyCycleVal(uint32_t dc, bool isFwd) {
		if (dc > this->max_period){
			return false;
		}
		this->cur_value = dc;

		if (isFwd){
			__HAL_TIM_SetCompare(this->htimer, this->channelA, dc);
			__HAL_TIM_SetCompare(this->htimer, this->channelB, 1);
		} else {
			__HAL_TIM_SetCompare(this->htimer, this->channelA, 1);
			__HAL_TIM_SetCompare(this->htimer, this->channelB, dc);
		}

		return true;
	}

	void Motor::halt() {
		__HAL_TIM_SET_COMPARE(this->htimer, this->channelA, 1);
		__HAL_TIM_SET_COMPARE(this->htimer, this->channelB, 1);
	}

	void Motor::setForward() {
		__HAL_TIM_SetCompare(this->htimer, this->channelA, this->max_period);
		__HAL_TIM_SetCompare(this->htimer, this->channelB, 1);
	}

	void Motor::setBackward() {
		__HAL_TIM_SetCompare(this->htimer, this->channelA, 1);
		__HAL_TIM_SetCompare(this->htimer, this->channelB, this->max_period);
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
