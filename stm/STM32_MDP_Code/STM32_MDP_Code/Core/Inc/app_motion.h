#ifndef INC_APP_MOTION_H_
#define INC_APP_MOTION_H_

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "app_parser.h"
#include "pid.h"

// application-layer motion logic
namespace AppMotion {

	/* Servo class that holds servo info */

	class Servo {
		public:
			Servo(TIM_HandleTypeDef *ctrl,
				  uint32_t channel,
				  uint32_t min,
				  uint32_t max,
				  uint32_t center
				  );

			~Servo() {
			}
			;
			void turnLeft();
			void turnRight();
			void turnFront();
			void turnToPos(uint32_t count);
			static const uint32_t TURN_DELAY_MS = 250UL;
		private:
			TIM_HandleTypeDef *htimer;
			uint32_t channel;
			uint32_t state;
			uint32_t MIN_PWM;
			uint32_t MAX_PWM;
			uint32_t CTR_PWM;
	};

	class Motor {
		public:
			Motor(TIM_HandleTypeDef *ctrl,
				  uint32_t channelA,
				  uint32_t channelB,
				  uint32_t max_period = 2000
				  );

			~Motor() {
			}
			;
			bool setSpeed(uint32_t percent, bool isFwd);
			bool _setDutyCycleVal(uint32_t dc, bool isFwd);
			void setForward(void);
			void setBackward(void);
			void halt(void);



		private:
			TIM_HandleTypeDef *htimer;
			uint32_t channelA;
			uint32_t channelB;
			uint32_t max_period;
			uint32_t cur_value;
	};


	/* Encoder class that holds encoder info */
	class Encoder {
		public:
			Encoder(TIM_HandleTypeDef *ctrl, uint32_t channel);
			~Encoder() {};
			uint32_t getCount();
			uint32_t getDelta(uint32_t ct1, uint32_t ct2);

		private:
			TIM_HandleTypeDef *htimer;
				uint32_t channel;
	};

	class MotionController {

		// Edit the following to control the servo's direction.

		#define CENTER_POS_PWM 757   //  756.9 //22 45
		#define LEFT_DELTA 11         // was 10
		#define RIGHT_DELTA 15        // was 10 then 15 20
		#define LEFT_POS_PWM (CENTER_POS_PWM - LEFT_DELTA)
		#define RIGHT_POS_PWM (CENTER_POS_PWM + RIGHT_DELTA)

		typedef struct {
			u_ctx* ctx;
			MotionController* i;
		} instance_wrapper; // for passing to class instance info to static task fn, workaround.


		public:

			MotionController(u_ctx *ctx);
			void start();
			void turn(bool isRight, bool isFwd, bool nostop,
					uint32_t arg);
			void moveConstantPWM(bool isFwd, uint32_t speed, uint32_t duration_ms);
			void move(bool isFwd, uint32_t arg, uint32_t speed, bool isCrawl, bool nostop);
			void emergencyStop();
			void task2ScanAndRot(bool dir);
			void task2ScanAndReturn(bool dir);
			void task2PassObstOne(bool dir);
			~MotionController() {
			}
			;
			u_ctx *ctx;
			static void motionTask(void *pv);
			static float map(float x, float in_min, float in_max, float out_min, float out_max) {
				return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
			}
			template <typename T> static int sgn(T val) {
				return (T(0) < val) - (val < T(0));
			}

		private:
			Motor *lmotor;
			Motor *rmotor;
			Servo *servo;
			Encoder *lencoder;
			Encoder *rencoder;
			pid_type_def left_pid, right_pid;
			pid_type_def sync_left_pid, sync_right_pid;
			bool emergency;
		};
}

#endif /* INC_APP_MOTION_H_ */
