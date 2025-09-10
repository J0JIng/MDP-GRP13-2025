#include "oled.h"
#include "cmsis_os2.h"
#include "app_main.h"
#include "app_display.h"
#include <cstring>
#include <cstdio>
namespace Display {

	void oledTask(void*) {

		OLED_Init();
		OLED_Clear();

		OLED_ShowString(0, 0, (uint8_t*) "TRD|OK");
		OLED_Display_On();
		OLED_DrawRect(46, 16, 51, 21, 1);
		OLED_DrawRect(46, 28, 51, 33, 1);
		OLED_DrawRect(46, 40, 51, 45, 1);
		OLED_DrawRect(46, 52, 51, 57, 1);
		OLED_DrawVerticalLine(55, 3, 58);

		OLED_ShowString(1, 13, (uint8_t*) "SELF");
		OLED_ShowString(1, 25, (uint8_t*) "PROC");
		OLED_ShowString(1, 37, (uint8_t*) "SENR");
		OLED_ShowString(1, 49, (uint8_t*) "MOTN");
		OLED_ShowString(60, 0, (uint8_t*) "gZ");
		OLED_ShowString(60, 12, (uint8_t*) "TRX QL");
		OLED_ShowString(60, 36, (uint8_t*) "YAW");
		OLED_Refresh_Gram();
		bool self = false;
		uint8_t buf[10] = { 0 };

		for (;;) {

			OLED_DrawRectWithFill(47, 17, 50, 20, self);
			OLED_DrawRectWithFill(47, 29, 50, 32, is_task_alive_struct.proc);
			OLED_DrawRectWithFill(47, 41, 50, 44, is_task_alive_struct.senr);
			OLED_DrawRectWithFill(47, 53, 50, 56, is_task_alive_struct.motn);

			memset(&buf, 0, sizeof(buf));
			snprintf((char*) buf, sizeof(buf), "%4.2f", sensor_data.imu->gyro[2]);
			if (is_task_alive_struct.senr) {
				OLED_ShowString(80, 0, (uint8_t*) &buf);

			} else {
				OLED_ShowString(80, 0, (uint8_t*) "NCAL");

			}

			memset(&buf, 0, sizeof(buf));
			snprintf((char*) buf, sizeof(buf), "%d", sensor_data.ql);
			OLED_ShowString(115, 12, (uint8_t*) &buf);

			memset(&buf, 0, sizeof(buf));
			snprintf((char*) buf, sizeof(buf), "%3.0f::%3.0f", sensor_data.ir_distL, sensor_data.ir_distR);
			OLED_ShowString(65, 24, (uint8_t*) &buf);

			memset(&buf, 0, sizeof(buf));
			snprintf((char*) buf, sizeof(buf), "%4.1f", sensor_data.yaw_abs);
			OLED_ShowString(85, 36, (uint8_t*) &buf);

			OLED_Refresh_Gram();

			self = !self;
			memset((void*) &is_task_alive_struct, 0, sizeof(isTaskAlive_t));
			osDelay(250);
		}
	}
}
