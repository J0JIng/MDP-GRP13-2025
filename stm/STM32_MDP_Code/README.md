# STM32_MDP_CODE

STM32_MDP_CODE is a FreeRTOS-based project for the STM32 platform. It demonstrates how to schedule and manage multiple sensing and actuation tasks in a real-time environment.

The following tasks are implemented:
1. StartDefaultTask – Blinks the on-board LED
2. oledTaskHandle – Handles OLED display updates
3. imuTaskHandle – Reads and processes IMU sensor data
4. motionTaskHandle – Controls motion-related operations (e.g., motors/servos)