/*
 * wheel_control.h
 *
 *  Created on: Mar 9, 2020
 *      Author: linux
 */

#ifndef INC_WHEEL_CONTROL_H_
#define INC_WHEEL_CONTROL_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "kalman_filter.h"
#include "pid.h"

enum WheelStatus{
	Stand = 0,
	TurnCkw,
	TurnC_ckw,
	Stoping,
	Stoped,
};

typedef struct{
	GPIO_TypeDef *gpio_port_en;
	uint16_t gpio_pin_en;

	TIM_HandleTypeDef *htim;
	uint32_t channel_ckw; //clockwise
	uint32_t channel_c_ckw;

	GPIO_TypeDef *gpio_port;
	uint16_t gpio_pin_1;
	uint16_t gpio_pin_2;

	enum WheelStatus status;
	uint8_t status_changed;
	uint32_t rpm;

	osThreadId taskHandle;

	uint32_t encoder_delay_us;
	float current_rpm;
	float current_freq;
	uint64_t time_us;
	uint64_t last_time_us;
	PID *pid;
	KalmanFilter kf;

	UART_HandleTypeDef *huart;
}WheelControl;

void wheel_init(WheelControl *wheel, UART_HandleTypeDef *huart, GPIO_TypeDef *gpio_port_en, uint16_t GPIO_Pin_en, TIM_HandleTypeDef *htim, uint32_t Channel_ckw,
		uint32_t Channel_c_ckw, GPIO_TypeDef *gpio_port, uint16_t GPIO_Pin_1, uint16_t GPIO_Pin_2);
void turn_clockwise(WheelControl *wheel, uint32_t rpm);
void turn_counter_clockwise(WheelControl *wheel, uint32_t rpm);
void wheel_control_task(void const *wheel);
void wheel_encoder_task(WheelControl *wheel);

#endif /* INC_WHEEL_CONTROL_H_ */
