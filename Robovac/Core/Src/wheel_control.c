/*
 * wheel_control.c
 *
 *  Created on: Mar 9, 2020
 *      Author: linux
 */

#include "wheel_control.h"
#include <stdlib.h>
#include <stdio.h>

void pid_task(const void *wheel) {
	WheelControl *w = (WheelControl*) wheel;

	double compare = 0;

	while (1) {
		if (w->time_us - w->last_time_us > 50000) {
			w->encoder_delay_us = 0;
			w->current_rpm = 0;
			w->current_freq = 0;
			//w->status = Stand;
			//w->status_changed = 1;
		}
		compare = pid_calc(w->pid, w->current_rpm, w->rpm);
		if (w->status == TurnCkw) {
			__HAL_TIM_SET_COMPARE(w->htim, w->channel_ckw, compare);
		} else if (w->status == TurnC_ckw) {
			__HAL_TIM_SET_COMPARE(w->htim, w->channel_c_ckw, compare);
		}



		osDelay(100);
	}
}

void wheel_init(WheelControl *wheel, UART_HandleTypeDef *huart,
		GPIO_TypeDef *gpio_port_en, uint16_t GPIO_Pin_en,
		TIM_HandleTypeDef *htim, uint32_t Channel_ckw, uint32_t Channel_c_ckw,
		GPIO_TypeDef *gpio_port, uint16_t GPIO_Pin_1, uint16_t GPIO_Pin_2) {

	wheel->huart = huart;
	kalman_filter_init(&wheel->kf, 1, 500, 0, 0);

	wheel->pid = pid_init(0.02, 0.02, 0, 0, TIM4->ARR);

	wheel->gpio_port_en = gpio_port_en;
	wheel->gpio_pin_en = GPIO_Pin_en;

	wheel->htim = htim;
	wheel->channel_ckw = Channel_ckw;
	wheel->channel_c_ckw = Channel_c_ckw;

	wheel->gpio_port = gpio_port;
	wheel->gpio_pin_1 = GPIO_Pin_1;
	wheel->gpio_pin_2 = GPIO_Pin_2;

	wheel->status_changed = 1;
	wheel->status = Stand;
	wheel->rpm = 0;

	osThreadDef(wheelTask, wheel_control_task, osPriorityNormal, 0, 128);
	wheel->taskHandle = osThreadCreate(osThread(wheelTask), (void*) wheel);

	osThreadDef(encTask, pid_task, osPriorityNormal, 0, 128);
	wheel->taskHandle = osThreadCreate(osThread(encTask), (void*) wheel);

	//return wheel;
}

void turn_clockwise(WheelControl *wheel, uint32_t rpm) {
	wheel->status_changed = 1;
	wheel->status = TurnCkw;
	wheel->rpm = rpm;
}

void turn_counter_clockwise(WheelControl *wheel, uint32_t rpm) {
	wheel->status_changed = 1;
	wheel->status = TurnC_ckw;
	wheel->rpm = rpm;
}

void wheel_control_task(void const *wheel) {
	WheelControl *w = (WheelControl*) wheel;
	while (1) {
		if (w->status_changed) {
			switch (w->status) {
			case Stand:
				HAL_TIM_PWM_Stop(w->htim, w->channel_ckw);
				HAL_TIM_PWM_Stop(w->htim, w->channel_c_ckw);

				setGPIOMode(w->gpio_port_en, w->gpio_pin_en,
				GPIO_MODE_OUTPUT_PP);
				HAL_GPIO_WritePin(w->gpio_port_en, w->gpio_pin_en,
						GPIO_PIN_SET);

				setGPIOMode(w->gpio_port, w->gpio_pin_1, GPIO_MODE_OUTPUT_PP);
				HAL_GPIO_WritePin(w->gpio_port, w->gpio_pin_1, GPIO_PIN_RESET);
				setGPIOMode(w->gpio_port, w->gpio_pin_2, GPIO_MODE_OUTPUT_PP);
				HAL_GPIO_WritePin(w->gpio_port, w->gpio_pin_2, GPIO_PIN_RESET);
				w->status_changed = 0;
				break;
			case TurnCkw:
				HAL_TIM_PWM_Stop(w->htim, w->channel_c_ckw);
				HAL_TIM_PWM_Start(w->htim, w->channel_ckw);
				setGPIOMode(w->gpio_port, w->gpio_pin_1, GPIO_MODE_INPUT);
				HAL_GPIO_WritePin(w->gpio_port, w->gpio_pin_1, GPIO_PIN_RESET);
				setGPIOMode(w->gpio_port, w->gpio_pin_2, GPIO_MODE_OUTPUT_PP);
				HAL_GPIO_WritePin(w->gpio_port, w->gpio_pin_2, GPIO_PIN_SET);
				w->status_changed = 0;
				w->last_time_us = w->time_us;
				//w->current_rpm = 0;
				//__HAL_TIM_SET_COMPARE(w->htim, w->channel_ckw, 1700);
				break;
			case TurnC_ckw:
				HAL_TIM_PWM_Stop(w->htim, w->channel_ckw);
				HAL_TIM_PWM_Start(w->htim, w->channel_c_ckw);
				setGPIOMode(w->gpio_port, w->gpio_pin_2, GPIO_MODE_INPUT);
				HAL_GPIO_WritePin(w->gpio_port, w->gpio_pin_2, GPIO_PIN_RESET);
				setGPIOMode(w->gpio_port, w->gpio_pin_1, GPIO_MODE_OUTPUT_PP);
				HAL_GPIO_WritePin(w->gpio_port, w->gpio_pin_1, GPIO_PIN_SET);
				w->status_changed = 0;
				w->last_time_us = w->time_us;
				//w->current_rpm = 0;
				//__HAL_TIM_SET_COMPARE(w->htim, w->channel_c_ckw, 1700);
				break;
			case Stoping:
				/*while(rpm > 0)
				 {

				 }*/

				break;
			}

		} /*else {

		 }*/
		osDelay(1);
	}
}

void wheel_encoder_task(WheelControl *w) {
	//__HAL_TIM_SET_COMPARE(wheel->htim, wheel->channel_ckw, wheel->rpm);
	w->encoder_delay_us = w->time_us - w->last_time_us;
	w->last_time_us = w->time_us;
	if (w->encoder_delay_us > 0) {
		w->current_freq = 1.0 / (w->encoder_delay_us * 0.000001);
		w->current_rpm = ((w->current_freq * 0.0625) * 60.0);

		if (w->current_rpm > 10000) {
		 w->current_rpm = 10000;
		 } else if (w->current_rpm < 0) {
			w->current_rpm = 0;
		}
		if (w->current_rpm < 11000 && w->current_rpm > 0) {
			//float test = w->current_rpm;

			w->current_rpm = kalman_filter_calc(&w->kf, w->current_rpm);

		}

	}

}
