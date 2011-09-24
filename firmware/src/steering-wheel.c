/* --------------------------------------------------------------------------                                 
    Template project main
    File name: template.c
    Author: Etienne Le Sueur
    Description: This is the steering wheel code. It makes use of the Scandal
	WaveSculptor driver. It should make no assumptions about what type of
	Waveculptor is connected. It should work with any WaveSculptor, however
	it has only been tested with the WS20 and WS22 models.

    Copyright (C) Etienne Le Sueur, 2011

    Created: 07/09/2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of the Sunswift Steering Wheel project
 * 
 * This tempalte is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with the project. If not, see <http://www.gnu.org/licenses/>.
 */

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/can.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/adc.h>

#define VELOCITY_MAX 40 /* max velocity in m/s */

uint32_t cruise = 0;
uint32_t hazards = 1;
uint32_t horn = 0;
float velocity = VELOCITY_MAX;

void speed_hold_handler();
void speed_down_handler();
void speed_up_handler();
void horn_handler();
void hazards_handler();

/* Do some general setup */
void setup(void) {
	GPIO_Init();

	/* LEDs */
	GPIO_SetFunction(PRCH_LED_PORT, PRCH_LED_BIT, GPIO_PIO);
	GPIO_SetFunction(CRUISE_LED_PORT, CRUISE_LED_BIT, GPIO_PIO);
	GPIO_SetFunction(REV_LED_PORT, REV_LED_BIT, GPIO_PIO);
	GPIO_SetFunction(RIGHT_LED_PORT, RIGHT_LED_BIT, GPIO_PIO);
	GPIO_SetFunction(LEFT_LED_PORT, LEFT_LED_BIT, GPIO_PIO);

	GPIO_SetDir(PRCH_LED_PORT, PRCH_LED_BIT, 1);
	GPIO_SetDir(CRUISE_LED_PORT, CRUISE_LED_BIT, 1);
	GPIO_SetDir(REV_LED_PORT, REV_LED_BIT, 1);
	GPIO_SetDir(RIGHT_LED_PORT, RIGHT_LED_BIT, 1);
	GPIO_SetDir(LEFT_LED_PORT, LEFT_LED_BIT, 1);

	/* Switches */

	/* Cruise switch */
	GPIO_SetFunction(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT, GPIO_PIO);
	GPIO_SetDir(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &speed_hold_handler);

	/* Set speed up switch */
	GPIO_SetFunction(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT, GPIO_PIO);
	GPIO_SetDir(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &speed_up_handler);

	/* Set speed down switch */
	GPIO_SetFunction(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT, GPIO_PIO);
	GPIO_SetDir(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &speed_down_handler);

	/* Horn switch */
	GPIO_SetFunction(HORN_SWITCH_PORT, HORN_SWITCH_PORT, GPIO_PIO);
	GPIO_SetDir(HORN_SWITCH_PORT, HORN_SWITCH_PORT, 0);

	/* we use a double edge here so that we can hold the horn button down */
	GPIO_RegisterInterruptHandler(HORN_SWITCH_PORT, HORN_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &horn_handler);

	/* Hazards switch */
	GPIO_SetFunction(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT, GPIO_PIO);
	GPIO_SetDir(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &hazards_handler);

	/* Left Regen paddle */
	GPIO_SetFunction(LEFT_PADDLE_PORT, LEFT_PADDLE_BIT, GPIO_FUNC1);

	/* Right Regen paddle */
	GPIO_SetFunction(RIGHT_PADDLE_PORT, RIGHT_PADDLE_BIT, GPIO_FUNC1);

} // setup

/* This is your main function! You should have an infinite loop in here that
 * does all the important stuff your node was designed for */
int main(void) {

	uint32_t regen_paddle_value = 0;
	uint32_t accelerator_paddle_value = 0;

	setup();

	scandal_init();

	ADC_Init(2);
	ADC_EnableChannel(REGEN_ADC_CHANNEL);
	ADC_EnableChannel(ACCELERATOR_ADC_CHANNEL);

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */

	/* Set LEDs to known states */
	left_led(1);
	right_led(1);

	scandal_delay(100); /* wait for the UART clocks to settle */

	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();

		/* Basic driver controls loop:

			precharged = 0
			velocity = 0
			motor_current = 0
			cruise = 0
			braking = 0

			while (1)

				if not precharged
					listen for precharge button press
						send to dcdc
							listen for message from dcdc
								precharged = 1

				else if precharged

					listen for precharge button press
						send to dcdc
							listen for message from dcdc
								precharged = 0

					if not braking

						listen for brake pedal sensor
							braking = sensor

						if not cruise && %accelerator > 0
							motor current = %accelerator
							listen for cruise button press
								cruise = current speed
						else If cruise
							listen for cruise button press
								cruise = 0
						else
							velocity = 0
							motor_current = 0

						if %regen > 0
							motor current = %regen

						if ws20
							velocity = m/s 
						else if ws22
							velocity = rpm

						send ws drive commands (motor_current, velocity) at least every 200ms
		*/

		ADC_Read(REGEN_ADC_CHANNEL);
		ADC_Read(ACCELERATOR_ADC_CHANNEL);

		if(sc_get_timer() >= one_sec_timer + 1000) {

			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_THROTTLE, ADCValue[ACCELERATOR_ADC_CHANNEL]);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_REGEN, ADCValue[REGEN_ADC_CHANNEL]);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_CRUISE, cruise);

			if (hazards) {
				toggle_left_led();
				toggle_right_led();
			}

			/* Update the timer */
			one_sec_timer = sc_get_timer();
		}
	}
}
