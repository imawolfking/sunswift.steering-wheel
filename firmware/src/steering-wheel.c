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

#include <string.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/can.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>

/* Do some general setur */
void setup(void) {
	GPIO_Init();
	GPIO_SetDir(2,8,1); //Green LED, Out
	GPIO_SetDir(2,7,1); //Yel LED, Out
} // setup

/* This is your main function! You should have an infinite loop in here that
 * does all the important stuff your node was designed for */
int main(void) {
	setup();

	scandal_init();

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */

	/* Set LEDs to known states */
	red_led(0);
	yellow_led(1);

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

		if(sc_get_timer() >= one_sec_timer + 1000) {
			/* Send a channel message with a blerg value at low priority on channel 0 */
			scandal_send_channel(TELEM_LOW, /* priority */
									0,      /* channel num */
									0xaa   /* value */
			);

			/* Twiddle the LEDs */
			toggle_yellow_led();
			toggle_red_led();

			/* Update the timer */
			one_sec_timer = sc_get_timer();
		}
	}
}
