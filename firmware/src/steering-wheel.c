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
#include <scandal/wavesculptor.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <project/leds_annexure.h>

#include <arch/can.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/adc.h>

#define VELOCITY_MAX 40 /* max velocity in m/s */

uint32_t  cruise = 0;                             /* are we in cruise? */
uint32_t  horn = 0;                               /* is the horn on */
uint32_t  brake = 0;                              /* is the brake on? */

uint32_t  precharging = 0;                        /* are we trying to precharge? */
uint32_t  discharging = 0;                        /* are we trying to discharge? */
uint32_t  precharged = 0;                         /* are we precharged? */
uint32_t  precharge_switch = 0;                   /* are we waiting for the button hold time? */
uint32_t  precharge_timeout = 0;                  /* have we timed out waiting? */
sc_time_t precharge_switch_on_time = 0;           /* the time someone pressed (and not released) the precharge switch */
sc_time_t precharge_discharge_request_time = 0;   /* the time we started sending out a precharge or discharge request */

float     velocity = VELOCITY_MAX;                /* what's the current velocity in m/s? */
float     bus_current = 0.0;                      /* what's our bus_current setpoint? */
float     motor_current = 0.0;                    /* what's our motor current setpoint? */
uint32_t  throttle = 0;                           /* what's the throttle paddle position? */
uint32_t  regen = 0;                              /* what's the regen paddle position? */
int32_t   current_velocity = 0.0;                 /* our current speed from the wavesculptor */

uint32_t  hazards = 1;                            /* are hazards on? */
uint32_t  hazards_state = 0;                      /* what's the current state of the hazards? */
uint32_t  left_indicator = 0;                     /* is the left indicator on? */
uint32_t  right_indicator = 0;                    /* is the right indicator on? */
uint32_t  left_indicator_state = 0;               /* what's the current state of the left indicator? */
uint32_t  right_indicator_state = 0;              /* what's the current state of the right indicator? */

extern void speed_hold_handler();
extern void speed_down_handler();
extern void speed_up_handler();
extern void horn_handler();
extern void hazards_handler();
extern void brake_handler();
extern void precharge_handler();
extern void left_indicator_handler();
extern void right_indicator_handler();

extern void init_ws_in_channels(void);
extern void handle_ws_drive_commands(float velocity, float bus_current, float motor_current);

extern void init_scandal_in_channels(void);
extern void precharge_status_handler(int32_t value, uint32_t src_time);

void setup(void) {
	GPIO_Init();

	/* LEDs */
	GPIO_SetFunction(PRCH_LED_PORT, PRCH_LED_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetFunction(CRUISE_LED_PORT, CRUISE_LED_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetFunction(REV_LED_PORT, REV_LED_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetFunction(RIGHT_LED_PORT, RIGHT_LED_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetFunction(LEFT_LED_PORT, LEFT_LED_BIT, GPIO_PIO, GPIO_MODE_NONE);

	GPIO_SetDir(PRCH_LED_PORT, PRCH_LED_BIT, 1);
	GPIO_SetDir(CRUISE_LED_PORT, CRUISE_LED_BIT, 1);
	GPIO_SetDir(REV_LED_PORT, REV_LED_BIT, 1);
	GPIO_SetDir(RIGHT_LED_PORT, RIGHT_LED_BIT, 1);
	GPIO_SetDir(LEFT_LED_PORT, LEFT_LED_BIT, 1);

	/* Switches */

	/* Cruise switch */
	GPIO_SetFunction(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &speed_hold_handler);

	/* Set speed up switch */
	GPIO_SetFunction(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &speed_up_handler);

	/* Set speed down switch */
	GPIO_SetFunction(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &speed_down_handler);

	/* Horn switch */
	GPIO_SetFunction(HORN_SWITCH_PORT, HORN_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(HORN_SWITCH_PORT, HORN_SWITCH_BIT, 0);

	/* we use a double edge here so that we can hold the horn button down */
	GPIO_RegisterInterruptHandler(HORN_SWITCH_PORT, HORN_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &horn_handler);

	/* Hazards switch */
	GPIO_SetFunction(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &hazards_handler);

	/* Left indicator switch */
	GPIO_SetFunction(LEFT_INDICATOR_SWITCH_PORT, LEFT_INDICATOR_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(LEFT_INDICATOR_SWITCH_PORT, LEFT_INDICATOR_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(LEFT_INDICATOR_SWITCH_PORT, LEFT_INDICATOR_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &left_indicator_handler);

	/* Right indicator switch */
	GPIO_SetFunction(RIGHT_INDICATOR_SWITCH_PORT, RIGHT_INDICATOR_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(RIGHT_INDICATOR_SWITCH_PORT, RIGHT_INDICATOR_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(RIGHT_INDICATOR_SWITCH_PORT, RIGHT_INDICATOR_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &right_indicator_handler);

	/* Brakes */
	GPIO_SetFunction(BRAKE_PORT, BRAKE_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(BRAKE_PORT, BRAKE_BIT, 0);

	/* we use a double edge here so that we can hold the horn button down */
	GPIO_RegisterInterruptHandler(BRAKE_PORT, BRAKE_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &brake_handler);

	/* Precharge */
	GPIO_SetFunction(PRECHARGE_SWITCH_PORT, PRECHARGE_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(PRECHARGE_SWITCH_PORT, PRECHARGE_SWITCH_BIT, 0);

	/* we use a double edge here so that we can hold the horn button down */
	GPIO_RegisterInterruptHandler(PRECHARGE_SWITCH_PORT, PRECHARGE_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &precharge_handler);

} // setup

int main(void) {
	setup();

	scandal_init();

	ADC_Init(2);
	ADC_EnableChannel(REGEN_ADC_CHANNEL);
	ADC_EnableChannel(ACCELERATOR_ADC_CHANNEL);

	init_ws_in_channels();
	init_scandal_in_channels();

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t hundred_ms_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t five_hundred_ms_timer = sc_get_timer(); /* Initialise the timer variable */

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

		float throttle_float = (float)(RIGHT_PADDLE_MAX - ((int32_t)ADCValue[ACCELERATOR_ADC_CHANNEL] - RIGHT_PADDLE_MIN)) / (float)RIGHT_PADDLE_MAX;
		float regen_float = (float)(LEFT_PADDLE_MAX - ((int32_t)ADCValue[REGEN_ADC_CHANNEL] - LEFT_PADDLE_MIN)) / (float)LEFT_PADDLE_MAX;

		/* If the throttle is below 5%, just zero it */
		if (throttle_float < 0.05)
			throttle_float = 0.0;

		/* If regen is below 5%, just zero it */
		if (regen_float < 0.05)
			regen_float = 0.0;

		/* If the throttle is above 100%, make it 100% */
		if (throttle_float > 1.0)
			throttle_float = 1.0;

		/* If regen is above 100%, make it 100% */
		if (regen_float > 1.0)
			regen_float = 1.0;

		/* calculate a percentage to send out as a CAN message for human viewing */
		throttle = (uint32_t)(throttle_float * 100.0);
		regen = (uint32_t)(regen_float * 100.0);

		/* turn on delay, the paddles seem to read crazy at turn on */
		if (sc_get_timer() > 2000) {
			if (!cruise && throttle_float > 0.0) {
				velocity = VELOCITY_MAX;
				bus_current = 1.0;
				motor_current = throttle_float;
				/* call the drive command handler */
				handle_ws_drive_commands(velocity, bus_current, motor_current);

			} else if (cruise) {
				velocity = current_velocity;
				bus_current = 1.0;
				motor_current = 1.0;
				/* call the drive command handler */
				handle_ws_drive_commands(velocity, bus_current, motor_current);
			}
		}

		/* stuff to do at 1s intervals */
		if(sc_get_timer() >= one_sec_timer + 1000) {

			/* send out the precharge channel, this is every second so that we ensure
			 * if we lose a message we don't get into a silly state */
			if (precharging)
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 1);
			else if (discharging)
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_START, 0);

			/* send out the human readable channels */
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_THROTTLE, throttle);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_REGEN, regen);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_CRUISE, cruise);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_VELOCITY, (int)(current_velocity*100.0));

			/* flash the hazards */
			if (hazards) {
				toggle_left_led();
				toggle_right_led();

				if (hazards_state) {
					hazards_state = 0;
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_LH_IND, 1);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_RH_IND, 1);
				} else {
					hazards_state = 1;
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_LH_IND, 0);
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_RH_IND, 0);
				}

			/* flash the left indicator */
			} else if (left_indicator) {
				/* if the right indicator is already on, turn if off */
				if (right_indicator)
					right_indicator = 0;

				toggle_left_led();

				if (left_indicator_state) {
					left_indicator_state = 0;
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_LH_IND, 1);
				} else {
					left_indicator_state = 1;
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_LH_IND, 0);
				}

			/* flash the right indicator */
			} else if (right_indicator) {
				/* if the left indicator is already on, turn if off */
				if (left_indicator)
					left_indicator = 0;

				toggle_right_led();

				if (right_indicator_state) {
					right_indicator_state = 0;
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_RH_IND, 1);
				} else {
					right_indicator_state = 1;
					scandal_send_channel(TELEM_LOW, STEERINGWHEEL_RH_IND, 0);
				}
			} else {
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_RH_IND, 0);
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_LH_IND, 0);
			}

			/* if we timed out waiting for a precharge or discharge, send a message */
			if (precharge_timeout > 0)
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_PRCH_TIMEOUT, 1);

			/* turn the brake lights on or off*/
			if (brake)
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_BRAKE, 1);
			else
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_BRAKE, 0);

			/* turn the horn on or off */
			if (horn)
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_HORN, 1);
			else
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_HORN, 0);

			/* Update the timer */
			one_sec_timer = sc_get_timer();
		}

		/* we timed out waiting for the smartDCDC */
		if ((precharging || discharging) && (sc_get_timer() >= precharge_discharge_request_time + PRECHARGE_DISCHARGE_TIMEOUT_MS)) {
			precharging = 0;
			discharging = 0;
			precharge_led(0);
			precharge_timeout = 20;
		}

		/* we also test for the precharge switch timeout. If we detect that the user has held the switch for long
		 * enough, we just let them have it */
		if (precharge_switch && (sc_get_timer() >= precharge_switch_on_time + PRECHARGE_SWITCH_HOLD_TIME_MS)) {
			if (precharged) {
				precharging = 0;
				discharging = 1;
			} else {
				precharging = 1;
				discharging = 0;
			}

			precharge_discharge_request_time = sc_get_timer();
			precharge_timeout = 0;
		}

		/* stuff to do at 100ms intervals */
		if(sc_get_timer() >= hundred_ms_timer + 100) {
			/* are we waiting for the button to be released */
			/* if we have a precharge timeout, flash here AND below in the 500ms timer */
			if ((precharge_switch && !precharging && !discharging) || precharge_timeout > 0)
				toggle_precharge_led();

			/* Update the timer */
			hundred_ms_timer = sc_get_timer();
		}

		/* stuff to do at 500ms intervals */
		if(sc_get_timer() >= five_hundred_ms_timer + 500) {
			/* are we waiting for a message from the smartdcdc? or have we timed out */
			if (precharging || discharging)
				toggle_precharge_led();

			if (precharge_timeout) {
				toggle_precharge_led();
				precharge_timeout--;
			}

			/* Update the timer */
			five_hundred_ms_timer = sc_get_timer();
		}
	}
}
