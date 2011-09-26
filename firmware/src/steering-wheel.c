/* --------------------------------------------------------------------------                                 
    Template project main
    File name: template.c
    Author: Etienne Le Sueur
    Description: This is the steering wheel code. It makes use of the Scandal
	WaveSculptor driver. It should make no assumptions about what type of
	Waveculptor is connected. It should work with any WaveSculptor, however
	it has only been tested with the WS20 and WS22 models.

	The precharge state machine in here is a little more complicated than before,
	but also more useful. The timeout for the button press is defined as
	PRECHARGE_SWITCH_HOLD_TIME_MS. While the button is pressed and the hold time
	is less than this, the LED will flash every 100ms. Once the hold timeout has
	expired, the LED will flash every 500ms (slower) while we wait for the 
	SmartDCDC to do its thing. However, we don't want to wait forever in this
	state, so we have PRECHARGE_DISCHARGE_TIMEOUT_MS. If this expires, the LED
	will flash in an erratic way for a few seconds and then we go back into the
	normal discharged state.

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
#include <project/conversion.h>

#include <arch/can.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/adc.h>

#define VELOCITY_MAX          40.0    /* max velocity in m/s */
#define VELOCITY_REVERSE_MAX  -10.0    /* max velocity in m/s */
#define WHEEL_DIAMETER        0.4064  /* wheel diameter in meters. currently = 16" tyre * 2.54cm = 0.4064 */

/* general variables */
uint32_t  horn = 0;                               /* is the horn on */
uint32_t  brake = 0;                              /* is the brake on? */
uint32_t  precharging = 0;                        /* are we trying to precharge? */
uint32_t  discharging = 0;                        /* are we trying to discharge? */
uint32_t  precharged = 0;                         /* are we precharged? */
uint32_t  precharge_switch = 0;                   /* are we waiting for the button hold time? */
uint32_t  precharge_timeout = 0;                  /* have we timed out waiting? */
sc_time_t precharge_switch_on_time = 0;           /* the time someone pressed (and not released) the precharge switch */
sc_time_t precharge_discharge_request_time = 0;   /* the time we started sending out a precharge or discharge request */
uint32_t  power_on_delay_completed = 0;           /* are we past the power on delay? */

/* wavesculptor related variables */
uint32_t  wavesculptor_model = WS20;              /* which wavesculptor is attached? */
int       cruise_led_flash = 0;                   /* flash the cruise led when we change speed */
int       cruise = 0;                             /* are we in cruise? */
float     bus_current = 0.0;                      /* what's our bus_current setpoint? */
float     motor_current = 0.0;                    /* what's our motor current setpoint? */
float     set_velocity = VELOCITY_MAX;            /* what's the current velocity in m/s? */
float     current_velocity = 0.0;                 /* our current speed from the wavesculptor */
int       reverse = 0;                            /* are we in reverse? */
int       reverse_switch = 0;                     /* are we waiting for the button hold time? */
sc_time_t reverse_switch_on_time = 0;             /* the time someone pressed (and not released) the reverse switch */
int       forward_switch = 0;                     /* are we waiting for the button hold time? */
sc_time_t forward_switch_on_time = 0;             /* the time someone pressed (and not released) the forward switch */
float     current_bus_voltage;                    /* the current bus voltage from the wavesculptor */

/* indicator related variables */
uint32_t  hazards = 1;                            /* are hazards on? */
uint32_t  hazards_state = 0;                      /* what's the current state of the hazards? */
uint32_t  left_indicator = 0;                     /* is the left indicator on? */
uint32_t  right_indicator = 0;                    /* is the right indicator on? */
uint32_t  left_indicator_state = 0;               /* what's the current state of the left indicator? */
uint32_t  right_indicator_state = 0;              /* what's the current state of the right indicator? */
uint32_t  rear_vision = 0;                        /* is the rear vision on? */

/* switch interrupt handlers */
extern void speed_hold_handler();
extern void speed_down_handler();
extern void speed_up_handler();
extern void horn_handler();
extern void hazards_handler();
extern void brake_handler();
extern void precharge_handler();
extern void left_indicator_handler();
extern void right_indicator_handler();
extern void reverse_handler();
extern void forward_handler();
extern void rear_vision_handler();

/* wavesculptor and inchannel handlers and init functions */
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

	/* Reverse switch */
	GPIO_SetFunction(REV_SWITCH_PORT, REV_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(REV_SWITCH_PORT, REV_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(REV_SWITCH_PORT, REV_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &reverse_handler);

	/* Forward switch */
	GPIO_SetFunction(FWD_SWITCH_PORT, FWD_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(FWD_SWITCH_PORT, FWD_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(FWD_SWITCH_PORT, FWD_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_DOUBLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &forward_handler);

	/* Rear vision switch */
	GPIO_SetFunction(REAR_VISION_SWITCH_PORT, REAR_VISION_SWITCH_BIT, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(REV_SWITCH_PORT, REV_SWITCH_BIT, 0);

	GPIO_RegisterInterruptHandler(REAR_VISION_SWITCH_PORT, REAR_VISION_SWITCH_BIT,
		GPIO_INTERRUPT_SENSE_EDGE, GPIO_INTERRUPT_SINGLE_EDGE, GPIO_INTERRUPT_EVENT_NONE,
		 &rear_vision_handler);

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

		/* Read the paddles */
		ADC_Read(REGEN_ADC_CHANNEL);
		ADC_Read(ACCELERATOR_ADC_CHANNEL);

		/* A bit of munging for top and bottom of the ranges */
		float throttle = (float)(RIGHT_PADDLE_MAX - ((int32_t)ADCValue[ACCELERATOR_ADC_CHANNEL] - RIGHT_PADDLE_MIN)) / (float)RIGHT_PADDLE_MAX;
		float regen = (float)(LEFT_PADDLE_MAX - ((int32_t)ADCValue[REGEN_ADC_CHANNEL] - LEFT_PADDLE_MIN)) / (float)LEFT_PADDLE_MAX;

		/* If the throttle is below 5%, just zero it */
		if (throttle < 0.05)
			throttle = 0.0;

		/* If regen is below 5%, just zero it */
		if (regen < 0.05)
			regen = 0.0;

		/* If the throttle is above 100%, make it 100% */
		if (throttle > 1.0)
			throttle = 1.0;

		/* If regen is above 100%, make it 100% */
		if (regen > 1.0)
			regen = 1.0;

		/* turn on delay, the paddles seem to read crazy at turn on. We also
		 * set another variable for when the timer rolls around, as it's only
		 * 32 bits. */
		if (sc_get_timer() > 2000 || power_on_delay_completed) {
			power_on_delay_completed = 1;

			set_velocity = 0.0;
			motor_current = 0.0;
			bus_current = 0.0;

			/* we want to move */
			if (!brake && !cruise && throttle > 0.0) {
				/* WS22 needs velocity in RPM */
				if (wavesculptor_model == WS22)
					set_velocity = mps2rpm(VELOCITY_MAX, WHEEL_DIAMETER);
				/* WS22 needs velocity in meters per second */
				else if (wavesculptor_model == WS20)
					set_velocity = VELOCITY_MAX;

				/* If we're reversing, set the speed to reverse_max */
				if (reverse)
					set_velocity = VELOCITY_REVERSE_MAX;

				bus_current = 1.0;
				motor_current = throttle;

			/* we're in cruise control, use velocity as control */
			} else if (cruise && !brake) {
				/* WS22 needs velocity in RPM */
				if (wavesculptor_model == WS22)
					set_velocity = mps2rpm(current_velocity, WHEEL_DIAMETER);
				/* WS22 needs velocity in meters per second */
				else if (wavesculptor_model == WS20)
					set_velocity = current_velocity;

				bus_current = 1.0;
				motor_current = 1.0;
			}

			/* regen */
			if (regen > 0.0) {
				cruise = 0;
				GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 1);

				set_velocity = 0.0;
				bus_current = 1.0;
				motor_current = 1.0 - regen;
			}

			/* Call the drive command handler.
			 * We do this regardless of the state of the throttle or precharge so that the
			 * the CAN bus load is consistent */
			handle_ws_drive_commands(set_velocity, bus_current, motor_current);
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
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_THROTTLE, (int)(throttle*100.0));
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_REGEN, (int)(regen*100.0));
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_CRUISE, cruise);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_VELOCITY, (int32_t)(current_velocity));
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_BUSVOLTS, (int32_t)(current_bus_voltage*100.0));
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_SET_VELOCITY, (int32_t)(set_velocity));
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_SET_CURRENT,  (int32_t)(motor_current));
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_SET_BUSCURRENT, (int32_t)(bus_current));
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_REVERSE, reverse);
			scandal_send_channel(TELEM_LOW, 40, reverse_switch);
			scandal_send_channel(TELEM_LOW, 41, forward_switch);

			/* send out the rear vision channel, this is every second so that we ensure
			 * if we lose a message we don't get into a silly state */
			if (rear_vision)
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_REAR_VISION, 1);
			else
				scandal_send_channel(TELEM_LOW, STEERINGWHEEL_REAR_VISION, 0);

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
				/* if the left indicator is already on, turn it off */
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
		if ((precharging || discharging) && 
			(sc_get_timer() >= precharge_discharge_request_time + PRECHARGE_DISCHARGE_TIMEOUT_MS)) {
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

		/* the user is holding the reverse switch. It we're past the timeout value, let them have it */
		if (reverse_switch && (sc_get_timer() >= reverse_switch_on_time + REVERSE_SWITCH_HOLD_TIME_MS)) {
			if (!reverse)
				reverse = 1;
		}

		/* the user is holding the forward switch. It we're past the timeout value, let them have it */
		if (forward_switch && (sc_get_timer() >= forward_switch_on_time + FWD_SWITCH_HOLD_TIME_MS)) {
			reverse = 0;
			reverse_led(0);
		}

		/* stuff to do at 100ms intervals */
		if(sc_get_timer() >= hundred_ms_timer + 100) {
			/* are we waiting for the button to be released */
			/* if we have a precharge timeout, flash here AND below in the 500ms timer */
			if ((precharge_switch && !precharging && !discharging) || precharge_timeout > 0)
				toggle_precharge_led();

			if (cruise_led_flash > 0) {
				toggle_cruise_led();
				cruise_led_flash--;

			/* make sure the LED is on if we're still in cruise */
			} else if (cruise) {
				cruise_led(1);
			}

			/* if the user is holding the reverse switch and the timeout hasn't passed, flash
			 * the LED fast */
			if ((reverse_switch && !reverse) || (forward_switch && reverse))
				toggle_reverse_led();

			/* Update the timer */
			hundred_ms_timer = sc_get_timer();
		}

		/* stuff to do at 500ms intervals */
		if(sc_get_timer() >= five_hundred_ms_timer + 500) {
			/* are we waiting for a message from the smartdcdc? */
			if (precharging || discharging)
				toggle_precharge_led();

			/* or have we timed out waiting for the smartDCDC? */
			if (precharge_timeout) {
				toggle_precharge_led();
				precharge_timeout--;
			}

			/* the reverse LED flashes slow if we're in reverse */
			if (reverse && !reverse_switch && !forward_switch)
				toggle_reverse_led();

			/* Update the timer */
			five_hundred_ms_timer = sc_get_timer();
		}
	}
}
