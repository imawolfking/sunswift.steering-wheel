#include <arch/gpio.h>
#include <scandal/timer.h>
#include <scandal/message.h>
#include <project/target_config.h>
#include <scandal/leds.h>
#include <project/leds_annexure.h>

extern int cruise;
extern int hazards;
extern int horn;
extern float velocity;
extern int brake;
extern int left_indicator;
extern int right_indicator;
extern sc_time_t precharge_discharge_request_time;
extern sc_time_t precharge_switch_on_time;
extern int precharge_timeout;
extern int precharging;
extern int discharging;
extern int precharged;
extern int precharge_switch;

static int last_speed_hold_time = 0;
void speed_hold_handler() {
	/* debouncing */
	if (sc_get_timer() >= last_speed_hold_time + 200) {
		if (cruise) {
			cruise = 0;
			GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 1);
		} else {
			cruise = 1;
			GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 0);
		}
	}

	last_speed_hold_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT);
}

static int last_speed_up_time = 0;
void speed_up_handler() {
	/* debouncing */
	if (sc_get_timer() - last_speed_up_time > 50)
		velocity += 0.5;

	last_speed_up_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT);
}

static int last_speed_down_time = 0;
void speed_down_handler() {
	/* debouncing */
	if (sc_get_timer() - last_speed_down_time > 50)
		velocity -= 0.5;

	last_speed_down_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT);
}

static int last_horn_time = 0;
void horn_handler() {
	/* debouncing */
	if (sc_get_timer() - last_horn_time > 50) {
		if (horn)
			horn = 0;
		else
			horn = 1;
	}

	last_horn_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(HORN_SWITCH_PORT, HORN_SWITCH_BIT);
}

static int last_hazard_time = 0;
void hazards_handler() {
	/* debouncing */
	if (sc_get_timer() - last_hazard_time > 50) {
		if (hazards) {
			hazards = 0;
			left_led(0);
			right_led(0);
		} else {
			hazards = 1;
		}
	}

	last_hazard_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT);
}

static int last_left_indicator_time = 0;
void left_indicator_handler() {
	/* debouncing */
	if (sc_get_timer() - last_left_indicator_time > 50) {
		if (left_indicator) {
			left_indicator = 0;
			left_led(0);
		} else {
			left_indicator = 1;
		}
	}

	last_left_indicator_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(LEFT_INDICATOR_SWITCH_PORT, LEFT_INDICATOR_SWITCH_BIT);
}

static int last_right_indicator_time = 0;
void right_indicator_handler() {
	/* debouncing */
	if (sc_get_timer() - last_right_indicator_time > 50) {
		if (right_indicator) {
			right_indicator = 0;
			right_led(0);
		} else {
			right_indicator = 1;
		}
	}

	last_right_indicator_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(RIGHT_INDICATOR_SWITCH_PORT, RIGHT_INDICATOR_SWITCH_BIT);
}

static int last_brake_time = 0;
void brake_handler() {
	/* debouncing */
	if (sc_get_timer() - last_brake_time > 50)
		brake ^= 1;

	last_brake_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(BRAKE_PORT, BRAKE_BIT);
}

static int last_precharge_time = 0;
void precharge_handler() {
	/* debouncing */
	if (sc_get_timer() >= last_precharge_time + 50) {
		/* precharge switch was on, has been released. Check time */
		if (precharge_switch) {
			precharge_switch = 0;

			/* the main loop also tests for the button press, only do this if we're not already precharging */ 
			if (!precharging) {
				/* we are now sending the precharge signal out */
				if (sc_get_timer() >= precharge_switch_on_time + PRECHARGE_SWITCH_HOLD_TIME_MS) {
					precharge_discharge_request_time = sc_get_timer();
					precharge_timeout = 0;
					if (precharged) {
						precharging = 0;
						discharging = 1;
					} else {
						precharging = 1;
						discharging = 0;
					}
				} else {
					precharge_led(0);
				}
			}

		/* precharge switch has been pressed */
		} else {
			precharge_switch = 1;
			precharge_switch_on_time = sc_get_timer();
		}
	}

	last_precharge_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(PRECHARGE_SWITCH_PORT, PRECHARGE_SWITCH_BIT);
}
