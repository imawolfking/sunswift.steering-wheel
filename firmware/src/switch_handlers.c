#include <arch/gpio.h>
#include <scandal/timer.h>
#include <scandal/message.h>
#include <project/target_config.h>
#include <scandal/leds.h>

extern int cruise;
extern int hazards;
extern int horn;
extern float velocity;

int last_speed_hold_time = 0;
void speed_hold_handler() {
	/* debouncing */
	if (sc_get_timer() - last_speed_hold_time > 200) {
		/* cruise is on, turn it off */
		if (cruise) {
			GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 1);
			cruise = 0;

		/* cruise is off turn it on */
		} else {
			GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 0);
			cruise = 1;
		}
	}

	last_speed_hold_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_HOLD_SWITCH_PORT, SPEED_HOLD_SWITCH_BIT);
}

int last_speed_up_time = 0;
void speed_up_handler() {
	/* debouncing */
	if (sc_get_timer() - last_speed_up_time > 200) {
		velocity += 0.5;
		scandal_send_channel(TELEM_LOW, STEERINGWHEEL_SET_VELOCITY, (int)velocity);
	}

	last_speed_up_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_UP_SWITCH_PORT, SPEED_UP_SWITCH_BIT);
}

int last_speed_down_time = 0;
void speed_down_handler() {
	/* debouncing */
	if (sc_get_timer() - last_speed_down_time > 200) {
		velocity -= 0.5;
		scandal_send_channel(TELEM_LOW, STEERINGWHEEL_SET_VELOCITY, (int)velocity);
	}

	last_speed_down_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(SPEED_DOWN_SWITCH_PORT, SPEED_DOWN_SWITCH_BIT);
}

int last_horn_time = 0;
void horn_handler() {
	/* debouncing */
	if (sc_get_timer() - last_horn_time > 200) {
		/* horn is on, turn it off */
		if (horn)
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_HORN, --horn);
		else 
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_HORN, ++horn);
	}

	last_horn_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(HORN_SWITCH_PORT, HORN_SWITCH_BIT);
}

int last_hazard_time = 0;
void hazards_handler() {
	/* debouncing */
	if (sc_get_timer() - last_horn_time > 200) {
		/* hazards are on, turn them off */
		if (hazards) {
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_RH_IND, --hazards);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_LH_IND, hazards);
			left_led(0);
			right_led(0);
		/* hazards are off, turn them on */
		} else {
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_RH_IND, ++hazards);
			scandal_send_channel(TELEM_LOW, STEERINGWHEEL_LH_IND, hazards);
		}
	}

	last_hazard_time = sc_get_timer();

	/* ack the interrupt */
	GPIO_IntClear(HAZARDS_SWITCH_PORT, HAZARDS_SWITCH_BIT);
}
