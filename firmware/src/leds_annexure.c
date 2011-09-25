#include <scandal/types.h>

#include <arch/gpio.h>

#include <project/target_config.h>
#include <project/leds_annexure.h>

void precharge_led(u08 on) {
	if (on)
		GPIO_SetValue(PRCH_LED_PORT, PRCH_LED_BIT, 0);
	else
		GPIO_SetValue(PRCH_LED_PORT, PRCH_LED_BIT, 1);
}

void toggle_precharge_led(void) {
	GPIO_ToggleValue(PRCH_LED_PORT, PRCH_LED_BIT);
}
