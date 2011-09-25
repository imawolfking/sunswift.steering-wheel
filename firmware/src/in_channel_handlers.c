#include <scandal/message.h>
#include <scandal/engine.h>
#include <arch/gpio.h>
#include <project/target_config.h>

extern int precharged;

void precharge_status_handler(int32_t value, uint32_t src_time) {
	if (value) {
		precharged = 1;
		GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 0);
	} else {
		precharged = 0;
		GPIO_SetValue(CRUISE_LED_PORT, CRUISE_LED_BIT, 0);
	}
}

void init_scandal_in_channels(void) {
	scandal_register_in_channel_handler(STEERINGWHEEL_PRECHARGE_STAT, &precharge_status_handler);
}

