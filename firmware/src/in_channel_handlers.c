#include <scandal/message.h>
#include <scandal/engine.h>
#include <arch/gpio.h>
#include <project/target_config.h>

extern int precharged;
extern int precharging;
extern int discharging;

void precharge_status_handler(int32_t value, uint32_t src_time) {
	if (value) {
		precharged = 1;
		precharging = 0;
		discharging = 0;
		GPIO_SetValue(PRCH_LED_PORT, PRCH_LED_BIT, 0);
	} else {
		precharged = 0;
		precharging = 0;
		discharging = 0;
		GPIO_SetValue(PRCH_LED_PORT, PRCH_LED_BIT, 1);
	}
}

void init_scandal_in_channels(void) {
	scandal_register_in_channel_handler(STEERINGWHEEL_PRECHARGE_STAT, &precharge_status_handler);
}

