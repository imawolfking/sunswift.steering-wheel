#include <arch/gpio.h>

/* LED defines */
#define PRCH_LED_PORT   PORT2
#define PRCH_LED_BIT    9

#define CRUISE_LED_PORT PORT1
#define CRUISE_LED_BIT  1

#define REV_LED_PORT    PORT3
#define REV_LED_BIT     1

#define RIGHT_LED_PORT  PORT2
#define RIGHT_LED_BIT   10

#define LEFT_LED_PORT   PORT3
#define LEFT_LED_BIT    0

#define BRAKE_PORT      PORT2
#define BRAKE_BIT       8

#define RED_LED_PORT     RIGHT_LED_PORT
#define RED_LED_BIT      RIGHT_LED_BIT
#define YELLOW_LED_PORT  LEFT_LED_PORT
#define YELLOW_LED_BIT   LEFT_LED_BIT

#define left_led(x)  yellow_led(x)
#define right_led(x) red_led(x)
#define toggle_left_led()  toggle_yellow_led()
#define toggle_right_led() toggle_red_led()

/* Switch defines */
#define HORN_SWITCH_PORT PORT2
#define HORN_SWITCH_BIT  3

#define HAZARDS_SWITCH_PORT PORT1
#define HAZARDS_SWITCH_BIT  4

#define INDICATORS_OFF_SWITCH_PORT PORT3
#define INDICATORS_OFF_SWITCH_BIT  3

#define RIGHT_INDICATOR_SWITCH_PORT PORT3
#define RIGHT_INDICATOR_SWITCH_BIT  3

#define LEFT_INDICATOR_SWITCH_PORT PORT1
#define LEFT_INDICATOR_SWITCH_BIT  5

#define SPEED_UP_SWITCH_PORT PORT0
#define SPEED_UP_SWITCH_BIT 11

#define SPEED_HOLD_SWITCH_PORT PORT2
#define SPEED_HOLD_SWITCH_BIT 11

#define FUNCY2_SWITCH_PORT PORT1
#define FUNCY2_SWITCH_BIT 10

#define FUNCY3_SWITCH_PORT PORT0
#define FUNCY3_SWITCH_BIT 8

#define SPEED_DOWN_SWITCH_PORT PORT2
#define SPEED_DOWN_SWITCH_BIT 2

#define FWD_SWITCH_PORT PORT0
#define FWD_SWITCH_BIT 7

#define START_SWITCH_PORT PORT0
#define START_SWITCH_BIT 6

#define REAR_VISION_SWITCH_PORT PORT2
#define REAR_VISION_SWITCH_BIT 5

#define FUNCY1_SWITCH_PORT PORT2
#define FUNCY1_SWITCH_BIT 4

#define REV_SWITCH_PORT PORT1
#define REV_SWITCH_BIT 9

#define LED_TOGGLE_TICKS 100 // 100 ticks = 1 Hz flash rate
#define FAST_LED_TOGGLE_TICKS 25 // 100 ticks = 1 Hz flash rate
#define COUNT_MAX		3 // how high to count on the LED display
