/* --------------------------------------------------------------------------                                 
    Template project main
    File name: main.c
    Author: Etienne Le Sueur
    Description: The template main file

    Copyright (C) Etienne Le Sueur, 2011

    Date: 07/09/2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of the Sunswift Template project
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
 * along with the project.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <arch/can.h>
#include <arch/uart.h>

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/led.h>
#include <scandal/utils.h>
#include <scandal/stdio.h>

#ifdef lpc11c14
#include <project/driver_config.h>
#include <project/target_config.h>

#include <arch/timer32.h>
#include <arch/gpio.h>
#include <arch/type.h>
#include <arch/i2c.h>
#else
#ifdef lpc1768

#else
#ifdef msp430f149
#include <msp430x14x.h>
#include <signal.h>
#include <project/hardware.h>

void init_clock(void) {
	volatile unsigned int i;
	
	/* XTAL = LF crystal, ACLK = LFXT1/1, DCO Rset = 4, XT2 = ON */
	BCSCTL1 = 0x04;
	
	/* Clear OSCOFF flag - start oscillator */
	_BIC_SR( OSCOFF );
	do {
		/* Clear OSCFault flag */
		IFG1 &= ~OFIFG; 
		/* Wait for flag to set */
		for( i = 255; i > 0; i-- )
			;
	} while(( IFG1 & OFIFG ) != 0);
	
	/* Set MCLK to XT2CLK and SMCLK to XT2CLK */
	BCSCTL2 = 0x88; 
}

void enable_can_interrupt(){
	P2IE = CAN_INT;
}

void disable_can_interrupt(){
	P2IE = 0x00;
}

interrupt (PORT2_VECTOR) port2int(void) {
	can_interrupt();
	P2IFG = 0x00;
}

#endif // msp430f149
#endif // lpc1768
#endif // lpc11c14

void setup_ports(void) {
#ifdef lpc11c14
	GPIOInit();
	GPIOSetDir(2,8,1); //Green LED, Out
	GPIOSetDir(2,7,1); //Yel LED, Out
#else
#ifdef lpc1768

#else
#ifdef msp43f149
	P1OUT = 0x00;
	P1SEL = 0x00;
	P1DIR = 0x00;
	P1IES = 0x00;
	P1IE  = 0x00;
	
	P2OUT = 0x00;
	P2SEL = 0x00;
	P2DIR = 0x00;
	P2IES = CAN_INT;
	P2IE  = 0x00;
	
	P3OUT = 0x00;
	P3SEL = TX | RX;
	P3DIR = TX;
	
	P4OUT = 0x00;
	P4SEL = 0x00;
	P4DIR = 0x00;
	
	P5OUT = CAN_CS;
	P5SEL = SIMO1 | SOMI1 | UCLK1;
	P5DIR = CAN_CS | SIMO1 | UCLK1 | YELLOWLED | REDLED;
	
	P6SEL = MEAS_12V_PIN;  
#endif // msp430f149
#endif // lpc1768
#endif // lpc11c14
}

void in_channel_0_handler(int32_t value, uint32_t src_time) {
	UART_printf("in_channel_0_handler got called with value %d time at source %u\n\r", (int)value, (unsigned int)src_time);
}

int main(void) {
	int i = 0;
	uint32_t value = 0xaa;

	setup_ports();

#ifdef msp430f149
	init_clock();
#endif

	scandal_init();

	UART_Init(115200);

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t test_in_timer = sc_get_timer(); /* Initialise the timer variable */

	/* Set LEDs to known states, i.e. on */
	red_led(1);
	yellow_led(0);

	scandal_delay(100); /* wait for the UART clocks to settle */

	/* Display welcome header over UART */
	UART_printf("Welcome to the template project! This is coming out over UART1\n\r");
	UART_printf("The 2 debug LEDs should blink at a rate of 1HZ\n\r");
	UART_printf("If you configure the in channel 0, I should print a message upon receipt of such a channel message\n\r");

	scandal_register_in_channel_handler(0, &in_channel_0_handler);

	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();

		/* Send a UART message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {
			/* Send the message */
			UART_printf("This is the 1 second timer... %u\n\r", i++);

			/* Send a channel message with a blerg value at low priority on channel 0 */
			scandal_send_channel(TELEM_LOW, /* priority */
									0,      /* channel num */
									value   /* value */
			);

			/* Twiddle the LEDs */
			toggle_yellow_led();
			toggle_red_led();

			/* Update the timer */
			one_sec_timer = sc_get_timer();
		}

		if(scandal_get_in_channel_rcvd_time(TEMPLATE_TEST_IN) > test_in_timer) {

			value = scandal_get_in_channel_value(TEMPLATE_TEST_IN);

			UART_printf("I received a channel message in the main loop on in_channel 0, value %u at time %d\n\r", 
				(unsigned int)value, (int)scandal_get_in_channel_rcvd_time(TEMPLATE_TEST_IN));

			if(scandal_get_in_channel_value(TEMPLATE_TEST_IN) == 1) {
				toggle_red_led();
			} else {
				toggle_yellow_led();
			}

			test_in_timer = scandal_get_in_channel_rcvd_time(TEMPLATE_TEST_IN);
		}

	}
}
