/* --------------------------------------------------------------------------
	Scandal Engine
	File name: led.c
	Author: Etienne Le Sueur
 
    Copyright (C) 2011 Etienne Le Sueur.

	Date: 29/08/2011
   -------------------------------------------------------------------------- */ 

/* 
 * This file is part of Scandal.
 * 
 * Scandal is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * Scandal is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with Scandal.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <scandal/types.h>

#ifdef lpc11c14
#include <project/driver_config.h>
#include <project/target_config.h>
#endif

#include <arch/gpio.h>

void red_led(u08 on) {
	if (on)
		GPIOSetValue(2,8,0);
	else
		GPIOSetValue(2,8,1);
}

void toggle_red_led(void) {
	GPIOToggleValue(2,8);
}

void yellow_led(u08 on) {
	if (on)
		GPIOSetValue(2,7,0);
	else
		GPIOSetValue(2,7,1);
}

void toggle_yellow_led(void) {
	GPIOToggleValue(2,7);
}

