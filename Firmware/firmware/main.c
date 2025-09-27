#include "config.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 

#include "xmega_clock.h"
#include "xmega_timer.h"
#include "pwm.h"
#include "dmx.h"
#include "default.h"
#include "debug_led.h"



int main(void)
{
	xmega_clock_init();
	default_init();

	debug_led_init();
	pwm_init();
	dmx_init();

	/* Enable interrupts: */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	PMIC.CTRL |= PMIC_MEDLVLEX_bm;
	PMIC.CTRL |= PMIC_HILVLEX_bm;
	sei();
		
	while (1) {
		dmx_handle();
		}

	return 0;
}

