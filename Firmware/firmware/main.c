#include "config.h"
#include <util/delay.h>
#include <inttypes.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "xmega_clock.h"
#include "xmega_timer.h"
#include "pwm.h"
#include "dmx.h"

int main(void)
{
	//xmega_clock_select_ext();
	xmega_clock_select_32mhz();
	
	pwm_init();
	dmx_init();

	/* Enable interrupts: */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	PMIC.CTRL |= PMIC_MEDLVLEX_bm;
	sei();
	
	LED0_SET(255, 0, 0);
	LED1_SET(255, 0, 0);
	LED2_SET(255, 0, 0);
	LED3_SET(255, 0, 0);
	LED4_SET(255, 0, 0);
	LED5_SET(255, 0, 0);
	
	while (1) {
		dmx_handle();
		}

	return 0;
}

