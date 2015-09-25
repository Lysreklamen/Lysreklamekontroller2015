#include <inttypes.h>
#include <avr/io.h>
#include <xmega_timer.h>
#include "pwm.h"

void pwm_init()
{
	/* Setup PWM output: */

	//TCC0
	xmega_timer_prescale(&TCC0, PRESCALER);
	xmega_timer_wgm(&TCC0, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCC0, PWM_PERIOD);
	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
	xmega_timer_cca(&TCC0, 0);
	xmega_timer_ccb(&TCC0, 0);
	xmega_timer_ccc(&TCC0, 0);
	xmega_timer_ccd(&TCC0, 0);
	xmega_timer_enable_a(&TCC0);
	xmega_timer_enable_b(&TCC0);
	xmega_timer_enable_c(&TCC0);
	xmega_timer_enable_d(&TCC0);

	//TCC1
	xmega_timer_prescale(&TCC1, PRESCALER);
	xmega_timer_wgm(&TCC1, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCC1, PWM_PERIOD);
	PORTC.DIRSET = PIN4_bm | PIN5_bm;
	xmega_timer_cca(&TCC1, 0);
	xmega_timer_ccb(&TCC1, 0);
	xmega_timer_enable_a(&TCC1);
	xmega_timer_enable_b(&TCC1);

	//TCD0
	xmega_timer_prescale(&TCD0, PRESCALER);
	xmega_timer_wgm(&TCD0, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCD0, PWM_PERIOD);
	PORTD.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
	xmega_timer_cca(&TCD0, 0);
	xmega_timer_ccb(&TCD0, 0);
	xmega_timer_ccc(&TCD0, 0);
	xmega_timer_enable_a(&TCD0);
	xmega_timer_enable_b(&TCD0);
	xmega_timer_enable_c(&TCD0);
	
	//TCE0
	xmega_timer_prescale(&TCE0, PRESCALER);
	xmega_timer_wgm(&TCE0, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCE0, PWM_PERIOD);
	PORTE.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
	xmega_timer_cca(&TCE0, 0);
	xmega_timer_ccb(&TCE0, 0);
	xmega_timer_ccc(&TCE0, 0);
	xmega_timer_ccd(&TCE0, 0);
	xmega_timer_enable_a(&TCE0);
	xmega_timer_enable_b(&TCE0);
	xmega_timer_enable_c(&TCE0);
	xmega_timer_enable_d(&TCE0);

	//TCC1
	xmega_timer_prescale(&TCE1, PRESCALER);
	xmega_timer_wgm(&TCE1, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCE1, PWM_PERIOD);
	PORTE.DIRSET = PIN4_bm | PIN5_bm;
	xmega_timer_cca(&TCE1, 0);
	xmega_timer_ccb(&TCE1, 0);
	xmega_timer_enable_a(&TCE1);
	xmega_timer_enable_b(&TCE1);
	
	//TCF0
	xmega_timer_prescale(&TCF0, PRESCALER);
	xmega_timer_wgm(&TCF0, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCF0, PWM_PERIOD);
	PORTF.DIRSET = PIN1_bm | PIN2_bm | PIN3_bm;
	xmega_timer_ccb(&TCF0, 0);
	xmega_timer_ccc(&TCF0, 0);
	xmega_timer_ccd(&TCF0, 0);
	xmega_timer_enable_b(&TCF0);
	xmega_timer_enable_c(&TCF0);
	xmega_timer_enable_d(&TCF0);
	
}
