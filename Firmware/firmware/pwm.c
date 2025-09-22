#include <stdbool.h>
#include <inttypes.h>
#include <avr/io.h>
#include <xmega_timer.h>
#include "pwm.h"

// Timere:
// TCC0  4  0r 0g 0b 1r
// TCC1  2  1g 1b
// TCD0  3  2r 2g 2b
// TCE0  4  4r 5r 5g 5b
// TCE1  2  4g 4b
// TCF0  3  3r 3g 3b

void pwm_init()
{
	/* Setup PWM output: */
	//TCC0
	xmega_timer_prescale(&TCC0, PRESCALER);
	xmega_timer_wgm(&TCC0, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCC0, PWM_PERIOD);
	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
	xmega_timer_enable_a(&TCC0);
	xmega_timer_enable_b(&TCC0);
	xmega_timer_enable_c(&TCC0);
	xmega_timer_enable_d(&TCC0);

	//TCC1
	xmega_timer_prescale(&TCC1, PRESCALER);
	xmega_timer_wgm(&TCC1, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCC1, PWM_PERIOD);
	PORTC.DIRSET = PIN4_bm | PIN5_bm;
	xmega_timer_enable_a(&TCC1);
	xmega_timer_enable_b(&TCC1);

	//TCD0
	xmega_timer_prescale(&TCD0, PRESCALER);
	xmega_timer_wgm(&TCD0, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCD0, PWM_PERIOD);
	PORTD.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
	xmega_timer_enable_a(&TCD0);
	xmega_timer_enable_b(&TCD0);
	xmega_timer_enable_c(&TCD0);
	
	//TCE0
	xmega_timer_prescale(&TCE0, PRESCALER);
	xmega_timer_wgm(&TCE0, TC_WGMODE_SS_gc);
	xmega_timer_period(&TCE0, PWM_PERIOD);
	PORTE.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
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
	xmega_timer_enable_b(&TCF0);
	xmega_timer_enable_c(&TCF0);
	xmega_timer_enable_d(&TCF0);
}


// TODO:
// dette skulle i teorien kunne være en bedre måte å addresere bufferene på:
register16_t* channel_ccbufs[] = {
	&TCC0.CCABUF, &TCC0.CCBBUF, &TCC0.CCCBUF, 
	&TCC0.CCDBUF, &TCC1.CCABUF, &TCC1.CCBBUF,
	&TCD0.CCABUF, &TCD0.CCBBUF, &TCD0.CCCBUF,
	&TCF0.CCBBUF, &TCF0.CCCBUF, &TCF0.CCDBUF,
	&TCE0.CCDBUF, &TCE1.CCABUF, &TCE1.CCBBUF,
	&TCE0.CCABUF, &TCE0.CCBBUF, &TCE0.CCCBUF
};

void pwm_set_ch(uint8_t ch, uint8_t value)
{	
	*(channel_ccbufs[ch]) = value;
	// uint16_t gamma = (uint16_t)value * (uint16_t)value ;
	// *(channel_ccbufs[ch]) = gamma
}

void pwm_set_led(uint8_t led, uint8_t red, uint8_t green, uint8_t blue)
{
	set_ch(led*3+0, red);
	set_ch(led*3+1, green);
	set_ch(led*3+2, blue);
}

void pwm_set_frame(uint8_t frame[])
{
	for (uint8_t i = 0 ; i < 18; i++ ){
		set_ch(i,frame[i]);
	}
}
