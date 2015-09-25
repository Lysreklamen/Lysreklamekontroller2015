#ifndef PWM_H
#define PWM_H

//32Mhz / 4 / 2^16 = 122Hz:
#define PRESCALER	TC_CLKSEL_DIV2_gc
#define PWM_PERIOD	65535

#define LED0_SET(r, g, b)	xmega_timer_cca(&TCC0, (r)); xmega_timer_ccb(&TCC0, (g)); xmega_timer_ccc(&TCC0, (b))
#define LED1_SET(r, g, b)	xmega_timer_ccd(&TCC0, (r)); xmega_timer_cca(&TCC1, (g)); xmega_timer_ccb(&TCC1, (b))
#define LED2_SET(r, g, b)	xmega_timer_cca(&TCD0, (r)); xmega_timer_ccb(&TCD0, (g)); xmega_timer_ccc(&TCD0, (b))
#define LED3_SET(r, g, b)	xmega_timer_ccb(&TCF0, (r)); xmega_timer_ccc(&TCF0, (g)); xmega_timer_ccd(&TCF0, (b))
#define LED4_SET(r, g, b)	xmega_timer_ccd(&TCE0, (r)); xmega_timer_cca(&TCE1, (g)); xmega_timer_ccb(&TCE1, (b))
#define LED5_SET(r, g, b)	xmega_timer_cca(&TCE0, (r)); xmega_timer_ccb(&TCE0, (g)); xmega_timer_ccc(&TCE0, (b))

#include <xmega_timer.h>
void pwm_init(void);

#endif
