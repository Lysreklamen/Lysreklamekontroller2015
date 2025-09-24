#ifndef PWM_H
#define PWM_H

#include <xmega_timer.h>

//32Mhz / 4 / 2^16 = 122Hz:
#define PRESCALER	TC_CLKSEL_DIV2_gc
#define PWM_PERIOD	65535

#define SQR(x) ((uint16_t)(x)*(uint16_t)(x))

#define LED0_SET(r, g, b)	xmega_timer_cca(&TCC0, SQR(r)); xmega_timer_ccb(&TCC0, SQR(g)); xmega_timer_ccc(&TCC0, SQR(b))
#define LED1_SET(r, g, b)	xmega_timer_ccd(&TCC0, SQR(r)); xmega_timer_cca(&TCC1, SQR(g)); xmega_timer_ccb(&TCC1, SQR(b))
#define LED2_SET(r, g, b)	xmega_timer_cca(&TCD0, SQR(r)); xmega_timer_ccb(&TCD0, SQR(g)); xmega_timer_ccc(&TCD0, SQR(b))
#define LED3_SET(r, g, b)	xmega_timer_ccb(&TCF0, SQR(r)); xmega_timer_ccc(&TCF0, SQR(g)); xmega_timer_ccd(&TCF0, SQR(b))
#define LED4_SET(r, g, b)	xmega_timer_ccd(&TCE0, SQR(r)); xmega_timer_cca(&TCE1, SQR(g)); xmega_timer_ccb(&TCE1, SQR(b))
#define LED5_SET(r, g, b)	xmega_timer_cca(&TCE0, SQR(r)); xmega_timer_ccb(&TCE0, SQR(g)); xmega_timer_ccc(&TCE0, SQR(b))

#define LED012345_SET(r, g, b) LED0_SET(r,g,b); LED1_SET(r,g,b); LED2_SET(r,g,b); LED3_SET(r,g,b); LED4_SET(r,g,b); LED5_SET(r,g,b);

void pwm_init(void);

void pwm_set_ch(uint8_t ch, uint8_t value);
void pwm_set_led(uint8_t led, uint8_t red, uint8_t green, uint8_t blue);
void pwm_set_frame(uint8_t frame[]);

#endif
