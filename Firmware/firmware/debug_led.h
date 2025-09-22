#ifndef __LED_H__
#define __LED_H__
 
/*
    PA1 DEBUG1
    PA2 DEBUG2
    PA3 DEBUG3
*/
#include <avr/io.h>

#define DEBUG1_bm (1 << 1)
#define DEBUG2_bm (1 << 2)
#define DEBUG3_bm (1 << 3)

#define debug_led_init() (PORTA.DIRSET |= DEBUG1_bm| DEBUG2_bm| DEBUG3_bm)

#define debug_led1_set() (PORTA.OUTSET |= DEBUG1_bm)
#define debug_led2_set() (PORTA.OUTSET |= DEBUG2_bm)
#define debug_led3_set() (PORTA.OUTSET |= DEBUG3_bm)

#define debug_led1_clr() (PORTA.OUTCLR |= DEBUG1_bm)
#define debug_led2_clr() (PORTA.OUTCLR |= DEBUG2_bm)
#define debug_led3_clr() (PORTA.OUTCLR |= DEBUG3_bm)

#define debug_led1_tgl() (PORTA.OUTTGL |= DEBUG1_bm)
#define debug_led2_tgl() (PORTA.OUTTGL |= DEBUG2_bm)
#define debug_led3_tgl() (PORTA.OUTTGL |= DEBUG3_bm)

#endif
