#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#include "xmega_clock.h"
#include "debug_led.h"

bool wait_for_clock(uint8_t mask, uint16_t timeout)
{
	// Wait for osc ready flag, timing out after ~200*timeout cycles
	uint16_t time = 0;
	while(time++ < timeout){
		if (OSC.STATUS & mask){
			return true;
		}
		_delay_loop_1(200/3);
	}
	return false;
}


bool xmega_clock_start_32mhz(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm; // |OSC_RC32KEN_bm
	//Wait for ready
	if (!wait_for_clock(OSC_RC32MRDY_bm, 1000)){ // 200 000 cycles, or ~100ms at 2MHz
		return false;
	}
	if (!wait_for_clock(OSC_RC32KRDY_bm, 1000)){
		return false;
	}
	// TODO: test:
	// DFLLRC32M.CTRL |= DFLL_ENABLE_bm;	
	return true;
}

void xmega_clock_select_32mhz(void)
{
	//Select 32Mhz:
	CPU_CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
}


bool xmega_clock_start_xtal(void)
{
	OSC.XOSCCTRL = OSC_XOSCSEL_XTAL_16KCLK_gc | OSC_FRQRANGE_12TO16_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;

	if (!wait_for_clock(OSC_XOSCRDY_bm, 10000)){
		return false;
	}

	// setup PLL
	OSC.PLLCTRL |= OSC_PLLSRC_XOSC_gc | (2 << OSC_PLLFAC_gm);
	
	if (!wait_for_clock(OSC_PLLRDY_bm, 10000)){
		return false;
	}
	return true;
}

void xmega_clock_select_xtal(void)
{
	// setup monitor
	CPU_CCP = CCP_IOREG_gc;
	OSC.XOSCFAIL |= OSC_XOSCFDEN_bm;

	//Select external:
	CPU_CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}

int xmega_clock_init(void)
{
	if (!xmega_clock_start_32mhz()){
		return false;
	}
	xmega_clock_select_32mhz();
	
	// todo: test
	// if (xmega_clock_start_xtal){
	// 	xmega_clock_select_xtal();
	// }
}

ISR(OSC_XOSCF_vect)
{
	// Dette betyr at oscillatoren ikke virker
	debug_led3_set();
	xmega_clock_select_32mhz();
}