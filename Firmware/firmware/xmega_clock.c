#include <avr/io.h>
#include "xmega_clock.h"

int xmega_clock_select_32mhz(void)
{

	//Enable oscillator:
	OSC.CTRL |= OSC_RC32MEN_bm;

	//Wait to ready?
	//TODO: TIMEOUT?
	while ((OSC.STATUS & OSC_RC32MRDY_bm) == 0);

	//Select 32Mhz:
	CPU_CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;

	//Disable 2MHz:
	OSC.CTRL &= ~(OSC_RC2MEN_bm); 

	//Success!
	return 0;
}

int xmega_clock_select_ext(void)
{
	//Enable external oscillator:
	OSC.XOSCCTRL = OSC_XOSCSEL_XTAL_16KCLK_gc | OSC_FRQRANGE_12TO16_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;

	//Wait to ready?
	while ((OSC.STATUS & OSC_XOSCRDY_bm) == 0);

	//Select external:
	CPU_CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_XOSC_gc;

	//Disable 2MHz:
	OSC.CTRL &= ~(OSC_RC2MEN_bm); 

	//Success!
	return 0;
	
}
