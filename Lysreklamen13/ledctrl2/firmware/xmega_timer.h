#ifndef XMEGA_TIMER_H
#define XMEGA_TIMER_H

#define xmega_timer_prescale(timer, div) ((timer)->CTRLA = (div))
#define xmega_timer_period(timer, per) ((timer)->PER = (per))
#define xmega_timer_ovf_interrupt(timer, lvl) ((timer)->INTCTRLA = ((timer)->INTCTRLA&~0x03) | (lvl))

#define xmega_timer_cca(timer, v) ((timer)->CCABUF = (v))
#define xmega_timer_ccb(timer, v) ((timer)->CCBBUF = (v))
#define xmega_timer_ccc(timer, v) ((timer)->CCCBUF = (v))
#define xmega_timer_ccd(timer, v) ((timer)->CCDBUF = (v))

#define xmega_timer_read_cca(timer) ((timer)->CCABUF)
#define xmega_timer_read_ccb(timer) ((timer)->CCBBUF)
#define xmega_timer_read_ccc(timer) ((timer)->CCCBUF)
#define xmega_timer_read_ccd(timer) ((timer)->CCDBUF)

#define xmega_timer_wgm(timer, mode) ((timer)->CTRLB = ((timer)->CTRLB&~TC0_WGMODE_gm) | (mode))

#define xmega_timer_enable_a(timer) ((timer)->CTRLB = ((timer)->CTRLB | TC0_CCAEN_bm));
#define xmega_timer_enable_b(timer) ((timer)->CTRLB = ((timer)->CTRLB | TC0_CCBEN_bm));
#define xmega_timer_enable_c(timer) ((timer)->CTRLB = ((timer)->CTRLB | TC0_CCCEN_bm));
#define xmega_timer_enable_d(timer) ((timer)->CTRLB = ((timer)->CTRLB | TC0_CCDEN_bm));

#define xmega_timer_get_cnt(timer) ((timer)->CNT)

#endif
