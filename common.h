#ifndef __COMMON_H
#define __COMMON_H

#include <cc1110.h>

extern void t1_isr(void) __interrupt T1_VECTOR;
extern void rftxrx_isr(void) __interrupt RFTXRX_VECTOR;
extern void rf_isr(void) __interrupt RF_VECTOR;

extern void rx0_isr(void) __interrupt URX0_VECTOR;
extern void tx0_isr(void) __interrupt UTX0_VECTOR;

#endif