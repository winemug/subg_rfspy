#ifndef HARDWARE_H
#define HARDWARE_H

#ifdef MOCK_HARDWARE
#include "mock_hardware.h"
#else
#include <cc1110.h>  // /usr/share/sdcc/include/mcs51/cc1110.h
#endif

#include "ioCCxx10_bitdef.h"
#include <stdint.h>

#define BIT0 0x1
#define BIT1 0x2
#define BIT2 0x4
#define BIT3 0x8
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define HARDWARE_LED_INIT P1DIR |= 2;
#define LED_PIN P1_1
#define SYSTEM_CLOCK_MHZ 26

#define MAX_MODE_REGISTERS 8

typedef struct register_setting {
	uint8_t	addr;
	uint8_t	value;
} register_setting;

typedef struct mode_registers {
  uint8_t count;
  register_setting registers[MAX_MODE_REGISTERS];
} mode_registers;

void init_gpios();
void led_set(uint8_t mode);

uint8_t get_register(uint8_t addr);
uint8_t set_register(uint8_t addr, uint8_t value);

void mode_registers_clear(mode_registers *mode);
void mode_registers_add(mode_registers *mode, uint8_t addr, uint8_t value);
void mode_registers_enact(mode_registers const *mode);

extern mode_registers __xdata tx_registers;
extern mode_registers __xdata rx_registers;

#define WDCLP1     0xA0  // Clear pattern 1
#define WDCLP2     0x50  // Clear pattern 2

inline void feed_watchdog() {
	WDCTL = WDCLP1 | WDCTL_EN;
	WDCTL = WDCLP2 | WDCTL_EN;
}

#endif
