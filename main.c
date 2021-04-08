#include "common.h"
#include "hardware.h"
#include "radio.h"
#include "timer.h"
#include "commands.h"
#include "serial.h"

void main(void)
{
    // Set the system clock source to HS XOSC and max CPU speed,
  // ref. [clk]=>[clk_xosc.c]
  SLEEP &= ~SLEEP_OSC_PD;
  while( !(SLEEP & SLEEP_XOSC_S) );
  CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
  while (CLKCON & CLKCON_OSC);
  SLEEP |= SLEEP_OSC_PD;

  init_gpios();
  led_set(true);
  // Global interrupt enable
  init_timer();
  configure_serial();
  configure_radio();
  led_set(false);
  delay(200);
  led_set(true);
  delay(300);
  led_set(false);

  // Start watchdog at 1s interval
  WDCTL = WDCTL_EN;

  while(1) {
    get_command();
  }
}
