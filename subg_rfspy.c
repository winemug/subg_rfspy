/* Control a cc1110 for sub-ghz RF comms over uart. */

#include <stdint.h>
#include <stdio.h>
#include "hardware.h"
#include "radio.h"
#include "timer.h"
#include "commands.h"
#include "subg_rfspy.h"

bool __xdata subg_rfspy_init_finished;
bool __xdata subg_rfspy_should_exit;

void subg_rfspy_main() {
  // Set the system clock source to HS XOSC and max CPU speed,
  // ref. [clk]=>[clk_xosc.c]
  SLEEP &= ~SLEEP_OSC_PD;
  while( !(SLEEP & SLEEP_XOSC_S) );
  CLKCON = (CLKCON & ~(CLKCON_CLKSPD | CLKCON_OSC)) | CLKSPD_DIV_1;
  while (CLKCON & CLKCON_OSC);
  SLEEP |= SLEEP_OSC_PD;

  init_gpios();

  // Global interrupt enable
  init_timer();
  EA = 1;

  configure_serial();
  configure_radio();

  //LED test
  led_set(true);
  delay(300);
  led_set(false);

  subg_rfspy_init_finished = true;

  // Start watchdog at 1s interval
  WDCTL = WDCTL_EN;

  while(!subg_rfspy_should_exit) {
    get_command();
  }
}
