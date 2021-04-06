#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <ioCCxx10_bitdef.h>
#include <cc1110.h>
#include "timer.h"
#include "hardware.h"
#include "common.h"

#define UART_BUF_LEN 128
#define FLUSH_TIMEOUT_MS 5000

static volatile uint8_t input_buffer_read_idx = 0;
static volatile uint8_t input_buffer_write_idx = 0;
static uint8_t __xdata input_buffer_mem[UART_BUF_LEN];
static volatile uint8_t ready_to_send = 0;

static volatile uint8_t output_buffer_read_idx = 0;
static volatile uint8_t output_buffer_write_idx = 0;
static uint8_t __xdata output_buffer_mem[UART_BUF_LEN];
volatile uint8_t serial_data_available;

void configure_serial()
{
}

void rx0_isr(void) __interrupt URX0_VECTOR
{
}

void tx0_isr(void) __interrupt UTX0_VECTOR
{
}

uint8_t serial_rx_avail()
{
  int remaining = input_buffer_write_idx - input_buffer_read_idx;
  if (remaining > 0) {
    return (uint8_t) remaining;
  } else {
    return 0;
  }
}

uint8_t serial_rx_byte()
{
  uint8_t s_data;
  if (!serial_data_available) {
    while(!serial_data_available) {
      feed_watchdog();
    }
  }
  s_data = input_buffer_mem[input_buffer_read_idx++];
  if (input_buffer_read_idx == input_buffer_write_idx) {
    serial_data_available = 0;
    input_buffer_read_idx = input_buffer_write_idx = 0;
  }
  return s_data;
}

uint16_t serial_rx_word()
{
  return (serial_rx_byte() << 8) + serial_rx_byte();
}

uint32_t serial_rx_long()
{
  return ((uint32_t)serial_rx_word() << 16) + serial_rx_word();
}

void serial_tx_byte(uint8_t tx_byte)
{
  output_buffer_mem[output_buffer_write_idx++] = tx_byte;
}

void serial_tx_word(uint16_t tx_word)
{
  serial_tx_byte(tx_word >> 8);
  serial_tx_byte(tx_word & 0xff);
}

void serial_tx_long(uint32_t tx_long)
{
  serial_tx_byte(tx_long >> 24);
  serial_tx_byte((tx_long >> 16) & 0xff);
  serial_tx_byte((tx_long >> 8) & 0xff);
  serial_tx_byte(tx_long & 0xff);
}

void serial_flush()
{
  uint32_t start_time;

  if (output_buffer_write_idx == 0) {
    return;
  }

  // Waiting for tx isr to send the data
  read_timer(&start_time);
  ready_to_send = 1;
  while(output_buffer_read_idx != output_buffer_write_idx) {
    feed_watchdog();
    if (check_elapsed(start_time, FLUSH_TIMEOUT_MS)) {
      break;
    }
  }

  output_buffer_read_idx = output_buffer_write_idx = 0;

  ready_to_send = 0;
}

void serial_tx_str(const char *str) {
  while(*str != 0) {
    serial_tx_byte(*str);
    str++;
  }
  serial_flush();
}
