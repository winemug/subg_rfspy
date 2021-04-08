#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <ioCCxx10_bitdef.h>
#include <cc1110.h>
#include "timer.h"
#include "hardware.h"
#include "common.h"

#define UART_BUF_SIZE 255

static volatile uint8_t __xdata uart_rx_buf[UART_BUF_SIZE + 1];
static volatile uint8_t __xdata uart_rx_idx = 0;
static volatile uint8_t __xdata uart_rx_finished = 0;

static volatile uint8_t __xdata uart_tx_buf[UART_BUF_SIZE + 1];
static volatile uint8_t __xdata uart_tx_idx = 0;
static volatile uint8_t __xdata uart_tx_finished = 0;


#define FLUSH_TIMEOUT_MS 5000

// Baudrate = 38400
#define UART_BAUD_M  131
#define UART_BAUD_E  10

void rx0_isr(void) __interrupt URX0_VECTOR
{
  URX0IF = 0;
  uart_rx_buf[uart_rx_idx++] = U0DBUF;
  if (uart_rx_idx > uart_rx_buf[0])
  {
    // stop rx
    URX0IE = 0;
    uart_rx_finished = 1;
  }
}

void tx0_isr(void) __interrupt UTX0_VECTOR
{
  UTX0IF = 0;
  if (uart_tx_idx > uart_tx_buf[0])
  {
    // stop tx
    IEN2 &= ~IEN2_UTX0IE;
    uart_tx_finished = 1;
  }
  else
  {
    U0DBUF = uart_tx_buf[uart_tx_idx++];
  }
}

void uart_start_tx()
{
  IEN2 &= ~IEN2_UTX0IE;
  UTX0IF = 0;
  U0CSR &= ~U0CSR_TX_BYTE;
  uart_tx_finished = 0;

  uart_tx_buf[0] = uart_tx_idx;
  U0DBUF = uart_tx_idx;
  if (uart_tx_idx == 0)
  {
    uart_tx_finished = 1;
  }
  else
  {
    uart_tx_idx = 1;
    EA = 1;
    IEN2 |= IEN2_UTX0IE;
  }
}

void uart_start_rx()
{
  URX0IE = 0;
  URX0IF = 0;
  U0CSR &= ~U0CSR_RX_BYTE;
  uart_rx_idx = 0;
  uart_rx_finished = 0;
  U0CSR |= U0CSR_RE;
  EA = 1;
  URX0IE = 1;
}

void configure_serial()
{
  PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;
  P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;
  // set baudrate
  U0BAUD = UART_BAUD_M;
  U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;
  // set usart mode uart
  U0CSR |= U0CSR_MODE;
  // set start bit level low, stop bit level high, 8n1, hw flow disabled
  U0UCR &= ~(U0UCR_START | U0UCR_SPB | U0UCR_PARITY | U0UCR_BIT9 | U0UCR_D9 | U0UCR_FLOW);
  U0UCR |= U0UCR_STOP;
  // lsb first
  U0GCR &= ~U0GCR_ORDER;
}

void uart_get_cmd()
{
  uart_start_rx();
  while(!uart_rx_finished)
    feed_watchdog();
  uart_rx_idx = 1;
}

uint8_t serial_rx_avail()
{
  if (!uart_rx_finished)
    return 0;
  return uart_rx_buf[0] - uart_rx_idx + 1;
}

uint8_t serial_rx_byte()
{
  return uart_rx_buf[uart_rx_idx++];
}

uint16_t serial_rx_word()
{
  return (serial_rx_byte() << 8) + serial_rx_byte();
}

uint32_t serial_rx_long()
{
  return ((uint32_t)serial_rx_word() << 16) + serial_rx_word();
}

void serial_flush()
{
  uart_start_tx();
  led_set(true);
  while(uart_tx_finished == 0)
    feed_watchdog();
  led_set(false);
  uart_tx_idx = 0;
}

void serial_tx_byte(uint8_t tx_byte)
{
  uart_tx_buf[++uart_tx_idx] = tx_byte;
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

void serial_tx_str(const char *str) {
  while(*str != 0) {
    serial_tx_byte(*str);
    str++;
  }
  serial_flush();
}
