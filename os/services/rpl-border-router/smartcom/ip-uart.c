/**
 * \file
 *         IP fallback interface UART driver
 * \author
 *         Raphael Löffel <loeffel@rte-ag.ch>
 */
/*---------------------------------------------------------------------------*/
#include "ip-uart.h"

#include "contiki-net.h"
#include "net/ipv6/uip.h"

#include <Board.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>


/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "IPUART"
#define LOG_LEVEL LOG_LEVEL_NONE
/*---------------------------------------------------------------------------*/

#define POLL_INTERVAL   5       /**< */
#define BUFFER_SIZE     256     /**< */

extern uint16_t uip_len;

typedef enum {
    eDrvUartStateIdle = 0,
    eDrvUartStateReceive,
    eDrvUartStateReceiveComplete
} EDrvUartState_t;


static bool initialized;
static UART_Handle uart_handle;
//static struct etimer et;
static GPTimerCC26XX_Handle timer_handle;

static EDrvUartState_t rx_state;
static uint32_t rx_cnt = 0;
static uint8_t rx_char_buf;
static uint8_t rx_buf[BUFFER_SIZE];
static uint8_t tx_buf[BUFFER_SIZE];
static GPTimerCC26XX_Value rx_load_val;

PROCESS(ip_uart_process, "IP UART interface");
process_event_t uart_rx_event;
//process_event_t uart_tx_event;
//AUTOSTART_PROCESSES(&ip_uart_process);

/*---------------------------------------------------------------------------*/
static void ip_uart_cb(UART_Handle handle, void *buf, size_t count) {
  bool new_data = false;

  // Timeout handling
  switch (rx_state) {
  case eDrvUartStateIdle:
    GPTimerCC26XX_setLoadValue(timer_handle, rx_load_val);
    GPTimerCC26XX_start(timer_handle);
    rx_cnt = 0;
    rx_state = eDrvUartStateReceive;
    new_data = true;
    break;
  case eDrvUartStateReceive:
    GPTimerCC26XX_setLoadValue(timer_handle, rx_load_val);
    new_data = true;
    break;
  case eDrvUartStateReceiveComplete:
    break;
  }

  // Get received data byte
  if (true == new_data) {
    if (sizeof(rx_buf) > rx_cnt) {
      rx_buf[rx_cnt] = rx_char_buf;
      rx_cnt++;
    }
    // Start next read
    UART_read(uart_handle, &rx_char_buf, 1);
  }
}

static void ip_uart_timer_cb(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
  // Stop timer
  GPTimerCC26XX_stop(timer_handle);

  rx_state = eDrvUartStateReceiveComplete;
  // Cancel reading until received data got proceeded
  UART_readCancel(uart_handle);
  // Send event to proceed received data
  process_post(PROCESS_BROADCAST, uart_rx_event, NULL);
}

void ip_uart_process_data( void )
{
  uint32_t len = uip_len;
  memcpy(tx_buf, uip_buf, len);
  uipbuf_clear();
  UART_write(uart_handle, tx_buf, len);
}

bool ip_uart_init( void )
{
  GPTimerCC26XX_Params timer_params;
  UART_Params uart_params;

  if(initialized) {
    return initialized;
  }

  rx_state = eDrvUartStateIdle;
  rx_cnt = 0;

  GPTimerCC26XX_Params_init(&timer_params);
  timer_params.width          = GPT_CONFIG_32BIT;
  timer_params.mode           = GPT_MODE_ONESHOT;
  timer_params.direction      = GPTimerCC26XX_DIRECTION_DOWN;
  timer_params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_ON;
  timer_handle = GPTimerCC26XX_open(CC1352R1_SMARTCOM_GPTIMER0A, &timer_params);
  if(NULL == timer_handle) {
    return initialized;
  }
  ClockP_FreqHz freq;
  ClockP_getCpuFreq(&freq);
//    rx_load_val = 48000000 / TI_UART_CONF_BAUD_RATE * 11 * 3;
  rx_load_val = 480000;
  GPTimerCC26XX_setLoadValue(timer_handle, rx_load_val);
  GPTimerCC26XX_registerInterrupt(timer_handle, ip_uart_timer_cb, GPT_INT_TIMEOUT);
//    GPTimerCC26XX_start(timer_handle);

  UART_Params_init(&uart_params);
  uart_params.baudRate = TI_UART_CONF_BAUD_RATE;
  uart_params.readMode = UART_MODE_CALLBACK;
  uart_params.writeMode = UART_MODE_BLOCKING;
  uart_params.readTimeout = 1000;
//    uart_params.writeTimeout ;
  uart_params.readCallback = ip_uart_cb;
  uart_params.readDataMode = UART_DATA_BINARY;
  uart_handle = UART_open(Board_UART1, &uart_params);
  if (NULL == uart_handle) {
    return initialized;
  }

  initialized = true;

  return initialized;
}

int_fast32_t ip_uart_read(void *buf, size_t buf_size)
{
  if(!initialized) {
    return UART_STATUS_ERROR;
  }
  return UART_read(uart_handle, buf, buf_size);
}

int_fast32_t ip_uart_write(const void *buf, size_t buf_size)
{
  if(!initialized) {
    return UART_STATUS_ERROR;
  }
  return UART_write(uart_handle, buf, buf_size);
}

void ip_uart_send(void)
{
  uint32_t len = uip_len;
  memcpy(tx_buf, uip_buf, len);
  UART_write(uart_handle, tx_buf, len);
}

static void (*input_callback)(void) = NULL;
void ip_uart_set_input_callback(void (*c)(void)) {
  input_callback = c;
}

void ip_uart_start(void)
{
  process_start(&ip_uart_process, NULL);
}

PROCESS_THREAD(ip_uart_process, ev, data)
{
  PROCESS_BEGIN();

  uart_rx_event = process_alloc_event();

  // Start reading
  UART_read(uart_handle, &rx_char_buf, 1);

  while (true) {
    PROCESS_WAIT_EVENT();
    // RX event
    if(ev == uart_rx_event) {
      // Send received data from host CPU to 6loWPAN stack
      uip_len = rx_cnt;
      memmove(&uip_buf[0], rx_buf, uip_len);
      if(input_callback) {
        input_callback();
      }
      tcpip_input();
      // Start reading
      rx_state = eDrvUartStateIdle;
      UART_read(uart_handle, &rx_char_buf, 1);
    }
  }

  PROCESS_END();
}
