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


/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "IPUART"
#define LOG_LEVEL LOG_LEVEL_ERR
/*---------------------------------------------------------------------------*/

#define BUFFER_SIZE       256       /**< TX and RX buffer size in bytes*/
#define MODBUS_CRC_POLY   0xA001    /**< Modbus CRC polynomial ( note: 0x8005 bit reversed )*/

extern uint16_t uip_len;

static volatile bool initialized;
static UART_Handle uart_handle;

static volatile int32_t rx_cnt = 0;
static uint8_t rx_buf[BUFFER_SIZE + sizeof(uint16_t)];
static uint8_t tx_buf[BUFFER_SIZE + sizeof(uint16_t)];

PROCESS(ip_uart_process, "IP UART interface");
process_event_t uart_rx_event;

/*---------------------------------------------------------------------------*/
void ip_uart_crc_modbus_init( volatile uint16_t * ptrToCrc )
{

  *ptrToCrc = 0xFFFF;
}

void ip_uart_crc_modbus_update( volatile uint16_t * ptrToCrc, uint8_t value )
{
  uint8_t     i;
  uint16_t    tempCrc;

  tempCrc = *ptrToCrc;
  /* Calc CRC */
  for (i=0, tempCrc ^= (uint16_t)value ; i < 8; i++) {
    if (tempCrc & 0x0001) {
      tempCrc = (tempCrc >> 1) ^ MODBUS_CRC_POLY;
    }
    else {
      tempCrc >>= 1;
    }
  }
  *ptrToCrc = tempCrc;
}

void ip_uart_crc_modbus_calc( volatile uint16_t * ptrToCrc, uint8_t * ptrToValue,  uint16_t numBytes )
{

  ip_uart_crc_modbus_init( ptrToCrc );

  while( numBytes-- ) {
    ip_uart_crc_modbus_update( ptrToCrc, *ptrToValue++ );
  }
}

static void (*input_callback)(void) = NULL;
void ip_uart_set_input_callback(void (*c)(void))
{
  input_callback = c;
}

static void ip_uart_cb(UART_Handle handle, void *buf, size_t count)
{
    rx_cnt = count;
    // Send event to proceed received data
    process_post(PROCESS_BROADCAST, uart_rx_event, NULL);
}

bool ip_uart_init( void )
{
  UART_Params uart_params;

  if(initialized) {
    return initialized;
  }

  // Initialize uart
  UART_Params_init(&uart_params);
  uart_params.baudRate = TI_UART_CONF_BAUD_RATE;
  uart_params.readMode = UART_MODE_CALLBACK;
  uart_params.writeMode = UART_MODE_BLOCKING;
// uart_params.readTimeout = 1000;
// uart_params.writeTimeout ;
  uart_params.readCallback = ip_uart_cb;
  uart_params.readDataMode = UART_DATA_BINARY;
  uart_handle = UART_open(Board_UART1, &uart_params);
  if (NULL == uart_handle) {
    return initialized;
  }

  // Configure partial read
  UART_control(uart_handle, UART_CMD_RESERVED + 0, NULL);

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
  if ((sizeof(tx_buf) - sizeof(uint16_t)) >= uip_len) {
    // Copy data to transmit
    uint32_t len = uip_len;
    memcpy(tx_buf, uip_buf, len);
    // Add CRC
    ip_uart_crc_modbus_calc((uint16_t *)&tx_buf[len], tx_buf, len);
    int32_t ret = UART_write(uart_handle, tx_buf, len + sizeof(uint16_t));
    if (0 == ret) {
      LOG_ERR("ip-uart: send failed: %d\n", (int)ret);
    }
  }
  else {
    LOG_ERR("ip-uart: send uip_buf too large\n");
  }
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
  rx_cnt = 0;
  memset(rx_buf, 0, sizeof(rx_buf));
  UART_read(uart_handle, rx_buf, sizeof(rx_buf));

  while (true) {
    PROCESS_WAIT_EVENT();
    // RX event
    if(ev == uart_rx_event) {
      // Send received data from host CPU to 6loWPAN stack
      if ((0 < rx_cnt) && (sizeof(uip_buf) >= rx_cnt)) {
        uint16_t crc;
        // Check CRC
        ip_uart_crc_modbus_calc(&crc, rx_buf, rx_cnt);
        if (0 == crc) {
          uip_len = rx_cnt;
          memmove(&uip_buf[0], rx_buf, uip_len);
          if(input_callback) {
            input_callback();
          }
          tcpip_input();
        }
        else {
          LOG_ERR("ip-uart: receive rx_buf crc void len=%ld ", rx_cnt);
          LOG_INFO_("dest=");
          LOG_INFO_6ADDR(&((struct uip_ip_hdr *)rx_buf)->destipaddr);
          LOG_ERR_("\n");
        }
      }
      else {
        LOG_ERR("ip-uart: receive rx_buf too large\n");
      }
      // Start reading
      rx_cnt = 0;
      memset(rx_buf, 0, sizeof(rx_buf));
      UART_read(uart_handle, rx_buf, sizeof(rx_buf));
    }
  }

  PROCESS_END();
}
