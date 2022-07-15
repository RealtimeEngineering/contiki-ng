/**
 * \file
 *         IP fallback interface
 * \author
 *         Raphael Löffel <loeffel@rte-ag.ch>
 */
/*---------------------------------------------------------------------------*/
#include "net/ipv6/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "ip-uart.h"

#include <string.h>
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "SLIP"
#define LOG_LEVEL LOG_LEVEL_NONE
/*---------------------------------------------------------------------------*/
void set_prefix_64(uip_ipaddr_t *);

static uip_ipaddr_t last_sender;

/*---------------------------------------------------------------------------*/
void
request_prefix(void)
{
  /* mess up uip_buf with a dirty request... */
  uip_buf[0] = '?';
  uip_buf[1] = 'P';
  uip_len = 2;
  ip_uart_write(uip_buf, uip_len);
  uipbuf_clear();
}
/*---------------------------------------------------------------------------*/
static void
ip_input_callback(void)
{
  LOG_DBG("SIN: %u\n", uip_len);
  /* Save the last sender received over SLIP to avoid bouncing the
     packet back if no route is found */
  uip_ipaddr_copy(&last_sender, &UIP_IP_BUF->srcipaddr);
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  ip_uart_init();
  ip_uart_start();
  ip_uart_set_input_callback(ip_input_callback);
}
/*---------------------------------------------------------------------------*/
static int
output(void)
{
  if(uip_ipaddr_cmp(&last_sender, &UIP_IP_BUF->srcipaddr)) {
    /* Do not bounce packets back over SLIP if the packet was received
       over SLIP */
    LOG_ERR("slip-bridge: Destination off-link but no route src=");
    LOG_ERR_6ADDR(&UIP_IP_BUF->srcipaddr);
    LOG_ERR_(" dst=");
    LOG_ERR_6ADDR(&UIP_IP_BUF->destipaddr);
    LOG_ERR_("\n");
  } else {
    LOG_DBG("SUT: %u\n", uip_len);
    ip_uart_send();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
const struct uip_fallback_interface rpl_interface = {
  init, output
};
/*---------------------------------------------------------------------------*/
