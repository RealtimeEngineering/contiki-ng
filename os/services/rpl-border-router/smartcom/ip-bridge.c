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
void get_ipv6(uip_ipaddr_t *);
int32_t get_version_sw(void);
void set_panid(uint16_t panid);

static uip_ipaddr_t last_sender;

/*---------------------------------------------------------------------------*/
static uint8_t
get_checksum(const uint8_t *pData, uint8_t size)
{
  uint32_t checksum = 0;

  while (size) {
    checksum += *pData;
    pData++;
    size--;
  }

  return (checksum % 0x100);
}
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
    if(uip_buf[0] == '!') {
      LOG_INFO("Got configuration message of type %c\n",
               uip_buf[1]);
      if(uip_buf[1] == 'P') {
        uip_ipaddr_t prefix;
        /* Here we set a prefix !!! */
        memset(&prefix, 0, 16);
        memcpy(&prefix, &uip_buf[2], 8);
        LOG_INFO("Setting prefix ");
        LOG_INFO_6ADDR(&prefix);
        LOG_INFO_("\n");
        set_prefix_64(&prefix);
      }
      else if(uip_buf[1] == 'I') {
        uint16_t panid;
        /* Here we set the PANID !!! */
        memcpy(&panid, &uip_buf[2], sizeof(panid));
        uint8_t checksum = get_checksum(uip_buf, 2+sizeof(panid));
        if (checksum == uip_buf[2+sizeof(panid)]) {
          LOG_INFO("Setting PANID %d ", panid);
          LOG_INFO_("\n");
          set_panid(panid);
        }
      }
      uipbuf_clear();
    } else if(uip_buf[0] == '?') {
      LOG_INFO("Got request message of type %c\n", uip_buf[1]);
      if(uip_buf[1] == 'V') {
        uint32_t version = get_version_sw();
        /* here we return the version of the node */
        uip_buf[0] = '?';
        uip_buf[1] = 'V';
        uip_buf[2] = version & 0x000000FF;
        uip_buf[3] = (version >> 8) & 0x000000FF;
        uip_buf[4] = (version >> 16) & 0x000000FF;
        uip_buf[5] = (version >> 24) & 0x000000FF;
        uip_buf[6] = get_checksum(uip_buf, 6);
        uip_len = 7;
        ip_uart_send();
      }
      else if(uip_buf[1] == 'I') {
        uip_ipaddr_t ip;
        get_ipv6(&ip);
        /* here we return the IPv6 of the node */
        uip_buf[0] = '?';
        uip_buf[1] = 'I';
        memcpy(&uip_buf[2], &ip.u8[0], 16);
        uip_buf[18] = get_checksum(uip_buf, 18);
        uip_len = 19;
        ip_uart_send();
      }
      uipbuf_clear();
    } else {
      /* Save the last sender received over SLIP to avoid bouncing the
         packet back if no route is found */
      uip_ipaddr_copy(&last_sender, &UIP_IP_BUF->srcipaddr);
    }

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
