/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2015 DENSO WAVE INCORPORATED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "stdint.h"
#include <stdlib.h>
#include <string.h>

#if defined(_USE_WIN_API)
#include <windows.h>
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#else
#include "dn_additional.h"
#endif

#include "dn_common.h"
#include "dn_device.h"
#include "dn_udp.h"
#include "dn_robotalk.h"

/**
 * @fn        HRESULT rtk_send_nak(const struct CONN_PARAM_COMMON *device, uint8_t from_id, uint8_t to_id)
 * @brief     Sends a NAK packet.
 * @param[in] device The common communication parameters.
 * @param[in] frm_id The id of host.
 * @param[in] to_id The id of the target client.
 */
static HRESULT
rtk_send_nak(const struct CONN_PARAM_COMMON *device, uint8_t from_id,
    uint8_t to_id)
{
  union RTK_PACKET *packet = NULL;
  HRESULT hr;

  packet = (union RTK_PACKET *) malloc(RTK_SIZE_HEADER + RTK_SIZE_CRC);
  if (packet == NULL)
    return E_OUTOFMEMORY;

  hr = rtk_param2packet(RTK_CMD_NAK, NULL, 0, from_id, to_id, packet);
  if (FAILED(hr))
    goto exit_proc;

  hr = rtk_send(device, packet);

exit_proc:
  if (packet != NULL) {
    free(packet);
    packet = NULL;
  }

  return hr;
}

/**
 * @fn         HRESULT rtk_param2packet(uint16_t command, const uint8_t *data, uint8_t len_data, uint8_t from_id, uint8_t to_id, union RTK_PACKET *dst)
 * @brief      Creates a ROBOTalk packet.
 * @param[in]  command The ROBOTalk command.
 * @param[in]  data The ROBOTalk data.
 * @param[in]  len_data The length of data.
 * @param[in]  from_id The id of host.
 * @param[in]  to_id The id of the target client.
 * @param[out] packet The created ROBOTalk packet.
 */
HRESULT
rtk_param2packet(uint16_t command, const uint8_t *data, uint8_t len_data,
    uint8_t from_id, uint8_t to_id, union RTK_PACKET *packet)
{
  if (packet == NULL)
    return E_INVALIDARG;
  if (len_data > 0 && data == NULL)
    return E_INVALIDARG;
  if (len_data > RTK_SIZE_DATA - RTK_SIZE_CRC)
    return E_INVALIDARG;

  packet->enq = RTK_ENQ;
  packet->len = len_data;

  /* Orders to ROBOTalk endian */
  memcpy_be(&packet->command, &command, RTK_SIZE_COMMAND);

  packet->from_id = from_id;
  packet->to_id = to_id;

  if (len_data > 0) {
    memcpy(packet->data, data, len_data);
  }

  return S_OK;
}

/**
 * @fn        uint16_t rtk_calc_crc(const union RTK_PACKET *packet)
 * @brief     Calculates CRC of the ROBOTalk packet.
 * @param[in] packet The ROBOTalk packet to be calculated.
 */
uint16_t
rtk_calc_crc(const union RTK_PACKET *packet)
{
  uint8_t pos, len_data;
  uint16_t crc = 0;

  if (packet != NULL) {
    len_data =
        (packet->len > (RTK_SIZE_DATA - RTK_SIZE_CRC)) ?
            (RTK_SIZE_DATA - RTK_SIZE_CRC) : packet->len;

    for (pos = 0; pos < RTK_SIZE_HEADER + len_data; pos++) {
      crc += (uint16_t) packet->buf[pos];
    }
  }

  return crc;
}

/**
 * @fn        HRESULT rtk_send(const struct CONN_PARAM_COMMON *device, union RTK_PACKET *packet_send)
 * @brief     Sends ROBOTalk packet.
 * @param[in] device The common communication parameters.
 * @param[in] packet_send The ROBOTalk packet to be sent.
 */
HRESULT
rtk_send(const struct CONN_PARAM_COMMON *device, union RTK_PACKET *packet_send)
{
  uint16_t crc;
  uint32_t len_send;
  void *parg = NULL;
  struct udpaddr opt_udp;
  HRESULT hr;

  hr = check_conn_param(device, RTK_CHECK_SEND);
  if (FAILED(hr))
    return hr;

  if (packet_send == NULL)
    return E_INVALIDARG;

  if (device->type == CONN_UDP) {
    if (device->arg == NULL)
      return E_INVALIDARG;
    opt_udp.flag = 0;
    opt_udp.addr = *(struct sockaddr_in *) device->arg;
    parg = &opt_udp;
  }

  len_send = RTK_SIZE_HEADER + RTK_SIZE_CRC + packet_send->len;
  if (len_send > RTK_SIZE_PACKET) {
    hr = E_TOO_MUCH_DATA;
    goto send_end;
  }

  /* Sets CRC */
  crc = rtk_calc_crc(packet_send);
  memcpy_be(&packet_send->data[packet_send->len], &crc, RTK_SIZE_CRC);

  hr = device->dn_send(device->sock, (const char *) packet_send->buf, len_send,
      parg);
  if (FAILED(hr))
    goto send_end;

  send_end: return hr;
}

/**
 * @fn         HRESULT rtk_recv(const struct CONN_PARAM_COMMON *device, union RTK_PACKET *packet_recv, int client, unsigned int retry_nak)
 * @brief      Receives ROBOTalk packet.
 * @param[in]  device The common communication parameters.
 * @param[out] packet_recv The ROBOTalk packet to be received.
 * @param[in]  client Flag that means the receiver is client(1) or not(0).
 * @param[in]  retry_nak The retry count when receiving the NAK packet.
 */
HRESULT
rtk_recv(const struct CONN_PARAM_COMMON *device, union RTK_PACKET *packet_recv,
    int client, unsigned int retry_nak)
{
  int flag_init = 0;
  unsigned int retry_cnt;
  uint16_t crc, crc_recv, command;
  uint32_t len_recv, len_recved, len_tmp;
  char *pos, buf_tmp[UDP_MAX_PACKET + 1] =
    { 0, };
  void *parg = NULL;
  struct udpaddr opt_udp;
  HRESULT hr;

  hr = check_conn_param(device, RTK_CHECK_RECV);
  if (FAILED(hr))
    return hr;

  if (packet_recv == NULL)
    return E_INVALIDARG;

  if (device->type == CONN_UDP) {
    if (device->arg == NULL)
      return E_INVALIDARG;
    opt_udp.flag = 0;
    parg = &opt_udp;
  }

  for (retry_cnt = 0; retry_cnt <= retry_nak; retry_cnt++) {
    len_recved = 0;
    while (1) {
      pos = (char *) memchr(buf_tmp, RTK_ENQ, len_recved);
      if (pos != NULL) {
        if (pos != buf_tmp) {
          len_recved -= ((long) pos - (long) buf_tmp);
          memcpy(buf_tmp, pos, len_recved);
        }
      } else {
        len_recved = 0;
      }

      if (len_recved >= RTK_SIZE_HEADER + RTK_SIZE_CRC)
        break;

      hr = device->dn_recv(device->sock, &buf_tmp[len_recved],
          UDP_MAX_PACKET - len_recved, &len_tmp, device->timeout, parg);

      if (device->type == CONN_UDP) {
        if (client || flag_init) {
          hr = SUCCEEDED(hr) ?
                  udp_check_sockaddr((struct sockaddr_in *) device->arg,
                      &opt_udp.addr) : hr;
        } else {
          flag_init = 1;
          *(struct sockaddr_in *) device->arg = opt_udp.addr;
        }
      }

      if (FAILED(hr))
        goto recv_end;

      len_recved += len_tmp;
    }

    len_recv = RTK_SIZE_HEADER + RTK_SIZE_CRC
        + ((union RTK_PACKET *) buf_tmp)->len;
    if (len_recv > RTK_SIZE_PACKET) {
      len_recved--;
      memcpy(buf_tmp, &buf_tmp[1], len_recved);
      continue;
    }

    memcpy(packet_recv->buf, buf_tmp,
        (len_recv < len_recved) ? len_recv : len_recved);

    while (len_recv > len_recved) {
      hr = device->dn_recv(device->sock,
          (char *) &packet_recv->buf[len_recved], len_recv - len_recved,
          &len_tmp, device->timeout, parg);

      if (device->type == CONN_UDP) {
        hr = SUCCEEDED(hr) ?
                udp_check_sockaddr((struct sockaddr_in *) device->arg,
                    &opt_udp.addr) : hr;
      }

      if (FAILED(hr))
        goto recv_end;

      len_recved += len_tmp;
    }

    /* Calculates CRC of the received packet */
    crc = rtk_calc_crc(packet_recv);
    memcpy_be(&crc_recv, &packet_recv->data[packet_recv->len], RTK_SIZE_CRC);

    /* Checks CRC */
    if (crc != crc_recv) {
      /* If CRC error occurs, then send NAK packet and retry */
      hr = rtk_send_nak(device, packet_recv->from_id, packet_recv->to_id);
      if (SUCCEEDED(hr)) {
        if (retry_cnt == retry_nak)
          hr = E_INVALIDPACKET;
        continue;
      }
    }

    /* Orders to host's endian */
    command = packet_recv->command;
    memcpy_be(&packet_recv->command, &command, RTK_SIZE_COMMAND);

    break;
  }

recv_end:
  return hr;
}
