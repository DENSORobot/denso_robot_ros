#ifndef DN_UDP_H_
#define DN_UDP_H_

/**
 * @file    dn_udp.h
 * @brief   UDP API file.
 * @details Defines UDP APIs.
 *
 * @version 1.0
 * @date    2014/11/06
 * @author  DENSO WAVE
 *
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

#ifndef _DN_EXP_UDP
#define _DN_EXP_UDP
#endif /* _DN_EXP_UDP */

/**
 * @def   UDP_MAX_PACKET
 * @brief The maximum buffer size of a UDP packet.
 */
#define UDP_MAX_PACKET (503)

/**
 * @def   UDP_MAX_DATA
 * @brief The maximum data size of a UDP packet.
 */
#define UDP_MAX_DATA (488)

/**
 * @struct udpaddr
 * @brief  A type definition for parameters of udp_send and udp_recv.
 */
struct udpaddr
{
  int flag;                /**< Flag of send and receive. */
  struct sockaddr_in addr; /**< IPv4 Address.             */
};

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn         HRESULT udp_open(void *param, int *sock)
   * @brief      Opens UDP socket.
   * @param[in]  param The pointer of Ethernet connection parameters: CONN_PARAM_ETH.
   * @param[out] sock The created socket.
   */
  _DN_EXP_UDP HRESULT
  udp_open(void *param, int *sock);

  /**
   * @fn            HRESULT udp_close(int *sock)
   * @brief         Closes the socket.
   * @param[in,out] sock The socket to be closed.
   */
  _DN_EXP_UDP HRESULT
  udp_close(int *sock);

  /**
   * @fn        HRESULT udp_send(int sock, const char *buf, uint32_t len_buf, void *arg)
   * @brief     Sends UDP packet.
   * @param[in] sock The socket to send.
   * @param[in] buf The buffer to be sent.
   * @param[in] len_buf The size of sent buffer.
   * @param[in] arg Special parameter: udpaddr.
   */
  _DN_EXP_UDP HRESULT
  udp_send(int sock, const char *buf, uint32_t len_buf, void *arg);

  /**
   * @fn         HRESULT udp_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved, uint32_t timeout, void *arg)
   * @brief      Receives UDP packet.
   * @param[in]  sock The socket to receive.
   * @param[out] buf The buffer to be received.
   * @param[in]  len_buf The allocated size of received buffer.
   * @param[out] len_recved The size of received buffer.
   * @param[in]  arg Special parameter: udpaddr.
   */
  _DN_EXP_UDP HRESULT
  udp_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved,
      uint32_t timeout, void *arg);

  /**
   * @fn        HRESULT udp_set_timeout(int sock, uint32_t timeout)
   * @brief     Sets timeout value to the UDP socket.
   * @param[in] sock The socket to be set.
   * @param[in] timeout Timeout value.
   */
  _DN_EXP_UDP HRESULT
  udp_set_timeout(int sock, uint32_t timeout);

  /**
   * @fn        HRESULT udp_clear(int sock, uint32_t timeout)
   * @brief     Clears the received buffer.
   * @param[in] sock The socket to be cleared.
   * @param[in] timeout Timeout value.
   */
  _DN_EXP_UDP HRESULT
  udp_clear(int sock, uint32_t timeout);

  /**
   * @fn        HRESULT udp_check_sockaddr(const struct sockaddr_in *sock_to, const struct sockaddr_in *sock_from)
   * @brief     Checks the socket address. If sock_to and sock_from are equivalent, then returns S_OK.
   * @param[in] sock_to The socket address of sender.
   * @param[in] sock_from The socket address of receiver.
   */
  _DN_EXP_UDP HRESULT
  udp_check_sockaddr(const struct sockaddr_in *sock_to,
      const struct sockaddr_in *sock_from);

#ifdef __cplusplus
}
#endif

#endif /* DN_UDP_H_ */
