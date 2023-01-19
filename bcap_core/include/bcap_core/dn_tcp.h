#ifndef DN_TCP_H_
#define DN_TCP_H_

/**
 * @file    dn_tcp.h
 * @brief   TCP API file.
 * @details Defines TCP APIs.
 *
 * @version 1.1
 * @date    2014/11/06
 * @date    2016/12/15 Adds connection timeout function for tcp_open_client.
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

#ifndef _DN_EXP_TCP
#define _DN_EXP_TCP
#endif /* _DN_EXP_TCP */

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn         HRESULT tcp_open_client(void *param, int *sock)
   * @brief      Opens TCP client.
   * @param[in]  param The pointer of Ethernet connection parameters: CONN_PARAM_ETH.
   * @param[out] sock The created socket.
   */
  _DN_EXP_TCP HRESULT
  tcp_open_client(void *param, int *sock);

  /**
   * @fn         HRESULT tcp_open_server(void *param, int *sock)
   * @brief      Opens TCP server.
   * @param[in]  param The pointer of Ethernet connection parameters: CONN_PARAM_ETH.
   * @param[out] sock The created socket.
   */
  _DN_EXP_TCP HRESULT
  tcp_open_server(void *param, int *sock);

  /**
   * @fn            HRESULT tcp_close(int *sock)
   * @brief         Closes the socket.
   * @param[in,out] sock The socket to be closed.
   */
  _DN_EXP_TCP HRESULT
  tcp_close(int *sock);

  /**
   * @fn        HRESULT tcp_send(int sock, const char *buf, uint32_t len_buf, void *arg)
   * @brief     Sends TCP packet.
   * @param[in] sock The socket to send.
   * @param[in] buf The buffer to be sent.
   * @param[in] len_buf The size of sent buffer.
   * @param[in] arg Special parameter: int.
   */
  _DN_EXP_TCP HRESULT
  tcp_send(int sock, const char *buf, uint32_t len_buf, void *arg);

  /**
   * @fn         HRESULT tcp_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved, uint32_t timeout, void *arg)
   * @brief      Receives TCP packet.
   * @param[in]  sock The socket to receive.
   * @param[out] buf The buffer to be received.
   * @param[in]  len_buf The allocated size of received buffer.
   * @param[out] len_recved The size of received buffer.
   * @param[in]  arg Special parameter: int.
   */
  _DN_EXP_TCP HRESULT
  tcp_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved,
      uint32_t timeout, void *arg);

  /**
   * @fn        HRESULT tcp_set_timeout(int sock, uint32_t timeout)
   * @brief     Sets timeout value to the TCP socket.
   * @param[in] sock The socket to be set.
   * @param[in] timeout Timeout value.
   */
  _DN_EXP_TCP HRESULT
  tcp_set_timeout(int sock, uint32_t timeout);

  /**
   * @fn        HRESULT tcp_clear(int sock, uint32_t timeout)
   * @brief     Clears the received buffer.
   * @param[in] sock The socket to be cleared.
   * @param[in] timeout Timeout value.
   */
  _DN_EXP_TCP HRESULT
  tcp_clear(int sock, uint32_t timeout);

  /**
   * @fn         HRESULT tcp_accept(int sock, int *client)
   * @brief      TCP server accepts a TCP client.
   * @param[in]  sock The TCP server socket.
   * @param[out] client The socket of the accepted TCP client.
   * @note       This function is not thread safe.
   * @note       Must take mutex among socket_open, socket_close and tcp_accept.
   */
  _DN_EXP_TCP HRESULT
  tcp_accept(int sock, int *client);

  /**
   * @fn        HRESULT tcp_set_keepalive(int sock, int enable, uint32_t idle, uint32_t interval, uint32_t count)
   * @brief     Sets keep alive option.
   * @param[in] sock The TCP socket.
   * @param[in] enable Enable option. 0: disable, 1: enable.
   * @param[in] idle Idle time until sending keep alive packet.
   * @param[in] interval Interval time between keep alive packet.
   * @param[in] count The number of sending keep alive packet.
   */
  _DN_EXP_TCP HRESULT
  tcp_set_keepalive(int sock, int enable, uint32_t idle, uint32_t interval,
      uint32_t count);

  /**
   * @fn        HRESULT tcp_set_nodelay(int sock, int enable)
   * @brief     Sets no delay option.
   * @param[in] sock The TCP socket.
   * @param[in] enable Enable option. 0: disable, 1: enable.
   */
  _DN_EXP_TCP HRESULT
  tcp_set_nodelay(int sock, int enable);

#ifdef __cplusplus
}
#endif

#endif /* DN_TCP_H_ */
