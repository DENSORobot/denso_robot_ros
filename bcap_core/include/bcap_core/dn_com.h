#ifndef DN_COM_H_
#define DN_COM_H_

/**
 * @file    dn_com.h
 * @brief   Serial API file.
 * @details Defines Serial APIs.
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

#ifndef _DN_EXP_COM
#define _DN_EXP_COM
#endif /* _DN_EXP_COM */

/**
 * @def  COM_BITS_CTS
 * @brief A definition for CTS flag.
 */

#if defined(_USE_WIN_API)
#define COM_BITS_CTS MS_CTS_ON
typedef DCB COM_STATE;
#elif defined(_USE_LINUX_API)
#define COM_BITS_CTS TIOCM_CTS
typedef struct termios COM_STATE;
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn         HRESULT com_open(void *param, int *sock)
   * @brief      Opens serial port.
   * @param[in]  param The pointer of serial connection parameters: CONN_PARAM_COM.
   * @param[out] sock The created socket.
   */
  _DN_EXP_COM HRESULT
  com_open(void *param, int *sock);

  /**
   * @fn            HRESULT com_close(int *sock)
   * @brief         Closes the socket.
   * @param[in,out] sock The socket to be closed.
   */
  _DN_EXP_COM HRESULT
  com_close(int *sock);

  /**
   * @fn        HRESULT com_send(int sock, const char *buf, uint32_t len_buf, void *arg)
   * @brief     Sends serial packet.
   * @param[in] sock The socket to send.
   * @param[in] buf The buffer to be sent.
   * @param[in] len_buf The size of sent buffer.
   * @param[in] arg Special parameter. Do not use.
   */
  _DN_EXP_COM HRESULT
  com_send(int sock, const char *buf, uint32_t len_buf, void *arg);

  /**
   * @fn         HRESULT com_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved, uint32_t timeout, void *arg)
   * @brief      Receives serial packet.
   * @param[in]  sock The socket to receive.
   * @param[out] buf The buffer to be received.
   * @param[in]  len_buf The allocated size of received buffer.
   * @param[out] len_recved The size of received buffer.
   * @param[in]  arg Special parameter. Do not use.
   */
  _DN_EXP_COM HRESULT
  com_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved,
      uint32_t timeout, void *arg);

  /**
   * @fn        HRESULT com_set_timeout(int sock, uint32_t timeout)
   * @brief     Sets timeout value to the serial socket.
   * @param[in] sock The socket to be set.
   * @param[in] timeout Timeout value.
   */
  _DN_EXP_COM HRESULT
  com_set_timeout(int sock, uint32_t timeout);

  /**
   * @fn        HRESULT com_clear(int sock, uint32_t timeout)
   * @brief     Clears the received buffer.
   * @param[in] sock The socket to be cleared.
   * @param[in] timeout Timeout value.
   */
  _DN_EXP_COM HRESULT
  com_clear(int sock, uint32_t timeout);

  /**
   * @fn         HRESULT com_get_state(int sock, COM_STATE *state)
   * @brief      Gets the serial socket parameters.
   * @param[in]  sock The socket to be gotten.
   * @param[out] state The gotten parameters.
   */
  _DN_EXP_COM HRESULT
  com_get_state(int sock, COM_STATE *state);

  /**
   * @fn        HRESULT com_set_state(int sock, COM_STATE *state)
   * @brief     Puts the serial socket parameters.
   * @param[in] sock The socket to be set.
   * @param[in] state The setting parameters.
   */
  _DN_EXP_COM HRESULT
  com_set_state(int sock, COM_STATE *state);

  /**
   * @fn        HRESULT com_get_modem_state(int sock, uint32_t *state)
   * @brief     Gets the serial port pin status.
   * @param[in] sock The socket to be gotten.
   * @param[in] state The gotten serial port pin status.
   */
  _DN_EXP_COM HRESULT
  com_get_modem_state(int sock, uint32_t *state);

#ifdef __cplusplus
}
#endif

#endif /* DN_COM_H_ */
