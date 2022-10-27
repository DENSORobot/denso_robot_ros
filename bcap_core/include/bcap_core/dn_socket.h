#ifndef DN_SOCKET_H_
#define DN_SOCKET_H_

/**
 * @file    dn_socket.h
 * @brief   Socket API file.
 * @details Defines socket APIs.
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

#ifndef _DN_EXP_SOCKET
#define _DN_EXP_SOCKET
#endif /* _DN_EXP_SOCKET */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn         HRESULT socket_open(int type, int *sock)
 * @brief      Creates a socket.
 * @param[in]  type The socket type. Allows SOCK_STREAM or SOCK_DGRAM.
 * @param[out] sock The created socket.
 * @note       This function is not thread safe.
 * @note       Must take mutex among socket_open, socket_close and tcp_accept.
 */
_DN_EXP_SOCKET HRESULT socket_open(int type, int *sock);

/**
 * @fn            HRESULT socket_close(int *sock)
 * @brief         Closes the socket.
 * @param[in,out] sock The socket to be closed.
 * @note          This function is not thread safe.
 * @note          Must take mutex among socket_open, socket_close and tcp_accept.
 */
_DN_EXP_SOCKET HRESULT socket_close(int *sock);

/**
 * @fn            HRESULT socket_bind(const struct CONN_PARAM_ETH *param, int *sock)
 * @brief         Binds the socket with param's source address and source port.
 * @param[in]     param The pointer of Ethernet connection parameters.
 * @param[in,out] sock The socket to be binded.
 */
_DN_EXP_SOCKET HRESULT socket_bind(const struct CONN_PARAM_ETH *param, int *sock);

/**
 * @fn        HRESULT socket_set_timeout(int sock, uint32_t timeout)
 * @brief     Sets timeout value to the socket.
 * @param[in] sock The socket to be set.
 * @param[in] timeout Timeout value.
 */
_DN_EXP_SOCKET HRESULT socket_set_timeout(int sock, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* DN_SOCKET_H_ */
