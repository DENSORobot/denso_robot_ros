#ifndef DN_DEVICE_H_
#define DN_DEVICE_H_

/**
 * @file    dn_device.h
 * @brief   Common device API file.
 * @details Defines common device APIs.
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

#ifndef _DN_EXP_DEVICE
#define _DN_EXP_DEVICE
#endif /* _DN_EXP_DEVICE */

/**
 * @def   DNGetLastError()
 * @brief A macro that gets last OS error
 */

/**
 * @def   OSSUCCEEDED(ret)
 * @brief A macro that returns TRUE/FALSE. If ret is OS succeeded, then returns TRUE.
 */

/**
 * @def   OSFAILED(ret)
 * @brief A macro that returns TRUE/FALSE. If ret is OS failed, then returns TRUE.
 */
#if defined(_USE_WIN_API)
#define DNGetLastError() GetLastError()
#define OSSUCCEEDED(ret) ((ret) != 0)
#define OSFAILED(ret)  ((ret) == 0)
#else
#if defined(_USE_LINUX_API)
#define DNGetLastError() errno
#define OSSUCCEEDED(ret) ((ret) ==  0)
#define OSFAILED(ret)  ((ret) == -1)
#endif

/**
 * @def   NOPARITY
 * @brief A definition for serial communication setting.
 */
#define NOPARITY (0)

/**
 * @def   ODDPARITY
 * @brief A definition for serial communication setting.
 */
#define ODDPARITY (1)

/**
 * @def   EVENPARITY
 * @brief A definition for serial communication setting.
 */
#define EVENPARITY (2)

/**
 * @def   ONESTOPBIT
 * @brief A definition for serial communication setting.
 */
#define ONESTOPBIT (0)

/**
 * @def   TWOSTOPBITS
 * @brief A definition for serial communication setting.
 */
#define TWOSTOPBITS (2)

#endif

/**
 * @def   DEV_BUF_MAX
 * @brief The maximum buffer size of a packet.
 */
#define DEV_BUF_MAX ((uint16_t)-1)

/**
 * @def   E_INVALIDPACKET
 * @brief Failed because the packet is invalid.
 */
#define E_INVALIDPACKET _HRESULT_TYPEDEF_(0x80010000L)

/**
 * @enum  CONN_PARAM_CHECK_FLAG
 * @brief Definitions for check_conn_param function.
 */
enum CONN_PARAM_CHECK_FLAG
{
  CHECK_TYPE_TCP = 0x0001,     /**< Allows to connect with TCP.          */
  CHECK_TYPE_UDP = 0x0002,     /**< Allows to connect with UDP.          */
  CHECK_TYPE_COM = 0x0004,     /**< Allows to connect with COM.          */
  CHECK_TYPE_ALL = 0x00FF,     /**< Allows to connect with all type.     */
  CHECK_FUNC_OPEN = 0x0100,    /**< Must be set the dn_open function.    */
  CHECK_FUNC_CLOSE = 0x0200,   /**< Must be set the dn_close function.   */
  CHECK_FUNC_SEND = 0x0400,    /**< Must be set the dn_send function.    */
  CHECK_FUNC_RECV = 0x0800,    /**< Must be set the dn_recv function.    */
  CHECK_FUNC_TIMEOUT = 0x1000, /**< Must be set the dn_timeout function. */
  CHECK_FUNC_CLEAR = 0x2000,   /**< Must be set the dn_clear function.   */
  CHECK_FUNC_ALL = 0xFF00,     /**< Must be set the all function.        */
};

/**
 * @enum  CONN_TYPE
 * @brief Connection type.
 */
enum CONN_TYPE
{
  CONN_TCP = CHECK_TYPE_TCP, /**< TCP connection.    */
  CONN_UDP = CHECK_TYPE_UDP, /**< UDP connection.    */
  CONN_COM = CHECK_TYPE_COM, /**< Serial connection. */
};

/**
 * @struct CONN_PARAM_ETH
 * @brief  A type definition for Ethernet connection parameters.
 * @note   These parameters are network byte order.
 */
struct CONN_PARAM_ETH
{
  uint32_t dst_addr; /**< Destination address. */
  uint16_t dst_port; /**< Destination port.    */
  uint32_t src_addr; /**< Source address.      */
  uint16_t src_port; /**< Source port.         */
};

/**
 * @struct CONN_PARAM_COM
 * @brief  A type definition for serial connection parameters.
 */
struct CONN_PARAM_COM
{
  int port;           /**< Port number. */
  uint32_t baud_rate; /**< Baud rate.   */
  char parity;        /**< Parity.      */
  char data_bits;     /**< Data bits.   */
  char stop_bits;     /**< Stop bits.   */
  char flow;          /**< Flow.        */
};

/**
 * @struct CONN_PARAM_COMMON
 * @brief  A type definition for common communication parameters.
 */
struct CONN_PARAM_COMMON
{
  int sock;         /**< File descriptor.   */
  int type;         /**< Connection type.   */
  uint32_t timeout; /**< Timeout value.     */
  void *arg;        /**< Special argument.  */
  HRESULT (*dn_open)(void *param, int *sock);
  HRESULT (*dn_close)(int *sock);
  HRESULT (*dn_send)(int sock, const char *buf, uint32_t len_buf, void *arg);
  HRESULT (*dn_recv)(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved,
      uint32_t timeout, void *arg);
  HRESULT (*dn_set_timeout)(int sock, uint32_t timeout);
  HRESULT (*dn_clear)(int sock, uint32_t timeout);
};

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn        int parse_conn_type(const char *opt)
   * @brief     Parses and returns the connection type.
   * @param[in] opt Connection option string.
   */
  _DN_EXP_DEVICE int
  parse_conn_type(const char *opt);

  /**
   * @fn         HRESULT parse_conn_param_ether(const char *opt, struct CONN_PARAM_ETH *param)
   * @brief      Parses Ethernet connection parameters.
   * @param[in]  opt Connection option string.
   * @param[out] param Parsed Ethernet connection parameters.
   * @note tcp[:<DestIP>[:<DestPort>[:<SourceIP>[:<SourcePort>]]]]
   * @note udp[:<DestIP>[:<DestPort>[:<SourceIP>[:<SourcePort>]]]]
   */
  _DN_EXP_DEVICE HRESULT
  parse_conn_param_ether(const char *opt, struct CONN_PARAM_ETH *param);

  /**
   * @fn         HRESULT parse_conn_param_serial(const char *opt, struct CONN_PARAM_COM *param)
   * @brief      Parses serial connection parameters.
   * @param[in]  opt Connection option string.
   * @param[out] param Parsed serial connection parameters.
   * @note com[:<COM Port>[:<BaudRate>[:<Parity>:<DataBits>:<StopBits>[:<Flow>]]]
   * @note <COM Port> := 0, 1, ...
   * @note <BaudRate> := 50, 75, ...
   * @note <Parity>   := N, O, E
   * @note <DataBits> := 5, 6, 7, 8
   * @note <StopBits> := 1, 2
   * @note <Flow>     := 0, 1, 2, 3
   */
  _DN_EXP_DEVICE HRESULT
  parse_conn_param_serial(const char *opt, struct CONN_PARAM_COM *param);

  /**
   * @fn        HRESULT check_timeout(int sock, uint32_t timeout)
   * @brief     Checks the communication timeout.
   * @param[in] sock File descriptor to be checked.
   * @param[in] timeout Timeout value.
   */
  _DN_EXP_DEVICE HRESULT
  check_timeout(int sock, uint32_t timeout);

  /**
   * @fn        HRESULT check_conn_param(const struct CONN_PARAM_COMMON *device, int flag)
   * @brief     Checks the communication parameters.
   * @param[in] device Communication parameters to be checked.
   * @param[in] flag Flags to be checked with CONN_PARAM_CHECK_FLAG.
   */
  _DN_EXP_DEVICE HRESULT
  check_conn_param(const struct CONN_PARAM_COMMON *device, int flag);

  /**
   * @fn         void memcpy_le(void *dst, const void *src, uint32_t len)
   * @brief      Orders to little endian.
   * @param[out] dst The pointer of ordered variable.
   * @param[in]  src The pointer of variable to be ordered.
   * @param[in]  len The number of buffers to be ordered.
   */
  _DN_EXP_DEVICE void
  memcpy_le(void *dst, const void *src, uint32_t len);

  /**
   * @fn         void memcpy_be(void *dst, const void *src, uint32_t len)
   * @brief      Orders to big endian.
   * @param[out] dst The pointer of ordered variable.
   * @param[in]  src The pointer of variable to be ordered.
   * @param[in]  len The number of buffers to be ordered.
   */
  _DN_EXP_DEVICE void
  memcpy_be(void *dst, const void *src, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* DN_DEVICE_H_ */
