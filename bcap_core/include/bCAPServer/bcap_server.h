#ifndef BCAP_CLIENT_H_
#define BCAP_CLIENT_H_

/**
 * @file    bcap_server.h
 * @brief   b-CAP Server API file.
 * @details Defines b-CAP Server APIs.
 *
 * @version 1.0
 * @date    2015/1/10
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

#ifndef _BCAP_EXP_SERVER
#define _BCAP_EXP_SERVER
#endif /* _BCAP_EXP_SERVER */

#include "../dn_common.h"
#include "../bcap_funcid.h"

#ifndef ERROR_BUSY
/**
 * @def   ERROR_BUSY
 * @brief The requested resource because executing process is busy.
 */
#define ERROR_BUSY (170L)
#endif

/**
 * @def   E_BUSY_PROC
 * @brief Failed because executing process is busy.
 */
#define E_BUSY_PROC OSERR2HRESULT(ERROR_BUSY)

/**
 * @def   BCAP_TCP_MAX
 * @brief A definition for the maximum count of TCP servers.
 * @note  You can change this parameter to 1 or more.
 */
#define BCAP_TCP_MAX (1)

/**
 * @def   BCAP_UDP_MAX
 * @brief A definition for the maximum count of UDP servers.
 * @note  You can change this parameter to 1 or more.
 */
#define BCAP_UDP_MAX (1)

/**
 * @def   BCAP_COM_MAX
 * @brief A definition for the maximum count of COM servers.
 * @note  You can change this parameter to 1 or more.
 */
#define BCAP_COM_MAX (1)

/**
 * @def   BCAP_CLIENT_MAX
 * @brief A definition for the maximum count of TCP clients.
 * @note  You can change this parameter to 1 or more.
 */
#define BCAP_CLIENT_MAX (20)

/**
 * @def   BCAP_OBJECT_MAX
 * @brief A definition for the maximum count of creatable objects in a thread.
 * @note  You can change this parameter to 1 or more.
 */
#define BCAP_OBJECT_MAX (1000)

/**
 * @def   INIT_EXEC_TIMEOUT
 * @brief A definition for the initial executing timeout.
 * @note  times in milliseconds.
 */
#define INIT_EXEC_TIMEOUT (180000)

/**
 * @def   MIN_WDT_INTERVAL
 * @brief A definition for the minimum watch dog timer interval.
 * @note  times in milliseconds.
 */
#define MIN_WDT_INTERVAL (80)

/**
 * @def   INIT_WDT_INTERVAL
 * @brief A definition for the initial watch dog timer interval.
 * @note  times in milliseconds.
 */
#define INIT_WDT_INTERVAL (INFINITE)

/**
 * @def   KEEPALIVE_ENABLE
 * @brief A definition for the keep alive enable option.
 */
#define KEEPALIVE_ENABLE (1)

/**
 * @def   KEEPALIVE_IDLE
 * @brief A definition for the keep alive idle option.
 * @note  times in seconds.
 */
#define KEEPALIVE_IDLE (7200)

/**
 * @def   KEEPALIVE_INTERVAL
 * @brief A definition for the keep alive interval option.
 * @note  times in seconds.
 */
#define KEEPALIVE_INTERVAL (75)

/**
 * @def   KEEPALIVE_COUNT
 * @brief A definition for the keep alive count option.
 */
#define KEEPALIVE_COUNT (9)

/**
 * @def   UDP_LIFELIMIT
 * @brief A definition for the life limit of a UDP connection.
 * @note  times in milliseconds.
 */
#define UDP_LIFELIMIT (10000)

typedef HRESULT
(*CALL_FUNC_BCAP)(VARIANT *vntArgs, int16_t Argc, VARIANT *vntRet);

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn        HRESULT bCap_SetCallFunc(int32_t Id, CALL_FUNC_BCAP func)
   * @brief     Sets a callback function.
   * @param[in] id The b-CAP function ID.
   * @param[in] func A callback function to be set.
   */
  _BCAP_EXP_SERVER HRESULT
  bCap_SetCallFunc(int32_t id, CALL_FUNC_BCAP func);

  /**
   * @fn         HRESULT bCap_Open_Server(const char *connect, uint32_t timeout, int *pfd)
   * @brief      Starts b-CAP server.
   * @param[in]  timeout Timeout value.
   * @param[out] pfd The pointer of File descriptor.
   */
  _BCAP_EXP_SERVER HRESULT
  bCap_Open_Server(const char *connect, uint32_t timeout, int *pfd);

  /**
   * @fn            HRESULT bCap_Close_Server(int *pfd)
   * @brief         Ends b-CAP server.
   * @param[in,out] pfd The pointer of File descriptor.
   */
  _BCAP_EXP_SERVER HRESULT
  bCap_Close_Server(int *pfd);

#ifdef __cplusplus
}
#endif

#endif /* BCAP_SERVER_H_ */
