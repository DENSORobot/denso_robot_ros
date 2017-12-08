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
#include <winsock2.h>
#pragma comment(lib, "wsock32.lib")
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#include <errno.h>
#include <sys/time.h>
#ifndef strnicmp
#define strnicmp strncasecmp
#endif
#else
#include "dn_additional.h"
#endif

#include "dn_common.h"
#include "dn_device.h"

#define _STR_ISNUMERIC "0123456789"

#define _TYPE_MIN     (3)
#define _PARA_MAX_COM (7)
#define _PARA_MAX_ETH (5)

/* Endian switching */
#ifndef __LITTLE_ENDIAN__
#ifndef __BIG_ENDIAN__
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define __LITTLE_ENDIAN__
#elif __BYTE_ORDER == __BIG_ENDIAN
#define __BIG_ENDIAN__
#endif
#endif
#endif

/**
 * @fn   int is_numeric(const char *src)
 * @brief  Checks the argument is numeric or not.
 * @param[in] src The characters to be checked.
 */
static int
is_numeric(const char *src)
{
  int len;

  /* Invalid argument */
  if ((src == NULL) || ((len = strlen(src)) == 0)) {
    return 0;
  }

  /* Only the head character can be '-' */
  if (*src == '-') {
    src++;
    len--;
  }

  return (strspn(src, _STR_ISNUMERIC) == len);
}

/**
 * @fn        int parse_conn_type(const char *opt)
 * @brief     Parses and returns the connection type.
 * @param[in] opt Connection option string.
 */
int
parse_conn_type(const char *opt)
{
  int type = -1; /* Invalid type */

  if (opt != NULL && strlen(opt) >= _TYPE_MIN) {
    if (strnicmp(opt, "tcp", _TYPE_MIN) == 0) {
      type = CONN_TCP;
    }
    else if (strnicmp(opt, "udp", _TYPE_MIN) == 0) {
      type = CONN_UDP;
    }
    else if (strnicmp(opt, "com", _TYPE_MIN) == 0) {
      type = CONN_COM;
    }
  }

  return type;
}

/**
 * @fn         HRESULT parse_conn_param_ether(const char *opt, struct CONN_PARAM_ETH *param)
 * @brief      Parses Ethernet connection parameters.
 * @param[in]  opt Connection option string.
 * @param[out] param Parsed Ethernet connection parameters.
 * @note tcp[:<DestIP>[:<DestPort>[:<SourceIP>[:<SourcePort>]]]]
 * @note udp[:<DestIP>[:<DestPort>[:<SourceIP>[:<SourcePort>]]]]
 */
HRESULT
parse_conn_param_ether(const char *opt, struct CONN_PARAM_ETH *param)
{
  int tmp, type, n = 0;
  uint32_t uitmp;
  char *top, *pos, *pos_param[_PARA_MAX_ETH];
  char *opt_cpy = NULL;
  HRESULT hr = E_INVALIDARG;

  if (param != NULL) {
    type = parse_conn_type(opt);
    if ((type == CONN_TCP) || (type == CONN_UDP)) {
      /* Copy connection option string */
      opt_cpy = (char *) malloc(strlen(opt) + 1);

      if (opt_cpy == NULL) {
        hr = E_OUTOFMEMORY;
        goto exit_proc;
      }

      strcpy(opt_cpy, opt);

      /* Split connection option string */
      top = opt_cpy;
      while (1) {
        if (n >= _PARA_MAX_ETH) {
          goto exit_proc;
        }

        pos_param[n++] = top;

        pos = strchr(top, ':');
        if (pos == NULL) {
          break;
        }

        *pos = '\0';
        top = (pos + 1);
      }

      /* Source Port */
      if (n >= 5) {
        if (is_numeric(pos_param[4]) == 0) {
          goto exit_proc;
        }
        tmp = atoi(pos_param[4]);
        if ((tmp < 0) || (tmp != (uint16_t) tmp)) {
          hr = DISP_E_OVERFLOW;
          goto exit_proc;
        }
        param->src_port = htons(tmp);
      }

      /* Source IP */
      if (n >= 4) {
        uitmp = inet_addr(pos_param[3]);
        if (uitmp == htonl(INADDR_NONE)
            && strcmp(pos_param[3], "255.255.255.255") != 0)
        {
          goto exit_proc;
        }
        param->src_addr = uitmp;
      }

      /* Dest Port */
      if (n >= 3) {
        if (is_numeric(pos_param[2]) == 0) {
          goto exit_proc;
        }
        tmp = atoi(pos_param[2]);
        if ((tmp < 0) || (tmp != (uint16_t) tmp)) {
          hr = DISP_E_OVERFLOW;
          goto exit_proc;
        }
        param->dst_port = htons(tmp);
      }

      /* Dest IP */
      if (n >= 2) {
        uitmp = inet_addr(pos_param[1]);
        if (uitmp == htonl(INADDR_NONE)
            && strcmp(pos_param[1], "255.255.255.255") != 0)
        {
          goto exit_proc;
        }
        param->dst_addr = uitmp;
      }

      hr = S_OK;
    }
  }

exit_proc:
  if (opt_cpy != NULL) {
    free(opt_cpy);
    opt_cpy = NULL;
  }

  return hr;
}

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
HRESULT
parse_conn_param_serial(const char *opt, struct CONN_PARAM_COM *param)
{
  int type, n = 0;
  long tmp;
  char *top, *pos, *pos_param[_PARA_MAX_COM];
  char *opt_cpy = NULL;
  HRESULT hr = E_INVALIDARG;

  if (param != NULL) {
    type = parse_conn_type(opt);
    if (type == CONN_COM) {
      /* Copy connection option string */
      opt_cpy = (char *) malloc(strlen(opt) + 1);

      if (opt_cpy == NULL) {
        hr = E_OUTOFMEMORY;
        goto exit_proc;
      }

      strcpy(opt_cpy, opt);

      /* Split connection option string */
      top = opt_cpy;
      while (1) {
        if (n >= _PARA_MAX_COM) {
          goto exit_proc;
        }

        pos_param[n++] = top;

        pos = strchr(top, ':');
        if (pos == NULL) {
          break;
        }

        *pos = '\0';
        top = (pos + 1);
      }

      if (n == 4 || n == 5) {
        goto exit_proc;
      }

      /* Flow */
      if (n >= 7) {
        if (is_numeric(pos_param[6]) == 0) {
          goto exit_proc;
        }
        tmp = atol(pos_param[6]);
        if (tmp < 0 || 3 < tmp) {
          hr = DISP_E_OVERFLOW;
          goto exit_proc;
        }
        param->flow = (char) tmp;
      }

      /* Stop bits */
      if (n >= 6) {
        tmp = (int) (atof(pos_param[5]) * 10.0);

        switch (tmp) {
          case 10:
            param->stop_bits = ONESTOPBIT;
            break;
          case 20:
            param->stop_bits = TWOSTOPBITS;
            break;
          default:
            goto exit_proc;
        }
      }

      /* Data bits */
      if (n >= 5) {
        if (is_numeric(pos_param[4]) == 0) {
          goto exit_proc;
        }
        tmp = atol(pos_param[4]);
        if (tmp < 5 || 8 < tmp) {
          hr = DISP_E_OVERFLOW;
          goto exit_proc;
        }
        param->data_bits = (char) tmp;
      }

      /* Parity */
      if (n >= 4) {
        if (strlen(pos_param[3]) != 1) {
          goto exit_proc;
        }

        switch (*pos_param[3]) {
          case L'N':
          case L'n':
            param->parity = NOPARITY;
            break;
          case L'O':
          case L'o':
            param->parity = ODDPARITY;
            break;
          case L'E':
          case L'e':
            param->parity = EVENPARITY;
            break;
          default:
            goto exit_proc;
        }
      }

      /* Baud rate */
      if (n >= 3) {
        if (is_numeric(pos_param[2]) == 0) {
          goto exit_proc;
        }
        param->baud_rate = (uint32_t) atol(pos_param[2]);
      }

      /* Port */
      if (n >= 2) {
        if (is_numeric(pos_param[1]) == 0) {
          goto exit_proc;
        }
        tmp = atol(pos_param[1]);
        if ((tmp < 0) || (tmp != (int) tmp)) {
          hr = DISP_E_OVERFLOW;
          goto exit_proc;
        }
        param->port = tmp;
      }

      hr = S_OK;
    }
  }

exit_proc:
  if (opt_cpy != NULL) {
    free(opt_cpy);
    opt_cpy = NULL;
  }

  return hr;
}

/**
 * @fn        HRESULT check_timeout(int sock, uint32_t timeout)
 * @brief     Checks the communication timeout.
 * @param[in] sock File descriptor to be checked.
 * @param[in] timeout Timeout value.
 */
HRESULT
check_timeout(int sock, uint32_t timeout)
{
  int ret;
  fd_set fds;
  struct timeval tv;
  HRESULT hr = S_OK;

  if (sock <= 0)
    return E_HANDLE;

  FD_ZERO(&fds); FD_SET(sock, &fds);

  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) * 1000;

  ret = select(sock + 1, &fds, NULL, NULL, &tv);
  if (ret == 0) {
    hr = E_TIMEOUT;
  }
  else if (ret < 0) {
    ret = DNGetLastError();
    hr = OSERR2HRESULT(ret);
  }

  return hr;
}

/**
 * @fn        HRESULT check_conn_param(const struct CONN_PARAM_COMMON *device, int flag)
 * @brief     Checks the communication parameters.
 * @param[in] device Communication parameters to be checked.
 * @param[in] flag Flags to be checked with CONN_PARAM_CHECK_FLAG.
 */
HRESULT
check_conn_param(const struct CONN_PARAM_COMMON *device, int flag)
{
  if (device == NULL)
    return E_INVALIDARG;

  /* Checks the socket */
  if (device->sock <= 0)
    return E_HANDLE;

  /* Checks the connection type */
  if (flag & CHECK_TYPE_ALL) {
    if (!(flag & device->type))
      return E_INVALIDARG;
  }

  /* Checks the dn_open function */
  if ((flag & CHECK_FUNC_OPEN) && (device->dn_open == NULL))
    return E_INVALIDARG;

  /* Checks the dn_close function */
  if ((flag & CHECK_FUNC_CLOSE) && (device->dn_close == NULL))
    return E_INVALIDARG;

  /* Checks the dn_send function */
  if ((flag & CHECK_FUNC_SEND) && (device->dn_send == NULL))
    return E_INVALIDARG;

  /* Checks the dn_recv function */
  if ((flag & CHECK_FUNC_RECV) && (device->dn_recv == NULL))
    return E_INVALIDARG;

  /* Checks the dn_timeout function */
  if ((flag & CHECK_FUNC_TIMEOUT) && (device->dn_set_timeout == NULL))
    return E_INVALIDARG;

  /* Checks the dn_clear function */
  if ((flag & CHECK_FUNC_CLEAR) && (device->dn_clear == NULL))
    return E_INVALIDARG;

  return S_OK;
}

/**
 * @fn         void memcpy_le(void *dst, const void *src, uint32_t len)
 * @brief      Orders to little endian.
 * @param[out] dst The pointer of ordered variable.
 * @param[in]  src The pointer of variable to be ordered.
 * @param[in]  len The number of buffers to be ordered.
 */
void
memcpy_le(void *dst, const void *src, uint32_t len)
{
#ifdef __BIG_ENDIAN__
  uint32_t i;
  uint8_t *pdst;
  uint8_t *psrc;

  psrc = (uint8_t *)(src) + len - 1;
  pdst = (uint8_t *)(dst);

  for (i = 0; i < len; i++) {
    *pdst++ = *psrc--;
  }
#else
  memcpy(dst, src, len);
#endif
}

/**
 * @fn         void memcpy_be(void *dst, const void *src, uint32_t len)
 * @brief      Orders to big endian.
 * @param[out] dst The pointer of ordered variable.
 * @param[in]  src The pointer of variable to be ordered.
 * @param[in]  len The number of buffers to be ordered.
 */
void
memcpy_be(void *dst, const void *src, uint32_t len)
{
#ifndef __BIG_ENDIAN__
  uint32_t i;
  uint8_t *pdst;
  uint8_t *psrc;

  psrc = (uint8_t *) (src) + len - 1;
  pdst = (uint8_t *) (dst);

  for (i = 0; i < len; i++) {
    *pdst++ = *psrc--;
  }
#else
  memcpy(dst, src, len);
#endif
}
