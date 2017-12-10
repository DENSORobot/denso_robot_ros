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
#include <stdio.h>
#include <string.h>

#if defined(_USE_WIN_API)
#include <windows.h>
#elif defined(_USE_LINUX_API)
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#define _BAUD_MAX    (15)
#define _BAUD_PAIR(val)   {val, B ## val}
static const unsigned int BAUD_RATE[_BAUD_MAX][2] =
  { _BAUD_PAIR(50), _BAUD_PAIR(75), _BAUD_PAIR(110), _BAUD_PAIR(134), _BAUD_PAIR(150),
    _BAUD_PAIR(200), _BAUD_PAIR(300), _BAUD_PAIR(600), _BAUD_PAIR(1200), _BAUD_PAIR(1800),
    _BAUD_PAIR(2400), _BAUD_PAIR(4800), _BAUD_PAIR(9600), _BAUD_PAIR(19200), _BAUD_PAIR(38400)};
#else
#include "dn_additional.h"
#endif

#include "dn_common.h"
#include "dn_device.h"
#include "dn_com.h"

#define _COM_PORT_MAX (256)

#define _FLOW_XINOUT   (1)
#define _FLOW_HARDWARE (2)

#if defined(_USE_WIN_API)
static HRESULT _com_open(const struct CONN_PARAM_COM *com_param, int *sock)
{
  char szName[16];
  COM_STATE state;
  HRESULT hr;

  sprintf(szName, "//./COM%d", com_param->port);
  *sock = (int)CreateFileA(szName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

  if(*sock < 0) {
    *sock = 0;
    hr = DNGetLastError();
    return OSERR2HRESULT(hr);
  }

  /* Clears the rest buffers */
  com_clear(*sock, 0);

  hr = com_get_state(*sock, &state);
  if(SUCCEEDED(hr)) {
    state.BaudRate = com_param->baud_rate;
    state.ByteSize = com_param->data_bits;
    state.Parity = com_param->parity;
    state.StopBits = com_param->stop_bits;

    if(com_param->flow & _FLOW_XINOUT) {
      state.fOutX = 1;
      state.fInX = 1;
    } else {
      state.fOutX = 0;
      state.fInX = 0;
    }

    if(com_param->flow & _FLOW_HARDWARE) {
      state.fOutxCtsFlow = 1;
      state.fRtsControl = RTS_CONTROL_HANDSHAKE;
    } else {
      state.fOutxCtsFlow = 0;
      state.fRtsControl = RTS_CONTROL_ENABLE;
    }
    state.fOutxDsrFlow = 0;
    state.fDtrControl = DTR_CONTROL_ENABLE;
    state.fDsrSensitivity = 0;

    hr = com_set_state(*sock, &state);
  }

  return hr;
}

static int _com_close(int sock)
{
  return CloseHandle((HANDLE)sock);
}

static int _com_send(int sock, const char *buf, uint32_t len_send, uint32_t *len_sended, void *arg)
{
  return WriteFile((HANDLE)sock, buf, len_send, (LPDWORD)len_sended, NULL);
}

static int _com_recv(int sock, char *buf, uint32_t len_recv, uint32_t *len_recved, uint32_t timeout, void *arg)
{
  int ret;

  *len_recved = 0;
  ret = ReadFile((HANDLE)sock, buf, len_recv, (LPDWORD)len_recved, NULL);

  return ret;
}

static HRESULT _com_set_timeout(int sock, uint32_t timeout)
{
  int ret;
  COMMTIMEOUTS stTimeOut;
  HRESULT hr = S_OK;

  stTimeOut.ReadIntervalTimeout = MAXDWORD;
  stTimeOut.ReadTotalTimeoutMultiplier = MAXDWORD;
  stTimeOut.ReadTotalTimeoutConstant = timeout;
  stTimeOut.WriteTotalTimeoutMultiplier = 1;
  stTimeOut.WriteTotalTimeoutConstant = timeout;
  ret = SetCommTimeouts((HANDLE)sock, &stTimeOut);
  if(OSFAILED(ret)) {
    ret = DNGetLastError();
    hr = OSERR2HRESULT(ret);
  }

  return hr;
}

static HRESULT _com_clear(int sock, uint32_t timeout)
{
  int ret;
  DWORD err;
  HRESULT hr = S_OK;

  ret = ClearCommError((HANDLE)sock, &err, NULL);
  if(OSSUCCEEDED(ret)) {
    ret = PurgeComm((HANDLE)sock, (PURGE_TXABORT | PURGE_RXABORT| PURGE_TXCLEAR | PURGE_RXCLEAR));
  }
  if(OSFAILED(ret)) {
    ret = DNGetLastError();
    hr = OSERR2HRESULT(ret);
  }

  return hr;
}

static int _com_get_state(int sock, COM_STATE *state)
{
  return GetCommState((HANDLE)sock, state);
}

static int _com_set_state(int sock, COM_STATE *state)
{
  return SetCommState((HANDLE)sock, state);
}

static int _com_get_modem_state(int sock, uint32_t *state)
{
  return GetCommModemStatus((HANDLE)sock, (DWORD *)state);
}
#elif defined(_USE_LINUX_API)
static HRESULT _com_open(const struct CONN_PARAM_COM *com_param, int *sock)
{
  int i;
  char szName[16];
  COM_STATE state;
  HRESULT hr;

  sprintf(szName, "/dev/ttyS%d", com_param->port);
  *sock = open(szName, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if(*sock < 0) {
    *sock = 0;
    hr = DNGetLastError();
    return OSERR2HRESULT(hr);
  }

  /* Clears the rest buffers */
  com_clear(*sock, 0);

  memset(&state, 0, sizeof(COM_STATE));

  state.c_cflag = (CLOCAL | CREAD);

  for(i = 0; i < _BAUD_MAX; i++) {
    if(com_param->baud_rate == BAUD_RATE[i][0]) {
      state.c_cflag |= BAUD_RATE[i][1];
      break;
    }
  }

  switch(com_param->data_bits) {
    case 5:
      state.c_cflag |= CS5;
      break;
    case 6:
      state.c_cflag |= CS6;
      break;
    case 7:
      state.c_cflag |= CS7;
      break;
    case 8:
      state.c_cflag |= CS8;
      break;
    default:
      break;
  }

  switch(com_param->parity) {
    case NOPARITY:
      state.c_cflag &= ~PARENB;
      break;
    case ODDPARITY:
      state.c_cflag |= PARENB;
      state.c_cflag |= PARODD;
      break;
    case EVENPARITY:
      state.c_cflag |= PARENB;
      state.c_cflag &= ~PARODD;
      break;
    default:
      break;
  }

  switch(com_param->stop_bits) {
    case ONESTOPBIT:
      state.c_cflag &= ~CSTOPB;
      break;
    case TWOSTOPBITS:
      state.c_cflag |= CSTOPB;
      break;
    default:
      break;
  }

  if(com_param->flow & _FLOW_XINOUT) {
    state.c_iflag |= IXON;
    state.c_iflag |= IXOFF;
  } else {
    state.c_iflag &= ~IXON;
    state.c_iflag &= ~IXOFF;
  }

  if(com_param->flow & _FLOW_HARDWARE) {
    state.c_cflag |= CRTSCTS;
  } else {
    state.c_cflag &= ~CRTSCTS;
  }

  hr = com_set_state(*sock, &state);

  return hr;
}

static int _com_close(int sock)
{
  return close(sock);
}

static int _com_send(int sock, const char *buf, uint32_t len_send, uint32_t *len_sended, void *arg)
{
  int ret;
  ret = write(sock, buf, len_send);
  *len_sended = ret;
  return ret;
}

static int _com_recv(int sock, char *buf, uint32_t len_recv, uint32_t *len_recved, uint32_t timeout, void *arg)
{
  int ret = 0;
  HRESULT hr;

  *len_recved = 0;
  hr = check_timeout(sock, timeout);
  if(SUCCEEDED(hr)) {
    ret = read(sock, buf, len_recv);
    *len_recved = ret;
  }

  return ret;
}

static HRESULT _com_set_timeout(int sock, uint32_t timeout)
{
  COM_STATE state;
  HRESULT hr;

  hr = com_get_state(sock, &state);
  if(SUCCEEDED(hr)) {
    state.c_cc[VMIN] = 0;
    state.c_cc[VTIME] = timeout * 10 / 1000;

    hr = com_set_state(sock, &state);
  }

  return hr;
}

static HRESULT _com_clear(int sock, uint32_t timeout)
{
  int ret;
  HRESULT hr = S_OK;

  ret = tcflush(sock, TCIFLUSH);
  if(OSFAILED(ret)) {
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

  ret = tcflush(sock, TCOFLUSH);
  if(OSFAILED(ret)) {
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

  return hr;
}

static int _com_get_state(int sock, COM_STATE *state)
{
  return tcgetattr(sock, state);
}

static int _com_set_state(int sock, COM_STATE *state)
{
  return tcsetattr(sock, TCSAFLUSH, state);
}

static int _com_get_modem_state(int sock, uint32_t *state)
{
  return ioctl(sock, TIOCMGET, (int *)state);
}
#endif

/**
 * @fn         HRESULT com_open(void *param, int *sock)
 * @brief      Opens serial port.
 * @param[in]  param The pointer of serial connection parameters: CONN_PARAM_COM.
 * @param[out] sock The created socket.
 */
HRESULT
com_open(void *param, int *sock)
{
  int port;
  HRESULT hr;
  const struct CONN_PARAM_COM *com_param = (const struct CONN_PARAM_COM *) param;

  if (param == NULL || sock == NULL)
    return E_INVALIDARG;

  /* Checks port range */
  port = com_param->port;
  if (port < 0 || _COM_PORT_MAX < port)
    return E_INVALIDARG;

  hr = _com_open(com_param, sock);

  return hr;
}

/**
 * @fn            HRESULT com_close(int *sock)
 * @brief         Closes the socket.
 * @param[in,out] sock The socket to be closed.
 */
HRESULT
com_close(int *sock)
{
  int ret;

  if (sock == NULL || *sock <= 0)
    return E_HANDLE;

  ret = _com_close(*sock);
  if (OSFAILED(ret)) {
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

  *sock = 0;

  return S_OK;
}

/**
 * @fn        HRESULT com_send(int sock, const char *buf, uint32_t len_buf, void *arg)
 * @brief     Sends serial packet.
 * @param[in] sock The socket to send.
 * @param[in] buf The buffer to be sent.
 * @param[in] len_buf The size of sent buffer.
 * @param[in] arg Special parameter. Do not use.
 */
HRESULT
com_send(int sock, const char *buf, uint32_t len_buf, void *arg)
{
  int ret;
  uint32_t len_send, len_sended;

  if (sock <= 0)
    return E_HANDLE;
  if (buf == NULL || strlen(buf) == 0)
    return E_INVALIDARG;

  len_send = (len_buf != 0) ? len_buf : strlen(buf);
  ret = _com_send(sock, buf, len_send, &len_sended, arg);

  if (OSFAILED(ret)) {
    ret = DNGetLastError();
#if defined(_USE_WIN_API)
    {
      DWORD err_flag;
      ClearCommError((HANDLE)sock, &err_flag, NULL);
    }
#endif
    return OSERR2HRESULT(ret);
  }

  if (len_send > len_sended) {
    return E_TIMEOUT;
  }

  return S_OK;
}

/**
 * @fn         HRESULT com_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved, uint32_t timeout, void *arg)
 * @brief      Receives serial packet.
 * @param[in]  sock The socket to receive.
 * @param[out] buf The buffer to be received.
 * @param[in]  len_buf The allocated size of received buffer.
 * @param[out] len_recved The size of received buffer.
 * @param[in]  arg Special parameter. Do not use.
 */
HRESULT
com_recv(int sock, char *buf, uint32_t len_buf, uint32_t *len_recved,
    uint32_t timeout, void *arg)
{
  int ret;

  if (sock <= 0)
    return E_HANDLE;
  if (buf == NULL || len_recved == NULL)
    return E_INVALIDARG;

  ret = _com_recv(sock, buf, len_buf, len_recved, timeout, arg);

  if (OSFAILED(ret)) {
    ret = DNGetLastError();
#if defined(_USE_WIN_API)
    {
      DWORD err_flag;
      ClearCommError((HANDLE)sock, &err_flag, NULL);
    }
#endif
    return OSERR2HRESULT(ret);
  }

  if (*len_recved == 0)
    return E_TIMEOUT;

  return S_OK;
}

/**
 * @fn        HRESULT com_set_timeout(int sock, uint32_t timeout)
 * @brief     Sets timeout value to the serial socket.
 * @param[in] sock The socket to be set.
 * @param[in] timeout Timeout value.
 */
HRESULT
com_set_timeout(int sock, uint32_t timeout)
{
  HRESULT hr = S_OK;

  if (sock <= 0)
    return E_HANDLE;

  hr = _com_set_timeout(sock, timeout);

  return hr;
}

/**
 * @fn        HRESULT com_clear(int sock, uint32_t timeout)
 * @brief     Clears the received buffer.
 * @param[in] sock The socket to be cleared.
 * @param[in] timeout Timeout value.
 */
HRESULT
com_clear(int sock, uint32_t timeout)
{
  HRESULT hr = S_OK;

  if (sock <= 0)
    return E_HANDLE;

  hr = _com_clear(sock, timeout);

  return hr;
}

/**
 * @fn         HRESULT com_get_state(int sock, COM_STATE *state)
 * @brief      Gets the serial socket parameters.
 * @param[in]  sock The socket to be gotten.
 * @param[out] state The gotten parameters.
 */
HRESULT
com_get_state(int sock, COM_STATE *state)
{
  int ret;
  HRESULT hr = S_OK;

  if (sock <= 0)
    return E_HANDLE;
  if (state == NULL)
    return E_INVALIDARG;

  ret = _com_get_state(sock, state);

  if (OSFAILED(ret)) {
    ret = DNGetLastError();
    hr = OSERR2HRESULT(ret);
  }

  return hr;
}

/**
 * @fn        HRESULT com_set_state(int sock, COM_STATE *state)
 * @brief     Puts the serial socket parameters.
 * @param[in] sock The socket to be set.
 * @param[in] state The setting parameters.
 */
HRESULT
com_set_state(int sock, COM_STATE *state)
{
  int ret;
  HRESULT hr = S_OK;

  if (sock <= 0)
    return E_HANDLE;
  if (state == NULL)
    return E_INVALIDARG;

  ret = _com_set_state(sock, state);

  if (OSFAILED(ret)) {
    ret = DNGetLastError();
    hr = OSERR2HRESULT(ret);
  }

  return hr;
}

/**
 * @fn        HRESULT com_get_modem_state(int sock, uint32_t *state)
 * @brief     Gets the serial port pin status.
 * @param[in] sock The socket to be gotten.
 * @param[in] state The gotten serial port pin status.
 */
HRESULT
com_get_modem_state(int sock, uint32_t *state)
{
  int ret;
  HRESULT hr = S_OK;

  if (sock <= 0)
    return E_HANDLE;
  if (state == NULL)
    return E_INVALIDARG;

  ret = _com_get_modem_state(sock, state);

  if (OSFAILED(ret)) {
    ret = DNGetLastError();
    hr = OSERR2HRESULT(ret);
  }

  return hr;
}
