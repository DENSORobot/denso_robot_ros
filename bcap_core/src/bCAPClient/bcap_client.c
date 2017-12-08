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

#include <stdarg.h>
#include "stdint.h"
#include <stdlib.h>
#include <string.h>

#if defined(_USE_WIN_API)
#include <winsock2.h>
#pragma comment(lib, "wsock32.lib")
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#include <termios.h>
#else
#include "dn_additional.h"
#endif

#include "dn_common.h"
#include "dn_device.h"
#include "dn_tcp.h"
#include "dn_udp.h"
#include "dn_com.h"
#include "dn_thread.h"
#include "bcap_common.h"
#include "bcap_funcid.h"
#include "bcap_client.h"

/**
 * @def   _RETRY_MIN
 * @brief A definition for the minimum retry count.
 */
#define _RETRY_MIN (1)

/**
 * @def   _RETRY_MAX
 * @brief A definition for the maximum retry count.
 */
#define _RETRY_MAX (7)

/**
 * @struct CONN_BCAP_CLIENT
 * @brief  b-CAP client communication object.
 */
struct CONN_BCAP_CLIENT
{
  struct CONN_PARAM_COMMON device; /* Common communication object */
  unsigned int retry;              /* Retry count: >= 0           */
  uint16_t serial;                 /* Serial number: 1 - 65535    */
  MUTEX mutex;                     /* Mutex object                */
};

static struct CONN_BCAP_CLIENT m_conn_param[BCAP_CONN_MAX];

/**
 * @fn    int find_open_address()
 * @brief Returns the open address of m_conn_param.
 * @note  If there is no open space, then returns 0.
 */
static int
find_open_address()
{
  int i, index = -1;

  for (i = 0; i < BCAP_CONN_MAX; i++) {
    if (m_conn_param[i].device.sock == 0) {
      index = i;
      break;
    }
  }

  return (index + 1);
}

/**
 * @fn        struct CONN_BCAP_CLIENT* check_address(int index)
 * @brief     Checks whether the index has been used or not.
 * @param[in] index The index of m_conn_param.
 * @note      If the index has not been used then returns NULL.
 */
static struct CONN_BCAP_CLIENT*
check_address(int index)
{
  index--;

  if (index < 0 || BCAP_CONN_MAX <= index) {
    return NULL;
  }
  else if (m_conn_param[index].device.sock == 0) {
    return NULL;
  }

  return &m_conn_param[index];
}

/**
 * @fn   HRESULT send_receive(int index, struct BCAP_PACKET *packet_send, struct BCAP_PACKET *packet_recv)
 * @brief  Sends and receives the b-CAP packet.
 * @param[in] index The index of m_conn_param.
 * @param[in] packet_send The b-CAP packet to be sent.
 * @param[out] packet_recv The received b-CAP packet.
 */
static HRESULT
send_receive(int index, struct BCAP_PACKET *packet_send,
    struct BCAP_PACKET *packet_recv)
{
  unsigned int retry_cnt;
  HRESULT hr;
  struct CONN_BCAP_CLIENT *bcap_param;

  bcap_param = check_address(index);
  if (bcap_param == NULL)
    return E_HANDLE;

  /* Locks mutex and must not returns this function without end of one. */
  hr = lock_mutex(&bcap_param->mutex, INFINITE);
  if (FAILED(hr))
    return hr;

  packet_send->reserv =
      (bcap_param->device.type == CONN_TCP) ? 0 : bcap_param->serial;
  packet_recv->id = E_FAIL;

  for (retry_cnt = 0; retry_cnt < bcap_param->retry; retry_cnt++) {
    packet_send->serial = bcap_param->serial++;
    if (bcap_param->serial == 0)
      bcap_param->serial = 1;

    hr = bcap_send(&bcap_param->device, packet_send);
    if (FAILED(hr))
      goto exit_proc;

    while (1) {
      packet_recv->argc = 1;
      hr = bcap_recv(&bcap_param->device, packet_recv, 1);
      if (FAILED(hr)) {
        goto retry_proc;
      } else {
        if ((packet_send->serial == packet_recv->serial)
            && (packet_recv->id != S_EXECUTING))
        {
          goto exit_proc;
        }
      }
    }
retry_proc:
    ;
  }

exit_proc:
  /* Unlocks mutex */
  unlock_mutex(&bcap_param->mutex);

  /* Sets the received return code to hr */
  if (SUCCEEDED(hr)) {
    hr = packet_recv->id;
  }

  return hr;
}

/**
 * @fn   invoke_method(int fd, int32_t id, int argc, char *format, ...)
 * @brief  Invoke b-CAP function.
 * @param[in] fd File descriptor.
 * @param[in] id The b-CAP function ID.
 * @param[in] argc The number of arguments. This is not contain the return value.
 * @param[in] format The format string of below arguments.
 * @note  format rule:
 * @note  Each type of arguments is written with the vt value.
 * @note  The splitter each of arguments is written with ","
 * @note  The splitter between the last argument and the return value is written with ":"
 */
static HRESULT
invoke_function(int fd, int32_t id, int argc, char *format, ...)
{
  int i, len;
  int32_t vt;
  void *pRet;
  char *chArg, *chRet, *chTmp;
  VARIANT *vntArg = NULL, vntRet;
  va_list list;
  struct BCAP_PACKET packet_send, packet_recv;
  HRESULT hr = S_OK;

  if (argc > 0) {
    vntArg = (VARIANT *) malloc(argc * sizeof(VARIANT));
    if (vntArg == NULL)
      return E_OUTOFMEMORY;
  }

  for (i = 0; i < argc; i++) {
    VariantInit(&vntArg[i]);
  }
  VariantInit(&vntRet);

  /* Input parameters */
  chArg = format;

  /* Output parameter */
  chRet = strchr(format, ':');
  if (chRet != NULL) {
    *chRet = '\0';
    chRet++;
  }

  va_start(list, format);

  /* Sets input parameters */
  i = 0;
  while ((*chArg) || (i < argc)) {
    len = 0;

    chTmp = strchr(chArg, ',');
    if (chTmp != NULL) {
      *chTmp = '\0';
      len++;
    }

    len += strlen(chArg);

    vt = atoi(chArg);

    switch (vt) {
      case VT_I4:
        vntArg[i].vt = VT_I4;
        vntArg[i].lVal = va_arg(list, int32_t);
        break;
      case VT_R4:
        vntArg[i].vt = VT_R4;
        vntArg[i].fltVal = (float) va_arg(list, double);
        break;
      case VT_BSTR:
        vntArg[i].vt = VT_BSTR;
        vntArg[i].bstrVal = va_arg(list, BSTR);
        break;
      case VT_VARIANT:
        vntArg[i] = va_arg(list, VARIANT);
        break;
      default:
        hr = E_INVALIDARG;
        goto exit_proc;
    }

    i++;
    chArg += len;
  }

  /* Sets send packet */
  packet_send.id = id;
  packet_send.argc = argc;
  packet_send.args = vntArg;

  /* Sets receive packet */
  packet_recv.argc = 1;
  packet_recv.args = &vntRet;

  vt = 0;
  pRet = NULL;
  if (chRet != NULL) {
    vt = atoi(chRet);
    pRet = va_arg(list, void *);

    if ((pRet != NULL) && (vt == VT_VARIANT)) {
      packet_recv.args = (VARIANT *) pRet;
    }
  }

  hr = send_receive(fd, &packet_send, &packet_recv);

  if (SUCCEEDED(hr)) {
    if ((pRet == NULL) || (vt != vntRet.vt)) {
      goto exit_proc;
    }

    switch (vt) {
      case VT_I4:
        *(int32_t *) pRet = vntRet.lVal;
        break;
      case VT_BSTR:
        *(BSTR *) pRet = SysAllocString(vntRet.bstrVal);
        break;
      default:
        break;
    }
  }

exit_proc:
  va_end(list);

  if (vntArg != NULL) {
    free(vntArg);
  }

  VariantClear(&vntRet);

  return hr;
}

HRESULT
bCap_Open_Client(const char* connect, uint32_t timeout, unsigned int retry,
    int* pfd)
{
  int index, *sock;
  extern uint32_t tcp_conn_timeout;
  HRESULT hr;
  void *conn_param;
  struct CONN_PARAM_ETH eth_param =
    { inet_addr("127.0.0.1"), htons(5007), 0, 0 };
  struct CONN_PARAM_COM com_param =
    { 1, 38400, NOPARITY, 8, ONESTOPBIT, 0 };
  struct CONN_BCAP_CLIENT *bcap_param;
  struct CONN_PARAM_COMMON *device;
  struct sockaddr_in *paddr;

  if (connect == NULL || pfd == NULL)
    return E_INVALIDARG;

  index = find_open_address();
  if (index == 0)
    return E_MAX_OBJECT;

  bcap_param = &m_conn_param[index - 1];
  device = &bcap_param->device;

  /* Initializes connection parameters */
  device->type = parse_conn_type(connect);
  switch (device->type) {
    case CONN_TCP:
      hr = parse_conn_param_ether(connect, &eth_param);
      conn_param = &eth_param;
      device->arg = NULL;
      device->dn_open = &tcp_open_client;
      device->dn_close = &tcp_close;
      device->dn_send = &tcp_send;
      device->dn_recv = &tcp_recv;
      device->dn_set_timeout = &tcp_set_timeout;
      tcp_conn_timeout = timeout;
      break;
    case CONN_UDP:
      hr = parse_conn_param_ether(connect, &eth_param);
      conn_param = &eth_param;
      paddr = (struct sockaddr_in *) malloc(sizeof(struct sockaddr_in));
      if (paddr == NULL) {
        hr = E_OUTOFMEMORY;
        break;
      }
      paddr->sin_addr.s_addr = eth_param.dst_addr;
      paddr->sin_port = eth_param.dst_port;
      paddr->sin_family = AF_INET;
      device->arg = (void *) paddr;
      device->dn_open = &udp_open;
      device->dn_close = &udp_close;
      device->dn_send = &udp_send;
      device->dn_recv = &udp_recv;
      device->dn_set_timeout = &udp_set_timeout;
      break;
    case CONN_COM:
      hr = parse_conn_param_serial(connect, &com_param);
      conn_param = &com_param;
      device->arg = NULL;
      device->dn_open = &com_open;
      device->dn_close = &com_close;
      device->dn_send = &com_send;
      device->dn_recv = &com_recv;
      device->dn_set_timeout = &com_set_timeout;
      break;
    default:
      hr = E_INVALIDARG;
      break;
  }

  if (FAILED(hr)) {
    if (device->arg != NULL) {
      free(device->arg);
      device->arg = NULL;
    }
    memset(bcap_param, 0, sizeof(struct CONN_BCAP_CLIENT));
    return hr;
  }

  /* Initializes mutex */
  hr = initialize_mutex(&bcap_param->mutex);
  if (FAILED(hr)) {
    if (device->arg != NULL) {
      free(device->arg);
      device->arg = NULL;
    }
    return hr;
  }

  /* Opens connection */
  sock = &device->sock;
  hr = device->dn_open(conn_param, sock);
  if (FAILED(hr)) {
    release_mutex(&bcap_param->mutex);
    if (device->arg != NULL) {
      free(device->arg);
      device->arg = NULL;
    }
    memset(bcap_param, 0, sizeof(struct CONN_BCAP_CLIENT));
    return hr;
  }

  hr = bCap_SetTimeout(index, timeout);
  if (FAILED(hr)) {
    bCap_Close_Client(&index);
    return hr;
  }

  hr = bCap_SetRetry(index, retry);
  if (FAILED(hr)) {
    bCap_Close_Client(&index);
    return hr;
  }

  /* Sets parameters */
  bcap_param->serial = 1;

  *pfd = index;

  return S_OK;
}

HRESULT
bCap_Close_Client(int *pfd)
{
  int index, *sock;
  struct CONN_BCAP_CLIENT *bcap_param;
  struct CONN_PARAM_COMMON *device;

  if (pfd == NULL)
    return E_HANDLE;

  index = *pfd;

  bcap_param = check_address(index);
  if (bcap_param == NULL)
    return E_HANDLE;

  device = &bcap_param->device;
  sock = &device->sock;

  /* Releases mutex */
  release_mutex(&bcap_param->mutex);

  /* Closes connection */
  device->dn_close(sock);

  /* Releases argument */
  if (device->arg != NULL) {
    free(device->arg);
    device->arg = NULL;
  }

  /* Resets connection parameters */
  memset(bcap_param, 0, sizeof(struct CONN_BCAP_CLIENT));

  *pfd = 0;

  return S_OK;
}

HRESULT
bCap_SetTimeout(int fd, uint32_t timeout)
{
  int *sock;
  HRESULT hr;
  struct CONN_BCAP_CLIENT *bcap_param;
  struct CONN_PARAM_COMMON *device;

  bcap_param = check_address(fd);
  if (bcap_param == NULL)
    return E_HANDLE;

  device = &bcap_param->device;
  sock = &device->sock;

  /* Locks mutex and must not returns this function without end of one. */
  hr = lock_mutex(&bcap_param->mutex, INFINITE);
  if (FAILED(hr))
    return hr;

  hr = device->dn_set_timeout(*sock, timeout);
  if (SUCCEEDED(hr)) {
    device->timeout = timeout;
  }

  /* Unlocks mutex */
  unlock_mutex(&bcap_param->mutex);

  return hr;
}

HRESULT
bCap_GetTimeout(int fd, uint32_t *timeout)
{
  struct CONN_BCAP_CLIENT *bcap_param;
  struct CONN_PARAM_COMMON *device;

  bcap_param = check_address(fd);
  if (bcap_param == NULL)
    return E_HANDLE;

  if (timeout == NULL)
    return E_INVALIDARG;

  device = &bcap_param->device;
  *timeout = device->timeout;

  return S_OK;
}

HRESULT
bCap_SetRetry(int fd, unsigned int retry)
{
  HRESULT hr;
  struct CONN_BCAP_CLIENT *bcap_param;
  struct CONN_PARAM_COMMON *device;

  bcap_param = check_address(fd);
  if (bcap_param == NULL)
    return E_HANDLE;

  device = &bcap_param->device;

  /* Locks mutex and must not returns this function without end of one. */
  hr = lock_mutex(&bcap_param->mutex, INFINITE);
  if (FAILED(hr))
    return hr;

  switch (device->type) {
    case CONN_TCP:
      bcap_param->retry = 1;
      break;
    case CONN_UDP:
    case CONN_COM:
      if (retry < _RETRY_MIN) {
        bcap_param->retry = _RETRY_MIN;
      }
      else if (_RETRY_MAX < retry) {
        bcap_param->retry = _RETRY_MAX;
      }
      else {
        bcap_param->retry = retry;
      }
      break;
    default:
      hr = E_HANDLE;
      break;
  }

  /* Unlocks mutex */
  unlock_mutex(&bcap_param->mutex);

  return hr;
}

HRESULT
bCap_GetRetry(int fd, unsigned int *retry)
{
  struct CONN_BCAP_CLIENT *bcap_param;

  bcap_param = check_address(fd);
  if (bcap_param == NULL)
    return E_HANDLE;

  if (retry == NULL)
    return E_INVALIDARG;

  *retry = bcap_param->retry;

  return S_OK;
}

HRESULT
bCap_ServiceStart(int fd, BSTR bstrOption)
{
  char format[] = "8";
  return invoke_function(fd, ID_SERVICE_START, 1, format, bstrOption);
}

HRESULT
bCap_ServiceStop(int fd)
{
  char format[] = "";
  return invoke_function(fd, ID_SERVICE_STOP, 0, format);
}

HRESULT
bCap_ControllerConnect(int fd, BSTR bstrName, BSTR bstrProvider,
    BSTR bstrMachine, BSTR bstrOption, uint32_t *hController)
{
  char format[] = "8,8,8,8:3";
  if (hController == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_CONNECT, 4, format, bstrName,
      bstrProvider, bstrMachine, bstrOption, hController);
}

HRESULT
bCap_ControllerDisconnect(int fd, uint32_t *hController)
{
  char format[] = "3";
  HRESULT hr;

  if (hController == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_CONTROLLER_DISCONNECT, 1, format, *hController);
  if (SUCCEEDED(hr)) {
    *hController = 0;
  }

  return hr;
}

HRESULT
bCap_ControllerGetExtension(int fd, uint32_t hController, BSTR bstrName,
    BSTR bstrOption, uint32_t *hExtension)
{
  char format[] = "3,8,8:3";
  if (hExtension == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_GETEXTENSION, 3, format, hController,
      bstrName, bstrOption, hExtension);
}

HRESULT
bCap_ControllerGetFile(int fd, uint32_t hController, BSTR bstrName,
    BSTR bstrOption, uint32_t *hFile)
{
  char format[] = "3,8,8:3";
  if (hFile == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_GETFILE, 3, format, hController,
      bstrName, bstrOption, hFile);
}

HRESULT
bCap_ControllerGetRobot(int fd, uint32_t hController, BSTR bstrName,
    BSTR bstrOption, uint32_t *hRobot)
{
  char format[] = "3,8,8:3";
  if (hRobot == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_GETROBOT, 3, format, hController,
      bstrName, bstrOption, hRobot);
}

HRESULT
bCap_ControllerGetTask(int fd, uint32_t hController, BSTR bstrName,
    BSTR bstrOption, uint32_t *hTask)
{
  char format[] = "3,8,8:3";
  if (hTask == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_GETTASK, 3, format, hController,
      bstrName, bstrOption, hTask);
}

HRESULT
bCap_ControllerGetVariable(int fd, uint32_t hController, BSTR bstrName,
    BSTR bstrOption, uint32_t *hVariable)
{
  char format[] = "3,8,8:3";
  if (hVariable == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_GETVARIABLE, 3, format, hController,
      bstrName, bstrOption, hVariable);
}

HRESULT
bCap_ControllerGetCommand(int fd, uint32_t hController, BSTR bstrName,
    BSTR bstrOption, uint32_t *hCommand)
{
  char format[] = "3,8,8:3";
  if (hCommand == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_GETCOMMAND, 3, format, hController,
      bstrName, bstrOption, hCommand);
}

HRESULT
bCap_ControllerGetExtensionNames(int fd, uint32_t hController, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_CONTROLLER_GETEXTENSIONNAMES, 2, format,
      hController, bstrOption, pVal);
}

HRESULT
bCap_ControllerGetFileNames(int fd, uint32_t hController, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_CONTROLLER_GETFILENAMES, 2, format, hController,
      bstrOption, pVal);
}

HRESULT
bCap_ControllerGetRobotNames(int fd, uint32_t hController, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_CONTROLLER_GETROBOTNAMES, 2, format,
      hController, bstrOption, pVal);
}

HRESULT
bCap_ControllerGetTaskNames(int fd, uint32_t hController, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_CONTROLLER_GETTASKNAMES, 2, format, hController,
      bstrOption, pVal);
}

HRESULT
bCap_ControllerGetVariableNames(int fd, uint32_t hController, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_CONTROLLER_GETVARIABLENAMES, 2, format,
      hController, bstrOption, pVal);
}

HRESULT
bCap_ControllerGetCommandNames(int fd, uint32_t hController, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_CONTROLLER_GETCOMMANDNAMES, 2, format,
      hController, bstrOption, pVal);
}

HRESULT
bCap_ControllerExecute(int fd, uint32_t hController, BSTR bstrCommand,
    VARIANT vntParam, VARIANT *pVal)
{
  char format[] = "3,8,12:12";
  return invoke_function(fd, ID_CONTROLLER_EXECUTE, 3, format, hController,
      bstrCommand, vntParam, pVal);
}

HRESULT
bCap_ControllerGetMessage(int fd, uint32_t hController, uint32_t *hMessage)
{
  char format[] = "3:3";
  if (hMessage == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_CONTROLLER_GETMESSAGE, 1, format, hController,
      hMessage);
}

HRESULT
bCap_ControllerGetAttribute(int fd, uint32_t hController, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_CONTROLLER_GETATTRIBUTE, 1, format, hController,
      pVal);
}

HRESULT
bCap_ControllerGetHelp(int fd, uint32_t hController, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_CONTROLLER_GETHELP, 1, format, hController,
      pVal);
}

HRESULT
bCap_ControllerGetName(int fd, uint32_t hController, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_CONTROLLER_GETNAME, 1, format, hController,
      pVal);
}

HRESULT
bCap_ControllerGetTag(int fd, uint32_t hController, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_CONTROLLER_GETTAG, 1, format, hController, pVal);
}

HRESULT
bCap_ControllerPutTag(int fd, uint32_t hController, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_CONTROLLER_PUTTAG, 2, format, hController,
      newVal);
}

HRESULT
bCap_ControllerGetID(int fd, uint32_t hController, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_CONTROLLER_GETID, 1, format, hController, pVal);
}

HRESULT
bCap_ControllerPutID(int fd, uint32_t hController, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_CONTROLLER_PUTID, 2, format, hController,
      newVal);
}

HRESULT
bCap_ExtensionGetVariable(int fd, uint32_t hExtension, BSTR bstrName,
    BSTR bstrOption, uint32_t *hVariable)
{
  char format[] = "3,8,8:3";
  if (hVariable == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_EXTENSION_GETVARIABLE, 3, format, hExtension,
      bstrName, bstrOption, hVariable);
}

HRESULT
bCap_ExtensionGetVariableNames(int fd, uint32_t hExtension, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_EXTENSION_GETVARIABLENAMES, 2, format,
      hExtension, bstrOption, pVal);
}

HRESULT
bCap_ExtensionExecute(int fd, uint32_t hExtension, BSTR bstrCommand,
    VARIANT vntParam, VARIANT *pVal)
{
  char format[] = "3,8,12:12";
  return invoke_function(fd, ID_EXTENSION_EXECUTE, 3, format, hExtension,
      bstrCommand, vntParam, pVal);
}

HRESULT
bCap_ExtensionGetAttribute(int fd, uint32_t hExtension, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_EXTENSION_GETATTRIBUTE, 1, format, hExtension,
      pVal);
}

HRESULT
bCap_ExtensionGetHelp(int fd, uint32_t hExtension, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_EXTENSION_GETHELP, 1, format, hExtension, pVal);
}

HRESULT
bCap_ExtensionGetName(int fd, uint32_t hExtension, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_EXTENSION_GETNAME, 1, format, hExtension, pVal);
}

HRESULT
bCap_ExtensionGetTag(int fd, uint32_t hExtension, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_EXTENSION_GETTAG, 1, format, hExtension, pVal);
}

HRESULT
bCap_ExtensionPutTag(int fd, uint32_t hExtension, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_EXTENSION_PUTTAG, 2, format, hExtension, newVal);
}

HRESULT
bCap_ExtensionGetID(int fd, uint32_t hExtension, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_EXTENSION_GETID, 1, format, hExtension, pVal);
}

HRESULT
bCap_ExtensionPutID(int fd, uint32_t hExtension, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_EXTENSION_PUTID, 2, format, hExtension, newVal);
}

HRESULT
bCap_ExtensionRelease(int fd, uint32_t *hExtension)
{
  char format[] = "3";
  HRESULT hr;

  if (hExtension == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_EXTENSION_RELEASE, 1, format, *hExtension);
  if (SUCCEEDED(hr)) {
    *hExtension = 0;
  }

  return hr;
}

HRESULT
bCap_FileGetFile(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption,
    uint32_t *hFile2)
{
  char format[] = "3,8,8:3";
  if (hFile2 == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_FILE_GETFILE, 3, format, hFile, bstrName,
      bstrOption, hFile2);
}

HRESULT
bCap_FileGetVariable(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption,
    uint32_t *hVariable)
{
  char format[] = "3,8,8:3";
  if (hVariable == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_FILE_GETVARIABLE, 3, format, hFile, bstrName,
      bstrOption, hVariable);
}

HRESULT
bCap_FileGetFileNames(int fd, uint32_t hFile, BSTR bstrOption, VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_FILE_GETFILENAMES, 2, format, hFile, bstrOption,
      pVal);
}

HRESULT
bCap_FileGetVariableNames(int fd, uint32_t hFile, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_FILE_GETVARIABLENAMES, 2, format, hFile,
      bstrOption, pVal);
}

HRESULT
bCap_FileExecute(int fd, uint32_t hFile, BSTR bstrCommand, VARIANT vntParam,
    VARIANT *pVal)
{
  char format[] = "3,8,12:12";
  return invoke_function(fd, ID_FILE_EXECUTE, 3, format, hFile, bstrCommand,
      vntParam, pVal);
}

HRESULT
bCap_FileCopy(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption)
{
  char format[] = "3,8,8";
  return invoke_function(fd, ID_FILE_COPY, 3, format, hFile, bstrName,
      bstrOption);
}

HRESULT
bCap_FileDelete(int fd, uint32_t hFile, BSTR bstrOption)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_FILE_DELETE, 2, format, hFile, bstrOption);
}

HRESULT
bCap_FileMove(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption)
{
  char format[] = "3,8,8";
  return invoke_function(fd, ID_FILE_MOVE, 3, format, hFile, bstrName,
      bstrOption);
}

HRESULT
bCap_FileRun(int fd, uint32_t hFile, BSTR bstrOption, BSTR *pVal)
{
  char format[] = "3,8:8";
  return invoke_function(fd, ID_FILE_RUN, 2, format, hFile, bstrOption, pVal);
}

HRESULT
bCap_FileGetDateCreated(int fd, uint32_t hFile, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_FILE_GETDATECREATED, 1, format, hFile, pVal);
}

HRESULT
bCap_FileGetDateLastAccessed(int fd, uint32_t hFile, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_FILE_GETDATELASTACCESSED, 1, format, hFile,
      pVal);
}

HRESULT
bCap_FileGetDateLastModified(int fd, uint32_t hFile, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_FILE_GETDATELASTMODIFIED, 1, format, hFile,
      pVal);
}

HRESULT
bCap_FileGetPath(int fd, uint32_t hFile, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_FILE_GETPATH, 1, format, hFile, pVal);
}

HRESULT
bCap_FileGetSize(int fd, uint32_t hFile, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_FILE_GETSIZE, 1, format, hFile, pVal);
}

HRESULT
bCap_FileGetType(int fd, uint32_t hFile, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_FILE_GETTYPE, 1, format, hFile, pVal);
}

HRESULT
bCap_FileGetValue(int fd, uint32_t hFile, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_FILE_GETVALUE, 1, format, hFile, pVal);
}

HRESULT
bCap_FilePutValue(int fd, uint32_t hFile, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_FILE_PUTVALUE, 2, format, hFile, newVal);
}

HRESULT
bCap_FileGetAttribute(int fd, uint32_t hFile, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_FILE_GETATTRIBUTE, 1, format, hFile, pVal);
}

HRESULT
bCap_FileGetHelp(int fd, uint32_t hFile, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_FILE_GETHELP, 1, format, hFile, pVal);
}

HRESULT
bCap_FileGetName(int fd, uint32_t hFile, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_FILE_GETNAME, 1, format, hFile, pVal);
}

HRESULT
bCap_FileGetTag(int fd, uint32_t hFile, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_FILE_GETTAG, 1, format, hFile, pVal);
}

HRESULT
bCap_FilePutTag(int fd, uint32_t hFile, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_FILE_PUTTAG, 2, format, hFile, newVal);
}

HRESULT
bCap_FileGetID(int fd, uint32_t hFile, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_FILE_GETID, 1, format, hFile, pVal);
}

HRESULT
bCap_FilePutID(int fd, uint32_t hFile, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_FILE_PUTID, 2, format, hFile, newVal);
}

HRESULT
bCap_FileRelease(int fd, uint32_t *hFile)
{
  char format[] = "3";
  HRESULT hr;

  if (hFile == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_FILE_RELEASE, 1, format, *hFile);
  if (SUCCEEDED(hr)) {
    *hFile = 0;
  }

  return hr;
}

HRESULT
bCap_RobotGetVariable(int fd, uint32_t hRobot, BSTR bstrName, BSTR bstrOption,
    uint32_t *hVariable)
{
  char format[] = "3,8,8:3";
  if (hVariable == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_ROBOT_GETVARIABLE, 3, format, hRobot, bstrName,
      bstrOption, hVariable);
}

HRESULT
bCap_RobotGetVariableNames(int fd, uint32_t hRobot, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_ROBOT_GETVARIABLENAMES, 2, format, hRobot,
      bstrOption, pVal);
}

HRESULT
bCap_RobotExecute(int fd, uint32_t hRobot, BSTR bstrCommand, VARIANT vntParam,
    VARIANT *pVal)
{
  char format[] = "3,8,12:12";
  return invoke_function(fd, ID_ROBOT_EXECUTE, 3, format, hRobot, bstrCommand,
      vntParam, pVal);
}

HRESULT
bCap_RobotAccelerate(int fd, uint32_t hRobot, int32_t lAxis, float fAccel,
    float fDecel)
{
  char format[] = "3,3,4,4";
  return invoke_function(fd, ID_ROBOT_ACCELERATE, 4, format, hRobot, lAxis,
      fAccel, fDecel);
}

HRESULT
bCap_RobotChange(int fd, uint32_t hRobot, BSTR bstrName)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_ROBOT_CHANGE, 2, format, hRobot, bstrName);
}

HRESULT
bCap_RobotChuck(int fd, uint32_t hRobot, BSTR bstrOption)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_ROBOT_CHUCK, 2, format, hRobot, bstrOption);
}

HRESULT
bCap_RobotDrive(int fd, uint32_t hRobot, int32_t lNo, float fMov,
    BSTR bstrOption)
{
  char format[] = "3,3,4,8";
  return invoke_function(fd, ID_ROBOT_DRIVE, 4, format, hRobot, lNo, fMov,
      bstrOption);
}

HRESULT
bCap_RobotGoHome(int fd, uint32_t hRobot)
{
  char format[] = "3";
  return invoke_function(fd, ID_ROBOT_GOHOME, 1, format, hRobot);
}

HRESULT
bCap_RobotHalt(int fd, uint32_t hRobot, BSTR bstrOption)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_ROBOT_HALT, 2, format, hRobot, bstrOption);
}

HRESULT
bCap_RobotHold(int fd, uint32_t hRobot, BSTR bstrOption)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_ROBOT_HOLD, 2, format, hRobot, bstrOption);
}

HRESULT
bCap_RobotMove(int fd, uint32_t hRobot, int32_t lComp, VARIANT vntPose,
    BSTR bstrOption)
{
  char format[] = "3,3,12,8";
  return invoke_function(fd, ID_ROBOT_MOVE, 4, format, hRobot, lComp, vntPose,
      bstrOption);
}

HRESULT
bCap_RobotRotate(int fd, uint32_t hRobot, VARIANT vntRotSuf, float fDeg,
    VARIANT vntPivot, BSTR bstrOption)
{
  char format[] = "3,12,4,12,8";
  return invoke_function(fd, ID_ROBOT_ROTATE, 5, format, hRobot, vntRotSuf,
      fDeg, vntPivot, bstrOption);
}

HRESULT
bCap_RobotSpeed(int fd, uint32_t hRobot, int32_t lAxis, float fSpeed)
{
  char format[] = "3,3,4";
  return invoke_function(fd, ID_ROBOT_SPEED, 3, format, hRobot, lAxis, fSpeed);
}

HRESULT
bCap_RobotUnchuck(int fd, uint32_t hRobot, BSTR bstrOption)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_ROBOT_UNCHUCK, 2, format, hRobot, bstrOption);
}

HRESULT
bCap_RobotUnhold(int fd, uint32_t hRobot, BSTR bstrOption)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_ROBOT_UNHOLD, 2, format, hRobot, bstrOption);
}

HRESULT
bCap_RobotGetAttribute(int fd, uint32_t hRobot, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_ROBOT_GETATTRIBUTE, 1, format, hRobot, pVal);
}

HRESULT
bCap_RobotGetHelp(int fd, uint32_t hRobot, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_ROBOT_GETHELP, 1, format, hRobot, pVal);
}

HRESULT
bCap_RobotGetName(int fd, uint32_t hRobot, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_ROBOT_GETNAME, 1, format, hRobot, pVal);
}

HRESULT
bCap_RobotGetTag(int fd, uint32_t hRobot, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_ROBOT_GETTAG, 1, format, hRobot, pVal);
}

HRESULT
bCap_RobotPutTag(int fd, uint32_t hRobot, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_ROBOT_PUTTAG, 2, format, hRobot, newVal);
}

HRESULT
bCap_RobotGetID(int fd, uint32_t hRobot, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_ROBOT_GETID, 1, format, hRobot, pVal);
}

HRESULT
bCap_RobotPutID(int fd, uint32_t hRobot, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_ROBOT_PUTID, 2, format, hRobot, newVal);
}

HRESULT
bCap_RobotRelease(int fd, uint32_t *hRobot)
{
  char format[] = "3";
  HRESULT hr;

  if (hRobot == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_ROBOT_RELEASE, 1, format, *hRobot);
  if (SUCCEEDED(hr)) {
    *hRobot = 0;
  }

  return hr;
}

HRESULT
bCap_TaskGetVariable(int fd, uint32_t hTask, BSTR bstrName, BSTR bstrOption,
    uint32_t *hVariable)
{
  char format[] = "3,8,8:3";
  if (hVariable == NULL)
    return E_INVALIDARG;
  return invoke_function(fd, ID_TASK_GETVARIABLE, 3, format, hTask, bstrName,
      bstrOption, hVariable);
}

HRESULT
bCap_TaskGetVariableNames(int fd, uint32_t hTask, BSTR bstrOption,
    VARIANT *pVal)
{
  char format[] = "3,8:12";
  return invoke_function(fd, ID_TASK_GETVARIABLENAMES, 2, format, hTask,
      bstrOption, pVal);
}

HRESULT
bCap_TaskExecute(int fd, uint32_t hTask, BSTR bstrCommand, VARIANT vntParam,
    VARIANT *pVal)
{
  char format[] = "3,8,12:12";
  return invoke_function(fd, ID_TASK_EXECUTE, 3, format, hTask, bstrCommand,
      vntParam, pVal);
}

HRESULT
bCap_TaskStart(int fd, uint32_t hTask, int32_t lMode, BSTR bstrOption)
{
  char format[] = "3,3,8";
  return invoke_function(fd, ID_TASK_START, 3, format, hTask, lMode, bstrOption);
}

HRESULT
bCap_TaskStop(int fd, uint32_t hTask, int32_t lMode, BSTR bstrOption)
{
  char format[] = "3,3,8";
  return invoke_function(fd, ID_TASK_STOP, 3, format, hTask, lMode, bstrOption);
}

HRESULT
bCap_TaskDelete(int fd, uint32_t hTask, BSTR bstrOption)
{
  char format[] = "3,8";
  return invoke_function(fd, ID_TASK_DELETE, 2, format, hTask, bstrOption);
}

HRESULT
bCap_TaskGetFileName(int fd, uint32_t hTask, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_TASK_GETFILENAME, 1, format, hTask, pVal);
}

HRESULT
bCap_TaskGetAttribute(int fd, uint32_t hTask, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_TASK_GETATTRIBUTE, 1, format, hTask, pVal);
}

HRESULT
bCap_TaskGetHelp(int fd, uint32_t hTask, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_TASK_GETHELP, 1, format, hTask, pVal);
}

HRESULT
bCap_TaskGetName(int fd, uint32_t hTask, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_TASK_GETNAME, 1, format, hTask, pVal);
}

HRESULT
bCap_TaskGetTag(int fd, uint32_t hTask, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_TASK_GETTAG, 1, format, hTask, pVal);
}

HRESULT
bCap_TaskPutTag(int fd, uint32_t hTask, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_TASK_PUTTAG, 2, format, hTask, newVal);
}

HRESULT
bCap_TaskGetID(int fd, uint32_t hTask, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_TASK_GETID, 1, format, hTask, pVal);
}

HRESULT
bCap_TaskPutID(int fd, uint32_t hTask, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_TASK_PUTID, 2, format, hTask, newVal);
}

HRESULT
bCap_TaskRelease(int fd, uint32_t *hTask)
{
  char format[] = "3";
  HRESULT hr;

  if (hTask == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_TASK_RELEASE, 1, format, *hTask);
  if (SUCCEEDED(hr)) {
    *hTask = 0;
  }

  return hr;
}

HRESULT
bCap_VariableGetDateTime(int fd, uint32_t hVariable, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_VARIABLE_GETDATETIME, 1, format, hVariable,
      pVal);
}

HRESULT
bCap_VariableGetValue(int fd, uint32_t hVariable, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_VARIABLE_GETVALUE, 1, format, hVariable, pVal);
}

HRESULT
bCap_VariablePutValue(int fd, uint32_t hVariable, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_VARIABLE_PUTVALUE, 2, format, hVariable, newVal);
}

HRESULT
bCap_VariableGetAttribute(int fd, uint32_t hVariable, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_VARIABLE_GETATTRIBUTE, 1, format, hVariable,
      pVal);
}

HRESULT
bCap_VariableGetHelp(int fd, uint32_t hVariable, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_VARIABLE_GETHELP, 1, format, hVariable, pVal);
}

HRESULT
bCap_VariableGetName(int fd, uint32_t hVariable, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_VARIABLE_GETNAME, 1, format, hVariable, pVal);
}

HRESULT
bCap_VariableGetTag(int fd, uint32_t hVariable, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_VARIABLE_GETTAG, 1, format, hVariable, pVal);
}

HRESULT
bCap_VariablePutTag(int fd, uint32_t hVariable, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_VARIABLE_PUTTAG, 2, format, hVariable, newVal);
}

HRESULT
bCap_VariableGetID(int fd, uint32_t hVariable, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_VARIABLE_GETID, 1, format, hVariable, pVal);
}

HRESULT
bCap_VariablePutID(int fd, uint32_t hVariable, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_VARIABLE_PUTID, 2, format, hVariable, newVal);
}

HRESULT
bCap_VariableGetMicrosecond(int fd, uint32_t hVariable, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_VARIABLE_GETMICROSECOND, 1, format, hVariable,
      pVal);
}

HRESULT
bCap_VariableRelease(int fd, uint32_t *hVariable)
{
  char format[] = "3";
  HRESULT hr;

  if (hVariable == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_VARIABLE_RELEASE, 1, format, *hVariable);
  if (SUCCEEDED(hr)) {
    *hVariable = 0;
  }

  return hr;
}

HRESULT
bCap_CommandExecute(int fd, uint32_t hCommand, int32_t lMode, VARIANT *pVal)
{
  char format[] = "3,3:12";
  return invoke_function(fd, ID_COMMAND_EXECUTE, 2, format, hCommand, lMode,
      pVal);
}

HRESULT
bCap_CommandCancel(int fd, uint32_t hCommand)
{
  char format[] = "3";
  return invoke_function(fd, ID_COMMAND_CANCEL, 1, format, hCommand);
}

HRESULT
bCap_CommandGetTimeout(int fd, uint32_t hCommand, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_COMMAND_GETTIMEOUT, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandPutTimeout(int fd, uint32_t hCommand, int32_t newVal)
{
  char format[] = "3,3";
  return invoke_function(fd, ID_COMMAND_PUTTIMEOUT, 2, format, hCommand, newVal);
}

HRESULT
bCap_CommandGetState(int fd, uint32_t hCommand, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_COMMAND_GETSTATE, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandGetParameters(int fd, uint32_t hCommand, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_COMMAND_GETPARAMETERS, 1, format, hCommand,
      pVal);
}

HRESULT
bCap_CommandPutParameters(int fd, uint32_t hCommand, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_COMMAND_PUTPARAMETERS, 2, format, hCommand,
      newVal);
}

HRESULT
bCap_CommandGetResult(int fd, uint32_t hCommand, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_COMMAND_GETRESULT, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandGetAttribute(int fd, uint32_t hCommand, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_COMMAND_GETATTRIBUTE, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandGetHelp(int fd, uint32_t hCommand, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_COMMAND_GETHELP, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandGetName(int fd, uint32_t hCommand, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_COMMAND_GETNAME, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandGetTag(int fd, uint32_t hCommand, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_COMMAND_GETTAG, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandPutTag(int fd, uint32_t hCommand, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_COMMAND_PUTTAG, 2, format, hCommand, newVal);
}

HRESULT
bCap_CommandGetID(int fd, uint32_t hCommand, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_COMMAND_GETID, 1, format, hCommand, pVal);
}

HRESULT
bCap_CommandPutID(int fd, uint32_t hCommand, VARIANT newVal)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_COMMAND_PUTID, 2, format, hCommand, newVal);
}

HRESULT
bCap_CommandRelease(int fd, uint32_t *hCommand)
{
  char format[] = "3";
  HRESULT hr;

  if (hCommand == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_COMMAND_RELEASE, 1, format, *hCommand);
  if (SUCCEEDED(hr)) {
    *hCommand = 0;
  }

  return hr;
}

HRESULT
bCap_MessageReply(int fd, uint32_t hMessage, VARIANT vntData)
{
  char format[] = "3,12";
  return invoke_function(fd, ID_MESSAGE_REPLY, 2, format, hMessage, vntData);
}

HRESULT
bCap_MessageClear(int fd, uint32_t hMessage)
{
  char format[] = "3";
  return invoke_function(fd, ID_MESSAGE_CLEAR, 1, format, hMessage);
}

HRESULT
bCap_MessageGetDateTime(int fd, uint32_t hMessage, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_MESSAGE_GETDATETIME, 1, format, hMessage, pVal);
}

HRESULT
bCap_MessageGetDescription(int fd, uint32_t hMessage, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_MESSAGE_GETDESCRIPTION, 1, format, hMessage,
      pVal);
}

HRESULT
bCap_MessageGetDestination(int fd, uint32_t hMessage, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_MESSAGE_GETDESTINATION, 1, format, hMessage,
      pVal);
}

HRESULT
bCap_MessageGetNumber(int fd, uint32_t hMessage, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_MESSAGE_GETNUMBER, 1, format, hMessage, pVal);
}

HRESULT
bCap_MessageGetSerialNumber(int fd, uint32_t hMessage, int32_t *pVal)
{
  char format[] = "3:3";
  return invoke_function(fd, ID_MESSAGE_GETSERIALNUMBER, 1, format, hMessage,
      pVal);
}

HRESULT
bCap_MessageGetSource(int fd, uint32_t hMessage, BSTR *pVal)
{
  char format[] = "3:8";
  return invoke_function(fd, ID_MESSAGE_GETSOURCE, 1, format, hMessage, pVal);
}

HRESULT
bCap_MessageGetValue(int fd, uint32_t hMessage, VARIANT *pVal)
{
  char format[] = "3:12";
  return invoke_function(fd, ID_MESSAGE_GETVALUE, 1, format, hMessage, pVal);
}

HRESULT
bCap_MessageRelease(int fd, uint32_t *hMessage)
{
  char format[] = "3";
  HRESULT hr;

  if (hMessage == NULL)
    return E_INVALIDARG;

  hr = invoke_function(fd, ID_MESSAGE_RELEASE, 1, format, *hMessage);
  if (SUCCEEDED(hr)) {
    *hMessage = 0;
  }

  return hr;
}
