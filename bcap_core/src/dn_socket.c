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

#if defined(_USE_WIN_API)
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#include "dn_additional.h"
#endif

#include "dn_common.h"
#include "dn_device.h"
#include "dn_socket.h"

int socket_counter;

#if defined(_USE_WIN_API)
static int _socket_close(int sock)
{
  shutdown(sock, SD_BOTH);
  return closesocket(sock);
}

static HRESULT _socket_bind(const struct CONN_PARAM_ETH *param, int *sock)
{
    int i, ret;
    HRESULT hr;
    DWORD d;
    char buffer[1024];
    SOCKET_ADDRESS_LIST *addressList = (SOCKET_ADDRESS_LIST *)buffer;

    /* Gets list of interfaces */
    if(WSAIoctl(*sock, SIO_ADDRESS_LIST_QUERY, NULL, 0, buffer, 1024, &d, NULL, NULL) != 0) {
      ret = DNGetLastError();
      return OSERR2HRESULT(ret);
    }

    /* Checks source address */
    if(param->src_addr == htonl(INADDR_LOOPBACK)) {
      hr = S_OK;
    } else {
      hr = E_FAIL;
      for(i = 0; i < addressList->iAddressCount; i++) {
        if(param->src_addr == ((SOCKADDR_IN *)addressList->Address[i].lpSockaddr)->sin_addr.s_addr) {
          hr = S_OK;
          break;
        }
      }
    }

    return hr;
}
#elif defined(_USE_LINUX_API)
static int _socket_close(int sock)
{
  shutdown(sock, SHUT_RDWR);
  return close(sock);
}

static HRESULT _socket_bind(const struct CONN_PARAM_ETH *param, int *sock)
{
  int ret;
  HRESULT hr;
  struct ifaddrs *ifaddr, *ifa;

  /* Gets list of interfaces */
  if(getifaddrs(&ifaddr) == -1) {
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

  /* Checks source address */
  hr = E_FAIL;
  for(ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if((ifa->ifa_addr == NULL) || (ifa->ifa_addr->sa_family != AF_INET)) {
      continue;
    }

    if(param->src_addr == (((struct sockaddr_in *)ifa->ifa_addr)->sin_addr.s_addr)) {
      hr = S_OK;
      break;
    }
  }

  freeifaddrs(ifaddr);

  return hr;
}
#endif

/**
 * @fn         HRESULT socket_open(int type, int *sock)
 * @brief      Creates a socket.
 * @param[in]  type The socket type. Allows SOCK_STREAM or SOCK_DGRAM.
 * @param[out] sock The created socket.
 * @note       This function is not thread safe.
 * @note       Must take mutex among socket_open, socket_close and tcp_accept.
 */
HRESULT
socket_open(int type, int *sock)
{
  int ret;

  if ((type != SOCK_STREAM) && (type != SOCK_DGRAM))
    return E_INVALIDARG;

  if (sock == NULL)
    return E_INVALIDARG;

#if defined(_USE_WIN_API)
  if(socket_counter == 0) {
    WSADATA wsaData;

    ret = WSAStartup(MAKEWORD(2, 0), &wsaData);
    if(ret != 0) {
      ret = DNGetLastError();
      return OSERR2HRESULT(ret);
    }

    if(MAKEWORD(2, 0) != wsaData.wVersion) {
      WSACleanup();
      return E_UNEXPECTED;
    }
  }
#endif

  *sock = socket(AF_INET, type, 0);
  if (*sock < 0) {
    *sock = 0;
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

  socket_counter++;

  return S_OK;
}

/**
 * @fn            HRESULT socket_close(int *sock)
 * @brief         Closes the socket.
 * @param[in,out] sock The socket to be closed.
 * @note          This function is not thread safe.
 * @note          Must take mutex among socket_open, socket_close and tcp_accept.
 */
HRESULT
socket_close(int *sock)
{
  int ret;

  if (sock == NULL || *sock <= 0)
    return E_HANDLE;

  ret = _socket_close(*sock);
  if (ret != 0) {
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

#if defined(_USE_WIN_API)
  if(socket_counter == 1) {
    WSACleanup();
  }
#endif

  *sock = 0;

  if (socket_counter > 0) {
    socket_counter--;
  }

  return S_OK;
}

/**
 * @fn            HRESULT socket_bind(const struct CONN_PARAM_ETH *param, int *sock)
 * @brief         Binds the socket with param's source address and source port.
 * @param[in]     param The pointer of Ethernet connection parameters.
 * @param[in,out] sock The socket to be binded.
 */
HRESULT
socket_bind(const struct CONN_PARAM_ETH *param, int *sock)
{
  int ret;
  struct sockaddr_in sockaddr =
    { AF_INET, };
  HRESULT hr = S_OK;

  if ((param == NULL) || (sock == NULL))
    return E_INVALIDARG;

  sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  sockaddr.sin_port = param->src_port;

  if ((param->src_addr != htonl(INADDR_ANY))
      && (param->src_addr != htonl(INADDR_NONE)))
  {
    hr = _socket_bind(param, sock);
    if (FAILED(hr)) {
      return hr;
    }

    sockaddr.sin_addr.s_addr = param->src_addr;
  }

  if ((sockaddr.sin_addr.s_addr != htonl(INADDR_ANY))
      || (sockaddr.sin_port != 0))
  {
    ret = bind(*sock, (struct sockaddr *) &sockaddr, sizeof(sockaddr));
    if (ret < 0) {
      ret = DNGetLastError();
      return OSERR2HRESULT(ret);
    }
  }

  return hr;
}

/**
 * @fn        HRESULT socket_set_timeout(int sock, uint32_t timeout)
 * @brief     Sets timeout value to the socket.
 * @param[in] sock The socket to be set.
 * @param[in] timeout Timeout value.
 */
HRESULT
socket_set_timeout(int sock, uint32_t timeout)
{
  int ret;
  struct timeval tv;

  if (sock <= 0)
    return E_HANDLE;

  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) * 1000;

  ret = setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *) &tv, sizeof(tv));
  if (ret < 0) {
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

  ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof(tv));
  if (ret < 0) {
    ret = DNGetLastError();
    return OSERR2HRESULT(ret);
  }

  return S_OK;
}
