#ifndef DN_ADDITIONAL_H_
#define DN_ADDITIONAL_H_

/**
 * @file    dn_additional.h
 * @brief   User own API file.
 * @details You can define some additional APIs.
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

#define INADDR_ANY  ((uint32_t)0x00000000)
#define INADDR_NONE (0xffffffff)

#define SOCKET uint16_t

#ifndef FD_SETSIZE
#define FD_SETSIZE (64)
#endif /* FD_SETSIZE */

typedef struct fd_set
{
  uint16_t fd_count;           /* how many are SET?   */
  SOCKET fd_array[FD_SETSIZE]; /* an array of SOCKETs */
} fd_set;

#define FD_SET(fd, fdsetp)
#define FD_ZERO(fdsetp)

struct timeval
{
  long tv_sec;  /* seconds          */
  long tv_usec; /* and microseconds */
};

#define DNGetLastError() (-1)
#define OSSUCCEEDED(ret) ((ret) ==  0)
#define OSFAILED(ret)    ((ret) == -1)

#define _E_BUSY_PROC (-1)

#define AF_INET      (2) /* internetwork: UDP, TCP, etc. */
#define SOCK_STREAM  (1) /* stream socket                */
#define SOCK_DGRAM   (2) /* datagram socket              */

#define SOL_SOCKET   (0xffff) /* options for socket level  */
#define SO_REUSEADDR (0x0004) /* allow local address reuse */
#define SO_KEEPALIVE (0x0008) /* keep connections alive    */
#define SO_SNDTIMEO  (0x1005) /* send timeout              */
#define SO_RCVTIMEO  (0x1006) /* receive timeout           */

#define IPPROTO_TCP  (6) /* tcp */
#define TCP_NODELAY  (0x0001)

#define MSG_PEEK     (0x02)

#define F_GETFL      (3) /* Get file status flags.  */
#define F_SETFL      (4) /* Set file status flags.  */
#define O_NONBLOCK   (04000)

typedef struct in_addr
{
  union
  {
    struct
    {
      uint8_t s_b1, s_b2, s_b3, s_b4;
    } S_un_b;
    struct
    {
      uint16_t s_w1, s_w2;
    } S_un_w;
    uint32_t S_addr;
  } S_un;

#define s_addr  S_un.S_addr      /* can be used for most tcp & ip code */
#define s_host  S_un.S_un_b.s_b2 // host on imp
#define s_net   S_un.S_un_b.s_b1 // network
#define s_imp   S_un.S_un_w.s_w2 // imp
#define s_impno S_un.S_un_b.s_b4 // imp #
#define s_lh    S_un.S_un_b.s_b3 // logical host
} IN_ADDR, *PIN_ADDR, *LPIN_ADDR;

struct sockaddr_in
{
  short sin_family;
  uint16_t sin_port;
  struct in_addr sin_addr;
  char sin_zero[8];
};

struct sockaddr
{
  uint16_t sa_family; /* address family                   */
  char sa_data[14];   /* up to 14 bytes of direct address */
};

typedef int socklen_t;

#define COM_BITS_CTS (0)
typedef int COM_STATE;

#define begin_thread(p_thread, function, arg)
#define exit_thread(thread)
#define dn_sleep(n)
typedef int MUTEX;
typedef int THREAD;
#define THRET void
#define THTYPE
#define STATUS_WAIT_0 ((uint32_t)0x00000000L)
#define WAIT_OBJECT_0 ((STATUS_WAIT_0) + 0)
typedef int EVENT;

#ifdef __cplusplus
extern "C"
{
#endif

  static uint16_t
  htons(uint16_t hostshort)
  {
    return 0;
  }

  static uint32_t
  htonl(uint32_t hostlong)
  {
    return 0;
  }

  static uint32_t
  inet_addr(const char *addr)
  {
    return 0;
  }

  static int
  select(int __nfds, fd_set *__restrict __readfds,
      fd_set *__restrict __writefds, fd_set *__restrict __exceptfds,
      struct timeval *__restrict __timeout)
  {
    return -1;
  }

  static int
  socket(int af, int type, int protocol)
  {
    return -1;
  }

  static int
  bind(SOCKET s, const struct sockaddr *name, int namelen)
  {
    return -1;
  }

  static int
  setsockopt(SOCKET s, int level, int optname, const char *optval, int optlen)
  {
    return -1;
  }

  static int
  connect(SOCKET s, const struct sockaddr *name, int namelen)
  {
    return -1;
  }

  static int
  listen(SOCKET s, int backlog)
  {
    return -1;
  }

  static SOCKET
  accept(SOCKET s, struct sockaddr *addr, int *addrlen)
  {
    return -1;
  }

  static int
  send(SOCKET s, const char *buf, int len, int flags)
  {
    return -1;
  }

  static int
  recv(SOCKET s, char *buf, int len, int flags)
  {
    return -1;
  }

  static int32_t
  _tcp_set_keepalive(int sock, int enable, uint32_t idle, uint32_t interval,
      uint32_t count)
  {
    return -1;
  }

  static int
  sendto(SOCKET s, const char *buf, int len, int flags,
      const struct sockaddr *to, int tolen)
  {
    return -1;
  }

  static int
  recvfrom(SOCKET s, char *buf, int len, int flags, struct sockaddr *from,
      int *fromlen)
  {
    return -1;
  }

  static int
  _socket_close(int sock)
  {
    return -1;
  }

  static int32_t
  _socket_bind(const void *param, int *sock)
  {
    return -1;
  }

  static int32_t
  _com_open(const void *com_param, int *sock)
  {
    return -1;
  }

  static int
  _com_close(int sock)
  {
    return -1;
  }

  static int
  _com_send(int sock, const char *buf, uint32_t len_send, uint32_t *len_sended,
      void *arg)
  {
    return -1;
  }

  static int
  _com_recv(int sock, char *buf, uint32_t len_recv, uint32_t *len_recved,
      uint32_t timeout, void *arg)
  {
    return -1;
  }

  static int32_t
  _com_set_timeout(int sock, uint32_t timeout)
  {
    return -1;
  }

  static int32_t
  _com_clear(int sock, uint32_t timeout)
  {
    return -1;
  }

  static int
  _com_get_state(int sock, COM_STATE *state)
  {
    return -1;
  }

  static int
  _com_set_state(int sock, COM_STATE *state)
  {
    return -1;
  }

  static int
  _com_get_modem_state(int sock, uint32_t *state)
  {
    return -1;
  }

  static int32_t
  _initialize_mutex(MUTEX *pmutex)
  {
    return -1;
  }

  static int32_t
  _release_mutex(MUTEX *pmutex)
  {
    return -1;
  }

  static int32_t
  _lock_mutex(MUTEX *pmutex, uint32_t timeout)
  {
    return -1;
  }

  static int32_t
  _unlock_mutex(MUTEX *pmutex)
  {
    return -1;
  }

  static int32_t
  _create_event(EVENT *pevt, int reset_mode, int init_signal)
  {
    return -1;
  }

  static int32_t
  _destroy_event(EVENT *pevt)
  {
    return -1;
  }

  static int32_t
  _set_event(EVENT *pevt)
  {
    return -1;
  }

  static int32_t
  _reset_event(EVENT *pevt)
  {
    return -1;
  }

  static int32_t
  _wait_event(EVENT *pevt, uint32_t timeout)
  {
    return -1;
  }

  static int32_t
  _wait_event_multi(EVENT **pevt, uint32_t count, uint32_t timeout,
      int wait_all)
  {
    return -1;
  }

  static uint32_t
  _gettimeofday_msec()
  {
    return 0;
  }


#ifdef __cplusplus
}
#endif

#endif /* DN_ADDITIONAL_H_ */
