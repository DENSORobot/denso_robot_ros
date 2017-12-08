/**
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
#include <process.h>
#include <winsock2.h>
#pragma comment(lib, "wsock32.lib")
#elif defined(_USE_LINUX_API)
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#else
#include "dn_additional.h"
#endif

#include "dn_common.h"
#include "dn_device.h"
#include "dn_udp.h"
#include "dn_com.h"
#include "dn_thread.h"
#include "dn_robotalk.h"
#include "dn_tpcomm.h"

#ifndef _DEBUG
#define _DEBUG (0)
#endif

/**
 * @def   _TP_CMD_SPECIAL
 * @brief A definition for the ROBOTalk command which means special command.
 */
#define _TP_CMD_SPECIAL     (0x0FFF)

/**
 * @def   _TIMER_INTERVAL
 * @brief A definition for the interval of the timer event
 * @note  times in milliseconds.
 */
#define _TIMER_INTERVAL     (10)

/**
 * @def   _TPERROR_TIMEOUT
 * @brief A definition for the timeout which is used to raise disconnect event.
 * @note  times in milliseconds.
 */
#define _TPERROR_TIMEOUT    (10000)

/**
 * @def   _TPLESS_INTERVAL
 * @brief A definition for the interval of the TP less event.
 * @note  times in milliseconds.
 */
#define _TPLESS_INTERVAL    (10000)

/**
 * @def   _PING_INTERVAL_CONNECT
 * @brief A definition for the sending interval of the ping packet when TP connected.
 * @note  times in milliseconds.
 */
#define _PING_INTERVAL_CONNECT   (300)

/**
 * @def   _PING_INTERVAL_TPERROR
 * @brief A definition for the sending interval of the ping packet when TP is error state.
 * @note  times in milliseconds.
 */
#define _PING_INTERVAL_TPERROR   (1000)

/**
 * @def   _TP_INIT_WAIT_CLIENT
 * @brief A definition for the wait time of the initialization when the client mode.
 * @note  times in milliseconds.
 */
#define _TP_INIT_WAIT_CLIENT   (100)

/**
 * @def   _TP_INIT_WAIT_SERVER
 * @brief A definition for the wait time of the initialization when the server mode.
 * @note  times in milliseconds.
 */
#define _TP_INIT_WAIT_SERVER   (3000)

/**
 * @struct TP_FLAGS
 * @brief  TP flag objects.
 */
struct TP_FLAGS
{
  uint8_t client :1;              /**< TP mode. (Client: = 1, Server: 0) */
  uint8_t init :1;                /**< Initialize flag.                  */
  uint8_t state :3;               /**< TP state.                         */
  uint8_t cts :1;                 /**< CTS signal.                       */
  volatile uint8_t timer_flag :1; /**< Alive flag for timer thread.      */
  volatile uint8_t recv_flag :1;  /**< Alive flag for receive thread.    */
};

/**
 * @struct CONN_PARAM_TP
 * @brief  TP communication object.
 */
struct CONN_PARAM_TP
{
  struct CONN_PARAM_COMMON device; /**< Common communication object.    */
  struct TP_FLAGS flags;           /**< TP flag objects                 */
  uint8_t from_id;                 /**< Sender ID.                      */
  uint8_t to_id;                   /**< Receiver ID.                    */
  uint32_t timer_clock;            /**< Time value for timer interval.  */
  uint32_t ping_clock;             /**< Time value for ping interval.   */
  uint32_t init_clock;             /**< Time value for initialize.      */
  uint32_t check_clock;            /**< Time value for check state.     */
  MUTEX mutex;                     /**< Mutex object.                   */
  EVENT evt;                       /**< Event object.                   */
  THREAD timer_thread;             /**< Timer thread handle.            */
  THREAD recv_thread;              /**< Receive thread handle.          */
  union RTK_PACKET last_packet;    /**< Last sent packet.               */
};

static struct CONN_PARAM_TP m_conn_param[TP_CONN_MAX];
static struct CALL_FUNC_TP m_call_func;

/**
 * @fn    int find_open_address()
 * @brief Returns the open address of m_conn_param.
 * @note  If there is no open space, then returns 0.
 */
static int
find_open_address()
{
  int i, index = -1;

  for (i = 0; i < TP_CONN_MAX; i++) {
    if (m_conn_param[i].device.sock == 0) {
      index = i;
      break;
    }
  }

  return (index + 1);
}

/**
 * @fn        struct CONN_PARAM_TP* check_address(int index)
 * @brief     Checks whether the index has been used or not.
 * @param[in] index The index of m_conn_param.
 * @note      If the index has not been used then returns NULL.
 */
static struct CONN_PARAM_TP*
check_address(int index)
{
  index--;

  if (index < 0 || TP_CONN_MAX <= index) {
    return NULL;
  }
  else if (m_conn_param[index].device.sock == 0) {
    return NULL;
  }

  return &m_conn_param[index];
}

/**
 * @fn        HRESULT tp_callfunc(uint16_t command, const uint8_t *data, uint8_t len_data)
 * @brief     Execute a callback function with the ROBOTalk command.
 * @param[in] command The received ROBOTalk command.
 * @param[in] data The received ROBOTalk data.
 * @param[in] len_data The length of received ROBOTalk data.
 */
static HRESULT
tp_callfunc(uint16_t command, const uint8_t *data, uint8_t len_data)
{
  void *param = NULL;

  /* If the function is not implemented, then returns E_NOTIMPL.  */
  HRESULT hr = E_NOTIMPL;

  /* Copies call back functions */
  struct CALL_FUNC_TP func = m_call_func;

  switch (command) {
    case _TP_CMD_SPECIAL: /* Changes TP state */
      if (len_data == sizeof(int)) {
        if (func.Call_TPState != NULL)
          hr = func.Call_TPState(*(int *) data);
      } else {
        hr = E_INVALIDARG;
      }
      break;

    case TP_CMD_REQ_ID: /* Received request ID command */
      hr = S_OK;
      break;

    case TP_CMD_KEYINFO: /* Received key information command */
      if (len_data == 8) {
        param = malloc(sizeof(struct TP_KEY_INFO));
        if (param != NULL) {
          /* Maps the received data to TP_KEY_INFO */
          memcpy_be(param, data, len_data);
          if (func.Call_TPKeyInfo != NULL)
            hr = func.Call_TPKeyInfo(*(struct TP_KEY_INFO *) param);
        } else {
          hr = E_OUTOFMEMORY;
        }
      } else {
        hr = E_INVALIDARG;
      }
      break;

    case TP_CMD_TOUTCHINFO: /* Received touch information command */
      if (len_data == 5) {
        param = malloc(sizeof(struct TP_TOUCH_INFO));
        if (param != NULL) {
          /* Maps the received data to TP_TOUCH_INFO */
          ((struct TP_TOUCH_INFO *) param)->mode = data[0];
          memcpy_be(&((struct TP_TOUCH_INFO *) param)->pos_x, &data[1], 2);
          ((struct TP_TOUCH_INFO *) param)->pos_x =
              ((struct TP_TOUCH_INFO *) param)->pos_x * 8 + 4;
          memcpy_be(&((struct TP_TOUCH_INFO *) param)->pos_y, &data[3], 2);
          ((struct TP_TOUCH_INFO *) param)->pos_y =
              ((struct TP_TOUCH_INFO *) param)->pos_y * 8 + 4;
          if (func.Call_TPTouchInfo != NULL)
            hr = func.Call_TPTouchInfo(*(struct TP_TOUCH_INFO *) param);
        } else {
          hr = E_OUTOFMEMORY;
        }
      } else {
        hr = E_INVALIDARG;
      }
      break;

#if (_DEBUG)
    default:
      if(func.Call_TPDefault != NULL) {
        hr = func.Call_TPDefault(command, data, len_data);
      }
      break;
#else
    default:
      break;
#endif
  }

  if (param != NULL) {
    free(param);
    param = NULL;
  }

  return hr;
}

/**
 * @fn        HRESULT tp_send(struct CONN_PARAM_TP *tp_param,
 * @fn        uint16_t command, uint8_t *data, uint8_t len_data)
 * @brief     Sends the ROBOTalk packet.
 * @param[in] tp_param TP communication object.
 * @param[in] command The sending ROBOTalk command.
 * @param[in] data The sending ROBOTalk data.
 * @param[in] len_data The length of sending ROBOTalk data.
 */
static HRESULT
tp_send(struct CONN_PARAM_TP *tp_param, uint16_t command, uint8_t *data,
    uint8_t len_data)
{
  struct CONN_PARAM_COMMON *device;
  union RTK_PACKET packet;
  HRESULT hr;

  device = &tp_param->device;

  hr = rtk_param2packet(command, data, len_data, tp_param->from_id,
      tp_param->to_id, &packet);
  if (SUCCEEDED(hr)) {
    hr = rtk_send(device, &packet);
    if (SUCCEEDED(hr)) {
      /* Keeps the last sent packet and updates the last send time. */
      tp_param->last_packet = packet;
      tp_param->ping_clock = gettimeofday_msec();
    }
  }

  return hr;
}

/**
 * @fn         HRESULT tp_recv(struct CONN_PARAM_TP *tp_param, unsigned int retry_nak,
 * @fn         uint16_t *command, uint8_t *data, uint8_t *len_data)
 * @brief      Receives the ROBOTalk packet.
 * @param[in]  tp_param TP communication object.
 * @param[in]  retry_nak The maximum retry count when received NAK packet.
 * @param[out] command The received ROBOTalk command.
 * @param[out] data The received ROBOTalk data.
 * @param[out] len_data The length of received ROBOTalk data.
 */
static HRESULT
tp_recv(struct CONN_PARAM_TP *tp_param, unsigned int retry_nak,
    uint16_t *command, uint8_t *data, uint8_t *len_data)
{
  unsigned int retry_cnt;
  uint32_t com_state;
  struct CONN_PARAM_COMMON *device;
  union RTK_PACKET packet;
  HRESULT hr = S_OK;

  device = &tp_param->device;

  for (retry_cnt = 0; retry_cnt <= retry_nak; retry_cnt++) {
    if (device->type == CONN_COM) {
      com_get_modem_state(device->sock, &com_state);
      /* If the CTS bit is off then changes TP state to TP_LESS */
      if (com_state & COM_BITS_CTS) { /* CTS bit is on  */
        /* If the previous CTS bit is off then clears temporary buffers. */
        if (!tp_param->flags.cts) {
          device->dn_clear(device->sock, device->timeout);
        }
        tp_param->flags.cts = 1;
      } else { /* CTS bit is off */
        tp_param->flags.cts = 0;
        hr = E_ACCESSDENIED;
        break;
      }
    }

    hr = rtk_recv(device, &packet, tp_param->flags.client, retry_nak);
    if (FAILED(hr))
      break;

    /* If received NAK packet then sends the last sent packet. */
    if (NativeCommand(packet.command) == RTK_CMD_NAK) {
      hr = rtk_send(device, &tp_param->last_packet);
      if (FAILED(hr)) {
        break;
      }
      if (retry_cnt == retry_nak) {
        hr = E_INVALIDPACKET;
      }
    } else {
      *command = packet.command;
      *len_data = packet.len;
      if (packet.len > 0) {
        memcpy(data, packet.data, packet.len);
      }

      /* Updates the receiver ID. */
      tp_param->to_id = packet.from_id;

      break;
    }
  }

  return hr;
}

/**
 * @fn         HRESULT send_receive(struct CONN_PARAM_TP *tp_param, unsigned int retry_timeout,
 * @fn         uint16_t command_send, uint8_t *data_send, uint8_t len_send,
 * @fn         uint16_t *command_recv, uint8_t *data_recv, uint8_t *len_recv)
 * @brief      Sends and receives the ROBOTalk packet.
 * @param[in]  index The index of m_conn_param.
 * @param[in]  retry_timeout The maximum retry count when receives timeout.
 * @param[in]  command_send The sending ROBOTalk command.
 * @param[in]  data_send The sending ROBOTalk data.
 * @param[in]  len_send The length of sending ROBOTalk data.
 * @param[out] command_recv The received ROBOTalk command.
 * @param[out] data_recv The received ROBOTalk data.
 * @param[out] len_recv The length of received ROBOTalk data.
 */
static HRESULT
send_receive(int index, unsigned int retry_timeout, uint16_t command_send,
    uint8_t *data_send, uint8_t len_send, uint16_t *command_recv,
    uint8_t *data_recv, uint8_t *len_recv)
{
  unsigned int retry_cnt;
  HRESULT hr;
  struct CONN_PARAM_TP *tp_param;

  tp_param = check_address(index);
  if (tp_param == NULL)
    return E_HANDLE;

  if (tp_param->flags.state == TP_TPLESS)
    return E_ACCESSDENIED;

  /* Locks mutex and must not returns this function without end of one. */
  hr = lock_mutex(&tp_param->mutex, INFINITE);
  if (FAILED(hr))
    return hr;

  for (retry_cnt = 0; retry_cnt <= retry_timeout; retry_cnt++) {
    /* If the timeout retry count is more than 0, then sets retry flag and counts. */
    if (retry_cnt > 0) {
      command_send |= RTK_RETRY_FLAG;
      command_send += RTK_RETRY_COUNT;
    }

    hr = tp_send(tp_param, command_send, data_send, len_send);
    if (FAILED(hr))
      break;

    hr = tp_recv(tp_param, TP_RETRY_NAK, command_recv, data_recv, len_recv);
    if (SUCCEEDED(hr) || hr != E_TIMEOUT) {
      if (SUCCEEDED(hr)) {
        switch (NativeCommand(*command_recv)) {
          case RTK_CMD_REJ:
            hr = E_FAIL;
            break;
          default:
            break;
        }
      }
      break;
    }
  }

  /* Unlocks mutex */
  unlock_mutex(&tp_param->mutex);

  return hr;
}

/**
 * @fn        HRESULT receive_execute(struct CONN_PARAM_TP *tp_param)
 * @brief     Receives the ROBOTalk packet and executes callback functions.
 * @param[in] tp_param TP communication object.
 */
static HRESULT
receive_execute(struct CONN_PARAM_TP *tp_param)
{
  int resp = 1, tmp_state = tp_param->flags.state;
  uint32_t cur, diff;
  union RTK_PACKET packet_send, packet_recv;
  HRESULT hr;

  /* Locks mutex and must not returns this function without end of one. */
  hr = lock_mutex(&tp_param->mutex, INFINITE);
  if (FAILED(hr))
    return hr;

  hr = tp_recv(tp_param, TP_RETRY_NAK, &packet_recv.command, packet_recv.data,
      &packet_recv.len);
  if (FAILED(hr)) {
    /* Updates temporary TP state. */
    if (hr == E_ACCESSDENIED) {
      tmp_state = TP_TPLESS;
    } else {
      tmp_state = TP_TPERROR;
    }
  } else {
    /* Executes callback functions. */
    switch (NativeCommand(packet_recv.command)) {
      /* If the received ROBOTalk command is not executable then do nothing. */
      case RTK_CMD_REJ:
      case RTK_CMD_ACK:
      case RTK_CMD_NAK:
        resp = 0;
        break;
      default:
        hr = tp_callfunc(NativeCommand(packet_recv.command), packet_recv.data,
            packet_recv.len);
        resp = 1;
        packet_send.command = (SUCCEEDED(hr) ? RTK_CMD_ACK : RTK_CMD_REJ);
        packet_send.len = 0;
        break;
    }

    /* Responds the result of callback functions */
    if (resp) {
      hr = tp_send(tp_param, packet_send.command, packet_send.data,
          packet_send.len);
      if (FAILED(hr))
        goto exit_proc;
    }

    if (tmp_state != TP_CONNECT) {
      /* If received the request ID command, then responds another command. */
      if (NativeCommand(packet_recv.command) == TP_CMD_REQ_ID) {
        hr = tp_send(tp_param, TP_CMD_PING, NULL, 0);
      } else {
        /* If received the another command, then changes temporary TP state. */
        tmp_state = TP_CONNECT;
      }
    }
  }

  /* Updates TP state */
  cur = gettimeofday_msec();
  diff = calc_time_diff(tp_param->check_clock, cur);

  packet_send.command = _TP_CMD_SPECIAL;
  packet_send.len = sizeof(int);

  switch (tp_param->flags.state) {
    case TP_CONNECT:
      switch (tmp_state) {
        case TP_CONNECT:
          tp_param->flags.state = tmp_state;
          tp_param->check_clock = cur;
          break;
        case TP_TPLESS:
          *(int *) packet_send.data = (int) TP_DISCONNECT;
          hr = tp_callfunc(packet_send.command, packet_send.data, packet_send.len);
          tp_param->flags.state = tmp_state;
          tp_param->check_clock = cur;
          break;
        case TP_TPERROR:
          if (diff > _TPERROR_TIMEOUT) {
            *(int *) packet_send.data = (int) TP_DISCONNECT;
            hr = tp_callfunc(packet_send.command, packet_send.data,
                packet_send.len);
            tp_param->flags.state = tmp_state;
            tp_param->check_clock = cur;
          }
          break;
        default:
          break;
      }
      break;
    case TP_TPLESS:
    case TP_TPERROR:
      switch (tmp_state) {
        case TP_CONNECT:
          *(int *) packet_send.data = (int) TP_CONNECT;
          hr = tp_callfunc(packet_send.command, packet_send.data, packet_send.len);
          tp_param->flags.state = tmp_state;
          tp_param->check_clock = cur;
          break;
        case TP_TPLESS:
        case TP_TPERROR:
          if (diff > _TPLESS_INTERVAL) {
            *(int *) packet_send.data = tmp_state;
            hr = tp_callfunc(packet_send.command, packet_send.data,
                packet_send.len);
            tp_param->flags.state = tmp_state;
            tp_param->check_clock = cur;
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

exit_proc:
  /* Unlocks mutex */
  unlock_mutex(&tp_param->mutex);

  return hr;
}

/**
 * @fn        HRESULT timer_event(struct CONN_PARAM_TP *tp_param)
 * @brief     Executes timer event procedure.
 * @param[in] tp_param TP communication object.
 */
static HRESULT
timer_event(struct CONN_PARAM_TP *tp_param)
{
  uint32_t cur, diff;
  HRESULT hr;

  /* Locks mutex and must not returns this function without end of one. */
  hr = lock_mutex(&tp_param->mutex, INFINITE);
  if (FAILED(hr))
    return hr;

  if (tp_param->flags.client) {
    tp_param->flags.init = 1;

    /* Sends request ID command until receiving ACK packet */
    cur = gettimeofday_msec();
    diff = calc_time_diff(tp_param->init_clock, cur);
    if ((diff > _TP_INIT_WAIT_CLIENT)
        && (tp_param->flags.state != TP_CONNECT))
    {

      hr = tp_send(tp_param, TP_CMD_REQ_ID, NULL, 0);
      if (SUCCEEDED(hr))
        hr = S_FALSE; /* Waits receiving thread */

      tp_param->init_clock = cur;
    }
  } else {
    if (!tp_param->flags.init) {
      /* Waits until connecting */
      cur = gettimeofday_msec();
      diff = calc_time_diff(tp_param->init_clock, cur);
      if ((diff > _TP_INIT_WAIT_SERVER)
          || (tp_param->flags.state == TP_CONNECT))
      {

        if (tp_param->flags.state != TP_CONNECT) {
          hr = tp_send(tp_param, TP_CMD_GET_KEYSTATE, NULL, 0);
        }

        tp_param->flags.init = 1;
      }
    } else {
      /* Sends ping packet */
      cur = gettimeofday_msec();
      diff = calc_time_diff(tp_param->ping_clock, cur);
      switch (tp_param->flags.state) {
        case TP_CONNECT:
          if (diff > _PING_INTERVAL_CONNECT) {
            hr = tp_send(tp_param, TP_CMD_PING, NULL, 0);
            if (SUCCEEDED(hr))
              hr = S_FALSE; /* Waits receiving thread */
          }
          break;
        case TP_TPERROR:
          if (diff > _PING_INTERVAL_TPERROR) {
            hr = tp_send(tp_param, TP_CMD_PING, NULL, 0);
            if (SUCCEEDED(hr))
              hr = S_FALSE; /* Waits receiving thread */
          }
          break;
        case TP_TPLESS:
          break;
        default:
          break;
      }
    }
  }

  /* Unlocks mutex */
  unlock_mutex(&tp_param->mutex);

  return hr;
}

/**
 * @fn   THRET recv_thread(void *arg)
 * @brief  The receiving thread.
 * @param[in] arg The argument of receiving thread: CONN_PARAM_TP.
 */
static THRET THTYPE
recv_thread(void *arg)
{
#if !defined(THRET)
  THRET ret = (THRET)NULL;
#endif

  uint32_t time_sleep;
  struct CONN_PARAM_TP *tp_param = (struct CONN_PARAM_TP *) arg;

  tp_param->check_clock = gettimeofday_msec();

  while (tp_param->flags.recv_flag) {
    time_sleep = (tp_param->flags.state == TP_CONNECT ? 0 : 300);
    dn_sleep(time_sleep);

    receive_execute(tp_param);

    set_event(&tp_param->evt);
    dn_sleep(0);
  }

  set_event(&tp_param->evt);

#if !defined(THRET)
  return ret;
#endif
}

/**
 * @fn   THRET timer_thread(void *arg)
 * @brief  The timer thread.
 * @param[in] arg The argument of timer thread: CONN_PARAM_TP.
 */
static THRET THTYPE
timer_thread(void *arg)
{
#if !defined(THRET)
  THRET ret = (THRET)NULL;
#endif

  uint32_t cur, diff;
  HRESULT hr;
  struct CONN_PARAM_TP *tp_param = (struct CONN_PARAM_TP *) arg;

  tp_param->init_clock = gettimeofday_msec();

  while (tp_param->flags.timer_flag) {
    /* Calculates time difference */
    cur = gettimeofday_msec();
    diff = calc_time_diff(tp_param->timer_clock, cur);

    /* Makes timer interval */
    if (_TIMER_INTERVAL > diff) {
      dn_sleep(_TIMER_INTERVAL - diff);
    }

    /* Sets last event time */
    tp_param->timer_clock = gettimeofday_msec();

    hr = timer_event(tp_param);

    /* Waits receiving thread */
    if (hr == S_FALSE) {
      reset_event(&tp_param->evt);
      wait_event(&tp_param->evt, INFINITE);
    }
  }

#if !defined(THRET)
  return ret;
#endif
}

HRESULT
TPComm_SetCallFunc(const struct CALL_FUNC_TP *func)
{
  if (func == NULL)
    return E_INVALIDARG;

  m_call_func = *func;

  return S_OK;
}

HRESULT
TPComm_Open(const char *connect, uint32_t timeout, int client, int *pfd)
{
  int index, *sock;
  HRESULT hr;
  void *conn_param;
  struct CONN_PARAM_ETH eth_param =
    { inet_addr("127.0.0.1"), htons(5007), htonl(INADDR_ANY), 0 };
  struct CONN_PARAM_COM com_param =
    { 1, 38400, NOPARITY, 8, ONESTOPBIT, 0 };
  struct CONN_PARAM_TP *tp_param;
  struct CONN_PARAM_COMMON *device;
  struct sockaddr_in *paddr;

  if (connect == NULL || pfd == NULL)
    return E_INVALIDARG;

  index = find_open_address();
  if (index == 0)
    return E_MAX_OBJECT;

  tp_param = &m_conn_param[index - 1];
  device = &tp_param->device;

  /* Initializes connection parameters */
  device->type = parse_conn_type(connect);
  switch (device->type) {
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
      device->dn_clear = &udp_clear;
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
      device->dn_clear = &com_clear;
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
    memset(tp_param, 0, sizeof(struct CONN_PARAM_TP));
    return hr;
  }

  /* Initializes mutex */
  hr = initialize_mutex(&tp_param->mutex);
  if (FAILED(hr)) {
    if (device->arg != NULL) {
      free(device->arg);
      device->arg = NULL;
    }
    return hr;
  }

  /* Initializes event */
  hr = create_event(&tp_param->evt, 0, 0);
  if (FAILED(hr)) {
    release_mutex(&tp_param->mutex);
    if (device->arg != NULL) {
      free(device->arg);
      device->arg = NULL;
    }
    memset(tp_param, 0, sizeof(struct CONN_PARAM_TP));
    return hr;
  }

  /* Opens connection */
  sock = &device->sock;
  hr = device->dn_open(conn_param, sock);
  if (FAILED(hr)) {
    release_mutex(&tp_param->mutex);
    destroy_event(&tp_param->evt);
    if (device->arg != NULL) {
      free(device->arg);
      device->arg = NULL;
    }
    memset(tp_param, 0, sizeof(struct CONN_PARAM_TP));
    return hr;
  }

  /* Sets timeout */
  hr = TPComm_SetTimeout(index, timeout);
  if (FAILED(hr)) {
    TPComm_Close(&index);
    return hr;
  }

  /* Sets parameters */
  tp_param->flags.client = (client ? 1 : 0);
  tp_param->flags.init = 0;
  tp_param->flags.state = TP_TPLESS;
  tp_param->flags.cts = 0;
  tp_param->from_id = (client ? TP_ID_CLIENT : TP_ID_SERVER);
  tp_param->to_id = TP_ID_CLIENT;

  /* Sets initialize time */
  tp_param->timer_clock = tp_param->ping_clock = tp_param->init_clock =
      tp_param->check_clock = gettimeofday_msec();

  /* Begins thread */
  tp_param->flags.timer_flag = 1;
  begin_thread(&tp_param->timer_thread, &timer_thread, tp_param);
  tp_param->flags.recv_flag = 1;
  begin_thread(&tp_param->recv_thread, &recv_thread, tp_param);

  *pfd = index;

  return S_OK;
}

HRESULT
TPComm_Close(int *pfd)
{
  int index, *sock;
  struct CONN_PARAM_TP *tp_param;
  struct CONN_PARAM_COMMON *device;

  if (pfd == NULL)
    return E_HANDLE;

  index = *pfd;

  tp_param = check_address(index);
  if (tp_param == NULL)
    return E_HANDLE;

  device = &tp_param->device;
  sock = &device->sock;

  /* Ends timer thread first */
  tp_param->flags.timer_flag = 0;
  exit_thread(tp_param->timer_thread);

  /* Ends receive thread second */
  tp_param->flags.recv_flag = 0;
  exit_thread(tp_param->recv_thread);

  /* Destroys event */
  destroy_event(&tp_param->evt);

  /* Releases mutex */
  release_mutex(&tp_param->mutex);

  /* Closes connection */
  device->dn_close(sock);

  /* Releases argument */
  if (device->arg != NULL) {
    free(device->arg);
    device->arg = NULL;
  }

  /* Resets connection parameters */
  memset(tp_param, 0, sizeof(struct CONN_PARAM_TP));

  *pfd = 0;

  return S_OK;
}

HRESULT
TPComm_SetTimeout(int fd, uint32_t timeout)
{
  int *sock;
  HRESULT hr;
  struct CONN_PARAM_TP *tp_param;
  struct CONN_PARAM_COMMON *device;

  tp_param = check_address(fd);
  if (tp_param == NULL)
    return E_HANDLE;

  device = &tp_param->device;
  sock = &device->sock;

  /* Locks mutex and must not returns this function without end of one. */
  hr = lock_mutex(&tp_param->mutex, INFINITE);
  if (FAILED(hr))
    return hr;

  hr = device->dn_set_timeout(*sock, timeout);
  if (SUCCEEDED(hr)) {
    device->timeout = timeout;
  }

  /* Unlocks mutex */
  unlock_mutex(&tp_param->mutex);

  return hr;
}

HRESULT
TPComm_GetTimeout(int fd, uint32_t *timeout)
{
  struct CONN_PARAM_TP *tp_param;
  struct CONN_PARAM_COMMON *device;

  tp_param = check_address(fd);
  if (tp_param == NULL)
    return E_HANDLE;

  if (timeout == NULL)
    return E_INVALIDARG;

  device = &tp_param->device;
  *timeout = device->timeout;

  return S_OK;
}

HRESULT
TPComm_GetTPState(int fd, int *state)
{
  struct CONN_PARAM_TP *tp_param;

  tp_param = check_address(fd);
  if (tp_param == NULL)
    return E_HANDLE;

  if (state == NULL)
    return E_INVALIDARG;

  *state = tp_param->flags.state;

  return S_OK;
}

HRESULT
TPComm_BEEP(int fd, int16_t time)
{
  uint8_t data[2];
  HRESULT hr;
  union RTK_PACKET packet;

  memcpy_be(data, &time, 2);

  hr = send_receive(fd, TP_RETRY_TIMEOUT, TP_CMD_BEEP, data, 2, &packet.command,
      packet.data, &packet.len);

  return hr;
}

HRESULT
TPComm_LED(int fd, int16_t number, int16_t state)
{
  uint8_t data[2];
  uint16_t command;
  HRESULT hr;
  union RTK_PACKET packet;

  switch (state) {
    case LED_OFF:
      command = TP_CMD_LED_OFF;
      break;
    case LED_ON:
      command = TP_CMD_LED_ON;
      break;
    case LED_FLASH:
      command = TP_CMD_LED_FLASH;
      break;
    default:
      return E_INVALIDARG;
  }

  memcpy_be(data, &number, 2);

  hr = send_receive(fd, TP_RETRY_TIMEOUT, command, data, 2, &packet.command,
      packet.data, &packet.len);

  return hr;
}

HRESULT
TPComm_LCD(int fd, int16_t contrast)
{
  uint8_t data[2];
  HRESULT hr;
  union RTK_PACKET packet;

  memcpy_be(data, &contrast, 2);

  hr = send_receive(fd, TP_RETRY_TIMEOUT, TP_CMD_LCD, data, 2, &packet.command,
      packet.data, &packet.len);

  return hr;
}

HRESULT
TPComm_DrawMiniTP(int fd, VARIANT commands)
{
  HRESULT hr;
  uint32_t ulPosCmd, ulNumCmd, ulNumPar, ulPosPar, ulPosSub, ulNumSub, ulLenStr,
      ulLenCmd;
  uint8_t chSub[4], data[248], len_data = 0;
  VARIANT *vntCmd, vntPar[2], vntSub[5];
  union RTK_PACKET packet;

  memset(vntPar, 0, sizeof(vntPar));
  memset(vntSub, 0, sizeof(vntSub));

  if (commands.vt != (VT_VARIANT | VT_ARRAY)) {
    return E_INVALIDARG;
  }

  ulNumCmd = (int32_t) commands.parray->rgsabound[0].cElements;

  SafeArrayAccessData(commands.parray, (void**) &vntCmd);
  for (ulPosCmd = 0; ulPosCmd < ulNumCmd; ulPosCmd++) {
    if (!(vntCmd[ulPosCmd].vt & VT_ARRAY)) {
      hr = E_INVALIDARG;
      goto exit_proc;
    }

    for (ulPosPar = 0; ulPosPar < 2; ulPosPar++) {
      VariantClear(&vntPar[ulPosPar]);
    }

    for (ulPosSub = 0; ulPosSub < 5; ulPosSub++) {
      VariantClear(&vntSub[ulPosSub]);
    }

    ulNumPar = ChangeVarType(vntCmd[ulPosCmd], VT_VARIANT, vntPar, 2);
    if (ulNumPar < 2) {
      hr = E_INVALIDARG;
      goto exit_proc;
    }

    hr = VariantChangeType(&vntPar[0], &vntPar[0], 0, VT_UI1);
    if (FAILED(hr))
      goto exit_proc;

    switch (vntPar[0].bVal) {
      case COLOR_FG:
      case COLOR_BG:
      case COLOR_FILL:
        if (248 < len_data + 3) {
          hr = E_OUTOFMEMORY;
          goto exit_proc;
        }

        hr = VariantChangeType(&vntPar[1], &vntPar[1], 0, VT_UI1);
        if (FAILED(hr))
          goto exit_proc;

        data[len_data] = vntPar[0].bVal;
        data[len_data + 1] = 0x1;
        data[len_data + 2] = vntPar[1].bVal;
        len_data += 3;

        break;
      case DRAW_STRING:
        ulNumSub = ChangeVarType(vntPar[1], VT_VARIANT, vntSub, 5);
        if (ulNumSub < 5) {
          hr = E_INVALIDARG;
          goto exit_proc;
        }

        for (ulPosSub = 0; ulPosSub < 5; ulPosSub++) {
          hr = VariantChangeType(&vntSub[ulPosSub], &vntSub[ulPosSub], 0,
              (ulPosSub == 0 ? VT_BSTR : VT_UI1));
          if (FAILED(hr))
            goto exit_proc;
        }

        ulLenStr = SysStringLen(vntSub[0].bstrVal);
        ulLenCmd = ulLenStr * 2 + 5;

        if (248 < (len_data + ulLenCmd + 2)) {
          hr = E_OUTOFMEMORY;
          goto exit_proc;
        }

        data[len_data] = vntPar[0].bVal;
        data[len_data + 1] = (uint8_t) ulLenCmd;
        data[len_data + 2] = vntSub[1].bVal;
        data[len_data + 3] = vntSub[2].bVal;
        data[len_data + 4] = vntSub[3].bVal;
        data[len_data + 5] = (uint8_t) ulLenStr;
        wcstombs((char*) &data[len_data + 6], vntSub[0].bstrVal, ulLenStr);
        data[len_data + ulLenStr + 6] = '\0';
        memset(&data[len_data + ulLenStr + 7], vntSub[4].bVal, ulLenStr);
        len_data += (uint8_t) (ulLenCmd + 2);

        break;
      case DRAW_LINE:
      case DRAW_RECT:
        ulNumSub = ChangeVarType(vntPar[1], VT_UI1, chSub, 4);
        if (ulNumSub < 4) {
          hr = E_INVALIDARG;
          goto exit_proc;
        }

        if (248 < len_data + 6) {
          hr = E_OUTOFMEMORY;
          goto exit_proc;
        }

        data[len_data] = vntPar[0].bVal;
        data[len_data + 1] = 0x04;
        memcpy(&data[len_data + 2], chSub, 4);
        len_data += 6;

        break;
      default:
        hr = E_INVALIDARG;
        goto exit_proc;
    }
  }

  if (SUCCEEDED(hr)) {
    hr = send_receive(fd, TP_RETRY_TIMEOUT, TP_CMD_MTP_DRAW, data, len_data,
        &packet.command, packet.data, &packet.len);
  }

exit_proc:
  SafeArrayUnaccessData(commands.parray);

  for (ulPosPar = 0; ulPosPar < 2; ulPosPar++) {
    VariantClear(&vntPar[ulPosPar]);
  }

  for (ulPosSub = 0; ulPosSub < 5; ulPosSub++) {
    VariantClear(&vntSub[ulPosSub]);
  }

  return hr;
}

HRESULT
TPComm_DrawString(int fd, BSTR bstr, uint8_t pos_x, uint8_t pos_y, uint8_t size,
    uint8_t attr, uint8_t color_fg, uint8_t color_bg)
{
  HRESULT hr;
  uint8_t *bData;
  VARIANT vntCmd, *vntPar, *vntSub1, *vntSub2;

  VariantInit(&vntCmd);

  vntCmd.vt = VT_VARIANT | VT_ARRAY;
  vntCmd.parray = SafeArrayCreateVector(VT_VARIANT, 0, 3);
  SafeArrayAccessData(vntCmd.parray, (void **) &vntPar);

  // Sets COLOR_FG
  vntPar[0].vt = VT_UI1 | VT_ARRAY;
  vntPar[0].parray = SafeArrayCreateVector(VT_UI1, 0, 2);
  SafeArrayAccessData(vntPar[0].parray, (void **) &bData);
  bData[0] = COLOR_FG;
  bData[1] = color_fg;
  SafeArrayUnaccessData(vntPar[0].parray);

  // Sets COLOR_BG
  vntPar[1].vt = VT_UI1 | VT_ARRAY;
  vntPar[1].parray = SafeArrayCreateVector(VT_UI1, 0, 2);
  SafeArrayAccessData(vntPar[1].parray, (void **) &bData);
  bData[0] = COLOR_BG;
  bData[1] = color_bg;
  SafeArrayUnaccessData(vntPar[1].parray);

  // Sets DRAW_STRING
  vntPar[2].vt = VT_VARIANT | VT_ARRAY;
  vntPar[2].parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
  SafeArrayAccessData(vntPar[2].parray, (void **) &vntSub1);
  vntSub1[0].vt = VT_UI1;
  vntSub1[0].bVal = DRAW_STRING;
  vntSub1[1].vt = VT_VARIANT | VT_ARRAY;
  vntSub1[1].parray = SafeArrayCreateVector(VT_VARIANT, 0, 5);
  SafeArrayAccessData(vntSub1[1].parray, (void **) &vntSub2);
  vntSub2[0].vt = VT_BSTR;
  vntSub2[0].bstrVal = SysAllocString(bstr);
  vntSub2[1].vt = VT_UI1;
  vntSub2[1].bVal = pos_x;
  vntSub2[2].vt = VT_UI1;
  vntSub2[2].bVal = pos_y;
  vntSub2[3].vt = VT_UI1;
  vntSub2[3].bVal = size;
  vntSub2[4].vt = VT_UI1;
  vntSub2[4].bVal = attr;
  SafeArrayUnaccessData(vntSub1[1].parray);
  SafeArrayUnaccessData(vntPar[2].parray);

  SafeArrayUnaccessData(vntCmd.parray);

  hr = TPComm_DrawMiniTP(fd, vntCmd);

  VariantClear(&vntCmd);

  return hr;
}

HRESULT
TPComm_DrawLine(int fd, uint8_t start_x, uint8_t start_y, uint8_t end_x,
    uint8_t end_y, uint8_t color_fg)
{
  HRESULT hr;
  uint8_t *bData;
  VARIANT vntCmd, *vntPar, *vntSub1;

  VariantInit(&vntCmd);

  vntCmd.vt = VT_VARIANT | VT_ARRAY;
  vntCmd.parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
  SafeArrayAccessData(vntCmd.parray, (void **) &vntPar);

  // Sets COLOR_FG
  vntPar[0].vt = VT_UI1 | VT_ARRAY;
  vntPar[0].parray = SafeArrayCreateVector(VT_UI1, 0, 2);
  SafeArrayAccessData(vntPar[0].parray, (void **) &bData);
  bData[0] = COLOR_FG;
  bData[1] = color_fg;
  SafeArrayUnaccessData(vntPar[0].parray);

  // Sets DRAW_LINE
  vntPar[1].vt = VT_VARIANT | VT_ARRAY;
  vntPar[1].parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
  SafeArrayAccessData(vntPar[1].parray, (void **) &vntSub1);
  vntSub1[0].vt = VT_UI1;
  vntSub1[0].bVal = DRAW_LINE;
  vntSub1[1].vt = VT_UI1 | VT_ARRAY;
  vntSub1[1].parray = SafeArrayCreateVector(VT_UI1, 0, 4);
  SafeArrayAccessData(vntSub1[1].parray, (void **) &bData);
  bData[0] = start_x;
  bData[1] = start_y;
  bData[2] = end_x;
  bData[3] = end_y;
  SafeArrayUnaccessData(vntSub1[1].parray);
  SafeArrayUnaccessData(vntPar[1].parray);

  SafeArrayUnaccessData(vntCmd.parray);

  hr = TPComm_DrawMiniTP(fd, vntCmd);

  VariantClear(&vntCmd);

  return hr;
}

HRESULT
TPComm_DrawRectangle(int fd, uint8_t start_x, uint8_t start_y, uint8_t end_x,
    uint8_t end_y, uint8_t color_fg, uint8_t color_bg)
{
  HRESULT hr;
  uint8_t *bData;
  VARIANT vntCmd, *vntPar, *vntSub1;

  VariantInit(&vntCmd);

  vntCmd.vt = VT_VARIANT | VT_ARRAY;
  vntCmd.parray = SafeArrayCreateVector(VT_VARIANT, 0, 3);
  SafeArrayAccessData(vntCmd.parray, (void **) &vntPar);

  // Sets COLOR_FG
  vntPar[0].vt = VT_UI1 | VT_ARRAY;
  vntPar[0].parray = SafeArrayCreateVector(VT_UI1, 0, 2);
  SafeArrayAccessData(vntPar[0].parray, (void **) &bData);
  bData[0] = COLOR_FG;
  bData[1] = color_fg;
  SafeArrayUnaccessData(vntPar[0].parray);

  // Sets COLOR_BG
  vntPar[1].vt = VT_UI1 | VT_ARRAY;
  vntPar[1].parray = SafeArrayCreateVector(VT_UI1, 0, 2);
  SafeArrayAccessData(vntPar[1].parray, (void **) &bData);
  bData[0] = COLOR_BG;
  bData[1] = color_bg;
  SafeArrayUnaccessData(vntPar[1].parray);

  // Sets DRAW_RECT
  vntPar[2].vt = VT_VARIANT | VT_ARRAY;
  vntPar[2].parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
  SafeArrayAccessData(vntPar[2].parray, (void **) &vntSub1);
  vntSub1[0].vt = VT_UI1;
  vntSub1[0].bVal = DRAW_RECT;
  vntSub1[1].vt = VT_UI1 | VT_ARRAY;
  vntSub1[1].parray = SafeArrayCreateVector(VT_UI1, 0, 4);
  SafeArrayAccessData(vntSub1[1].parray, (void **) &bData);
  bData[0] = start_x;
  bData[1] = start_y;
  bData[2] = end_x;
  bData[3] = end_y;
  SafeArrayUnaccessData(vntSub1[1].parray);
  SafeArrayUnaccessData(vntPar[2].parray);

  SafeArrayUnaccessData(vntCmd.parray);

  hr = TPComm_DrawMiniTP(fd, vntCmd);

  VariantClear(&vntCmd);

  return hr;
}
