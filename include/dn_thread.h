#ifndef DN_THREAD_H_
#define DN_THREAD_H_

/**
 * @file    dn_thread.h
 * @brief   Thread and mutex API file.
 * @details Defines thread and mutex APIs.
 *
 * @version 1.2
 * @date    2014/11/06
 * @date    2014/12/19 Adds wait_event_multi function.
 * @date    2015/11/10 Bug fix. Solve the deadlock on wait_event in Linux.
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

#ifndef _DN_EXP_THREAD
#define _DN_EXP_THREAD
#endif /* _DN_EXP_THREAD */

/**
 * @def       begin_thread(p_thread, function, arg)
 * @brief     A macro that begins thread.
 * @param[in] p_thread The pointer of thread handle: THREAD.
 * @param[in] function The thread function.
 * @param[in] arg The argument of thread function.
 */

/**
 * @def       exit_thread(thread)
 * @brief     A macro that ends thread.
 * @param[in] thread The thread handle: THREAD.
 */

/**
 * @def       dn_sleep(n)
 * @brief     A macro that sleeps.
 * @param[in] n The sleep time[ms].
 */

#if defined(_USE_WIN_API)
#define begin_thread(p_thread, function, arg) { *p_thread = (HANDLE)_beginthreadex(NULL, 0, function, arg, 0, NULL); }
#define exit_thread(thread) { if(thread != NULL) { WaitForSingleObject(thread, INFINITE); CloseHandle(thread); thread = NULL; } }
#define dn_sleep(n) Sleep(n)
typedef HANDLE MUTEX;
typedef HANDLE THREAD;
typedef unsigned int THRET;
#define THTYPE __stdcall
typedef HANDLE EVENT;
#elif defined(_USE_LINUX_API)
#define begin_thread(p_thread, function, arg) pthread_create(p_thread, NULL, function, arg)
#define exit_thread(thread) pthread_join(thread, NULL)
#define dn_sleep(n) usleep((n) * 1000)
typedef pthread_mutex_t MUTEX;
typedef pthread_t THREAD;
typedef void *THRET;
#define THTYPE
#define STATUS_WAIT_0 ((uint32_t)0x00000000L)
#define WAIT_OBJECT_0 ((STATUS_WAIT_0) + 0)
typedef struct EVENT
  {
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int mode;
    int signal;
  }EVENT;
#endif

/**
 * @def   INFINITE
 * @brief A definition for infinite wait.
 */
#ifndef INFINITE
#define INFINITE ((uint32_t)-1)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn            HRESULT initialize_mutex(MUTEX *pmutex)
   * @brief         Initializes mutex handle.
   * @param[in,out] pmutex The pointer of mutex handle to be initialized.
   */
  _DN_EXP_THREAD HRESULT
  initialize_mutex(MUTEX *pmutex);

  /**
   * @fn            HRESULT release_mutex(MUTEX *pmutex)
   * @brief         Releases mutex handle.
   * @param[in,out] pmutex The pointer of mutex handle to be released.
   * @note          Must not be reused until it is reinitialized
   */
  _DN_EXP_THREAD HRESULT
  release_mutex(MUTEX *pmutex);

  /**
   * @fn            HRESULT lock_mutex(MUTEX *pmutex, uint32_t timeout)
   * @brief         Locks mutex handle.
   * @param[in,out] pmutex The pointer of mutex handle to be locked.
   * @param[in]     timeout Timeout value.
   */
  _DN_EXP_THREAD HRESULT
  lock_mutex(MUTEX *pmutex, uint32_t timeout);

  /**
   * @fn            HRESULT unlock_mutex(MUTEX *pmutex)
   * @brief         Unlocks mutex handle.
   * @param[in,out] pmutex The pointer of mutex handle to be unlocked.
   */
  _DN_EXP_THREAD HRESULT
  unlock_mutex(MUTEX *pmutex);

  /**
   * @fn            HRESULT create_event(EVENT *pevt, int reset_mode, int init_signal)
   * @brief         Creates a event object.
   * @param[in,out] pevt The pointer of event object to be created.
   * @param[in]     reset_mode If (reset_mode != 0) then the event object is set manual reset mode, else auto reset mode.
   * @param[in]     init_signal If (init_signal != 0) then the event object is signaled at first, else non signaled.
   */
  _DN_EXP_THREAD HRESULT
  create_event(EVENT *pevt, int reset_mode, int init_signal);

  /**
   * @fn            HRESULT destroy_event(EVENT *pevt)
   * @brief         Destroys a event object.
   * @param[in,out] pevt The pointer of event object to be destroyed.
   * @note          Must not be reused until it is recreated.
   */
  _DN_EXP_THREAD HRESULT
  destroy_event(EVENT *pevt);

  /**
   * @fn            HRESULT set_event(EVENT *pevt)
   * @brief         Sets a event.
   * @param[in,out] pevt The pointer of event object to be set.
   */
  _DN_EXP_THREAD HRESULT
  set_event(EVENT *pevt);

  /**
   * @fn            HRESULT reset_event(EVENT *pevt)
   * @brief         Resets a event.
   * @param[in,out] pevt The pointer of event object to be reset.
   */
  _DN_EXP_THREAD HRESULT
  reset_event(EVENT *pevt);

  /**
   * @fn            HRESULT wait_event(EVENT *pevt, uint32_t timeout)
   * @brief         Waits a event.
   * @param[in,out] pevt The pointer of event object to be waited.
   * @param[in]     timeout Timeout value.
   */
  _DN_EXP_THREAD HRESULT
  wait_event(EVENT *pevt, uint32_t timeout);

  /**
   * @fn            HRESULT wait_event_multi(EVENT **pevt, uint32_t count, uint32_t timeout, int wait_all)
   * @brief         Waits multiple events.
   * @param[in,out] pevt The pointer of event objects to be waited.
   * @param[in]     count The number of event objects.
   * @param[in]     timeout Timeout value.
   * @param[in]     wait_all If (wait_all != 0) then waits until all event objects are signaled, else waits until a event object is signaled.
   */
  _DN_EXP_THREAD HRESULT
  wait_event_multi(EVENT **pevt, uint32_t count, uint32_t timeout,
      int wait_all);

  /**
   * @def       calc_time_diff(start, end)
   * @brief     A macro that calculates the time difference between start and end.
   * @param[in] start The start time [ms].
   * @param[in] end The end time [ms].
   */
#define calc_time_diff(start, end) (((end) >= (start)) ? ((end) - (start)) : ((((uint32_t)-1) - (end)) + (start)))

  /**
   * @fn    uint32_t gettimeofday_msec()
   * @brief Gets the current time value [ms].
   */
  _DN_EXP_THREAD uint32_t
  gettimeofday_msec();

#ifdef __cplusplus
}
#endif

#endif /* DN_THREAD_H_ */
