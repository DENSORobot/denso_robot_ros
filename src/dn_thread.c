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

#if defined(_USE_WIN_API)
#include <process.h>
#include <windows.h>
#elif defined(_USE_LINUX_API)
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#else
#include "dn_additional.h"
#endif

#include "dn_common.h"
#include "dn_thread.h"

#if defined(_USE_LINUX_API)
/**
 * @fn    HRESULT create_timeout(struct timespec *spec, uint32_t timeout)
 * @brief   Creates timeout value.
 * @param[out]  spec Timeout value to be created.
 * @param[in]  timeout Timeout value to be set. times in milliseconds.
 */
static HRESULT create_timeout(struct timespec *spec, uint32_t timeout)
{
  uint32_t tmp_usec;
  struct timeval cur;

  /* Gets current time */
  gettimeofday(&cur, NULL);

  /* Sets timeout seconds */
  spec->tv_sec = cur.tv_sec + (timeout / 1000);

  /* Temporary calcurates timeout microseconds */
  tmp_usec = cur.tv_usec + ((timeout % 1000) * 1000);

  /* Adds timeout seconds */
  spec->tv_sec += (tmp_usec / 1000000);

  /* Sets timeout nanoseconds */
  spec->tv_nsec = (tmp_usec % 1000000) * 1000;

  return S_OK;
}
#endif

/**
 * @fn            HRESULT initialize_mutex(MUTEX *pmutex)
 * @brief         Initializes mutex handle.
 * @param[in,out] pmutex The pointer of mutex handle to be initialized.
 */
HRESULT
initialize_mutex(MUTEX *pmutex)
{
  HRESULT hr = S_OK;

  if (pmutex == NULL)
    return E_INVALIDARG;

#if defined(_USE_WIN_API)
  *pmutex = CreateMutex(NULL, FALSE, NULL);
  if(*pmutex == NULL) {
    hr = E_UNEXPECTED;
  }
#elif defined(_USE_LINUX_API)
  pthread_mutex_init(pmutex, NULL);
#else
  hr = _initialize_mutex(pmutex);
#endif

  return hr;
}

/**
 * @fn            HRESULT release_mutex(MUTEX *pmutex)
 * @brief         Releases mutex handle.
 * @param[in,out] pmutex The pointer of mutex handle to be released.
 * @note          Must not be reused until it is reinitialized
 */
HRESULT
release_mutex(MUTEX *pmutex)
{
  HRESULT hr = S_OK;

  if (pmutex == NULL)
    return E_INVALIDARG;

#if defined(_USE_WIN_API)
  if(*pmutex != NULL) {
    BOOL ret;
    ret = CloseHandle(*pmutex);
    if(ret != 0) {
      *pmutex = NULL;
    } else {
      hr = E_ACCESSDENIED;
    }
  }
#elif defined(_USE_LINUX_API)
  int ret;
  ret = pthread_mutex_destroy(pmutex);
  if(ret != 0) {
    hr = E_ACCESSDENIED;
  }
#else
  hr = _release_mutex(pmutex);
#endif

  return hr;
}

/**
 * @fn            HRESULT lock_mutex(MUTEX *pmutex, uint32_t timeout)
 * @brief         Locks mutex handle.
 * @param[in,out] pmutex The pointer of mutex handle to be locked.
 * @param[in]     timeout Timeout value.
 */
HRESULT
lock_mutex(MUTEX *pmutex, uint32_t timeout)
{
  HRESULT hr = E_INVALIDARG;

  if (pmutex != NULL) {
#if defined(_USE_WIN_API)
    if(*pmutex != NULL) {
      DWORD dwRet = WaitForSingleObject(*pmutex, timeout);
      if(dwRet == WAIT_OBJECT_0) {
        hr = S_OK;
      } else {
        hr = E_TIMEOUT;
      }
    }
#elif defined(_USE_LINUX_API)
    int ret;
    if(timeout == INFINITE) {
      ret = pthread_mutex_lock(pmutex);
    } else {
      struct timespec spec;
      create_timeout(&spec, timeout);
      ret = pthread_mutex_timedlock(pmutex, &spec);
    }
    if(ret == 0) {
      hr = S_OK;
    }
    else if(ret == ETIMEDOUT) {
      hr = E_TIMEOUT;
    }
#else
    hr = _lock_mutex(pmutex, timeout);
#endif
  }

  return hr;
}

/**
 * @fn            HRESULT unlock_mutex(MUTEX *pmutex)
 * @brief         Unlocks mutex handle.
 * @param[in,out] pmutex The pointer of mutex handle to be unlocked.
 */
HRESULT
unlock_mutex(MUTEX *pmutex)
{
  HRESULT hr = E_INVALIDARG;

  if (pmutex != NULL) {
#if defined(_USE_WIN_API)
    if(*pmutex != NULL) {
      ReleaseMutex(*pmutex);
      hr = S_OK;
    }
#elif defined(_USE_LINUX_API)
    int ret;
    ret = pthread_mutex_unlock(pmutex);
    if(ret == 0) {
      hr = S_OK;
    }
#else
    hr = _unlock_mutex(pmutex);
#endif
  }

  return hr;
}

/**
 * @fn            HRESULT create_event(EVENT *pevt, int reset_mode, int init_signal)
 * @brief         Creates a event object.
 * @param[in,out] pevt The pointer of event object to be created.
 * @param[in]     reset_mode If (reset_mode != 0) then the event object is set manual reset mode, else auto reset mode.
 * @param[in]     init_signal If (init_signal != 0) then the event object is signaled at first, else non signaled.
 */
HRESULT
create_event(EVENT *pevt, int reset_mode, int init_signal)
{
  HRESULT hr = S_OK;

  if (pevt == NULL)
    return E_INVALIDARG;

#if defined(_USE_WIN_API)
  *pevt = CreateEvent(NULL, reset_mode, init_signal, NULL);
  if(*pevt == NULL) {
    hr = E_UNEXPECTED;
  }
#elif defined(_USE_LINUX_API)
  pthread_mutex_init(&pevt->mutex, NULL);
  pthread_cond_init(&pevt->cond, NULL);
  pevt->mode = (reset_mode != 0);
  pevt->signal = (init_signal != 0);
#else
  hr = _create_event(pevt, reset_mode, init_signal);
#endif

  return hr;
}

/**
 * @fn            HRESULT destroy_event(EVENT *pevt)
 * @brief         Destroys a event object.
 * @param[in,out] pevt The pointer of event object to be destroyed.
 * @note          Must not be reused until it is recreated.
 */
HRESULT
destroy_event(EVENT *pevt)
{
  HRESULT hr = S_OK;

  if (pevt == NULL)
    return E_INVALIDARG;

#if defined(_USE_WIN_API)
  if(*pevt != NULL) {
      BOOL ret;
      ret = CloseHandle(*pevt);
      if(ret != 0) {
        *pevt = NULL;
      } else {
        hr = E_ACCESSDENIED;
      }
    }
#elif defined(_USE_LINUX_API)
  int ret;
  ret = pthread_mutex_destroy(&pevt->mutex);
  if(ret != 0) {
    return E_ACCESSDENIED;
  }

  ret = pthread_cond_destroy(&pevt->cond);
  if(ret != 0) {
    return E_ACCESSDENIED;
  }

  pevt->mode = 0;
  pevt->signal = 0;
#else
  hr = _destroy_event(pevt);
#endif

  return hr;
}

/**
 * @fn            HRESULT set_event(EVENT *pevt)
 * @brief         Sets a event.
 * @param[in,out] pevt The pointer of event object to be set.
 */
HRESULT
set_event(EVENT *pevt)
{
  HRESULT hr = E_INVALIDARG;

  if (pevt != NULL) {
#if defined(_USE_WIN_API)
    if(*pevt != NULL) {
      SetEvent(*pevt);
      hr = S_OK;
    }
#elif defined(_USE_LINUX_API)
    int ret;
    ret = pthread_mutex_lock(&pevt->mutex);
    if(ret == 0) {
      pthread_cond_broadcast(&pevt->cond);
      pevt->signal = 1;
      pthread_mutex_unlock(&pevt->mutex);
      hr = S_OK;
    }
#else
    hr = _set_event(pevt);
#endif
  }

  return hr;
}

/**
 * @fn            HRESULT reset_event(EVENT *pevt)
 * @brief         Resets a event.
 * @param[in,out] pevt The pointer of event object to be reset.
 */
HRESULT
reset_event(EVENT *pevt)
{
  HRESULT hr = E_INVALIDARG;

  if (pevt != NULL) {
#if defined(_USE_WIN_API)
    if(*pevt != NULL) {
      ResetEvent(*pevt);
      hr = S_OK;
    }
#elif defined(_USE_LINUX_API)
    int ret;
    ret = pthread_mutex_lock(&pevt->mutex);
    if(ret == 0) {
      pevt->signal = 0;
      pthread_mutex_unlock(&pevt->mutex);
      hr = S_OK;
    }
#else
    hr = _reset_event(pevt);
#endif
  }

  return hr;
}

/**
 * @fn            HRESULT wait_event(EVENT *pevt, uint32_t timeout)
 * @brief         Waits a event.
 * @param[in,out] pevt The pointer of event object to be waited.
 * @param[in]     timeout Timeout value.
 */
HRESULT
wait_event(EVENT *pevt, uint32_t timeout)
{
  HRESULT hr = E_INVALIDARG;

  if (pevt != NULL) {
#if defined(_USE_WIN_API)
    if(*pevt != NULL) {
      DWORD dwRet = WaitForSingleObject(*pevt, timeout);
      if(dwRet == WAIT_OBJECT_0) {
        hr = S_OK;
      } else {
        hr = E_TIMEOUT;
      }
    }
#elif defined(_USE_LINUX_API)
    int ret;
    if(timeout == INFINITE) {
      ret = pthread_mutex_lock(&pevt->mutex);
      if(ret == 0) {
        if(pevt->signal == 0) {
          pthread_cond_wait(&pevt->cond, &pevt->mutex);
        }
        if(pevt->mode == 0) {
          pevt->signal = 0;
        }
        pthread_mutex_unlock(&pevt->mutex);
      }
    } else {
      ret = pthread_mutex_lock(&pevt->mutex);
      if(ret == 0) {
        if(pevt->signal == 0) {
          struct timespec spec;
          create_timeout(&spec, timeout);
          ret = pthread_cond_timedwait(&pevt->cond, &pevt->mutex, &spec);
        }
        if((ret == 0) && (pevt->mode == 0)) {
          pevt->signal = 0;
        }
        pthread_mutex_unlock(&pevt->mutex);
      }
    }
    if(ret == 0) {
      hr = S_OK;
    }
    else if(ret == ETIMEDOUT) {
      hr = E_TIMEOUT;
    }
#else
    hr = _wait_event(pevt, timeout);
#endif
  }

  return hr;
}

#if defined(_USE_LINUX_API)
struct multi_arg
{
  EVENT **pevt;
  HRESULT hr;
  uint32_t count;
  uint32_t index;
  uint32_t timeout;
  uint32_t stamp;
  int wait_all;
  volatile int *flag_all;
};

static void *_wait_event_multi(void *arg)
{
  uint32_t start, end;
  struct multi_arg *parg = (struct multi_arg *)arg;

  /* Initializes return value */
  parg->hr = E_TIMEOUT;
  parg->stamp = 0;

  /* Sets start time */
  start = gettimeofday_msec();

  /* Waits single event object */
  while(*parg->flag_all) {
    parg->hr = wait_event(parg->pevt[parg->index], 1);

    /* Gets finished time */
    end = gettimeofday_msec();

    if(SUCCEEDED(parg->hr) || (parg->hr != E_TIMEOUT)) {
      /* Sets finished time to return value */
      parg->stamp = end;
      break;
    }

    /* Checks timeout */
    if((parg->timeout != INFINITE) && (calc_time_diff(start, end) >= parg->timeout)) {
      break;
    }
  }

  if(parg->wait_all == 0) {
    /* Signals all event objects */
    *parg->flag_all = 0;
  }

  return arg;
}
#endif

/**
 * @fn            HRESULT wait_event_multi(EVENT **pevt, uint32_t count, uint32_t timeout, int wait_all)
 * @brief         Waits multiple events.
 * @param[in,out] pevt The pointer of event objects to be waited.
 * @param[in]     count The number of event objects.
 * @param[in]     timeout Timeout value.
 * @param[in]     wait_all If (wait_all != 0) then waits until all event objects are signaled, else waits until a event object is signaled.
 */
HRESULT
wait_event_multi(EVENT **pevt, uint32_t count, uint32_t timeout, int wait_all)
{
  HRESULT hr = E_INVALIDARG;

  if (pevt != NULL) {
#if defined(_USE_WIN_API)
    if(*pevt != NULL) {
      uint32_t i;
      DWORD dwRet;
      HANDLE *evts = (HANDLE *)malloc(count * sizeof(HANDLE));

      if(evts == NULL) {
        hr = E_OUTOFMEMORY;
        goto exit_proc;
      }

      if(count == 0) {
        hr = E_INVALIDARG;
        goto exit_proc;
      }

      for(i = 0; i < count; i++) {
        evts[i] = *pevt[i];
      }

      dwRet = WaitForMultipleObjects(count, evts, wait_all, timeout);
      if(dwRet < WAIT_OBJECT_0 + count) {
        hr = dwRet;
      } else {
        hr = E_TIMEOUT;
      }

exit_proc:
      if(evts != NULL) {
        free(evts);
      }
    }
#elif defined(_USE_LINUX_API)
    volatile int flag_all = 1;
    uint32_t i, min_index = (uint32_t)-1,
    base_stamp, diff_stamp, min_stamp = (uint32_t)-1;
    THREAD *pthread = (THREAD *)malloc(count * sizeof(THREAD));
    struct multi_arg *pret, *parg = (struct multi_arg *)malloc(count * sizeof(struct multi_arg));

    if((pthread == NULL) || (parg == NULL)) {
      hr = E_OUTOFMEMORY;
      goto exit_proc;
    }

    if(count == 0) {
      hr = E_INVALIDARG;
      goto exit_proc;
    }

    /* Sets base time */
    base_stamp = gettimeofday_msec();

    /* Waits single event object */
    for(i = 0; i < count; i++) {
      parg[i].pevt = pevt;
      parg[i].count = count;
      parg[i].index = i;
      parg[i].timeout = timeout;
      parg[i].wait_all = wait_all;
      parg[i].flag_all = &flag_all;
      pthread_create(&pthread[i], NULL, _wait_event_multi, &parg[i]);
    }

    /* Checks result */
    hr = S_OK;
    for(i = 0; i < count; i++) {
      pthread_join(pthread[i], (void **)&pret);

      if(SUCCEEDED(pret->hr)) {
        diff_stamp = calc_time_diff(base_stamp, pret->stamp);
        if(diff_stamp < min_stamp) {
          min_index = i;
          min_stamp = diff_stamp;
        }
      } else {
        hr = pret->hr;
      }
    }

    if(wait_all == 0) {
      if(min_index != (uint32_t)-1) {
        hr = WAIT_OBJECT_0 + min_index;
      }
    }

exit_proc:
    if(pthread != NULL) {
      free(pthread);
    }

    if(parg != NULL) {
      free(parg);
    }
#else
    hr = _wait_event_multi(pevt, count, timeout, wait_all);
#endif
  }

  return hr;
}

/**
 * @fn    uint32_t gettimeofday_msec()
 * @brief Gets the current time value [ms].
 */
uint32_t
gettimeofday_msec()
{
#if defined(_USE_WIN_API)
  return GetTickCount();
#elif defined(_USE_LINUX_API)
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint32_t)(tv.tv_sec * 1e+3 + tv.tv_usec * 1e-3);
#else
  return _gettimeofday_msec();
#endif
}
