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

#ifndef _BCAP_SERVICE_H_
#define _BCAP_SERVICE_H_

#include <ros/ros.h>
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>

#include "variant_allocator.h"
#include "bcap_service/bcap.h"
#include "bcap_core/dn_common.h"
#include "bcap_core/dn_device.h"

typedef std::pair<int32_t,uint32_t> KeyHandle;
typedef std::vector<KeyHandle>      KeyHandle_Vec;

struct variant_deleter {
  void operator()(VARIANT *p) const {
    VariantClear(p);
    delete p;
  }
};

typedef boost::interprocess::unique_ptr<VARIANT, variant_deleter> VARIANT_Ptr;
typedef std::vector<VARIANT, VariantAllocator<VARIANT> >          VARIANT_Vec;

namespace bcap_service {

class BCAPService
{
 public:
  BCAPService();
  virtual ~BCAPService();

  void parseParams();

  HRESULT Connect();
  HRESULT Disconnect();

  HRESULT StartService(ros::NodeHandle& node);
  HRESULT StopService();

  const std::string& get_Type() const;
  void put_Type(const std::string& type);

  uint32_t get_Timeout() const;
  void put_Timeout(uint32_t value);

  unsigned int get_Retry() const;
  void put_Retry(unsigned int value);

  HRESULT ExecFunction(
      int32_t func_id, VARIANT_Vec& vntArgs,
      VARIANT_Ptr& vntRet);

 private:
  bool CallFunction(
      bcap::Request &req,
      bcap::Response &res);

  // Connect parameters
  std::string m_type, m_addr;
  int m_port, m_timeout, m_retry, m_wait;

  // Socket parameters
  int m_fd;

  // ServiceStart parameters
  int m_wdt, m_invoke;

  // Handle vector
  KeyHandle_Vec m_vecKH;

  // Service
  ros::ServiceServer m_svr;
};

}

typedef boost::shared_ptr<bcap_service::BCAPService> BCAPService_Ptr;

#endif
