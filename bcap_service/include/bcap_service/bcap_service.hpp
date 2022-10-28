#ifndef BCAP_SERVICE_HPP__
#define BCAP_SERVICE_HPP__

// Std include
#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>

// Custom include
#include "bcap_core/dn_common.h"
#include "bcap_core/dn_device.h"
#include "bcap_service/variant_allocator.h"
#include "bcap_service/visibility_control.h"


namespace bcap_service {

typedef std::pair<int32_t,uint32_t> KeyHandle;
typedef std::vector<KeyHandle>      KeyHandle_Vec;

struct VARIANT_MSG
{
  int16_t vt;
  std::string value;
};

struct BCAP_SERVICE_REQ
{
  int32_t func_id;
  std::vector<VARIANT_MSG> vntreq;
};

struct BCAP_SERVICE_RES
{
  int32_t result;
  VARIANT_MSG vntres;
};

struct variant_deleter {
  void operator()(VARIANT *p) const {
    VariantClear(p);
    delete p;
  }
};

typedef std::vector<VARIANT, VariantAllocator<VARIANT> >          VARIANT_Vec;
typedef boost::interprocess::unique_ptr<VARIANT, variant_deleter> VARIANT_Ptr;


class BcapService
{
public:
    BcapService();
    virtual ~BcapService();

    BCAP_SERVICE_PUBLIC 
    HRESULT Connect();

    BCAP_SERVICE_PUBLIC 
    HRESULT Disconnect();

    BCAP_SERVICE_PUBLIC 
    HRESULT StartService();

    BCAP_SERVICE_PUBLIC 
    HRESULT StopService();

    BCAP_SERVICE_PUBLIC
    const std::string& get_Type() const;

    BCAP_SERVICE_PUBLIC
    void put_Type(const std::string& type);

    BCAP_SERVICE_PUBLIC
    uint32_t get_Timeout() const;

    BCAP_SERVICE_PUBLIC
    void put_Timeout(uint32_t value);

    BCAP_SERVICE_PUBLIC
    unsigned int get_Retry() const;

    BCAP_SERVICE_PUBLIC
    void put_Retry(unsigned int value);

    BCAP_SERVICE_PUBLIC
    HRESULT CallFunction(BCAP_SERVICE_REQ& req, BCAP_SERVICE_RES& res);

    BCAP_SERVICE_PUBLIC
    HRESULT ExecFunction(
        int32_t func_id, VARIANT_Vec& vntArgs,
        VARIANT_Ptr& vntRet);

private:
    // bool CallFunction(
    //     bcap::Request &req,
    //     bcap::Response &res);

    // Connect parameters
    std::string type_, addr_;
    int port_, timeout_, retry_, wait_;

    // Socket parameters
    int fd_;

    // ServiceStart parameters
    int wdt_, invoke_;

    // Handle vector
    KeyHandle_Vec vecKH_;
};

typedef boost::shared_ptr<bcap_service::BcapService> BcapService_Ptr;

} /* End of bcap_service */

#endif