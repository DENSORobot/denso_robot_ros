#ifndef DENSO_BASE_HPP__
#define DENSO_BASE_HPP__

// Std include
#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <boost/thread/mutex.hpp>

// Custom include
#include "bcap_core/bcap_funcid.h"
#include "bcap_service/bcap_service.hpp"
#include "denso_robot_core/tinyxml2.h"
#include "denso_robot_core/visibility_control.h"

using namespace tinyxml2;

#define MESSAGE_QUEUE (1)
#define BCAP_VAR_DEFAULT_DURATION (1000) /* [ms] */

namespace denso_robot_core
{

typedef std::vector<std::string> Name_Vec;
typedef std::vector<uint32_t> Handle_Vec;
typedef std::vector<bcap_service::BcapService_Ptr> Service_Vec;

class DensoBase
{

public:
    static constexpr int BCAP_GET_OBJECT_ARGS = 3;
    static constexpr int BCAP_GET_OBJECTNAMES_ARGS = 2;
    enum
    {
        SRV_MIN = 0,
        SRV_ACT = SRV_MIN,
        SRV_WATCH,
        SRV_MAX = SRV_WATCH
    };

public:
    DENSO_ROBOT_CORE_PUBLIC
    static std::string ConvertBSTRToString(const BSTR bstr);

    DENSO_ROBOT_CORE_PUBLIC
    static BSTR ConvertStringToBSTR(const std::string& str);

public:
    DensoBase(const std::string& name, const int* mode) : parent_(NULL), name_(name), mode_(mode), serving_(false)
    {
    }
    DensoBase(DensoBase* parent, Service_Vec& service, Handle_Vec& handle, const std::string& name, const int* mode)
        : parent_(parent), name_(name), mode_(mode), serving_(false)
    {
        vecService_ = service;
        vecHandle_ = handle;
    }
    virtual ~DensoBase()
    {
        StopService();
    }

    virtual HRESULT InitializeBCAP()
    {
        return S_OK;
    }

    virtual HRESULT TerminateBCAP()
    {
        return S_OK;
    }

    virtual HRESULT StartService() = 0;

    virtual HRESULT StopService()
    {
        return S_OK;
    }

    virtual bool Update()
    {
        return true;
    }

    const std::string& Name() const
    {
        return name_;
    }


protected:
    HRESULT AddVariable(int32_t get_id, const std::string& name,
                        std::vector<boost::shared_ptr<class DensoVariable> >& vecVar, int16_t vt = VT_EMPTY,
                        bool bRead = false, bool bWrite = false, bool bID = false,
                        int iDuration = BCAP_VAR_DEFAULT_DURATION);

    HRESULT AddVariable(int32_t get_id, const XMLElement* xmlVar,
                        std::vector<boost::shared_ptr<class DensoVariable> >& vecVar);

    HRESULT AddObject(int32_t get_id, const std::string& name, Handle_Vec& vecHandle);

    HRESULT GetObjectNames(int32_t func_id, Name_Vec& vecName);

    HRESULT get_Object(const std::vector<boost::shared_ptr<DensoBase> >& vecBase, int index,
                        boost::shared_ptr<DensoBase>* obj);

    HRESULT get_Object(const std::vector<boost::shared_ptr<DensoBase> >& vecBase, const std::string& name,
                        boost::shared_ptr<DensoBase>* obj);

protected:
    DensoBase* parent_;

    Service_Vec vecService_;
    Handle_Vec vecHandle_;

    std::string name_;
    const int* mode_;

    bool serving_;
    boost::mutex mtxSrv_;
};

typedef boost::shared_ptr<DensoBase> DensoBase_Ptr;
typedef std::vector<DensoBase_Ptr> DensoBase_Vec;
    
} // namespace denso_robot_core




#endif