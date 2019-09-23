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

#ifndef DENSO_BASE_H
#define DENSO_BASE_H

#include <ros/ros.h>
#include "bcap_core/bcap_funcid.h"
#include "bcap_service/bcap_service.h"

// Message (std_msgs)
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std_msgs;

// Message (original)
#include "denso_robot_core/Joints.h"
#include "denso_robot_core/ExJoints.h"
#include "denso_robot_core/PoseData.h"
#include "denso_robot_core/UserIO.h"

// Action
#include <actionlib/server/simple_action_server.h>
using namespace actionlib;

// Action (original)
#include "denso_robot_core/MoveStringAction.h"
#include "denso_robot_core/MoveValueAction.h"
#include "denso_robot_core/DriveStringAction.h"
#include "denso_robot_core/DriveValueAction.h"

#include "denso_robot_core/tinyxml2.h"
using namespace tinyxml2;

#define MESSAGE_QUEUE (1)
#define BCAP_VAR_DEFAULT_DURATION (1000) /* [ms] */

namespace denso_robot_core {

typedef std::vector<std::string>     Name_Vec;
typedef std::vector<uint32_t>        Handle_Vec;
typedef std::vector<BCAPService_Ptr> Service_Vec;

class DensoBase
{
public:
  enum {
    SRV_MIN = 0,
    SRV_ACT = SRV_MIN,
    SRV_WATCH,
    SRV_MAX = SRV_WATCH
  };

public:
  static std::string ConvertBSTRToString(const BSTR bstr);
  static BSTR ConvertStringToBSTR(const std::string& str);

public:
  DensoBase(const std::string& name, const int* mode)
    : m_parent(NULL),
      m_name(name), m_mode(mode), m_serving(false)
  {

  }

  DensoBase(DensoBase* parent,
     Service_Vec& service, Handle_Vec& handle,
     const std::string& name, const int* mode)
    : m_parent(parent),
      m_name(name), m_mode(mode), m_serving(false)
  {
    m_vecService = service;
    m_vecHandle  = handle;
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

  virtual HRESULT StartService(ros::NodeHandle& node) = 0;

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
    return m_name;
  }

  std::string RosName() const;

protected:
  HRESULT AddVariable(int32_t get_id,
     const std::string& name,
     std::vector<boost::shared_ptr<class DensoVariable> >& vecVar,
     int16_t vt = VT_EMPTY, bool bRead = false, bool bWrite = false,
     bool bID = false, int iDuration = BCAP_VAR_DEFAULT_DURATION);

  HRESULT AddVariable(int32_t get_id,
     const XMLElement *xmlVar,
     std::vector<boost::shared_ptr<class DensoVariable> >& vecVar);

  HRESULT AddObject(int32_t get_id, const std::string& name,
     Handle_Vec& vecHandle);

  HRESULT GetObjectNames(int32_t func_id,
     Name_Vec& vecName);

  HRESULT get_Object(const std::vector<boost::shared_ptr<DensoBase> >& vecBase,
     int index, boost::shared_ptr<DensoBase> *obj);

  HRESULT get_Object(const std::vector<boost::shared_ptr<DensoBase> >& vecBase,
     const std::string& name, boost::shared_ptr<DensoBase> *obj);

protected:
  DensoBase* m_parent;

  Service_Vec m_vecService;
  Handle_Vec  m_vecHandle;

  std::string m_name;
  const int* m_mode;

  bool m_serving;
  boost::mutex  m_mtxSrv;
};

typedef boost::shared_ptr<DensoBase> DensoBase_Ptr;
typedef std::vector<DensoBase_Ptr> DensoBase_Vec;

}

#endif // DENSO_BASE_H
