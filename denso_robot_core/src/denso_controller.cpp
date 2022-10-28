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

#include "denso_robot_core/denso_controller.hpp"
#include "boost/make_shared.hpp"

namespace denso_robot_core
{
DensoController::DensoController(const std::string& name, const int* mode)
  : DensoBase(name, mode)
{
  for (int srvs = DensoBase::SRV_MIN; srvs <= DensoBase::SRV_MAX; srvs++)
  {
    BcapService_Ptr service = boost::make_shared<bcap_service::BcapService>();
    // service->parseParams();
    switch (srvs)
    {
      case DensoBase::SRV_ACT:
        service->put_Type("udp");
        break;
      default:
        service->put_Type("tcp");
        break;
    }
    vecService_.push_back(service);
  }
}

DensoController::~DensoController()
{
}

HRESULT DensoController::InitializeBCAP(const std::string& filename)
{
  HRESULT hr;
  XMLError ret;
  XMLDocument xmlDoc;
  XMLElement *xmlCtrl, *xmlRob, *xmlTsk;

  for (int srvs = DensoBase::SRV_MIN; srvs <= DensoBase::SRV_MAX; srvs++)
  {
    hr = vecService_[srvs]->Connect();
    if (FAILED(hr))
      return hr;
  }

  ret = xmlDoc.LoadFile(filename.c_str());
  if (ret != XML_SUCCESS)
    return E_FAIL;

  hr = AddController();
  if (FAILED(hr))
    return hr;

  xmlCtrl = xmlDoc.FirstChildElement(DensoController::XML_CTRL_NAME);
  if (xmlCtrl == NULL)
    return E_FAIL;

  hr = AddVariable(xmlCtrl);
  if (FAILED(hr))
    return hr;

  xmlRob = xmlCtrl->FirstChildElement(DensoRobot::XML_ROBOT_NAME);
  if (xmlRob == NULL)
    return E_FAIL;

  hr = AddRobot(xmlRob);
  if (FAILED(hr))
    return hr;

  xmlTsk = xmlCtrl->FirstChildElement(DensoTask::XML_TASK_NAME);
  if (xmlTsk == NULL)
    return E_FAIL;

  hr = AddTask(xmlTsk);

  return hr;
}

HRESULT DensoController::StartService()
{
  DensoRobot_Vec::iterator itRob;
  for (itRob = vecRobot_.begin(); itRob != vecRobot_.end(); itRob++)
  {
    (*itRob)->StartService();
  }

  DensoTask_Vec::iterator itTsk;
  for (itTsk = vecTask_.begin(); itTsk != vecTask_.end(); itTsk++)
  {
    (*itTsk)->StartService();
  }

  DensoVariable_Vec::iterator itVar;
  for (itVar = vecVar_.begin(); itVar != vecVar_.end(); itVar++)
  {
    (*itVar)->StartService();
  }

  serving_ = true;

  return S_OK;
}

HRESULT DensoController::StopService()
{
  mtxSrv_.lock();
  serving_ = false;
  mtxSrv_.unlock();

  DensoRobot_Vec::iterator itRob;
  for (itRob = vecRobot_.begin(); itRob != vecRobot_.end(); itRob++)
  {
    (*itRob)->StopService();
  }

  DensoTask_Vec::iterator itTsk;
  for (itTsk = vecTask_.begin(); itTsk != vecTask_.end(); itTsk++)
  {
    (*itTsk)->StopService();
  }

  DensoVariable_Vec::iterator itVar;
  for (itVar = vecVar_.begin(); itVar != vecVar_.end(); itVar++)
  {
    (*itVar)->StopService();
  }

  return S_OK;
}

bool DensoController::Update()
{
  boost::mutex::scoped_lock lockSrv(mtxSrv_);
  if (!serving_)
    return false;

  DensoRobot_Vec::iterator itRob;
  for (itRob = vecRobot_.begin(); itRob != vecRobot_.end(); itRob++)
  {
    (*itRob)->Update();
  }

  DensoTask_Vec::iterator itTsk;
  for (itTsk = vecTask_.begin(); itTsk != vecTask_.end(); itTsk++)
  {
    (*itTsk)->Update();
  }

  DensoVariable_Vec::iterator itVar;
  for (itVar = vecVar_.begin(); itVar != vecVar_.end(); itVar++)
  {
    (*itVar)->Update();
  }

  return true;
}

HRESULT DensoController::get_Robot(int index, DensoRobot_Ptr* robot)
{
  if (robot == NULL)
  {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), vecRobot_.begin(), vecRobot_.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, index, &pBase);
  if (SUCCEEDED(hr))
  {
    *robot = boost::dynamic_pointer_cast<DensoRobot>(pBase);
  }

  return hr;
}

HRESULT DensoController::get_Task(const std::string& name, DensoTask_Ptr* task)
{
  if (task == NULL)
  {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), vecTask_.begin(), vecTask_.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, name, &pBase);
  if (SUCCEEDED(hr))
  {
    *task = boost::dynamic_pointer_cast<DensoTask>(pBase);
  }

  return hr;
}

HRESULT DensoController::get_Variable(const std::string& name, DensoVariable_Ptr* var)
{
  if (var == NULL)
  {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), vecVar_.begin(), vecVar_.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, name, &pBase);
  if (SUCCEEDED(hr))
  {
    *var = boost::dynamic_pointer_cast<DensoVariable>(pBase);
  }

  return hr;
}

HRESULT DensoController::AddTask(XMLElement* xmlElem)
{
  HRESULT hr;

  Name_Vec vecName;
  hr = DensoBase::GetObjectNames(ID_CONTROLLER_GETTASKNAMES, vecName);
  if (SUCCEEDED(hr))
  {
    for (size_t objs = 0; objs < vecName.size(); objs++)
    {
      Handle_Vec vecHandle;
      hr = DensoBase::AddObject(ID_CONTROLLER_GETTASK, vecName[objs], vecHandle);
      if (FAILED(hr))
        break;

      DensoTask_Ptr tsk(new DensoTask(this, vecService_, vecHandle, vecName[objs], mode_));

      hr = tsk->InitializeBCAP(xmlElem);
      if (FAILED(hr))
        break;

      vecTask_.push_back(tsk);
    }
  }

  return hr;
}

HRESULT DensoController::AddVariable(const std::string& name)
{
  return DensoBase::AddVariable(ID_CONTROLLER_GETVARIABLE, name, vecVar_);
}

HRESULT DensoController::AddVariable(XMLElement* xmlElem)
{
  HRESULT hr = S_OK;
  XMLElement* xmlVar;

  for (xmlVar = xmlElem->FirstChildElement(DensoVariable::XML_VARIABLE_NAME); xmlVar != NULL;
       xmlVar = xmlVar->NextSiblingElement(DensoVariable::XML_VARIABLE_NAME))
  {
    hr = DensoBase::AddVariable(ID_CONTROLLER_GETVARIABLE, xmlVar, vecVar_);
    if (FAILED(hr))
      break;
  }

  return hr;
}

/**
 * CaoController::Execute("ClearError")
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoController::ExecClearError()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  for (argc = 0; argc < BCAP_CONTROLLER_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());

    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_I4;
        vntTmp->ulVal = vecHandle_[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"ClearError");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_CONTROLLER_EXECUTE, vntArgs, vntRet);
}

/**
 * CaoController::Execute("GetCurErrorCount")
 * Do NOT call on b-CAP Slave.
 * @param[out] count The number of errors being occurred
 * @return HRESULT
 */
HRESULT DensoController::ExecGetCurErrorCount(int& count)
{
  HRESULT hr;

  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  for (argc = 0; argc < BCAP_CONTROLLER_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());

    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_I4;
        vntTmp->lVal = vecHandle_[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"GetCurErrorCount");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_CONTROLLER_EXECUTE, vntArgs, vntRet);

  if (FAILED(hr) || (vntRet->vt != VT_I4))
  {
    return hr;
  }
  count = vntRet->lVal;

  return hr;
}

/**
 * CaoController::Execute("GetCurErrorInfo")
 * Do NOT call on b-CAP Slave.
 * @param[in] error_index Index number
 * @param[out] error_code Error Code
 * @param[out] error_message Error message
 * @return HRESULT
 */
HRESULT DensoController::ExecGetCurErrorInfo(int error_index, HRESULT& error_code, std::string& error_message)
{
  HRESULT hr;

  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VARIANT* elements;

  for (argc = 0; argc < BCAP_CONTROLLER_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());

    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_I4;
        vntTmp->lVal = vecHandle_[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"GetCurErrorInfo");
        break;
      case 2:
        vntTmp->vt = VT_I4;
        vntTmp->lVal = error_index;
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_CONTROLLER_EXECUTE, vntArgs, vntRet);

  if (FAILED(hr) || (vntRet->vt != (VT_ARRAY | VT_VARIANT)))
  {
    return hr;
  }

  SafeArrayAccessData(vntRet->parray, (void**)&elements);
  if (elements[0].vt == VT_I4)
  {
    error_code = elements[0].lVal;
  }
  if (elements[1].vt == VT_BSTR)
  {
    error_message = ConvertBSTRToString(elements[1].bstrVal);
  }
  SafeArrayUnaccessData(vntRet->parray);

  return hr;
}

/**
 * CaoController::Execute("GetErrorDescription")
 * Do NOT call on b-CAP Slave.
 * @param[in] error_code Error Code
 * @param[out] error_description Error description
 * @return HRESULT
 */
HRESULT DensoController::ExecGetErrorDescription(HRESULT error_code, std::string& error_description)
{
  HRESULT hr;

  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  // VARIANT* elements;

  for (argc = 0; argc < BCAP_CONTROLLER_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());

    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_I4;
        vntTmp->lVal = vecHandle_[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"GetErrorDescription");
        break;
      case 2:
        vntTmp->vt = VT_I4;
        vntTmp->lVal = error_code;
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_CONTROLLER_EXECUTE, vntArgs, vntRet);

  if (FAILED(hr) || (vntRet->vt != VT_BSTR))
  {
    return hr;
  }
  error_description = ConvertBSTRToString(vntRet->bstrVal);

  return hr;
}

}  // namespace denso_robot_core
