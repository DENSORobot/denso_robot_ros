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

#include "denso_robot_core/denso_controller.h"

namespace denso_robot_core {

DensoController::DensoController(const std::string& name, const int* mode)
  : DensoBase(name, mode)
{
  for(int srvs = DensoBase::SRV_MIN; srvs <= DensoBase::SRV_MAX; srvs++) {
    BCAPService_Ptr service = boost::make_shared<bcap_service::BCAPService>();
    service->parseParams();
    switch(srvs){
      case DensoBase::SRV_ACT:
        service->put_Type("udp");
        break;
      default:
        service->put_Type("tcp");
        break;
    }
    m_vecService.push_back(service);
  }
}

DensoController::~DensoController()
{

}

HRESULT DensoController::InitializeBCAP(const std::string& filename)
{
  HRESULT      hr;
  XMLError     ret;
  XMLDocument  xmlDoc;
  XMLElement  *xmlCtrl, *xmlRob, *xmlTsk;

  for(int srvs = DensoBase::SRV_MIN; srvs <= DensoBase::SRV_MAX; srvs++) {
    hr = m_vecService[srvs]->Connect();
    if(FAILED(hr)) return hr;
  }

  ret = xmlDoc.LoadFile(filename.c_str());
  if(ret != XML_SUCCESS) return E_FAIL;

  hr = AddController();
  if(FAILED(hr)) return hr;

  xmlCtrl = xmlDoc.FirstChildElement(XML_CTRL_NAME);
  if(xmlCtrl == NULL) return E_FAIL;

  hr = AddVariable(xmlCtrl);
  if(FAILED(hr)) return hr;

  xmlRob = xmlCtrl->FirstChildElement(XML_ROBOT_NAME);
  if(xmlRob == NULL) return E_FAIL;

  hr = AddRobot(xmlRob);
  if(FAILED(hr)) return hr;

  xmlTsk = xmlCtrl->FirstChildElement(XML_TASK_NAME);
  if(xmlTsk == NULL) return E_FAIL;

  hr = AddTask(xmlTsk);

  return hr;
}

HRESULT DensoController::StartService(ros::NodeHandle& node)
{
  DensoRobot_Vec::iterator itRob;
  for(itRob  = m_vecRobot.begin();
      itRob != m_vecRobot.end();
      itRob++)
  {
    (*itRob)->StartService(node);
  }

  DensoTask_Vec::iterator itTsk;
  for(itTsk  = m_vecTask.begin();
      itTsk != m_vecTask.end();
      itTsk++)
  {
    (*itTsk)->StartService(node);
  }

  DensoVariable_Vec::iterator itVar;
  for(itVar  = m_vecVar.begin();
      itVar != m_vecVar.end();
      itVar++)
  {
    (*itVar)->StartService(node);
  }

  m_serving = true;

  return S_OK;
}

HRESULT DensoController::StopService()
{
  m_mtxSrv.lock();
  m_serving = false;
  m_mtxSrv.unlock();

  DensoRobot_Vec::iterator itRob;
  for(itRob  = m_vecRobot.begin();
      itRob != m_vecRobot.end();
      itRob++)
  {
    (*itRob)->StopService();
  }

  DensoTask_Vec::iterator itTsk;
  for(itTsk  = m_vecTask.begin();
      itTsk != m_vecTask.end();
      itTsk++)
  {
    (*itTsk)->StopService();
  }

  DensoVariable_Vec::iterator itVar;
  for(itVar  = m_vecVar.begin();
      itVar != m_vecVar.end();
      itVar++)
  {
    (*itVar)->StopService();
  }

  return S_OK;
}

bool DensoController::Update()
{
  boost::mutex::scoped_lock lockSrv(m_mtxSrv);
  if(!m_serving) return false;

  DensoRobot_Vec::iterator itRob;
  for(itRob  = m_vecRobot.begin();
      itRob != m_vecRobot.end();
      itRob++)
  {
    (*itRob)->Update();
  }

  DensoTask_Vec::iterator itTsk;
  for(itTsk  = m_vecTask.begin();
      itTsk != m_vecTask.end();
      itTsk++)
  {
    (*itTsk)->Update();
  }

  DensoVariable_Vec::iterator itVar;
  for(itVar  = m_vecVar.begin();
      itVar != m_vecVar.end();
      itVar++)
  {
    (*itVar)->Update();
  }

  return true;
}

HRESULT DensoController::get_Robot(int index, DensoRobot_Ptr* robot)
{
  if(robot == NULL) {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), m_vecRobot.begin(), m_vecRobot.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, index, &pBase);
  if(SUCCEEDED(hr)) {
    *robot = boost::dynamic_pointer_cast<DensoRobot>(pBase);
  }

  return hr;
}

HRESULT DensoController::get_Task(const std::string& name, DensoTask_Ptr* task)
{
  if(task == NULL) {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), m_vecTask.begin(), m_vecTask.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, name, &pBase);
  if(SUCCEEDED(hr)) {
    *task = boost::dynamic_pointer_cast<DensoTask>(pBase);
  }

  return hr;
}

HRESULT DensoController::get_Variable(const std::string& name, DensoVariable_Ptr* var)
{
  if(var == NULL) {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), m_vecVar.begin(), m_vecVar.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, name, &pBase);
  if(SUCCEEDED(hr)) {
    *var = boost::dynamic_pointer_cast<DensoVariable>(pBase);
  }

  return hr;
}

HRESULT DensoController::AddTask(XMLElement *xmlElem)
{
  int objs;
  HRESULT hr;

  Name_Vec   vecName;
  hr = DensoBase::GetObjectNames(ID_CONTROLLER_GETTASKNAMES, vecName);
  if(SUCCEEDED(hr)) {
    for(objs = 0; objs < vecName.size(); objs++) {
      Handle_Vec vecHandle;
      hr = DensoBase::AddObject(
          ID_CONTROLLER_GETTASK, vecName[objs], vecHandle);
      if(FAILED(hr)) break;

      DensoTask_Ptr tsk(new DensoTask(this,
          m_vecService, vecHandle, vecName[objs], m_mode));

      hr = tsk->InitializeBCAP(xmlElem);
      if(FAILED(hr)) break;

      m_vecTask.push_back(tsk);
    }
  }

  return hr;
}

HRESULT DensoController::AddVariable(const std::string& name)
{
  return DensoBase::AddVariable(ID_CONTROLLER_GETVARIABLE,
      name, m_vecVar);
}

HRESULT DensoController::AddVariable(XMLElement *xmlElem)
{
  HRESULT hr = S_OK;
  XMLElement *xmlVar;

  for(xmlVar = xmlElem->FirstChildElement(XML_VARIABLE_NAME);
      xmlVar!= NULL;
      xmlVar = xmlVar->NextSiblingElement(XML_VARIABLE_NAME)) {

      hr = DensoBase::AddVariable(ID_CONTROLLER_GETVARIABLE,
          xmlVar, m_vecVar);
      if(FAILED(hr)) break;
  }

  return hr;
}

}
