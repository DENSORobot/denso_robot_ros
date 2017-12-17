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

#include "denso_robot_core/denso_robot.h"

#define NAME_ARMGROUP "_armgroup"

namespace denso_robot_core {

DensoRobot::DensoRobot(DensoBase* parent,
    Service_Vec& service, Handle_Vec& handle,
    const std::string& name, const int* mode)
  : DensoBase(parent, service, handle, name, mode),
    m_ArmGroup(0)
{

}

DensoRobot::~DensoRobot()
{

}

HRESULT DensoRobot::InitializeBCAP(XMLElement *xmlElem)
{
  return AddVariable(xmlElem);
}

HRESULT DensoRobot::StartService(ros::NodeHandle& node)
{
  std::string tmpName = DensoBase::RosName();

  if(*m_mode == 0) {
    m_subArmGroup = node.subscribe<Int32>(
        tmpName + NAME_ARMGROUP, MESSAGE_QUEUE,
        &DensoRobot::Callback_ArmGroup, this);
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

HRESULT DensoRobot::StopService()
{
  m_mtxSrv.lock();
  m_serving = false;
  m_mtxSrv.unlock();

  m_subArmGroup.shutdown();

  DensoVariable_Vec::iterator itVar;
  for(itVar  = m_vecVar.begin();
      itVar != m_vecVar.end();
      itVar++)
  {
    (*itVar)->StopService();
  }

  return S_OK;
}

bool DensoRobot::Update()
{
  boost::mutex::scoped_lock lockSrv(m_mtxSrv);
  if(!m_serving) return false;

  DensoVariable_Vec::iterator itVar;
  for(itVar  = m_vecVar.begin();
      itVar != m_vecVar.end();
      itVar++)
  {
    (*itVar)->Update();
  }

  return true;
}

void DensoRobot::ChangeArmGroup(int number)
{
  m_ArmGroup = number;
}

void DensoRobot::Callback_ArmGroup(const Int32::ConstPtr& msg)
{
  ChangeArmGroup(msg->data);
}

HRESULT DensoRobot::get_Variable(const std::string& name, DensoVariable_Ptr* var)
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

HRESULT DensoRobot::AddVariable(const std::string& name)
{
  return DensoBase::AddVariable(ID_ROBOT_GETVARIABLE,
      name, m_vecVar);
}

HRESULT DensoRobot::AddVariable(XMLElement *xmlElem)
{
  HRESULT hr = S_OK;
  XMLElement *xmlVar;

  for(xmlVar = xmlElem->FirstChildElement(XML_VARIABLE_NAME);
      xmlVar!= NULL;
      xmlVar = xmlVar->NextSiblingElement(XML_VARIABLE_NAME))
  {
    hr = DensoBase::AddVariable(ID_ROBOT_GETVARIABLE,
        xmlVar, m_vecVar);
    if(FAILED(hr)) break;
  }

  return hr;
}

HRESULT DensoRobot::CreatePoseData(
    const PoseData &pose, VARIANT &vnt)
{
  uint32_t i, j;
  uint32_t num = 3 +
    (((pose.exjoints.mode != 0) && (pose.exjoints.joints.size() > 0)) ? 1 : 0);
  float   *pfltval;
  VARIANT *pvntval;

  vnt.vt = (VT_ARRAY | VT_VARIANT);
  vnt.parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

  SafeArrayAccessData(vnt.parray, (void**)&pvntval);

  for(i = 0; i < num; i++) {
    switch(i){
      case 0:
	pvntval[i].vt = (VT_ARRAY | VT_R4);
	pvntval[i].parray = SafeArrayCreateVector(VT_R4, 0, pose.value.size());
	SafeArrayAccessData(pvntval[i].parray, (void**)&pfltval);
	std::copy(pose.value.begin(), pose.value.end(), pfltval);
	SafeArrayUnaccessData(pvntval[i].parray);
	break;
      case 1:
	pvntval[i].vt = VT_I4;
	pvntval[i].lVal  = pose.type;
	break;
      case 2:
	pvntval[i].vt = VT_I4;
	pvntval[i].lVal  = pose.pass;
        break;
      case 3:
	CreateExJoints(pose.exjoints, pvntval[i]);
        break;
    }
  }

  SafeArrayUnaccessData(vnt.parray);

  return S_OK;
}

HRESULT DensoRobot::CreateExJoints(
    const ExJoints &exjoints, VARIANT &vnt)
{
  uint32_t i, j;
  uint32_t num = 1 + exjoints.joints.size();
  VARIANT *pvntval, *pjntval;

  vnt.vt = (VT_ARRAY | VT_VARIANT);
  vnt.parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

  SafeArrayAccessData(vnt.parray, (void**)&pvntval);

  for(i = 0; i < num; i++) {
    if(i == 0) {
      pvntval[0].vt   = VT_I4;
      pvntval[0].lVal = exjoints.mode;
    } else {
      pvntval[i].vt = (VT_ARRAY | VT_VARIANT);
      pvntval[i].parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
      SafeArrayAccessData(pvntval[i].parray, (void**)&pjntval);
      pjntval[0].vt     = VT_I4;
      pjntval[0].lVal   = exjoints.joints.at(i-1).joint;
      pjntval[1].vt     = VT_R4;
      pjntval[1].fltVal = exjoints.joints.at(i-1).value;
      SafeArrayUnaccessData(pvntval[i].parray);
    }
  }

  SafeArrayUnaccessData(vnt.parray);

  return S_OK;
}

}
