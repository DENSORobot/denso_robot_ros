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

#include <stdlib.h>
#include <sstream>
#include "bcap_service/bcap_service.h"
#include "bcap_core/bcap_funcid.h"
#include "bcap_core/bCAPClient/bcap_client.h"
#include "bcap_core/RACString/rac_string.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bcap_service");

  HRESULT hr;
  ros::NodeHandle node;

  bcap_service::BCAPService bcapsrv;

  bcapsrv.parseParams();

  hr = bcapsrv.Connect();
  if(FAILED(hr)) {
    ROS_ERROR("Failed to connect. (%X)", hr);
    return 1;
  } else {
    bcapsrv.StartService(node);

    ros::spin();

    bcapsrv.StopService();
    bcapsrv.Disconnect();
    return 0;
  }
}

namespace bcap_service {

BCAPService::BCAPService()
  : m_type(""), m_addr(""),
    m_port(0), m_timeout(0), m_retry(0), m_wait(0),
    m_fd(0), m_wdt(0), m_invoke(0)
{

}

BCAPService::~BCAPService()
{
  StopService();
  Disconnect();
}

void BCAPService::parseParams()
{
  ros::NodeHandle node;

  if(!node.getParam("conn_type", m_type))
  {
    m_type = "tcp";
  }

  if(!node.getParam("ip_address", m_addr))
  {
    m_addr = "192.168.0.1";
  }

  if(!node.getParam("port_number", m_port))
  {
    m_port = 5007;
  }

  if(!node.getParam("timeout", m_timeout))
  {
    m_timeout = 3000;
  }

  if(!node.getParam("retry_count", m_retry))
  {
    m_retry = 5;
  }

  if(!node.getParam("wait_time", m_wait))
  {
    m_wait = 0;
  }

  if(!node.getParam("watchdog_timer", m_wdt))
  {
    m_wdt = 400;
  }

  if(!node.getParam("invoke_timeout", m_invoke))
  {
    m_invoke = 180000;
  }
}

HRESULT BCAPService::Connect()
{
  HRESULT hr;
  std::stringstream  ss1;
  std::wstringstream ss2;

  ros::Duration(m_wait).sleep();

  ss1 << m_type << ":" << m_addr << ":" << m_port;
  hr = bCap_Open_Client(ss1.str().c_str(), m_timeout, m_retry, &m_fd);
  if(SUCCEEDED(hr)) {
    ss2 << L",WDT=" << m_wdt << L",InvokeTimeout=" << m_invoke;
    BSTR bstrOption = SysAllocString(ss2.str().c_str());
    hr = bCap_ServiceStart(m_fd, bstrOption);
    SysFreeString(bstrOption);
  }

  return hr;
}

HRESULT BCAPService::Disconnect()
{
  if(m_fd > 0) {
    KeyHandle_Vec::iterator it;
    for(it = m_vecKH.begin(); it != m_vecKH.end(); it++) {
      switch(it->first){
        case ID_CONTROLLER_DISCONNECT:
          bCap_ControllerDisconnect(m_fd, &it->second);
          break;
        case ID_EXTENSION_RELEASE:
          bCap_ExtensionRelease(m_fd, &it->second);
          break;
        case ID_FILE_RELEASE:
          bCap_FileRelease(m_fd, &it->second);
          break;
        case ID_ROBOT_RELEASE:
          bCap_RobotRelease(m_fd, &it->second);
          break;
        case ID_TASK_RELEASE:
          bCap_TaskRelease(m_fd, &it->second);
          break;
        case ID_VARIABLE_RELEASE:
          bCap_VariableRelease(m_fd, &it->second);
          break;
        case ID_COMMAND_RELEASE:
          bCap_CommandRelease(m_fd, &it->second);
          break;
        case ID_MESSAGE_RELEASE:
          bCap_MessageRelease(m_fd, &it->second);
          break;
      }
    }

    m_vecKH.clear();

    bCap_ServiceStop(m_fd);

    bCap_Close_Client(&m_fd);
  }

  return S_OK;
}

HRESULT BCAPService::StartService(ros::NodeHandle& node)
{
  m_svr = node.advertiseService("bcap_service", &BCAPService::CallFunction, this);
  return S_OK;
}

HRESULT BCAPService::StopService()
{
  m_svr.shutdown();
  return S_OK;
}

const std::string& BCAPService::get_Type() const
{
  return m_type;
}

void BCAPService::put_Type(const std::string& type)
{
  if(m_fd == 0) {
    m_type = type;
  }
}

uint32_t BCAPService::get_Timeout() const
{
  uint32_t value = 0;
  if(FAILED(bCap_GetTimeout(m_fd, &value))) {
    value = m_timeout;
  }
  return value;
}

void BCAPService::put_Timeout(uint32_t value)
{
  if(SUCCEEDED(bCap_SetTimeout(m_fd, value))) {
    m_timeout = value;
  }
}

unsigned int BCAPService::get_Retry() const
{
  unsigned int value = 0;
  if(FAILED(bCap_GetRetry(m_fd, &value))) {
    value = m_retry;
  }
  return value;
}

void BCAPService::put_Retry(unsigned int value)
{
  if(SUCCEEDED(bCap_SetRetry(m_fd, value))) {
    m_retry = value;
  }
}

bool BCAPService::CallFunction(bcap::Request &req, bcap::Response &res)
{
  HRESULT     hr = S_OK;
  char      *chRet = NULL;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());
  VariantInit(vntRet.get());

  for(int i = 0; i < req.vntArgs.size(); i++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());
    hr = ConvertRacStr2Variant(
        req.vntArgs[i].vt, req.vntArgs[i].value.c_str(),
        vntTmp.get());
    if(FAILED(hr)) goto err_proc;
    vntArgs.push_back(*vntTmp.get());
  }

  hr = ExecFunction(req.func_id, vntArgs, vntRet);
  if(FAILED(hr)) goto err_proc;

  hr = ConvertVariant2RacStr(*vntRet.get(), &chRet);
  if(FAILED(hr)) goto err_proc;

  res.HRESULT = S_OK;
  res.vntRet.vt = vntRet->vt;
  res.vntRet.value = chRet;

  free(chRet);

post_proc:
  return true;

err_proc:
  res.HRESULT = hr;
  res.vntRet.vt = VT_EMPTY;
  res.vntRet.value = "";

  goto post_proc;
}

HRESULT BCAPService::ExecFunction(
    int32_t func_id, VARIANT_Vec& vntArgs,
    VARIANT_Ptr& vntRet)
{
  HRESULT hr = E_INVALIDARG;
  KeyHandle tmpKH(0, 0);

  try {
    switch(func_id) {
      // Controller
      case ID_CONTROLLER_CONNECT:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerConnect(m_fd,
                vntArgs.at(0).bstrVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal, vntArgs.at(3).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_CONTROLLER_DISCONNECT;
        break;
      case ID_CONTROLLER_DISCONNECT:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_ControllerDisconnect(m_fd,
                &vntArgs.at(0).ulVal);
        break;
      case ID_CONTROLLER_GETEXTENSION:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetExtension(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_EXTENSION_RELEASE;
        break;
      case ID_CONTROLLER_GETFILE:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetFile(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_FILE_RELEASE;
        break;
      case ID_CONTROLLER_GETROBOT:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetRobot(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_ROBOT_RELEASE;
        break;
      case ID_CONTROLLER_GETTASK:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetTask(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_TASK_RELEASE;
        break;
      case ID_CONTROLLER_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetVariable(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_CONTROLLER_GETCOMMAND:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetCommand(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_COMMAND_RELEASE;
        break;
      case ID_CONTROLLER_GETEXTENSIONNAMES:
        hr = bCap_ControllerGetExtensionNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETFILENAMES:
        hr = bCap_ControllerGetFileNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETROBOTNAMES:
        hr = bCap_ControllerGetRobotNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETTASKNAMES:
        hr = bCap_ControllerGetTaskNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETVARIABLENAMES:
        hr = bCap_ControllerGetVariableNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETCOMMANDNAMES:
        hr = bCap_ControllerGetCommandNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_EXECUTE:
        hr = bCap_ControllerExecute(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_CONTROLLER_GETMESSAGE:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetMessage(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->ulVal);
        tmpKH.first = ID_MESSAGE_RELEASE;
        break;
      case ID_CONTROLLER_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_ControllerGetAttribute(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_CONTROLLER_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_ControllerGetHelp(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_CONTROLLER_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_ControllerGetName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_CONTROLLER_GETTAG:
        hr = bCap_ControllerGetTag(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_PUTTAG:
        hr = bCap_ControllerPutTag(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_CONTROLLER_GETID:
        hr = bCap_ControllerGetID(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_PUTID:
        hr = bCap_ControllerPutID(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      // Extension
      case ID_EXTENSION_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_ExtensionGetVariable(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_EXTENSION_GETVARIABLENAMES:
        hr = bCap_ExtensionGetVariableNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_EXTENSION_EXECUTE:
        hr = bCap_ExtensionExecute(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_EXTENSION_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_ExtensionGetAttribute(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_EXTENSION_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_ExtensionGetHelp(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_EXTENSION_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_ExtensionGetName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_EXTENSION_GETTAG:
        hr = bCap_ExtensionGetTag(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_EXTENSION_PUTTAG:
        hr = bCap_ExtensionPutTag(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_EXTENSION_GETID:
        hr = bCap_ExtensionGetID(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_EXTENSION_PUTID:
        hr = bCap_ExtensionPutID(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_EXTENSION_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_ExtensionRelease(m_fd,
                &vntArgs.at(0).ulVal);
        break;
      // File
      case ID_FILE_GETFILE:
        vntRet->vt = VT_UI4;
        hr = bCap_FileGetFile(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_FILE_RELEASE;
        break;
      case ID_FILE_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_FileGetVariable(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_FILE_GETFILENAMES:
        hr = bCap_FileGetFileNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_FILE_GETVARIABLENAMES:
        hr = bCap_FileGetVariableNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_FILE_EXECUTE:
        hr = bCap_FileExecute(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_FILE_COPY:
        hr = bCap_FileCopy(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal);
        break;
      case ID_FILE_DELETE:
        hr = bCap_FileDelete(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_FILE_MOVE:
        hr = bCap_FileMove(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal);
        break;
      case ID_FILE_RUN:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileRun(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETDATECREATED:
        hr = bCap_FileGetDateCreated(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_GETDATELASTACCESSED:
        hr = bCap_FileGetDateLastAccessed(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_GETDATELASTMODIFIED:
        hr = bCap_FileGetDateLastModified(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_GETPATH:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetPath(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETSIZE:
        vntRet->vt = VT_I4;
        hr = bCap_FileGetSize(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_FILE_GETTYPE:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetType(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETVALUE:
        hr = bCap_FileGetValue(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_PUTVALUE:
        hr = bCap_FilePutValue(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_FILE_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_FileGetAttribute(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_FILE_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetHelp(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETTAG:
        hr = bCap_FileGetTag(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_PUTTAG:
        hr = bCap_FilePutTag(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_FILE_GETID:
        hr = bCap_FileGetID(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_PUTID:
        hr = bCap_FilePutID(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_FILE_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_FileRelease(m_fd,
                &vntArgs.at(0).ulVal);
        break;
      // Robot
      case ID_ROBOT_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_RobotGetVariable(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_ROBOT_GETVARIABLENAMES:
        hr = bCap_RobotGetVariableNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_ROBOT_EXECUTE:
        hr = bCap_RobotExecute(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_ROBOT_ACCELERATE:
        hr = bCap_RobotAccelerate(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).fltVal, vntArgs.at(3).fltVal);
        break;
      case ID_ROBOT_CHANGE:
        hr = bCap_RobotChange(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_CHUCK:
        hr = bCap_RobotChuck(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_DRIVE:
        hr = bCap_RobotDrive(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).fltVal, vntArgs.at(3).bstrVal);
        break;
      case ID_ROBOT_GOHOME:
        hr = bCap_RobotGoHome(m_fd,
                vntArgs.at(0).ulVal);
        break;
      case ID_ROBOT_HALT:
        hr = bCap_RobotHalt(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_HOLD:
        hr = bCap_RobotHold(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_MOVE:
        hr = bCap_RobotMove(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2), vntArgs.at(3).bstrVal);
        break;
      case ID_ROBOT_ROTATE:
        hr = bCap_RobotRotate(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1), vntArgs.at(2).fltVal, vntArgs.at(3), vntArgs.at(4).bstrVal);
        break;
      case ID_ROBOT_SPEED:
        hr = bCap_RobotSpeed(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).fltVal);
        break;
      case ID_ROBOT_UNCHUCK:
        hr = bCap_RobotUnchuck(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_UNHOLD:
        hr = bCap_RobotUnhold(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_RobotGetAttribute(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_ROBOT_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_RobotGetHelp(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_ROBOT_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_RobotGetName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_ROBOT_GETTAG:
        hr = bCap_RobotGetTag(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_ROBOT_PUTTAG:
        hr = bCap_RobotPutTag(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_ROBOT_GETID:
        hr = bCap_RobotGetID(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_ROBOT_PUTID:
        hr = bCap_RobotPutID(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_ROBOT_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_RobotRelease(m_fd,
                &vntArgs.at(0).ulVal);
        break;
      // Task
      case ID_TASK_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_TaskGetVariable(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_TASK_GETVARIABLENAMES:
        hr = bCap_TaskGetVariableNames(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_TASK_EXECUTE:
        hr = bCap_TaskExecute(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_TASK_START:
        hr = bCap_TaskStart(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).bstrVal);
        break;
      case ID_TASK_STOP:
        hr = bCap_TaskStop(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).bstrVal);
        break;
      case ID_TASK_DELETE:
        hr = bCap_TaskDelete(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_TASK_GETFILENAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_TaskGetFileName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_TASK_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_TaskGetAttribute(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_TASK_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_TaskGetHelp(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_TASK_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_TaskGetName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_TASK_GETTAG:
        hr = bCap_TaskGetTag(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_TASK_PUTTAG:
        hr = bCap_TaskPutTag(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_TASK_GETID:
        hr = bCap_TaskGetID(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_TASK_PUTID:
        hr = bCap_TaskPutID(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_TASK_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_TaskRelease(m_fd,
                &vntArgs.at(0).ulVal);
        break;
      // Variable
      case ID_VARIABLE_GETDATETIME:
        hr = bCap_VariableGetDateTime(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_GETVALUE:
        hr = bCap_VariableGetValue(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_PUTVALUE:
        hr = bCap_VariablePutValue(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_VARIABLE_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_VariableGetAttribute(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_VARIABLE_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_VariableGetHelp(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_VARIABLE_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_VariableGetName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_VARIABLE_GETTAG:
        hr = bCap_VariableGetTag(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_PUTTAG:
        hr = bCap_VariablePutTag(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_VARIABLE_GETID:
        hr = bCap_VariableGetID(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_PUTID:
        hr = bCap_VariablePutID(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_VARIABLE_GETMICROSECOND:
        vntRet->vt = VT_I4;
        hr = bCap_VariableGetMicrosecond(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_VARIABLE_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_VariableRelease(m_fd,
                &vntArgs.at(0).ulVal);
        break;
      // Command
      case ID_COMMAND_EXECUTE:
        hr = bCap_CommandExecute(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal,
                vntRet.get());
        break;
      case ID_COMMAND_CANCEL:
        hr = bCap_CommandCancel(m_fd,
                vntArgs.at(0).ulVal);
        break;
      case ID_COMMAND_GETTIMEOUT:
        vntRet->vt = VT_I4;
        hr = bCap_CommandGetTimeout(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_COMMAND_PUTTIMEOUT:
        hr = bCap_CommandPutTimeout(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal);
        break;
      case ID_COMMAND_GETSTATE:
        vntRet->vt = VT_I4;
        hr = bCap_CommandGetState(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_COMMAND_GETPARAMETERS:
        hr = bCap_CommandGetParameters(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_PUTPARAMETERS:
        hr = bCap_CommandPutParameters(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_COMMAND_GETRESULT:
        hr = bCap_CommandGetResult(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_CommandGetAttribute(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_COMMAND_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_CommandGetHelp(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_COMMAND_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_CommandGetName(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_COMMAND_GETTAG:
        hr = bCap_CommandGetTag(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_PUTTAG:
        hr = bCap_CommandPutTag(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_COMMAND_GETID:
        hr = bCap_CommandGetID(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_PUTID:
        hr = bCap_CommandPutID(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_COMMAND_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_CommandRelease(m_fd,
                &vntArgs.at(0).ulVal);
        break;
      case ID_MESSAGE_REPLY:
        hr = bCap_MessageReply(m_fd,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_MESSAGE_CLEAR:
        hr = bCap_MessageClear(m_fd,
                vntArgs.at(0).ulVal);
        break;
      case ID_MESSAGE_GETDATETIME:
        hr = bCap_MessageGetDateTime(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_MESSAGE_GETDESCRIPTION:
        vntRet->vt = VT_BSTR;
        hr = bCap_MessageGetDescription(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_MESSAGE_GETDESTINATION:
        vntRet->vt = VT_BSTR;
        hr = bCap_MessageGetDestination(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_MESSAGE_GETNUMBER:
        vntRet->vt = VT_I4;
        hr = bCap_MessageGetNumber(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_MESSAGE_GETSERIALNUMBER:
        vntRet->vt = VT_I4;
        hr = bCap_MessageGetSerialNumber(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_MESSAGE_GETSOURCE:
        vntRet->vt = VT_BSTR;
        hr = bCap_MessageGetSource(m_fd,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_MESSAGE_GETVALUE:
        hr = bCap_MessageGetValue(m_fd,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_MESSAGE_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_MessageRelease(m_fd,
                &vntArgs.at(0).ulVal);
        break;
    }
  } catch(std::out_of_range&) {
    hr = DISP_E_BADINDEX;
  }

  if(hr == S_OK) {
    if(tmpKH.first > 0) {
      tmpKH.second = vntRet->ulVal;
      m_vecKH.push_back(tmpKH);
    }
    else if(tmpKH.second > 0) {
      KeyHandle_Vec::iterator it;
      for(it = m_vecKH.begin(); it != m_vecKH.end(); it++) {
        if((func_id == it->first)
           && (tmpKH.second == it->second))
        {
          m_vecKH.erase(it);
          break;
        }
      }
    }
  }

  return hr;
}

}

