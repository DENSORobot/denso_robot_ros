#include "bcap_service/bcap_service.hpp"
#include "bcap_core/bcap_funcid.h"
#include "bcap_core/bCAPClient/bcap_client.h"
#include "bcap_core/RACString/rac_string.h"
#include <stdlib.h>
#include <sstream>
#include <chrono>
#include <thread>
namespace bcap_service {

BcapService::BcapService(std::string addr, int port)
  : type_("tcp"), addr_(addr),
    port_(port), timeout_(3000), retry_(5), wait_(0),
    fd_(0), wdt_(400), invoke_(180000)
{

}

BcapService::~BcapService()
{
  Disconnect();
}

HRESULT BcapService::Connect()
{
    HRESULT hr;
    std::stringstream  ss1;
    std::wstringstream ss2;
    sleep(wait_);
    ss1 << type_ << ":" << addr_ << ":" << port_;
    hr = bCap_Open_Client(ss1.str().c_str(), timeout_, retry_, &fd_);
    if(SUCCEEDED(hr)) {
        ss2 << L",WDT=" << wdt_ << L",InvokeTimeout=" << invoke_;
        BSTR bstrOption = SysAllocString(ss2.str().c_str());
        hr = bCap_ServiceStart(fd_, bstrOption);
        SysFreeString(bstrOption);
    }
    return hr;
}

HRESULT BcapService::Disconnect()
{
  if(fd_ > 0) {
    KeyHandle_Vec::iterator it;
    for(it = vecKH_.begin(); it != vecKH_.end(); it++) {
      switch(it->first){
        case ID_CONTROLLER_DISCONNECT:
          bCap_ControllerDisconnect(fd_, &it->second);
          break;
        case ID_EXTENSION_RELEASE:
          bCap_ExtensionRelease(fd_, &it->second);
          break;
        case ID_FILE_RELEASE:
          bCap_FileRelease(fd_, &it->second);
          break;
        case ID_ROBOT_RELEASE:
          bCap_RobotRelease(fd_, &it->second);
          break;
        case ID_TASK_RELEASE:
          bCap_TaskRelease(fd_, &it->second);
          break;
        case ID_VARIABLE_RELEASE:
          bCap_VariableRelease(fd_, &it->second);
          break;
        case ID_COMMAND_RELEASE:
          bCap_CommandRelease(fd_, &it->second);
          break;
        case ID_MESSAGE_RELEASE:
          bCap_MessageRelease(fd_, &it->second);
          break;
      }
    }

    vecKH_.clear();

    bCap_ServiceStop(fd_);

    bCap_Close_Client(&fd_);
  }

  return S_OK;
}


const std::string& BcapService::get_Type() const
{
  return type_;
}

void BcapService::set_type(const std::string& type)
{
  if(fd_ == 0) {
    type_ = type;
  }
}

uint32_t BcapService::get_timeout() const
{
  uint32_t value = 0;
  if(FAILED(bCap_GetTimeout(fd_, &value))) {
    value = timeout_;
  }
  return value;
}

void BcapService::set_timeout(uint32_t value)
{
  if(SUCCEEDED(bCap_SetTimeout(fd_, value))) {
    timeout_ = value;
  }
}

unsigned int BcapService::get_retry() const
{
  unsigned int value = 0;
  if(FAILED(bCap_GetRetry(fd_, &value))) {
    value = retry_;
  }
  return value;
}

void BcapService::set_retry(unsigned int value)
{
  if(SUCCEEDED(bCap_SetRetry(fd_, value))) {
    retry_ = value;
  }
}

HRESULT BcapService::CallFunction(BCAP_SERVICE_REQ& req, BCAP_SERVICE_RES& res)
{
  HRESULT     hr = S_OK;
  char      *chRet = NULL;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for(size_t i = 0; i < req.vntreq.size(); i++) {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());
    hr = ConvertRacStr2Variant(
        req.vntreq[i].vt, req.vntreq[i].value.c_str(),
        vntTmp.get());
    if(FAILED(hr)) goto err_proc;
    vntArgs.push_back(*vntTmp.get());
  }

  hr = ExecFunction(req.func_id, vntArgs, vntRet);
  if(FAILED(hr)) goto err_proc;

  hr = ConvertVariant2RacStr(*vntRet.get(), &chRet);
  if(FAILED(hr)) goto err_proc;

  res.result = S_OK;
  res.vntres.vt = vntRet->vt;
  res.vntres.value = chRet;

  free(chRet);

post_proc:
  return true;

err_proc:
  res.result = hr;
  res.vntres.vt = VT_EMPTY;
  res.vntres.value = "";

  goto post_proc;
}

HRESULT BcapService::ExecFunction(
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
        hr = bCap_ControllerConnect(fd_,
                vntArgs.at(0).bstrVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal, vntArgs.at(3).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_CONTROLLER_DISCONNECT;
        break;
      case ID_CONTROLLER_DISCONNECT:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_ControllerDisconnect(fd_,
                &vntArgs.at(0).ulVal);
        break;
      case ID_CONTROLLER_GETEXTENSION:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetExtension(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_EXTENSION_RELEASE;
        break;
      case ID_CONTROLLER_GETFILE:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetFile(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_FILE_RELEASE;
        break;
      case ID_CONTROLLER_GETROBOT:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetRobot(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_ROBOT_RELEASE;
        break;
      case ID_CONTROLLER_GETTASK:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetTask(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_TASK_RELEASE;
        break;
      case ID_CONTROLLER_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetVariable(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_CONTROLLER_GETCOMMAND:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetCommand(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_COMMAND_RELEASE;
        break;
      case ID_CONTROLLER_GETEXTENSIONNAMES:
        hr = bCap_ControllerGetExtensionNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETFILENAMES:
        hr = bCap_ControllerGetFileNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETROBOTNAMES:
        hr = bCap_ControllerGetRobotNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETTASKNAMES:
        hr = bCap_ControllerGetTaskNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETVARIABLENAMES:
        hr = bCap_ControllerGetVariableNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_GETCOMMANDNAMES:
        hr = bCap_ControllerGetCommandNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_EXECUTE:
        hr = bCap_ControllerExecute(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_CONTROLLER_GETMESSAGE:
        vntRet->vt = VT_UI4;
        hr = bCap_ControllerGetMessage(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->ulVal);
        tmpKH.first = ID_MESSAGE_RELEASE;
        break;
      case ID_CONTROLLER_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_ControllerGetAttribute(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_CONTROLLER_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_ControllerGetHelp(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_CONTROLLER_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_ControllerGetName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_CONTROLLER_GETTAG:
        hr = bCap_ControllerGetTag(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_PUTTAG:
        hr = bCap_ControllerPutTag(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_CONTROLLER_GETID:
        hr = bCap_ControllerGetID(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_CONTROLLER_PUTID:
        hr = bCap_ControllerPutID(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      // Extension
      case ID_EXTENSION_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_ExtensionGetVariable(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_EXTENSION_GETVARIABLENAMES:
        hr = bCap_ExtensionGetVariableNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_EXTENSION_EXECUTE:
        hr = bCap_ExtensionExecute(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_EXTENSION_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_ExtensionGetAttribute(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_EXTENSION_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_ExtensionGetHelp(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_EXTENSION_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_ExtensionGetName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_EXTENSION_GETTAG:
        hr = bCap_ExtensionGetTag(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_EXTENSION_PUTTAG:
        hr = bCap_ExtensionPutTag(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_EXTENSION_GETID:
        hr = bCap_ExtensionGetID(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_EXTENSION_PUTID:
        hr = bCap_ExtensionPutID(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_EXTENSION_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_ExtensionRelease(fd_,
                &vntArgs.at(0).ulVal);
        break;
      // File
      case ID_FILE_GETFILE:
        vntRet->vt = VT_UI4;
        hr = bCap_FileGetFile(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_FILE_RELEASE;
        break;
      case ID_FILE_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_FileGetVariable(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_FILE_GETFILENAMES:
        hr = bCap_FileGetFileNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_FILE_GETVARIABLENAMES:
        hr = bCap_FileGetVariableNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_FILE_EXECUTE:
        hr = bCap_FileExecute(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_FILE_COPY:
        hr = bCap_FileCopy(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal);
        break;
      case ID_FILE_DELETE:
        hr = bCap_FileDelete(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_FILE_MOVE:
        hr = bCap_FileMove(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal);
        break;
      case ID_FILE_RUN:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileRun(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETDATECREATED:
        hr = bCap_FileGetDateCreated(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_GETDATELASTACCESSED:
        hr = bCap_FileGetDateLastAccessed(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_GETDATELASTMODIFIED:
        hr = bCap_FileGetDateLastModified(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_GETPATH:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetPath(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETSIZE:
        vntRet->vt = VT_I4;
        hr = bCap_FileGetSize(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_FILE_GETTYPE:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetType(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETVALUE:
        hr = bCap_FileGetValue(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_PUTVALUE:
        hr = bCap_FilePutValue(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_FILE_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_FileGetAttribute(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_FILE_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetHelp(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_FileGetName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_FILE_GETTAG:
        hr = bCap_FileGetTag(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_PUTTAG:
        hr = bCap_FilePutTag(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_FILE_GETID:
        hr = bCap_FileGetID(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_FILE_PUTID:
        hr = bCap_FilePutID(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_FILE_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_FileRelease(fd_,
                &vntArgs.at(0).ulVal);
        break;
      // Robot
      case ID_ROBOT_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_RobotGetVariable(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_ROBOT_GETVARIABLENAMES:
        hr = bCap_RobotGetVariableNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_ROBOT_EXECUTE:
        hr = bCap_RobotExecute(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_ROBOT_ACCELERATE:
        hr = bCap_RobotAccelerate(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).fltVal, vntArgs.at(3).fltVal);
        break;
      case ID_ROBOT_CHANGE:
        hr = bCap_RobotChange(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_CHUCK:
        hr = bCap_RobotChuck(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_DRIVE:
        hr = bCap_RobotDrive(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).fltVal, vntArgs.at(3).bstrVal);
        break;
      case ID_ROBOT_GOHOME:
        hr = bCap_RobotGoHome(fd_,
                vntArgs.at(0).ulVal);
        break;
      case ID_ROBOT_HALT:
        hr = bCap_RobotHalt(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_HOLD:
        hr = bCap_RobotHold(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_MOVE:
        hr = bCap_RobotMove(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2), vntArgs.at(3).bstrVal);
        break;
      case ID_ROBOT_ROTATE:
        hr = bCap_RobotRotate(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1), vntArgs.at(2).fltVal, vntArgs.at(3), vntArgs.at(4).bstrVal);
        break;
      case ID_ROBOT_SPEED:
        hr = bCap_RobotSpeed(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).fltVal);
        break;
      case ID_ROBOT_UNCHUCK:
        hr = bCap_RobotUnchuck(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_UNHOLD:
        hr = bCap_RobotUnhold(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_ROBOT_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_RobotGetAttribute(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_ROBOT_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_RobotGetHelp(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_ROBOT_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_RobotGetName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_ROBOT_GETTAG:
        hr = bCap_RobotGetTag(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_ROBOT_PUTTAG:
        hr = bCap_RobotPutTag(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_ROBOT_GETID:
        hr = bCap_RobotGetID(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_ROBOT_PUTID:
        hr = bCap_RobotPutID(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_ROBOT_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_RobotRelease(fd_,
                &vntArgs.at(0).ulVal);
        break;
      // Task
      case ID_TASK_GETVARIABLE:
        vntRet->vt = VT_UI4;
        hr = bCap_TaskGetVariable(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2).bstrVal,
                &vntRet->ulVal);
        tmpKH.first = ID_VARIABLE_RELEASE;
        break;
      case ID_TASK_GETVARIABLENAMES:
        hr = bCap_TaskGetVariableNames(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal,
                vntRet.get());
        break;
      case ID_TASK_EXECUTE:
        hr = bCap_TaskExecute(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal, vntArgs.at(2),
                vntRet.get());
        break;
      case ID_TASK_START:
        hr = bCap_TaskStart(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).bstrVal);
        break;
      case ID_TASK_STOP:
        hr = bCap_TaskStop(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal, vntArgs.at(2).bstrVal);
        break;
      case ID_TASK_DELETE:
        hr = bCap_TaskDelete(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).bstrVal);
        break;
      case ID_TASK_GETFILENAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_TaskGetFileName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_TASK_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_TaskGetAttribute(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_TASK_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_TaskGetHelp(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_TASK_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_TaskGetName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_TASK_GETTAG:
        hr = bCap_TaskGetTag(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_TASK_PUTTAG:
        hr = bCap_TaskPutTag(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_TASK_GETID:
        hr = bCap_TaskGetID(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_TASK_PUTID:
        hr = bCap_TaskPutID(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_TASK_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_TaskRelease(fd_,
                &vntArgs.at(0).ulVal);
        break;
      // Variable
      case ID_VARIABLE_GETDATETIME:
        hr = bCap_VariableGetDateTime(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_GETVALUE:
        hr = bCap_VariableGetValue(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_PUTVALUE:
        hr = bCap_VariablePutValue(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_VARIABLE_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_VariableGetAttribute(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_VARIABLE_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_VariableGetHelp(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_VARIABLE_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_VariableGetName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_VARIABLE_GETTAG:
        hr = bCap_VariableGetTag(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_PUTTAG:
        hr = bCap_VariablePutTag(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_VARIABLE_GETID:
        hr = bCap_VariableGetID(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_VARIABLE_PUTID:
        hr = bCap_VariablePutID(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_VARIABLE_GETMICROSECOND:
        vntRet->vt = VT_I4;
        hr = bCap_VariableGetMicrosecond(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_VARIABLE_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_VariableRelease(fd_,
                &vntArgs.at(0).ulVal);
        break;
      // Command
      case ID_COMMAND_EXECUTE:
        hr = bCap_CommandExecute(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal,
                vntRet.get());
        break;
      case ID_COMMAND_CANCEL:
        hr = bCap_CommandCancel(fd_,
                vntArgs.at(0).ulVal);
        break;
      case ID_COMMAND_GETTIMEOUT:
        vntRet->vt = VT_I4;
        hr = bCap_CommandGetTimeout(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_COMMAND_PUTTIMEOUT:
        hr = bCap_CommandPutTimeout(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1).lVal);
        break;
      case ID_COMMAND_GETSTATE:
        vntRet->vt = VT_I4;
        hr = bCap_CommandGetState(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_COMMAND_GETPARAMETERS:
        hr = bCap_CommandGetParameters(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_PUTPARAMETERS:
        hr = bCap_CommandPutParameters(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_COMMAND_GETRESULT:
        hr = bCap_CommandGetResult(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_GETATTRIBUTE:
        vntRet->vt = VT_I4;
        hr = bCap_CommandGetAttribute(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_COMMAND_GETHELP:
        vntRet->vt = VT_BSTR;
        hr = bCap_CommandGetHelp(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_COMMAND_GETNAME:
        vntRet->vt = VT_BSTR;
        hr = bCap_CommandGetName(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_COMMAND_GETTAG:
        hr = bCap_CommandGetTag(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_PUTTAG:
        hr = bCap_CommandPutTag(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_COMMAND_GETID:
        hr = bCap_CommandGetID(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_COMMAND_PUTID:
        hr = bCap_CommandPutID(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_COMMAND_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_CommandRelease(fd_,
                &vntArgs.at(0).ulVal);
        break;
      case ID_MESSAGE_REPLY:
        hr = bCap_MessageReply(fd_,
                vntArgs.at(0).ulVal, vntArgs.at(1));
        break;
      case ID_MESSAGE_CLEAR:
        hr = bCap_MessageClear(fd_,
                vntArgs.at(0).ulVal);
        break;
      case ID_MESSAGE_GETDATETIME:
        hr = bCap_MessageGetDateTime(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_MESSAGE_GETDESCRIPTION:
        vntRet->vt = VT_BSTR;
        hr = bCap_MessageGetDescription(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_MESSAGE_GETDESTINATION:
        vntRet->vt = VT_BSTR;
        hr = bCap_MessageGetDestination(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_MESSAGE_GETNUMBER:
        vntRet->vt = VT_I4;
        hr = bCap_MessageGetNumber(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_MESSAGE_GETSERIALNUMBER:
        vntRet->vt = VT_I4;
        hr = bCap_MessageGetSerialNumber(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->lVal);
        break;
      case ID_MESSAGE_GETSOURCE:
        vntRet->vt = VT_BSTR;
        hr = bCap_MessageGetSource(fd_,
                vntArgs.at(0).ulVal,
                &vntRet->bstrVal);
        break;
      case ID_MESSAGE_GETVALUE:
        hr = bCap_MessageGetValue(fd_,
                vntArgs.at(0).ulVal,
                vntRet.get());
        break;
      case ID_MESSAGE_RELEASE:
        tmpKH.second = vntArgs.at(0).ulVal;
        hr = bCap_MessageRelease(fd_,
                &vntArgs.at(0).ulVal);
        break;
    }
  } catch(std::out_of_range&) {
    hr = DISP_E_BADINDEX;
  }

  if(hr == S_OK) {
    if(tmpKH.first > 0) {
      tmpKH.second = vntRet->ulVal;
      vecKH_.push_back(tmpKH);
    }
    else if(tmpKH.second > 0) {
      KeyHandle_Vec::iterator it;
      for(it = vecKH_.begin(); it != vecKH_.end(); it++) {
        if((func_id == it->first)
           && (tmpKH.second == it->second))
        {
          vecKH_.erase(it);
          break;
        }
      }
    }
  }

  return hr;
}

}

