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

#include "denso_robot_core/denso_robot.hpp"

namespace denso_robot_core
{
enum
{
  NUPOSITION_ = 7,
  NUJOINT_ = 8,
  NUTRANS_ = 10
};

enum
{
  ACT_RESET = -1,
  ACT_NONE = 0,
  ACT_MOVESTRING,
  ACT_MOVEVALUE,
  ACT_DRIVEEXSTRING,
  ACT_DRIVEEXVALUE,
  ACT_DRIVEAEXSTRING,
  ACT_DRIVEAEXVALUE,
};

DensoRobot::DensoRobot(DensoBase* parent, Service_Vec& service, Handle_Vec& handle, const std::string& name,
                       const int* mode)
  : DensoBase(parent, service, handle, name, mode)
  , ArmGroup_(0)
  , curAct_(ACT_RESET)
  , memTimeout_(0)
  , memRetry_(0)
  , tsfmt_(0)
  , timestamp_(0)
  , sendfmt_(0)
  , send_miniio_(0)
  , send_handio_(0)
  , recvfmt_(0)
  , recv_miniio_(0)
  , recv_handio_(0)
  , send_userio_offset_(DEFAULT_MIN_USERIO_OFFSET)
  , send_userio_size_(1)
  , recv_userio_offset_(DEFAULT_MIN_USERIO_OFFSET)
  , recv_userio_size_(1)
{
  tsfmt_ = TSFMT_MILLISEC;

  sendfmt_ = SENDFMT_MINIIO | SENDFMT_HANDIO;

  recvfmt_ = RECVFMT_POSE_PJ | RECVFMT_MINIIO | RECVFMT_HANDIO;
}

DensoRobot::~DensoRobot()
{
}

HRESULT DensoRobot::InitializeBCAP(XMLElement* xmlElem)
{
  return AddVariable(xmlElem);
}

HRESULT DensoRobot::StartService()
{
  // std::string tmpName = DensoBase::RosName();

  if (*mode_ == 0)
  {
    // subSpeed_ = node.subscribe<Float32>(tmpName + NAME_SPEED, MESSAGE_QUEUE, &DensoRobot::Callback_Speed, this);

    // subChangeTool_ = node.subscribe<Int32>(tmpName + NAME_CHANGETOOL, MESSAGE_QUEUE,
    //                                         boost::bind(&DensoRobot::Callback_Change, this, "Tool", _1));

    // subChangeWork_ = node.subscribe<Int32>(tmpName + NAME_CHANGEWORK, MESSAGE_QUEUE,
    //                                         boost::bind(&DensoRobot::Callback_Change, this, "Work", _1));

    // actMoveString_ = boost::make_shared<SimpleActionServer<MoveStringAction> >(
    //     node, DensoBase::RosName() + NAME_MOVESTRING, boost::bind(&DensoRobot::Callback_MoveString, this, _1), false);

    // actMoveString_->registerPreemptCallback(boost::bind(&DensoRobot::Callback_Cancel, this));

    // actMoveString_->start();

    // actMoveValue_ = boost::make_shared<SimpleActionServer<MoveValueAction> >(
    //     node, DensoBase::RosName() + NAME_MOVEVALUE, boost::bind(&DensoRobot::Callback_MoveValue, this, _1), false);

    // actMoveValue_->registerPreemptCallback(boost::bind(&DensoRobot::Callback_Cancel, this));

    // actMoveValue_->start();

    // actDriveExString_ = boost::make_shared<SimpleActionServer<DriveStringAction> >(
    //     node, DensoBase::RosName() + NAME_DRIVEEXSTRING,
    //     boost::bind(&DensoRobot::Callback_DriveString, this, "DriveEx", _1), false);

    // actDriveExString_->registerPreemptCallback(boost::bind(&DensoRobot::Callback_Cancel, this));

    // actDriveExString_->start();

    // actDriveExValue_ = boost::make_shared<SimpleActionServer<DriveValueAction> >(
    //     node, DensoBase::RosName() + NAME_DRIVEEXVALUE,
    //     boost::bind(&DensoRobot::Callback_DriveValue, this, "DriveEx", _1), false);

    // actDriveExValue_->registerPreemptCallback(boost::bind(&DensoRobot::Callback_Cancel, this));

    // actDriveExValue_->start();

    // actDriveAExString_ = boost::make_shared<SimpleActionServer<DriveStringAction> >(
    //     node, DensoBase::RosName() + NAME_DRIVEAEXSTRING,
    //     boost::bind(&DensoRobot::Callback_DriveString, this, "DriveAEx", _1), false);

    // actDriveAExString_->registerPreemptCallback(boost::bind(&DensoRobot::Callback_Cancel, this));

    // actDriveAExString_->start();

    // actDriveAExValue_ = boost::make_shared<SimpleActionServer<DriveValueAction> >(
    //     node, DensoBase::RosName() + NAME_DRIVEAEXVALUE,
    //     boost::bind(&DensoRobot::Callback_DriveValue, this, "DriveAEx", _1), false);

    // actDriveAExValue_->registerPreemptCallback(boost::bind(&DensoRobot::Callback_Cancel, this));

    // actDriveAExValue_->start();

    // subArmGroup_ = node.subscribe<Int32>(tmpName + NAME_ARMGROUP, MESSAGE_QUEUE, &DensoRobot::Callback_ArmGroup, this);
  }

  DensoVariable_Vec::iterator itVar;
  for (itVar = vecVar_.begin(); itVar != vecVar_.end(); itVar++)
  {
    (*itVar)->StartService();
  }

  serving_ = true;

  curAct_ = ACT_NONE;

  return S_OK;
}

HRESULT DensoRobot::StopService()
{
  mtxSrv_.lock();
  serving_ = false;
  mtxSrv_.unlock();

  DensoVariable_Vec::iterator itVar;
  for (itVar = vecVar_.begin(); itVar != vecVar_.end(); itVar++)
  {
    (*itVar)->StopService();
  }

  mtxAct_.lock();
  curAct_ = ACT_RESET;
  mtxAct_.unlock();

  // subSpeed_.shutdown();
  // subChangeTool_.shutdown();
  // subChangeWork_.shutdown();
  // actMoveString_.reset();
  // actMoveValue_.reset();
  // actDriveExString_.reset();
  // actDriveExValue_.reset();
  // actDriveAExString_.reset();
  // actDriveAExValue_.reset();

  return S_OK;
}

bool DensoRobot::Update()
{
  boost::mutex::scoped_lock lockSrv(mtxSrv_);
  if (!serving_)
    return false;

  DensoVariable_Vec::iterator itVar;
  for (itVar = vecVar_.begin(); itVar != vecVar_.end(); itVar++)
  {
    (*itVar)->Update();
  }

  // Action_Feedback();

  return true;
}

HRESULT DensoRobot::ExecTakeArm()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());
  int32_t* pval;

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"TakeArm");
        break;
      case 2:
        vntTmp->vt = (VT_ARRAY | VT_I4);
        vntTmp->parray = SafeArrayCreateVector(VT_I4, 0, 2);
        SafeArrayAccessData(vntTmp->parray, (void**)&pval);
        pval[0] = ArmGroup_;  // Arm group
        pval[1] = 1L;          // Keep
        SafeArrayUnaccessData(vntTmp->parray);
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

HRESULT DensoRobot::ExecGiveArm()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"GiveArm");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

void DensoRobot::ChangeArmGroup(int number)
{
  ArmGroup_ = number;
}

// void DensoRobot::Callback_ArmGroup(const Int32::ConstPtr& msg)
// {
//   ChangeArmGroup(msg->data);
// }

HRESULT DensoRobot::get_Variable(const std::string& name, DensoVariable_Ptr* var)
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

HRESULT DensoRobot::AddVariable(const std::string& name)
{
  return DensoBase::AddVariable(ID_ROBOT_GETVARIABLE, name, vecVar_);
}

HRESULT DensoRobot::AddVariable(XMLElement* xmlElem)
{
  HRESULT hr = S_OK;
  XMLElement* xmlVar;

  for (xmlVar = xmlElem->FirstChildElement(DensoVariable::XML_VARIABLE_NAME); xmlVar != NULL;
       xmlVar = xmlVar->NextSiblingElement(DensoVariable::XML_VARIABLE_NAME))
  {
    hr = DensoBase::AddVariable(ID_ROBOT_GETVARIABLE, xmlVar, vecVar_);
    if (FAILED(hr))
      break;
  }

  return hr;
}

HRESULT DensoRobot::ExecMove(int comp, const VARIANT_Ptr& pose, const std::string& option)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr))
  {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_ROBOT_MOVE_ARGS; argc++)
    {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc)
      {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_I4;
          vntTmp->lVal = comp;
          break;
        case 2:
          VariantCopy(vntTmp.get(), pose.get());
          break;
        case 3:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = ConvertStringToBSTR(option);
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_MOVE, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecDrive(const std::string& name, const VARIANT_Ptr& option)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr))
  {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (int argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++)
    {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc)
      {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = ConvertStringToBSTR(name);
          break;
        case 2:
          VariantCopy(vntTmp.get(), option.get());
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecSpeed(float value)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr))
  {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_ROBOT_SPEED_ARGS; argc++)
    {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc)
      {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_I4;
          vntTmp->lVal = -1;
          break;
        case 2:
          vntTmp->vt = VT_R4;
          vntTmp->fltVal = value;
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_SPEED, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecChange(const std::string& value)
{
  HRESULT hr;

  hr = ExecTakeArm();
  if (SUCCEEDED(hr))
  {
    int argc;
    VARIANT_Vec vntArgs;
    VARIANT_Ptr vntRet(new VARIANT());

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_ROBOT_CHANGE_ARGS; argc++)
    {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc)
      {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
          break;
        case 1:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = ConvertStringToBSTR(value);
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_CHANGE, vntArgs, vntRet);

    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecHalt()
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  for (argc = 0; argc < BCAP_ROBOT_HALT_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());

    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = vecHandle_[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_ROBOT_HALT, vntArgs, vntRet);
}

HRESULT DensoRobot::ExecCurJnt(std::vector<double>& pose)
{
  HRESULT hr;

  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = vecHandle_[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"HighCurJntEx");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);

  if (SUCCEEDED(hr) && (vntRet->vt == (VT_ARRAY | VT_R8)))
  {
    uint32_t num;
    double* pdblval;

    num = vntRet->parray->rgsabound->cElements;
    SafeArrayAccessData(vntRet->parray, (void**)&pdblval);
    pose.resize(num - 1);
    std::copy(&pdblval[1], &pdblval[num], pose.begin());
    SafeArrayUnaccessData(vntRet->parray);
  }

  return hr;
}

HRESULT DensoRobot::ExecSlaveMove(const std::vector<double>& pose, std::vector<double>& joint)
{
  HRESULT hr = S_OK;
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"slvMove");
        break;
      case 2:
        hr = CreateSendParameter(pose, vntTmp, send_miniio_, send_handio_, recv_userio_offset_, recv_userio_size_,
                                 send_userio_offset_, send_userio_size_, send_userio_);
        if (FAILED(hr))
          return hr;
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
  if (SUCCEEDED(hr))
  {
    HRESULT hrTmp = ParseRecvParameter(vntRet, position_, joint_, trans_, recv_miniio_, recv_handio_, timestamp_,
                                       recv_userio_, current_);

    joint = joint_;

    if (FAILED(hrTmp))
    {
      hr = hrTmp;
    }
  }

  return hr;
}

int DensoRobot::get_SendFormat() const
{
  return sendfmt_;
}

/**
 * DensoRobot::put_SendFormat() has been deprecated.
 */
void DensoRobot::put_SendFormat(int format)
{
  // // ROS_WARN("DensoRobot::put_SendFormat() has been deprecated.");
  switch (format)
  {
    case SENDFMT_NONE:
    case SENDFMT_HANDIO:
    case SENDFMT_MINIIO:
    case SENDFMT_HANDIO | SENDFMT_MINIIO:
    case SENDFMT_USERIO:
    case SENDFMT_USERIO | SENDFMT_HANDIO:
      sendfmt_ = format;
      break;
    default:
      // // ROS_WARN("Failed to put_SendFormat.");
      break;
  }
}

void DensoRobot::set_SendFormat(int format)
{
  sendfmt_ = format;
}

int DensoRobot::get_RecvFormat() const
{
  return recvfmt_;
}

/**
 * DensoRobot::put_RecvFormat() has been deprecated.
 */
void DensoRobot::put_RecvFormat(int format)
{
  // // ROS_WARN("DensoRobot::put_RecvFormat() has been deprecated.");
  int pose = format & RECVFMT_POSE;
  if ((RECVFMT_NONE <= pose) && (pose <= RECVFMT_POSE_TJ))
  {
    switch (format & ~RECVFMT_POSE)
    {
      case RECVFMT_NONE:
      case RECVFMT_TIME:
      case RECVFMT_HANDIO:
      case RECVFMT_CURRENT:
      case RECVFMT_MINIIO:
      case RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_HANDIO:
      case RECVFMT_TIME | RECVFMT_MINIIO:
      case RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_USERIO:
      case RECVFMT_HANDIO | RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_HANDIO | RECVFMT_USERIO:
      case RECVFMT_CURRENT | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_MINIIO:
      case RECVFMT_CURRENT | RECVFMT_HANDIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_HANDIO:
      case RECVFMT_CURRENT | RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_HANDIO | RECVFMT_MINIIO:
      case RECVFMT_CURRENT | RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_USERIO:
      case RECVFMT_CURRENT | RECVFMT_MINIIO | RECVFMT_USERIO:
      case RECVFMT_TIME | RECVFMT_CURRENT | RECVFMT_HANDIO | RECVFMT_USERIO:
        recvfmt_ = format;
        break;
      default:
        // // ROS_WARN("Failed to put_RecvFormat.");
        break;
    }
  }
  else
  {
    // // ROS_WARN("Failed to put_RecvFormat.");
  }
}

void DensoRobot::set_RecvFormat(int format)
{
  recvfmt_ = format;
}

int DensoRobot::get_TimeFormat() const
{
  return tsfmt_;
}

void DensoRobot::put_TimeFormat(int format)
{
  if ((format == TSFMT_MILLISEC) || (format == TSFMT_MICROSEC))
  {
    tsfmt_ = format;
  }
  else
  {
    // ROS_WARN("Failed to put_TimeFormat.");
  }
}

unsigned int DensoRobot::get_MiniIO() const
{
  return recv_miniio_;
}

void DensoRobot::put_MiniIO(unsigned int value)
{
  send_miniio_ = value;
}

unsigned int DensoRobot::get_HandIO() const
{
  return recv_handio_;
}

void DensoRobot::put_HandIO(unsigned int value)
{
  send_handio_ = value;
}

void DensoRobot::put_SendUserIO(const UserIO& value)
{
  if (value.offset < DEFAULT_MIN_USERIO_OFFSET)
  {
    // ROS_WARN("User I/O offset has to be greater than %d.", UserIO::MIN_USERIO_OFFSET - 1);
    return;
  }

  if (value.offset % DEFAULT_USERIO_ALIGNMENT)
  {
    // ROS_WARN("User I/O offset has to be multiple of %d.", DEFAULT_USERIO_ALIGNMENT);
    return;
  }

  if (value.size <= 0)
  {
    // ROS_WARN("User I/O size has to be greater than 0.");
    return;
  }

  if (value.size < value.value.size())
  {
    // ROS_WARN("User I/O size has to be equal or greater than the value length.");
    return;
  }

  send_userio_offset_ = value.offset;
  send_userio_size_ = value.size;
  send_userio_ = value.value;
}

void DensoRobot::put_RecvUserIO(const UserIO& value)
{
  if (value.offset < DEFAULT_MIN_USERIO_OFFSET)
  {
    // ROS_WARN("User I/O offset has to be greater than %d.", UserIO::MIN_USERIO_OFFSET - 1);
    return;
  }

  if (value.offset % DEFAULT_USERIO_ALIGNMENT)
  {
    // ROS_WARN("User I/O offset has to be multiple of %d.", DEFAULT_USERIO_ALIGNMENT);
    return;
  }

  if (value.size <= 0)
  {
    // ROS_WARN("User I/O size has to be greater than 0.");
    return;
  }

  recv_userio_offset_ = value.offset;
  recv_userio_size_ = value.size;
}

void DensoRobot::get_RecvUserIO(UserIO& value) const
{
  value.offset = recv_userio_offset_;
  value.size = recv_userio_.size();
  value.value = recv_userio_;
}

void DensoRobot::get_Current(std::vector<double>& current) const
{
  current = current_;
}

int DensoRobot::get_Timestamp() const
{
  return timestamp_;
}

HRESULT DensoRobot::CreatePoseData(const PoseData& pose, VARIANT& vnt)
{
  uint32_t i, j;
  uint32_t num = 3 + (((pose.exjoints.mode != 0) && (pose.exjoints.joints.size() > 0)) ? 1 : 0);
  float* pfltval;
  VARIANT* pvntval;

  vnt.vt = (VT_ARRAY | VT_VARIANT);
  vnt.parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

  SafeArrayAccessData(vnt.parray, (void**)&pvntval);

  for (i = 0; i < num; i++)
  {
    switch (i)
    {
      case 0:
        pvntval[i].vt = (VT_ARRAY | VT_R4);
        pvntval[i].parray = SafeArrayCreateVector(VT_R4, 0, pose.value.size());
        SafeArrayAccessData(pvntval[i].parray, (void**)&pfltval);
        std::copy(pose.value.begin(), pose.value.end(), pfltval);
        SafeArrayUnaccessData(pvntval[i].parray);
        break;
      case 1:
        pvntval[i].vt = VT_I4;
        pvntval[i].lVal = pose.type;
        break;
      case 2:
        pvntval[i].vt = VT_I4;
        pvntval[i].lVal = pose.pass;
        break;
      case 3:
        CreateExJoints(pose.exjoints, pvntval[i]);
        break;
    }
  }

  SafeArrayUnaccessData(vnt.parray);

  return S_OK;
}

HRESULT DensoRobot::CreateExJoints(const ExJoints& exjoints, VARIANT& vnt)
{
  uint32_t i, j;
  uint32_t num = 1 + exjoints.joints.size();
  VARIANT *pvntval, *pjntval;

  vnt.vt = (VT_ARRAY | VT_VARIANT);
  vnt.parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

  SafeArrayAccessData(vnt.parray, (void**)&pvntval);

  for (i = 0; i < num; i++)
  {
    if (i == 0)
    {
      pvntval[0].vt = VT_I4;
      pvntval[0].lVal = exjoints.mode;
    }
    else
    {
      pvntval[i].vt = (VT_ARRAY | VT_VARIANT);
      pvntval[i].parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);
      SafeArrayAccessData(pvntval[i].parray, (void**)&pjntval);
      pjntval[0].vt = VT_I4;
      pjntval[0].lVal = exjoints.joints.at(i - 1).joint;
      pjntval[1].vt = VT_R4;
      pjntval[1].fltVal = exjoints.joints.at(i - 1).value;
      SafeArrayUnaccessData(pvntval[i].parray);
    }
  }

  SafeArrayUnaccessData(vnt.parray);

  return S_OK;
}

HRESULT DensoRobot::ChangeMode(int mode)
{
  HRESULT hr = S_OK;

  if (*mode_ == 0)
  {
    // Change to slave mode
    if (mode != 0)
    {
      hr = ExecSlaveMode("slvSendFormat", sendfmt_);
      if (FAILED(hr))
      {
        // ROS_ERROR("Invalid argument value (send_format = 0x%x)", sendfmt_);
        return hr;
      }
      hr = ExecSlaveMode("slvRecvFormat", recvfmt_, tsfmt_);
      if (FAILED(hr))
      {
        // ROS_ERROR("Invalid argument value (recv_format = 0x%x)", recvfmt_);
        return hr;
      }
      hr = ExecTakeArm();
      if (FAILED(hr))
        return hr;

      hr = ExecSlaveMode("slvChangeMode", mode);
      if (FAILED(hr))
        return hr;

      memTimeout_ = vecService_[DensoBase::SRV_ACT]->get_Timeout();
      memRetry_ = vecService_[DensoBase::SRV_ACT]->get_Retry();
      if (mode & DensoRobot::SLVMODE_SYNC_WAIT)
      {
        vecService_[DensoBase::SRV_ACT]->put_Timeout(this->SLVMODE_TIMEOUT_SYNC);
      }
      else
      {
        vecService_[DensoBase::SRV_ACT]->put_Timeout(this->SLVMODE_TIMEOUT_ASYNC);
      }
      // ROS_INFO("bcap-slave timeout changed to %d msec [mode: 0x%X]", vecService_[DensoBase::SRV_ACT]->get_Timeout(),
              //  mode);
      vecService_[DensoBase::SRV_ACT]->put_Retry(3);
    }
  }
  else
  {
    vecService_[DensoBase::SRV_ACT]->put_Timeout(memTimeout_);
    vecService_[DensoBase::SRV_ACT]->put_Retry(memRetry_);

    hr = ExecSlaveMode("slvChangeMode", mode);
    ExecGiveArm();
  }

  return hr;
}

HRESULT DensoRobot::ExecSlaveMode(const std::string& name, int32_t format, int32_t option)
{
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntRet(new VARIANT());
  int32_t* pval;

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_ROBOT_EXECUTE_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    switch (argc)
    {
      case 0:
        vntTmp->vt = VT_UI4;
        vntTmp->ulVal = vecHandle_[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = ConvertStringToBSTR(name);
        break;
      case 2:
        if (option == 0)
        {
          vntTmp->vt = VT_I4;
          vntTmp->lVal = format;
        }
        else
        {
          vntTmp->vt = (VT_ARRAY | VT_I4);
          vntTmp->parray = SafeArrayCreateVector(VT_I4, 0, 2);
          SafeArrayAccessData(vntTmp->parray, (void**)&pval);
          pval[0] = format;
          pval[1] = option;
          SafeArrayUnaccessData(vntTmp->parray);
        }
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return vecService_[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

HRESULT DensoRobot::CreateSendParameter(const std::vector<double>& pose, VARIANT_Ptr& send, const int miniio,
                                        const int handio, const int recv_userio_offset, const int recv_userio_size,
                                        const int send_userio_offset, const int send_userio_size,
                                        const std::vector<uint8_t>& send_userio)
{
  int type = *mode_ & SLVMODE_POSE;

  // Check pose type
  int joints = 0;
  switch (type)
  {
    case SLVMODE_POSE_P:
      joints = NUPOSITION_;
      break;
    case SLVMODE_POSE_J:
      joints = NUJOINT_;
      break;
    case SLVMODE_POSE_T:
      joints = NUTRANS_;
      break;
    default:
      return E_FAIL;
  }

  // if(joints < pose.size()) {
  //  return E_FAIL;
  //}

  // Check send format
  bool send_hio, send_mio, send_uio, recv_uio;
  send_hio = sendfmt_ & SENDFMT_HANDIO;
  send_mio = sendfmt_ & SENDFMT_MINIIO;
  send_uio = sendfmt_ & SENDFMT_USERIO;

  if (send_uio)
  {
    if (send_userio_size < send_userio.size())
    {
      return E_FAIL;
    }
  }

  // Check receive format
  recv_uio = recvfmt_ & RECVFMT_USERIO;

  // Number of arguments
  int num = 1 + send_hio + send_mio + 3 * send_uio + 2 * recv_uio;

  VARIANT* pvnt;
  double* pdbl;
  uint8_t* pbool;

  // Number of joints + option
  joints += 1;
  if (num == 1)
  {
    // Pose only
    send->vt = (VT_ARRAY | VT_R8);
    send->parray = SafeArrayCreateVector(VT_R8, 0, joints);
    SafeArrayAccessData(send->parray, (void**)&pdbl);
    memset(pdbl, 0, joints * sizeof(double));
    std::copy(pose.begin(), pose.end(), pdbl);
    SafeArrayUnaccessData(send->parray);
  }
  else
  {
    send->vt = (VT_ARRAY | VT_VARIANT);
    send->parray = SafeArrayCreateVector(VT_VARIANT, 0, num);

    SafeArrayAccessData(send->parray, (void**)&pvnt);

    int offset = 0;

    // Pose
    {
      pvnt[offset].vt = (VT_ARRAY | VT_R8);
      pvnt[offset].parray = SafeArrayCreateVector(VT_R8, 0, joints);
      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      memset(pdbl, 0, joints * sizeof(double));
      std::copy(pose.begin(), pose.end(), pdbl);
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Mini I/O
    if (send_mio)
    {
      pvnt[offset].vt = VT_I4;
      pvnt[offset].lVal = miniio;

      offset++;
    }

    // Send User I/O
    if (send_uio)
    {
      pvnt[offset + 0].vt = VT_I4;
      pvnt[offset + 0].lVal = send_userio_offset;

      pvnt[offset + 1].vt = VT_I4;
      pvnt[offset + 1].lVal = send_userio_size * DEFAULT_USERIO_ALIGNMENT;

      pvnt[offset + 2].vt = (VT_ARRAY | VT_UI1);
      pvnt[offset + 2].parray = SafeArrayCreateVector(VT_UI1, 0, send_userio_size);
      SafeArrayAccessData(pvnt[offset + 2].parray, (void**)&pbool);
      memset(pbool, 0, send_userio_size * sizeof(uint8_t));
      std::copy(send_userio.begin(), send_userio.end(), pbool);
      SafeArrayUnaccessData(pvnt[offset + 2].parray);

      offset += 3;
    }

    // Receive User I/O
    if (recv_uio)
    {
      pvnt[offset + 0].vt = VT_I4;
      pvnt[offset + 0].lVal = recv_userio_offset;

      pvnt[offset + 1].vt = VT_I4;
      pvnt[offset + 1].lVal = recv_userio_size * DEFAULT_USERIO_ALIGNMENT;

      offset += 2;
    }

    // Hand I/O
    if (send_hio)
    {
      pvnt[offset].vt = VT_I4;
      pvnt[offset].lVal = handio;

      offset++;
    }

    SafeArrayUnaccessData(send->parray);
  }

  return S_OK;
}

HRESULT DensoRobot::ParseRecvParameter(const VARIANT_Ptr& recv, std::vector<double>& position,
                                       std::vector<double>& joint, std::vector<double>& trans, int& miniio, int& handio,
                                       int& timestamp, std::vector<uint8_t>& recv_userio, std::vector<double>& current)
{
  int type = recvfmt_ & SLVMODE_POSE;

  // Check pose type
  int j = 0, j1 = 0, j2 = 0, joints = 0;
  std::vector<double>*pose1 = NULL, *pose2 = NULL;

  switch (type)
  {
    case RECVFMT_POSE_P:
      j1 = NUPOSITION_;
      pose1 = &position;
      break;
    case RECVFMT_POSE_J:
      j1 = NUJOINT_;
      pose1 = &joint;
      break;
    case RECVFMT_POSE_T:
      j1 = NUTRANS_;
      pose1 = &trans;
      break;
    case RECVFMT_POSE_PJ:
      j1 = NUPOSITION_;
      j2 = NUJOINT_;
      pose1 = &position;
      pose2 = &joint;
      break;
    case RECVFMT_POSE_TJ:
      j1 = NUTRANS_;
      j2 = NUJOINT_;
      pose1 = &trans;
      pose2 = &joint;
      break;
    default:
      return E_FAIL;
  }

  joints = j1 + j2;

  // Check receive format
  bool recv_ts, recv_hio, recv_mio, recv_uio, recv_crt;
  recv_ts = recvfmt_ & RECVFMT_TIME;
  recv_hio = recvfmt_ & RECVFMT_HANDIO;
  recv_mio = recvfmt_ & RECVFMT_MINIIO;
  recv_uio = recvfmt_ & RECVFMT_USERIO;
  recv_crt = recvfmt_ & RECVFMT_CURRENT;

  // Number of arguments
  int num = 1 + recv_ts + recv_hio + recv_mio + recv_uio + recv_crt;

  HRESULT hr = S_OK;
  VARIANT* pvnt;
  double* pdbl;
  uint8_t* pbool;

  if (recv->vt == (VT_ARRAY | VT_R8))
  {
    if (joints != recv->parray->rgsabound->cElements)
    {
      return E_FAIL;
    }

    // Pose only
    SafeArrayAccessData(recv->parray, (void**)&pdbl);
    pose1->resize(j1);
    std::copy(pdbl, &pdbl[j1], pose1->begin());
    if (pose2 != NULL)
    {
      pose2->resize(j2);
      std::copy(&pdbl[j1], &pdbl[joints], pose2->begin());
    }
    SafeArrayUnaccessData(recv->parray);
  }
  else if (recv->vt == (VT_ARRAY | VT_VARIANT))
  {
    if (num != recv->parray->rgsabound->cElements)
    {
      return E_FAIL;
    }

    SafeArrayAccessData(recv->parray, (void**)&pvnt);

    int offset = 0;

    // Timestamp
    if (recv_ts)
    {
      if (pvnt[offset].vt != VT_I4)
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      timestamp = pvnt[offset].lVal;

      offset++;
    }

    // Pose
    {
      if ((pvnt[offset].vt != (VT_ARRAY | VT_R8)) || (joints != pvnt[offset].parray->rgsabound->cElements))
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      pose1->resize(j1);
      std::copy(pdbl, &pdbl[j1], pose1->begin());
      if (pose2 != NULL)
      {
        pose2->resize(j2);
        std::copy(&pdbl[j1], &pdbl[joints], pose2->begin());
      }
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Mini I/O
    if (recv_mio)
    {
      if (pvnt[offset].vt != VT_I4)
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      miniio = pvnt[offset].lVal;

      offset++;
    }

    // User I/O
    if (recv_uio)
    {
      if (pvnt[offset].vt != (VT_ARRAY | VT_UI1))
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pbool);
      recv_userio.resize(pvnt[offset].parray->rgsabound->cElements);
      std::copy(pbool, &pbool[pvnt[offset].parray->rgsabound->cElements], recv_userio.begin());
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

    // Hand I/O
    if (recv_hio)
    {
      if (pvnt[offset].vt != VT_I4)
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      handio = pvnt[offset].lVal;

      offset++;
    }

    // Current
    if (recv_crt)
    {
      if ((pvnt[offset].vt != (VT_ARRAY | VT_R8)) || (8 != pvnt[offset].parray->rgsabound->cElements))
      {
        hr = E_FAIL;
        goto exit_proc;
      }

      SafeArrayAccessData(pvnt[offset].parray, (void**)&pdbl);
      current.resize(8);
      std::copy(pdbl, &pdbl[8], current.begin());
      SafeArrayUnaccessData(pvnt[offset].parray);

      offset++;
    }

  exit_proc:
    SafeArrayUnaccessData(recv->parray);
  }
  else
  {
    return E_FAIL;
  }

  return hr;
}

// void DensoRobot::Callback_MoveString(const MoveStringGoalConstPtr& goal)
// {
//   HRESULT hr;
//   MoveStringResult res;

//   // Set current action
//   boost::mutex::scoped_lock lockAct(mtxAct_);
//   if (curAct_ != ACT_NONE)
//   {
//     if (curAct_ != ACT_RESET)
//     {
//       res.HRESULT = E_FAIL;
//       actMoveString_->setAborted(res);
//     }
//     return;
//   }
//   curAct_ = ACT_MOVESTRING;
//   lockAct.unlock();

//   VARIANT_Ptr vntPose(new VARIANT());
//   VariantInit(vntPose.get());
//   vntPose->vt = VT_BSTR;
//   vntPose->bstrVal = ConvertStringToBSTR(goal->pose);

//   hr = ExecMove(goal->comp, vntPose, goal->option);

//   // Reset current action
//   mtxAct_.lock();
//   if (curAct_ == ACT_MOVESTRING)
//   {
//     if (SUCCEEDED(hr))
//     {
//       res.HRESULT = S_OK;
//       actMoveString_->setSucceeded(res);
//     }
//     else
//     {
//       res.HRESULT = hr;
//       actMoveString_->setAborted(res);
//     }
//     curAct_ = ACT_NONE;
//   }
//   mtxAct_.unlock();
// }

// void DensoRobot::Callback_MoveValue(const MoveValueGoalConstPtr& goal)
// {
//   HRESULT hr;
//   MoveValueResult res;

//   // Set current action
//   boost::mutex::scoped_lock lockAct(mtxAct_);
//   if (curAct_ != ACT_NONE)
//   {
//     if (curAct_ != ACT_RESET)
//     {
//       res.HRESULT = E_FAIL;
//       actMoveValue_->setAborted(res);
//     }
//     return;
//   }
//   curAct_ = ACT_MOVEVALUE;
//   lockAct.unlock();

//   VARIANT_Ptr vntPose(new VARIANT());
//   VariantInit(vntPose.get());
//   CreatePoseData(goal->pose, *vntPose.get());

//   hr = ExecMove(goal->comp, vntPose, goal->option);

//   // Reset current action
//   mtxAct_.lock();
//   if (curAct_ == ACT_MOVEVALUE)
//   {
//     if (SUCCEEDED(hr))
//     {
//       res.HRESULT = S_OK;
//       actMoveValue_->setSucceeded(res);
//     }
//     else
//     {
//       res.HRESULT = hr;
//       actMoveValue_->setAborted(res);
//     }
//     curAct_ = ACT_NONE;
//   }
//   mtxAct_.unlock();
// }

// void DensoRobot::Callback_DriveString(const std::string& name, const DriveStringGoalConstPtr& goal)
// {
//   HRESULT hr;
//   DriveStringResult res;
//   BSTR* pbstr;

//   int act = 0;
//   boost::shared_ptr<SimpleActionServer<DriveStringAction> > actSvr;

//   if (!name.compare("DriveEx"))
//   {
//     act = ACT_DRIVEEXSTRING;
//     actSvr = actDriveExString_;
//   }
//   else if (!name.compare("DriveAEx"))
//   {
//     act = ACT_DRIVEAEXSTRING;
//     actSvr = actDriveAExString_;
//   }
//   else
//     return;

//   // Set current action
//   boost::mutex::scoped_lock lockAct(mtxAct_);
//   if (curAct_ != ACT_NONE)
//   {
//     if (curAct_ != ACT_RESET)
//     {
//       res.HRESULT = E_FAIL;
//       actSvr->setAborted(res);
//     }
//     return;
//   }
//   curAct_ = act;
//   lockAct.unlock();

//   VARIANT_Ptr vntOpt(new VARIANT());
//   VariantInit(vntOpt.get());
//   vntOpt->vt = (VT_ARRAY | VT_BSTR);
//   vntOpt->parray = SafeArrayCreateVector(VT_BSTR, 0, 2);
//   SafeArrayAccessData(vntOpt->parray, (void**)&pbstr);
//   pbstr[0] = ConvertStringToBSTR(goal->pose);
//   pbstr[1] = ConvertStringToBSTR(goal->option);
//   SafeArrayUnaccessData(vntOpt->parray);

//   hr = ExecDrive(name, vntOpt);

//   // Reset current action
//   mtxAct_.lock();
//   if (curAct_ == act)
//   {
//     if (SUCCEEDED(hr))
//     {
//       res.HRESULT = S_OK;
//       actSvr->setSucceeded(res);
//     }
//     else
//     {
//       res.HRESULT = hr;
//       actSvr->setAborted(res);
//     }
//     curAct_ = ACT_NONE;
//   }
//   mtxAct_.unlock();
// }

// void DensoRobot::Callback_DriveValue(const std::string& name, const DriveValueGoalConstPtr& goal)
// {
//   HRESULT hr;
//   DriveValueResult res;
//   VARIANT *pvntval, *pvntjnt;

//   int act = 0;
//   boost::shared_ptr<SimpleActionServer<DriveValueAction> > actSvr;

//   if (!name.compare("DriveEx"))
//   {
//     act = ACT_DRIVEEXVALUE;
//     actSvr = actDriveExValue_;
//   }
//   else if (!name.compare("DriveAEx"))
//   {
//     act = ACT_DRIVEAEXVALUE;
//     actSvr = actDriveAExValue_;
//   }
//   else
//     return;

//   // Set current action
//   boost::mutex::scoped_lock lockAct(mtxAct_);
//   if (curAct_ != ACT_NONE)
//   {
//     if (curAct_ != ACT_RESET)
//     {
//       res.HRESULT = E_FAIL;
//       actSvr->setAborted(res);
//     }
//     return;
//   }
//   curAct_ = act;
//   lockAct.unlock();

//   VARIANT_Ptr vntOpt(new VARIANT());
//   VariantInit(vntOpt.get());

//   vntOpt->vt = (VT_ARRAY | VT_VARIANT);
//   vntOpt->parray = SafeArrayCreateVector(VT_VARIANT, 0, 2);

//   SafeArrayAccessData(vntOpt->parray, (void**)&pvntval);

//   pvntval[0].vt = (VT_ARRAY | VT_VARIANT);
//   pvntval[0].parray = SafeArrayCreateVector(VT_VARIANT, 0, goal->pose.size());

//   SafeArrayAccessData(pvntval[0].parray, (void**)&pvntjnt);

//   for (int i = 0; i < goal->pose.size(); i++)
//   {
//     PoseData pd;
//     pd.value.push_back(goal->pose.at(i).joint);
//     pd.value.push_back(goal->pose.at(i).value);
//     pd.type = -1;
//     pd.pass = (i == 0) ? goal->pass : 0;
//     CreatePoseData(pd, pvntjnt[i]);
//   }

//   SafeArrayUnaccessData(pvntval[0].parray);

//   pvntval[1].vt = VT_BSTR;
//   pvntval[1].bstrVal = ConvertStringToBSTR(goal->option);

//   SafeArrayUnaccessData(vntOpt->parray);

//   hr = ExecDrive(name, vntOpt);

//   // Reset current action
//   mtxAct_.lock();
//   if (curAct_ == act)
//   {
//     if (SUCCEEDED(hr))
//     {
//       res.HRESULT = S_OK;
//       actSvr->setSucceeded(res);
//     }
//     else
//     {
//       res.HRESULT = hr;
//       actSvr->setAborted(res);
//     }
//     curAct_ = ACT_NONE;
//   }
//   mtxAct_.unlock();
// }

// void DensoRobot::Callback_Speed(const Float32::ConstPtr& msg)
// {
//   ExecSpeed(msg->data);
// }

// void DensoRobot::Callback_Change(const std::string& name, const Int32::ConstPtr& msg)
// {
//   std::stringstream ss;
//   ss << name << msg->data;
//   ExecChange(ss.str());
// }

// void DensoRobot::Callback_Cancel()
// {
//   boost::mutex::scoped_lock lockAct(mtxAct_);

//   if (curAct_ > ACT_NONE)
//   {
//     ExecHalt();

//     switch (curAct_)
//     {
//       case ACT_MOVESTRING:
//         actMoveString_->setPreempted();
//         break;
//       case ACT_MOVEVALUE:
//         actMoveValue_->setPreempted();
//         break;
//       case ACT_DRIVEEXSTRING:
//         actDriveExString_->setPreempted();
//         break;
//       case ACT_DRIVEEXVALUE:
//         actDriveExValue_->setPreempted();
//         break;
//       case ACT_DRIVEAEXSTRING:
//         actDriveAExString_->setPreempted();
//         break;
//       case ACT_DRIVEAEXVALUE:
//         actDriveAExValue_->setPreempted();
//         break;
//     }

//     curAct_ = ACT_NONE;
//   }
// }

// void DensoRobot::Action_Feedback()
// {
//   boost::mutex::scoped_lock lockAct(mtxAct_);

//   if (curAct_ > ACT_NONE)
//   {
//     HRESULT hr;
//     std::vector<double> pose;

//     MoveStringFeedback fbMvStr;
//     MoveValueFeedback fbMvVal;
//     DriveStringFeedback fbDrvStr;
//     DriveValueFeedback fbDrvVal;

//     hr = ExecCurJnt(pose);

//     if (SUCCEEDED(hr))
//     {
//       switch (curAct_)
//       {
//         case ACT_MOVESTRING:
//           fbMvStr.pose = pose;
//           actMoveString_->publishFeedback(fbMvStr);
//           break;
//         case ACT_MOVEVALUE:
//           fbMvVal.pose = pose;
//           actMoveValue_->publishFeedback(fbMvVal);
//           break;
//         case ACT_DRIVEEXSTRING:
//           fbDrvStr.pose = pose;
//           actDriveExString_->publishFeedback(fbDrvStr);
//           break;
//         case ACT_DRIVEEXVALUE:
//           fbDrvVal.pose = pose;
//           actDriveExValue_->publishFeedback(fbDrvVal);
//           break;
//         case ACT_DRIVEAEXSTRING:
//           fbDrvStr.pose = pose;
//           actDriveAExString_->publishFeedback(fbDrvStr);
//           break;
//         case ACT_DRIVEAEXVALUE:
//           fbDrvVal.pose = pose;
//           actDriveAExValue_->publishFeedback(fbDrvVal);
//           break;
//       }
//     }
//   }
// }

}  // namespace denso_robot_core
