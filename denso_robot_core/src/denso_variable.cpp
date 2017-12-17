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

#include "denso_robot_core/denso_variable.h"

#define NAME_READ  "_Read"
#define NAME_WRITE "_Write"
#define NAME_ID    "_ID"

namespace denso_robot_core {

DensoVariable::DensoVariable(DensoBase* parent,
    Service_Vec& service, Handle_Vec& handle,
    const std::string& name, const int* mode,
    int16_t vt, bool Read, bool Write, bool ID, int Duration)
  : DensoBase(parent, service, handle, name, mode),
    m_vt(vt), m_bRead(Read), m_bWrite(Write), m_bID(ID)
{
  m_Duration    = ros::Duration(Duration / 1000, (Duration % 1000) * 1000);
  m_pubTimePrev = ros::Time::now();
}

DensoVariable::~DensoVariable()
{

}

HRESULT DensoVariable::StartService(ros::NodeHandle& node)
{
  if(*m_mode != 0) {
    return S_FALSE;
  }

  // Message name
  std::string tmpName = m_parent->RosName();
  if(tmpName != "") tmpName.append("/");
  tmpName.append(DensoBase::RosName());

  if(m_bRead) {
    switch(m_vt) {
      case VT_I4:
	m_pubValue = node.advertise<Int32>(
	    tmpName + NAME_READ, MESSAGE_QUEUE);
	break;
      case VT_R4:
	m_pubValue = node.advertise<Float32>(
	    tmpName + NAME_READ, MESSAGE_QUEUE);
	break;
      case VT_R8:
	m_pubValue = node.advertise<Float64>(
	    tmpName + NAME_READ, MESSAGE_QUEUE);
	break;
      case VT_BSTR:
	m_pubValue = node.advertise<String>(
	    tmpName + NAME_READ, MESSAGE_QUEUE);
	break;
      case VT_BOOL:
	m_pubValue = node.advertise<Bool>(
	    tmpName + NAME_READ, MESSAGE_QUEUE);
	break;
      case (VT_ARRAY | VT_R4):
	m_pubValue = node.advertise<Float32MultiArray>(
	    tmpName + NAME_READ, MESSAGE_QUEUE);
        break;
      case (VT_ARRAY | VT_R8):
	m_pubValue = node.advertise<Float64MultiArray>(
	    tmpName + NAME_READ, MESSAGE_QUEUE);
	break;
      default:
        return E_FAIL;
    }
  }

  if(m_bWrite) {
    switch(m_vt) {
      case VT_I4:
	m_subValue = node.subscribe<Int32>(
	    tmpName + NAME_WRITE, MESSAGE_QUEUE,
	    &DensoVariable::Callback_I32, this);
	break;
      case VT_R4:
	m_subValue = node.subscribe<Float32>(
	    tmpName + NAME_WRITE, MESSAGE_QUEUE,
	    &DensoVariable::Callback_F32, this);
	break;
      case VT_R8:
	m_subValue = node.subscribe<Float64>(
	    tmpName + NAME_WRITE, MESSAGE_QUEUE,
	    &DensoVariable::Callback_F64, this);
	break;
      case VT_BSTR:
	m_subValue = node.subscribe<String>(
	    tmpName + NAME_WRITE, MESSAGE_QUEUE,
	    &DensoVariable::Callback_String, this);
	break;
      case VT_BOOL:
	m_subValue = node.subscribe<Bool>(
	    tmpName + NAME_WRITE, MESSAGE_QUEUE,
	    &DensoVariable::Callback_Bool, this);
	break;
      case (VT_ARRAY | VT_R4):
	m_subValue = node.subscribe<Float32MultiArray>(
	    tmpName + NAME_WRITE, MESSAGE_QUEUE,
	    &DensoVariable::Callback_F32Array, this);
        break;
      case (VT_ARRAY | VT_R8):
	m_subValue = node.subscribe<Float64MultiArray>(
	    tmpName + NAME_WRITE, MESSAGE_QUEUE,
	    &DensoVariable::Callback_F64Array, this);
	break;
      default:
        return E_FAIL;
    }
  }

  if(m_bID) {
    m_subID = node.subscribe<Int32>(
        tmpName + NAME_ID, MESSAGE_QUEUE,
        &DensoVariable::Callback_ID, this);
  }

  m_serving = true;

  return S_OK;
}

HRESULT DensoVariable::StopService()
{
  m_mtxSrv.lock();
  m_serving = false;
  m_mtxSrv.unlock();

  m_pubValue.shutdown();
  m_subValue.shutdown();
  m_subID.shutdown();
  return S_OK;
}

bool DensoVariable::Update()
{
  boost::mutex::scoped_lock lockSrv(m_mtxSrv);
  if(!m_serving) return false;

  if(m_bRead) {
    HRESULT hr;

    Int32  varI; Float32 varF; Float64 varD;
    String varS; Bool    varIO;
    Float32MultiArray varFArray;
    Float64MultiArray varDArray;

    uint32_t num;
    float  *pfltval;
    double *pdblval;

    ros::Time pubTimeCur = ros::Time::now();

    if(pubTimeCur - m_pubTimePrev > m_Duration) {
      VARIANT_Ptr vntRet(new VARIANT());
      VariantInit(vntRet.get());

      hr = ExecGetValue(vntRet);
      if(SUCCEEDED(hr)) {
	if(vntRet->vt == m_vt) {
	  switch(m_vt){
	    case VT_I4:
	      varI.data = vntRet->lVal;
	      m_pubValue.publish(varI);
	      break;
	    case VT_R4:
	      varF.data = vntRet->fltVal;
	      m_pubValue.publish(varF);
	      break;
	    case VT_R8:
	      varD.data = vntRet->dblVal;
	      m_pubValue.publish(varD);
	      break;
	    case VT_BSTR:
	      varS.data = ConvertBSTRToString(vntRet->bstrVal);
	      m_pubValue.publish(varS);
	      break;
	    case VT_BOOL:
	      varIO.data = (vntRet->boolVal != VARIANT_FALSE) ? true : false;
	      m_pubValue.publish(varIO);
	      break;
	    case (VT_ARRAY | VT_R4):
	      num = vntRet->parray->rgsabound->cElements;
	      SafeArrayAccessData(vntRet->parray, (void**)&pfltval);
	      varFArray.data.resize(num);
	      std::copy(pfltval, &pfltval[num], varFArray.data.begin());
	      SafeArrayUnaccessData(vntRet->parray);
	      m_pubValue.publish(varFArray);
	      break;
	    case (VT_ARRAY | VT_R8):
	      num = vntRet->parray->rgsabound->cElements;
	      SafeArrayAccessData(vntRet->parray, (void**)&pdblval);
	      varDArray.data.resize(num);
	      std::copy(pdblval, &pdblval[num], varDArray.data.begin());
	      SafeArrayUnaccessData(vntRet->parray);
	      m_pubValue.publish(varDArray);
	      break;
	  }
	}
      }

      m_pubTimePrev = pubTimeCur;
    }
  }

  return true;
}

HRESULT DensoVariable::ExecGetValue(VARIANT_Ptr& value)
{
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntHandle(new VARIANT());

  VariantInit(vntHandle.get());

  vntHandle->vt = VT_UI4;
  vntHandle->ulVal = m_vecHandle[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  return m_vecService[DensoBase::SRV_WATCH]->ExecFunction(
      ID_VARIABLE_GETVALUE, vntArgs, value);
}

HRESULT DensoVariable::ExecPutValue(const VARIANT_Ptr& value)
{
  HRESULT hr;
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntHandle(new VARIANT());
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  vntHandle->vt = VT_UI4;
  vntHandle->ulVal = m_vecHandle[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  vntArgs.push_back(*value.get());

  hr = m_vecService[DensoBase::SRV_WATCH]->ExecFunction(
      ID_VARIABLE_PUTVALUE, vntArgs, vntRet);
  if(SUCCEEDED(hr)) {
      Update();
  }

  return hr;
}

HRESULT DensoVariable::ExecPutID(const int id)
{
  HRESULT hr;
  int argc;
  VARIANT_Vec vntArgs;
  VARIANT_Ptr vntHandle(new VARIANT());
  VARIANT_Ptr vntValue(new VARIANT());
  VARIANT_Ptr vntRet(new VARIANT());

  VariantInit(vntRet.get());

  vntHandle->vt = VT_UI4;
  vntHandle->ulVal = m_vecHandle[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  vntValue->vt = VT_I4;
  vntValue->lVal = id;
  vntArgs.push_back(*vntValue.get());

  hr = m_vecService[DensoBase::SRV_WATCH]->ExecFunction(
      ID_VARIABLE_PUTID, vntArgs, vntRet);
  if(SUCCEEDED(hr)) {
      Update();
  }

  return hr;
}

void DensoVariable::Callback_I32(const Int32::ConstPtr& msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_I4;
  vntVal->lVal = msg->data;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F32(const Float32::ConstPtr& msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_R4;
  vntVal->fltVal = msg->data;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F64(const Float64::ConstPtr& msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_R8;
  vntVal->dblVal = msg->data;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_String(const String::ConstPtr& msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_BSTR;
  vntVal->bstrVal = ConvertStringToBSTR(msg->data);

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_Bool(const Bool::ConstPtr& msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  vntVal->vt = VT_BOOL;
  vntVal->boolVal = (msg->data != 0) ? VARIANT_TRUE : VARIANT_FALSE;

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F32Array(const Float32MultiArray::ConstPtr& msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  float *pval;

  vntVal->vt = (VT_ARRAY | VT_R4);
  vntVal->parray = SafeArrayCreateVector(VT_R4, 0, msg->data.size());

  SafeArrayAccessData(vntVal->parray, (void**)&pval);
  std::copy(msg->data.begin(), msg->data.end(), pval);
  SafeArrayUnaccessData(vntVal->parray);

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_F64Array(const Float64MultiArray::ConstPtr& msg)
{
  VARIANT_Ptr vntVal(new VARIANT());
  double *pval;

  vntVal->vt = (VT_ARRAY | VT_R8);
  vntVal->parray = SafeArrayCreateVector(VT_R8, 0, msg->data.size());

  SafeArrayAccessData(vntVal->parray, (void**)&pval);
  std::copy(msg->data.begin(), msg->data.end(), pval);
  SafeArrayUnaccessData(vntVal->parray);

  ExecPutValue(vntVal);
}

void DensoVariable::Callback_ID(const Int32::ConstPtr &msg)
{
  ExecPutID(msg->data);
}

}
