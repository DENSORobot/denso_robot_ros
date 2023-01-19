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

#include "denso_robot_core/denso_variable.hpp"

namespace denso2
{
DensoVariable::DensoVariable(DensoBase* parent, Service_Vec& service, Handle_Vec& handle, const std::string& name,
                             const int* mode, int16_t vt, bool Read, bool Write, bool ID, int Duration)
  : DensoBase(parent, service, handle, name, mode), m_vt(vt), bRead_(Read), bWrite_(Write), bID_(ID)
{
  // m_Duration = ros::Duration(Duration / 1000, (Duration % 1000) * 1000);
  // m_pubTimePrev = ros::Time::now();
}

DensoVariable::~DensoVariable()
{
}

HRESULT DensoVariable::StartService()
{
  if (*mode_ != 0)
  {
    return S_FALSE;
  }

  if (bRead_)
  {
    switch (m_vt)
    {
     
    }
  }

  if (bWrite_)
  {
    
  }

  serving_ = true;

  return S_OK;
}

HRESULT DensoVariable::StopService()
{
  mtxSrv_.lock();
  serving_ = false;
  mtxSrv_.unlock();

  return S_OK;
}

bool DensoVariable::Update()
{
  boost::mutex::scoped_lock lockSrv(mtxSrv_);
  if (!serving_)
    return false;

  if (bRead_)
  {
    HRESULT hr;
    int32_t varI;
    // Int32 varI;
    float varF;
    double varD;
    std::string varS;
    bool varIO;
    std::vector<float> varFArray;
    std::vector<double> varDArray;

    uint32_t num;
    float* pfltval;
    double* pdblval;


    {
      VARIANT_Ptr vntRet(new VARIANT());
      VariantInit(vntRet.get());

      hr = ExecGetValue(vntRet);
      if (SUCCEEDED(hr))
      {
        if (vntRet->vt == m_vt)
        {
          switch (m_vt)
          {
            case VT_I4:
              varI = vntRet->lVal;
              break;
            case VT_R4:
              varF = vntRet->fltVal;
              break;
            case VT_R8:
              varD = vntRet->dblVal;
              break;
            case VT_BSTR:
              varS = ConvertBSTRToString(vntRet->bstrVal);
              break;
            case VT_BOOL:
              varIO = (vntRet->boolVal != VARIANT_FALSE) ? true : false;
              break;
            case (VT_ARRAY | VT_R4):
              num = vntRet->parray->rgsabound->cElements;
              SafeArrayAccessData(vntRet->parray, (void**)&pfltval);
              varFArray.resize(num);
              std::copy(pfltval, &pfltval[num], varFArray.begin());
              SafeArrayUnaccessData(vntRet->parray);
              break;
            case (VT_ARRAY | VT_R8):
              num = vntRet->parray->rgsabound->cElements;
              SafeArrayAccessData(vntRet->parray, (void**)&pdblval);
              varDArray.resize(num);
              std::copy(pdblval, &pdblval[num], varDArray.begin());
              SafeArrayUnaccessData(vntRet->parray);
              break;
          }
        }
      }
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
  vntHandle->ulVal = vecHandle_[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  return vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_VARIABLE_GETVALUE, vntArgs, value);
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
  vntHandle->ulVal = vecHandle_[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  vntArgs.push_back(*value.get());

  hr = vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_VARIABLE_PUTVALUE, vntArgs, vntRet);
  if (SUCCEEDED(hr))
  {
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
  vntHandle->ulVal = vecHandle_[DensoBase::SRV_WATCH];

  vntArgs.push_back(*vntHandle.get());

  vntValue->vt = VT_I4;
  vntValue->lVal = id;
  vntArgs.push_back(*vntValue.get());

  hr = vecService_[DensoBase::SRV_WATCH]->ExecFunction(ID_VARIABLE_PUTID, vntArgs, vntRet);
  if (SUCCEEDED(hr))
  {
    Update();
  }

  return hr;
}
}  // namespace denso2
