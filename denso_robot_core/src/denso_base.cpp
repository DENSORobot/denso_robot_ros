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
#include "denso_robot_core/denso_base.h"
#include "denso_robot_core/denso_variable.h"

namespace denso_robot_core
{
std::string DensoBase::ConvertBSTRToString(const BSTR bstr)
{
  std::string strRet;
  char* chTmp;

  chTmp = ConvertWideChar2MultiByte(bstr);
  if (chTmp != NULL)
  {
    strRet = chTmp;
    free(chTmp);
  }

  return strRet;
}

BSTR DensoBase::ConvertStringToBSTR(const std::string& str)
{
  BSTR strRet = NULL;
  wchar_t* chTmp;

  chTmp = ConvertMultiByte2WideChar(str.c_str());
  if (chTmp != NULL)
  {
    strRet = SysAllocString(chTmp);
    free(chTmp);
  }

  return strRet;
}

std::string DensoBase::RosName() const
{
  std::string tmpName = m_name;

  // Replace space to _
  std::replace(tmpName.begin(), tmpName.end(), ' ', '_');

  // Replace backslash to /
  std::replace(tmpName.begin(), tmpName.end(), '\\', '/');

  // Erase @
  size_t c;
  while ((c = tmpName.find_first_of("@")) != std::string::npos)
  {
    tmpName.erase(c, 1);
  }

  // Erase *
  while ((c = tmpName.find_first_of("*")) != std::string::npos)
  {
    tmpName.erase(c, 1);
  }

  return tmpName;
}

HRESULT DensoBase::AddVariable(int32_t get_id, const std::string& name, DensoVariable_Vec& vecVar, int16_t vt,
                               bool bRead, bool bWrite, bool bID, int iDuration)
{
  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), vecVar.begin(), vecVar.end());

  if (E_HANDLE == get_Object(vecBase, name, NULL))
  {
    Handle_Vec vecHandle;
    HRESULT hr = AddObject(get_id, name, vecHandle);
    if (FAILED(hr))
      return hr;

    DensoVariable_Ptr var(
        new DensoVariable(this, m_vecService, vecHandle, name, m_mode, vt, bRead, bWrite, bID, iDuration));

    vecVar.push_back(var);
  }

  return S_OK;
}

HRESULT DensoBase::AddVariable(int32_t get_id, const XMLElement* xmlVar, DensoVariable_Vec& vecVar)
{
  const char* chTmp;

  std::string name;
  int16_t vt = VT_EMPTY;
  bool bRead = false, bWrite = false, bID = false;
  int iDuration = BCAP_VAR_DEFAULT_DURATION;

  name = xmlVar->GetText();

  chTmp = xmlVar->Attribute(DensoVariable::XML_ATTR_VARTYPE);
  if (chTmp != NULL)
    vt = atoi(chTmp);

  chTmp = xmlVar->Attribute(DensoVariable::XML_ATTR_READ);
  if (chTmp != NULL)
    bRead = (strcasecmp(chTmp, "true") == 0);

  chTmp = xmlVar->Attribute(DensoVariable::XML_ATTR_WRITE);
  if (chTmp != NULL)
    bWrite = (strcasecmp(chTmp, "true") == 0);

  chTmp = xmlVar->Attribute(DensoVariable::XML_ATTR_ID);
  if (chTmp != NULL)
    bID = (strcasecmp(chTmp, "true") == 0);

  chTmp = xmlVar->Attribute(DensoVariable::XML_ATTR_DURATION);
  if (chTmp != NULL)
    iDuration = atoi(chTmp);

  Handle_Vec vecHandle;
  HRESULT hr = AddObject(get_id, name, vecHandle);
  if (FAILED(hr))
    return hr;

  DensoVariable_Ptr var(
      new DensoVariable(this, m_vecService, vecHandle, name, m_mode, vt, bRead, bWrite, bID, iDuration));

  vecVar.push_back(var);

  return S_OK;
}

HRESULT DensoBase::AddObject(int32_t get_id, const std::string& name, Handle_Vec& vecHandle)
{
  int srvs, argc;
  HRESULT hr;

  for (srvs = SRV_MIN; srvs <= SRV_MAX; srvs++)
  {
    VARIANT_Ptr vntRet(new VARIANT());
    VARIANT_Vec vntArgs;

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_GET_OBJECT_ARGS; argc++)
    {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      switch (argc)
      {
        case 0:
          vntTmp->vt = VT_UI4;
          vntTmp->ulVal = m_vecHandle[srvs];
          break;
        case 1:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = ConvertStringToBSTR(name);
          break;
        case 2:
          vntTmp->vt = VT_BSTR;
          vntTmp->bstrVal = SysAllocString(L"");
          break;
      }

      vntArgs.push_back(*vntTmp.get());
    }

    hr = m_vecService[srvs]->ExecFunction(get_id, vntArgs, vntRet);
    if (FAILED(hr))
      break;

    vecHandle.push_back(vntRet->ulVal);
  }

  return hr;
}

HRESULT DensoBase::GetObjectNames(int32_t func_id, Name_Vec& vecName)
{
  uint32_t argc, i, num;
  HRESULT hr;
  VARIANT_Ptr vntRet(new VARIANT());
  VARIANT_Vec vntArgs;

  VariantInit(vntRet.get());

  for (argc = 0; argc < BCAP_GET_OBJECTNAMES_ARGS; argc++)
  {
    VARIANT_Ptr vntTmp(new VARIANT());
    VariantInit(vntTmp.get());

    if (argc == 0)
    {
      vntTmp->vt = VT_UI4;
      vntTmp->ulVal = m_vecHandle[SRV_WATCH];
    }
    else
    {
      vntTmp->vt = VT_BSTR;
      vntTmp->bstrVal = SysAllocString(L"");
    }

    vntArgs.push_back(*vntTmp.get());
  }

  hr = m_vecService[SRV_WATCH]->ExecFunction(func_id, vntArgs, vntRet);
  if (SUCCEEDED(hr))
  {
    BSTR* pbstr;
    VARIANT* pvnt;
    switch (vntRet->vt)
    {
      case (VT_ARRAY | VT_BSTR):
        num = vntRet->parray->rgsabound->cElements;
        SafeArrayAccessData(vntRet->parray, (void**)&pbstr);
        for (i = 0; i < num; i++)
        {
          vecName.push_back(ConvertBSTRToString(pbstr[i]));
        }
        SafeArrayUnaccessData(vntRet->parray);
        break;
      case (VT_ARRAY | VT_VARIANT):
        num = vntRet->parray->rgsabound->cElements;
        SafeArrayAccessData(vntRet->parray, (void**)&pvnt);
        for (i = 0; i < num; i++)
        {
          if (pvnt[i].vt != VT_BSTR)
          {
            hr = E_FAIL;
            break;
          }
          vecName.push_back(ConvertBSTRToString(pvnt[i].bstrVal));
        }
        SafeArrayUnaccessData(vntRet->parray);
        break;
      default:
        hr = S_FALSE;
        break;
    }
  }

  return hr;
}

HRESULT DensoBase::get_Object(const DensoBase_Vec& vecBase, int index, DensoBase_Ptr* obj)
{
  try
  {
    if (obj != NULL)
    {
      *obj = vecBase.at(index);
    }
  }
  catch (std::out_of_range&)
  {
    return E_HANDLE;
  }

  return S_OK;
}

HRESULT DensoBase::get_Object(const DensoBase_Vec& vecBase, const std::string& name, DensoBase_Ptr* obj)
{
  DensoBase_Vec::const_iterator it;
  for (it = vecBase.begin(); it != vecBase.end(); it++)
  {
    if (!strcasecmp((*it)->Name().c_str(), name.c_str()))
    {
      if (obj != NULL)
      {
        *obj = *it;
      }
      return S_OK;
    }
  }

  return E_HANDLE;
}

}  // namespace denso_robot_core
