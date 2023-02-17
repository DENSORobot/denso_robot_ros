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
#include "denso_robot_core/denso_controller_rc9.h"

namespace denso_robot_core
{
DensoControllerRC9::DensoControllerRC9(const std::string& name, const int* mode, const ros::Duration dt)
  : DensoController(name, mode, dt)
{
}

DensoControllerRC9::~DensoControllerRC9()
{
}

HRESULT DensoControllerRC9::AddController()
{
  static const std::string CTRL_CONNECT_OPTION[BCAP_CONTROLLER_CONNECT_ARGS]
                                               = { "", "CaoProv.DENSO.VRC9", "localhost", "" };

  HRESULT hr = E_FAIL;
  int srvs, argc;

  if (m_duration != ros::Duration(0.008))
  {
    ROS_ERROR("Invalid argument value [bcap_slave_control_cycle_msec]");
    return E_INVALIDARG;
  }

  for (srvs = DensoBase::SRV_MIN; srvs <= DensoBase::SRV_MAX; srvs++)
  {
    std::stringstream ss;
    std::string strTmp;
    VARIANT_Ptr vntRet(new VARIANT());
    VARIANT_Vec vntArgs;

    VariantInit(vntRet.get());

    for (argc = 0; argc < BCAP_CONTROLLER_CONNECT_ARGS; argc++)
    {
      VARIANT_Ptr vntTmp(new VARIANT());
      VariantInit(vntTmp.get());

      vntTmp->vt = VT_BSTR;

      if (argc == 0)
      {
        strTmp = "";
        if (m_name != "")
        {
          ss << ros::this_node::getNamespace() << m_name << srvs;
          strTmp = ss.str();
        }
      }
      else
      {
        strTmp = CTRL_CONNECT_OPTION[argc];
      }

      vntTmp->bstrVal = ConvertStringToBSTR(strTmp);

      vntArgs.push_back(*vntTmp.get());
    }

    hr = m_vecService[srvs]->ExecFunction(ID_CONTROLLER_CONNECT, vntArgs, vntRet);
    if (FAILED(hr))
      break;

    m_vecHandle.push_back(vntRet->ulVal);
  }

  return hr;
}

HRESULT DensoControllerRC9::AddRobot(XMLElement* xmlElem)
{
  int objs;
  HRESULT hr;

  Name_Vec vecName;
  hr = DensoBase::GetObjectNames(ID_CONTROLLER_GETROBOTNAMES, vecName);
  if (SUCCEEDED(hr))
  {
    for (objs = 0; objs < vecName.size(); objs++)
    {
      Handle_Vec vecHandle;
      hr = DensoBase::AddObject(ID_CONTROLLER_GETROBOT, vecName[objs], vecHandle);
      if (FAILED(hr))
        break;

      DensoRobot_Ptr rob(new DensoRobotRC9(this, m_vecService, vecHandle, vecName[objs], m_mode));
      hr = rob->InitializeBCAP(xmlElem);
      if (FAILED(hr))
        break;

      m_vecRobot.push_back(rob);
    }
  }

  return hr;
}

HRESULT DensoControllerRC9::get_Robot(int index, DensoRobotRC9_Ptr* robot)
{
  if (robot == NULL)
  {
    return E_INVALIDARG;
  }

  DensoBase_Vec vecBase;
  vecBase.insert(vecBase.end(), m_vecRobot.begin(), m_vecRobot.end());

  DensoBase_Ptr pBase;
  HRESULT hr = DensoBase::get_Object(vecBase, index, &pBase);
  if (SUCCEEDED(hr))
  {
    *robot = boost::dynamic_pointer_cast<DensoRobotRC9>(pBase);
  }

  return hr;
}

/**
 * CaoController::Execute("ManualReset")
 * Clear safety-state with RC9 v1.4.x or later.
 * Use ExecResetStoState() for earlier versions.
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoControllerRC9::ExecManualReset()
{
  /*
   * RC9 v1.4.x or later: ManualReset
   * RC9 v1.3.x or earlier: ResetStoState
   */
  std::tuple<int, int, int> min_version = std::make_tuple(1, 4, 0);
  std::tuple<int, int, int> current_version = get_SoftwareVersion();
  if (current_version < min_version)
  {
    return ExecResetStoState();
  }

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
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"ManualReset");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_CONTROLLER_EXECUTE, vntArgs, vntRet);
}

/**
 * [Deprecated] CaoController::Execute("ResetStoState")
 * Reset the STO(Safe Torque Off) state.
 * Use ExecManualReset() for RC9 v1.4.x or later.
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoControllerRC9::ExecResetStoState()
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
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_WATCH];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"ResetStoState");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_WATCH]->ExecFunction(ID_CONTROLLER_EXECUTE, vntArgs, vntRet);
}

}  // namespace denso_robot_core
