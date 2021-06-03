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

#include "denso_robot_core/denso_controller_rc8_cobotta.h"

namespace denso_robot_core
{
HRESULT DensoControllerRC8Cobotta::AddRobot(XMLElement* xmlElem)
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

      DensoRobot_Ptr rob(new DensoRobotRC8Cobotta(this, m_vecService, vecHandle, vecName[objs], m_mode));
      hr = rob->InitializeBCAP(xmlElem);
      if (FAILED(hr))
        break;

      m_vecRobot.push_back(rob);
    }
  }

  return hr;
}

HRESULT DensoControllerRC8Cobotta::get_Robot(int index, DensoRobotRC8Cobotta_Ptr* robot)
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
    *robot = boost::dynamic_pointer_cast<DensoRobotRC8Cobotta>(pBase);
  }

  return hr;
}

/**
 * Clear an error that occurs in the controller.
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoControllerRC8Cobotta::ExecClearError()
{
  DensoRobotRC8Cobotta_Ptr pRob;
  HRESULT hr;

  hr = this->get_Robot(0, &pRob);
  if (FAILED(hr))
  {
    return hr;
  }
  hr = pRob->ExecManualResetPreparation();
  if (FAILED(hr))
  {
    return hr;
  }
  hr = DensoControllerRC8::ExecClearError();
  if (FAILED(hr))
  {
    return hr;
  }

  return hr;
}

/**
 * Reset the STO(Safe Torque Off) state.
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoControllerRC8Cobotta::ExecResetStoState()
{
  DensoRobotRC8Cobotta_Ptr pRob;
  HRESULT hr;

  hr = this->get_Robot(0, &pRob);
  if (FAILED(hr))
  {
    return hr;
  }
  hr = pRob->ExecManualResetPreparation();
  if (FAILED(hr))
  {
    return hr;
  }
  hr = pRob->ExecMotionPreparation();
  if (FAILED(hr))
  {
    return hr;
  }

  return hr;
}

/**
 * @param robot_name
 * @return True if robot_name is COBOTTA.
 */
bool DensoControllerRC8Cobotta::IsCobotta(const std::string& robot_name)
{
  std::string cobotta = ROBOT_NAME_RC8_COBOTTA;

  if (std::equal(cobotta.begin(),cobotta.end(),
                 robot_name.begin(), robot_name.begin() + cobotta.length()))
  {
    return true;
  }

  return false;
}

}  // namespace denso_robot_core
