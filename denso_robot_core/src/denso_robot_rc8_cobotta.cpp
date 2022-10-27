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

#include "denso_robot_core/denso_robot_rc8_cobotta.h"

namespace denso_robot_core
{

/**
 * When you control your COBOTTA by any means other than RC8 provider
 * (such as to control your COBOTTA directly from b-CAP),
 * before sending safety-related commands (e.g; motion preparation, error reset)
 * from the control means, send this command to confirm
 * if the safety-related command is intentionally sent.
 * If you don't send this command beforehand, "83500372" error will occur.
 *
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoRobotRC8Cobotta::ExecManualResetPreparation()
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
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"ManualResetPreparation");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

/**
 * The motion preparation is performed automatically. This command performs
 * the same behavior as MotionPriparation of PAC command.
 * When safety data was sent to COBOTTA by using COBOTTA parameter tool,
 * TP requires motion preparation because the parameters need to be confirmed.
 * Therefore, this command cannot be executed immediately after the safety data was sent.
 * For details on MotionPreparation, see the COBOTTA manual.
 *
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoRobotRC8Cobotta::ExecMotionPreparation()
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
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"MotionPreparation");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}

/**
 * To execute CALSET automatically at the startup. This command performs
 * the same behavior as AutoCal of PAC command.
 * To use the command, set the use condition parameter 252 "CALSET on start-up"
 * to "1: DoNot" to stop the display of CALSET window at the startup of TP.
 * Also, the start-up privilege should be set to Ethernet.
 * For details on AutoCal, see the COBOTTA manual.
 *
 * Do NOT call on b-CAP Slave.
 * @return HRESULT
 */
HRESULT DensoRobotRC8Cobotta::ExecAutoCal()
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
        vntTmp->ulVal = m_vecHandle[DensoBase::SRV_ACT];
        break;
      case 1:
        vntTmp->vt = VT_BSTR;
        vntTmp->bstrVal = SysAllocString(L"AutoCal");
        break;
    }

    vntArgs.push_back(*vntTmp.get());
  }

  return m_vecService[DensoBase::SRV_ACT]->ExecFunction(ID_ROBOT_EXECUTE, vntArgs, vntRet);
}
}  // namespace denso_robot_core
