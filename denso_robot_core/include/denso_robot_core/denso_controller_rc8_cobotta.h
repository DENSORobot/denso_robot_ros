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

#ifndef DENSO_CONTROLLER_RC8_COBOTTA_H
#define DENSO_CONTROLLER_RC8_COBOTTA_H

#include "denso_robot_core/denso_controller_rc8.h"
#include "denso_robot_core/denso_robot_rc8_cobotta.h"

namespace denso_robot_core
{

class DensoControllerRC8Cobotta : public DensoControllerRC8
{
public:
  static constexpr const char* ROBOT_NAME_RC8_COBOTTA = "CVR038A1";

  using DensoControllerRC8::DensoControllerRC8;
  virtual ~DensoControllerRC8Cobotta() = default;

  HRESULT get_Robot(int index, DensoRobotRC8Cobotta_Ptr* robot);

  virtual HRESULT ExecClearError() override;
  virtual HRESULT ExecResetStoState() override;

  static bool IsCobotta(const std::string& robot_name);

private:
  HRESULT AddRobot(XMLElement* xmlElem);

};
}  // namespace denso_robot_core

#endif  // DENSO_CONTROLLER_RC8_COBOTTA_H
