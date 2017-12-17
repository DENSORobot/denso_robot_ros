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

#include <ros/ros.h>
#include "bcap_service/bcap.h"

using namespace bcap_service;

int main(int argc, char **argv)
{
  if ((argc < 4) || (argc % 2 != 0))
  {
    ROS_WARN("Usage: bcap_service_test func_id vt value ...");
    return 1;
  }

  ros::init(argc, argv, "bcap_service_test");

  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<bcap>("bcap_service");

  int i;
  bcap packet;

  packet.request.func_id = atoi(argv[1]);

  for(i = 0; i < (argc - 2) / 2; i++) {
    variant vnt;
    vnt.vt    = atoi(argv[2 + 2 * i]);
    vnt.value = argv[3 + 2 * i];
    packet.request.vntArgs.push_back(vnt);
  }

  if (client.call(packet))
  {
    ROS_INFO("HRESULT: %X", packet.response.HRESULT);
    ROS_INFO("vntRet: vt := %d, value := %s", packet.response.vntRet.vt, packet.response.vntRet.value.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call.");
    return 1;
  }

  return 0;

}
