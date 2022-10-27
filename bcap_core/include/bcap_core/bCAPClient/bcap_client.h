#ifndef BCAP_CLIENT_H_
#define BCAP_CLIENT_H_

/**
 * @file    bcap_client.h
 * @brief   b-CAP Client API file.
 * @details Defines b-CAP Client APIs.
 *
 * @version 1.0
 * @date    2014/11/06
 * @author  DENSO WAVE
 *
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

#ifndef _BCAP_EXP_CLIENT
#define _BCAP_EXP_CLIENT
#endif /* _BCAP_EXP_CLIENT */

#include "../dn_common.h"

/**
 * @def   BCAP_CONN_MAX
 * @brief A definition for the maximum count of b-CAP client connections.
 * @note  You can change this parameter to 1 or more.
 */
#define BCAP_CONN_MAX (20)

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn         HRESULT bCap_Open_Client(const char *connect, uint32_t timeout, unsigned int retry, int *pfd)
   * @brief      Starts b-CAP communication.
   * @param[in]  connect Connection parameters.
   * @param[in]  timeout Timeout value.
   * @param[in]  retry Retry value.
   * @param[out] pfd The pointer of File descriptor.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_Open_Client(const char *connect, uint32_t timeout, unsigned int retry,
      int *pfd);

  /**
   * @fn            HRESULT bCap_Close_Client(int *pfd)
   * @brief         Ends b-CAP communication.
   * @param[in,out] pfd The pointer of File descriptor.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_Close_Client(int *pfd);

  /**
   * @fn        HRESULT bCap_SetTimeout(int fd, uint32_t timeout)
   * @brief     Sets timeout.
   * @param[in] fd File descriptor.
   * @param[in] timeout Timeout value to be set.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_SetTimeout(int fd, uint32_t timeout);

  /**
   * @fn         HRESULT bCap_GetTimeout(int fd, uint32_t *timeout)
   * @brief      Gets timeout.
   * @param[in]  fd File descriptor.
   * @param[out] timeout The gotten timeout value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_GetTimeout(int fd, uint32_t *timeout);

  /**
   * @fn        HRESULT bCap_SetRetry(int fd, unsigned int retry)
   * @brief     Sets retry count.
   * @param[in] fd File descriptor.
   * @param[in] retry Retry count to be set.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_SetRetry(int fd, unsigned int retry);

  /**
   * @fn        HRESULT bCap_GetRetry(int fd, unsigned int *retry)
   * @brief     Gets retry count.
   * @param[in] fd File descriptor.
   * @param[in] retry The gotten retry count.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_GetRetry(int fd, unsigned int *retry);

  /**
   * @fn        HRESULT bCap_ServiceStart(int fd, BSTR bstrOption)
   * @brief     Send the b-CAP ID_SERVICE_START packet.
   * @param[in] fd File descriptor.
   * @param[in] bstrOpt Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ServiceStart(int fd, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_ServiceStop(int fd)
   * @brief     Send the b-CAP ID_SERVICE_STOP packet.
   * @param[in] fd File descriptor.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ServiceStop(int fd);

  /**
   * @fn         HRESULT bCap_ControllerConnect(int fd, BSTR bstrName, BSTR bstrProvider, BSTR bstrMachine, BSTR bstrOption, uint32_t *hController)
   * @brief      Send the b-CAP ID_CONTROLLER_CONNECT packet.
   * @param[in]  fd File descriptor.
   * @param[in]  bstrName Controller name.
   * @param[in]  bstrProvider Provider name.
   * @param[in]  bstrMachine Machine name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hController Controller handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerConnect(int fd, BSTR bstrName, BSTR bstrProvider,
      BSTR bstrMachine, BSTR bstrOption, uint32_t *hController);

  /**
   * @fn            HRESULT bCap_ControllerDisconnect(int fd, uint32_t *hController)
   * @brief         Send the b-CAP ID_CONTROLLER_DISCONNECT packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hController Controller handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerDisconnect(int fd, uint32_t *hController);

  /**
   * @fn         HRESULT bCap_ControllerGetExtension(int fd, uint32_t hController, BSTR bstrName, BSTR bstrOption, uint32_t *hExtension)
   * @brief      Send the b-CAP ID_CONTROLLER_GETEXTENSION packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrName Extension name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hExtension Extension handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetExtension(int fd, uint32_t hController, BSTR bstrName,
      BSTR bstrOption, uint32_t *hExtension);

  /**
   * @fn         HRESULT bCap_ControllerGetFile(int fd, uint32_t hController, BSTR bstrName, BSTR bstrOption, uint32_t *hFile)
   * @brief      Send the b-CAP ID_CONTROLLER_GETFILE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hFile File handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetFile(int fd, uint32_t hController, BSTR bstrName,
      BSTR bstrOption, uint32_t *hFile);

  /**
   * @fn         HRESULT bCap_ControllerGetRobot(int fd, uint32_t hController, BSTR bstrName, BSTR bstrOption, uint32_t *hRobot)
   * @brief      Send the b-CAP ID_CONTROLLER_GETROBOT packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hRobot Robot handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetRobot(int fd, uint32_t hController, BSTR bstrName,
      BSTR bstrOption, uint32_t *hRobot);

  /**
   * @fn         HRESULT bCap_ControllerGetTask(int fd, uint32_t hController, BSTR bstrName, BSTR bstrOption, uint32_t *hTask)
   * @brief      Send the b-CAP ID_CONTROLLER_GETTASK packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hTask Task handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetTask(int fd, uint32_t hController, BSTR bstrName,
      BSTR bstrOption, uint32_t *hTask);

  /**
   * @fn         HRESULT bCap_ControllerGetVariable(int fd, uint32_t hController, BSTR bstrName, BSTR bstrOption, uint32_t *hVariable)
   * @brief      Send the b-CAP ID_CONTROLLER_GETVARIABLE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hVariable Variable handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetVariable(int fd, uint32_t hController, BSTR bstrName,
      BSTR bstrOption, uint32_t *hVariable);

  /**
   * @fn         HRESULT bCap_ControllerGetCommand(int fd, uint32_t hController, BSTR bstrName, BSTR bstrOption, uint32_t *hCommand)
   * @brief      Send the b-CAP ID_CONTROLLER_GETCOMMAND packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hCommand Command handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetCommand(int fd, uint32_t hController, BSTR bstrName,
      BSTR bstrOption, uint32_t *hCommand);

  /**
   * @fn         HRESULT bCap_ControllerGetExtensionNames(int fd, uint32_t hController, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETEXTENSIONNAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetExtensionNames(int fd, uint32_t hController,
      BSTR bstrOption, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetFileNames(int fd, uint32_t hController, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETFILENAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetFileNames(int fd, uint32_t hController, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetRobotNames(int fd, uint32_t hController, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETROBOTNAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetRobotNames(int fd, uint32_t hController, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetTaskNames(int fd, uint32_t hController, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETTASKNAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetTaskNames(int fd, uint32_t hController, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetVariableNames(int fd, uint32_t hController, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETVARIABLENAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetVariableNames(int fd, uint32_t hController, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetCommandNames(int fd, uint32_t hController, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETCOMMANDNAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetCommandNames(int fd, uint32_t hController, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ControllerExecute(int fd, uint32_t hController, BSTR bstrCommand, VARIANT vntParam, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_EXECUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[in]  bstrCommand Executing command name.
   * @param[in]  vntParam Executing parameters.
   * @param[out] pVal Result value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerExecute(int fd, uint32_t hController, BSTR bstrCommand,
      VARIANT vntParam, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetMessage(int fd, uint32_t hController, uint32_t *hMessage)
   * @brief      Send the b-CAP ID_CONTROLLER_GETMESSAGE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[out] hMessage Message handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetMessage(int fd, uint32_t hController, uint32_t *hMessage);

  /**
   * @fn         HRESULT bCap_ControllerGetAttribute(int fd, uint32_t hController, int32_t *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETATTRIBUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[out] pVal The gotten attribute value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetAttribute(int fd, uint32_t hController, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetHelp(int fd, uint32_t hController, BSTR *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETHELP packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[out] pVal The gotten help string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetHelp(int fd, uint32_t hController, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetName(int fd, uint32_t hController, BSTR *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETNAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[out] pVal The gotten name string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetName(int fd, uint32_t hController, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_ControllerGetTag(int fd, uint32_t hController, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETTAG packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[out] pVal The gotten tag value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetTag(int fd, uint32_t hController, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_ControllerPutTag(int fd, uint32_t hController, VARIANT newVal)
   * @brief     Send the b-CAP ID_CONTROLLER_PUTTAG packet.
   * @param[in] fd File descriptor.
   * @param[in] hController Controller handle.
   * @param[in] newVal The tag value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerPutTag(int fd, uint32_t hController, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_ControllerGetID(int fd, uint32_t hController, VARIANT *pVal)
   * @brief      Send the b-CAP ID_CONTROLLER_GETID packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Controller handle.
   * @param[out] pVal The gotten ID.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerGetID(int fd, uint32_t hController, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_ControllerPutID(int fd, uint32_t hController, VARIANT newVal)
   * @brief     Send the b-CAP ID_CONTROLLER_PUTID packet.
   * @param[in] fd File descriptor.
   * @param[in] hController Controller handle.
   * @param[in] newVal The ID to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ControllerPutID(int fd, uint32_t hController, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_ExtensionGetVariable(int fd, uint32_t hExtension, BSTR bstrName, BSTR bstrOption, uint32_t *hVariable)
   * @brief      Send the b-CAP ID_EXTENSION_GETVARIABLE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Extension handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hVariable Variable handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionGetVariable(int fd, uint32_t hExtension, BSTR bstrName,
      BSTR bstrOption, uint32_t *hVariable);

  /**
   * @fn         HRESULT bCap_ExtensionGetVariableNames(int fd, uint32_t hExtension, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_EXTENSION_GETVARIABLENAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hExtension Extension handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionGetVariableNames(int fd, uint32_t hExtension, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ExtensionExecute(int fd, uint32_t hExtension, BSTR bstrCommand, VARIANT vntParam, VARIANT *pVal)
   * @brief      Send the b-CAP ID_EXTENSION_EXECUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hExtension Extension handle.
   * @param[in]  bstrCommand Executing command name.
   * @param[in]  vntParam Executing parameters.
   * @param[out] pVal Result value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionExecute(int fd, uint32_t hExtension, BSTR bstrCommand,
      VARIANT vntParam, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_ExtensionGetAttribute(int fd, uint32_t hExtension, int32_t *pVal)
   * @brief      Send the b-CAP ID_EXTENSION_GETATTRIBUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hExtension Extension handle.
   * @param[out] pVal The gotten attribute value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionGetAttribute(int fd, uint32_t hExtension, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_ExtensionGetHelp(int fd, uint32_t hExtension, BSTR *pVal)
   * @brief      Send the b-CAP ID_EXTENSION_GETHELP packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hExtension Extension handle.
   * @param[out] pVal The gotten help string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionGetHelp(int fd, uint32_t hExtension, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_ExtensionGetName(int fd, uint32_t hExtension, BSTR *pVal)
   * @brief      Send the b-CAP ID_EXTENSION_GETNAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hExtension Extension handle.
   * @param[out] pVal The gotten name string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionGetName(int fd, uint32_t hExtension, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_ExtensionGetTag(int fd, uint32_t hExtension, VARIANT *pVal)
   * @brief      Send the b-CAP ID_EXTENSION_GETTAG packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hExtension Extension handle.
   * @param[out] pVal The gotten tag value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionGetTag(int fd, uint32_t hExtension, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_ExtensionPutTag(int fd, uint32_t hExtension, VARIANT newVal)
   * @brief     Send the b-CAP ID_EXTENSION_PUTTAG packet.
   * @param[in] fd File descriptor.
   * @param[in] hExtension Extension handle.
   * @param[in] newVal The tag value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionPutTag(int fd, uint32_t hExtension, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_ExtensionGetID(int fd, uint32_t hExtension, VARIANT *pVal)
   * @brief      Send the b-CAP ID_EXTENSION_GETID packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hExtension Extension handle.
   * @param[out] pVal The gotten ID.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionGetID(int fd, uint32_t hExtension, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_ExtensionPutID(int fd, uint32_t hExtension, VARIANT newVal)
   * @brief     Send the b-CAP ID_EXTENSION_PUTID packet.
   * @param[in] fd File descriptor.
   * @param[in] hExtension Extension handle.
   * @param[in] newVal The ID to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionPutID(int fd, uint32_t hExtension, VARIANT newVal);

  /**
   * @fn            HRESULT bCap_ExtensionRelease(int fd, uint32_t *hExtension)
   * @brief         Send the b-CAP ID_EXTENSION_RELEASE packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hExtension Extension handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_ExtensionRelease(int fd, uint32_t *hExtension);

  /**
   * @fn         HRESULT bCap_FileGetFile(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption, uint32_t *hFile2)
   * @brief      Send the b-CAP ID_FILE_GETFILE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hFile2 File handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetFile(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption,
      uint32_t *hFile2);

  /**
   * @fn         HRESULT bCap_FileGetVariable(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption, uint32_t *hVariable)
   * @brief      Send the b-CAP ID_FILE_GETVARIABLE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hVariable Variable handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetVariable(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption,
      uint32_t *hVariable);

  /**
   * @fn         HRESULT bCap_FileGetFileNames(int fd, uint32_t hFile, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETFILENAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetFileNames(int fd, uint32_t hFile, BSTR bstrOption, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_FileGetVariableNames(int fd, uint32_t hFile, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETVARIABLENAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetVariableNames(int fd, uint32_t hFile, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_FileExecute(int fd, uint32_t hFile, BSTR bstrCommand, VARIANT vntParam, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_EXECUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[in]  bstrCommand Executing command name.
   * @param[in]  vntParam Executing parameters.
   * @param[out] pVal Result value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileExecute(int fd, uint32_t hFile, BSTR bstrCommand, VARIANT vntParam,
      VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_FileCopy(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption)
   * @brief     Send the b-CAP ID_FILE_COPY packet.
   * @param[in] fd File descriptor.
   * @param[in] hFile File handle.
   * @param[in] bstrName Copied file name.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileCopy(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_FileDelete(int fd, uint32_t hFile, BSTR bstrOption)
   * @brief     Send the b-CAP ID_FILE_DELETE packet.
   * @param[in] fd File descriptor.
   * @param[in] hFile File handle.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileDelete(int fd, uint32_t hFile, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_FileMove(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption)
   * @brief     Send the b-CAP ID_FILE_MOVE packet.
   * @param[in] fd File descriptor.
   * @param[in] hFile File handle.
   * @param[in] bstrName Moved file name.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileMove(int fd, uint32_t hFile, BSTR bstrName, BSTR bstrOption);

  /**
   * @fn         HRESULT bCap_FileRun(int fd, uint32_t hFile, BSTR bstrOption, BSTR *pVal)
   * @brief      Send the b-CAP ID_FILE_RUN packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The created task name.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileRun(int fd, uint32_t hFile, BSTR bstrOption, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_FileGetDateCreated(int fd, uint32_t hFile, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETDATECREATED packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten time value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetDateCreated(int fd, uint32_t hFile, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_FileGetDateLastAccessed(int fd, uint32_t hFile, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETDATELASTACCESSED packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten time value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetDateLastAccessed(int fd, uint32_t hFile, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_FileGetDateLastModified(int fd, uint32_t hFile, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETDATELASTMODIFIED packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten time value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetDateLastModified(int fd, uint32_t hFile, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_FileGetPath(int fd, uint32_t hFile, BSTR *pVal)
   * @brief      Send the b-CAP ID_FILE_GETPATH packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten path.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetPath(int fd, uint32_t hFile, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_FileGetSize(int fd, uint32_t hFile, int32_t *pVal)
   * @brief      Send the b-CAP ID_FILE_GETSIZE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten size.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetSize(int fd, uint32_t hFile, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_FileGetType(int fd, uint32_t hFile, BSTR *pVal)
   * @brief      Send the b-CAP ID_FILE_GETTYPE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten type.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetType(int fd, uint32_t hFile, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_FileGetValue(int fd, uint32_t hFile, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETVALUE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetValue(int fd, uint32_t hFile, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_FilePutValue(int fd, uint32_t hFile, VARIANT newVal)
   * @brief     Send the b-CAP ID_FILE_PUTVALUE packet.
   * @param[in] fd File descriptor.
   * @param[in] hFile File handle.
   * @param[in] newVal The value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FilePutValue(int fd, uint32_t hFile, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_FileGetAttribute(int fd, uint32_t hFile, int32_t *pVal)
   * @brief      Send the b-CAP ID_FILE_GETATTRIBUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten attribute value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetAttribute(int fd, uint32_t hFile, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_FileGetHelp(int fd, uint32_t hFile, BSTR *pVal)
   * @brief      Send the b-CAP ID_FILE_GETHELP packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten help string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetHelp(int fd, uint32_t hFile, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_FileGetName(int fd, uint32_t hFile, BSTR *pVal)
   * @brief      Send the b-CAP ID_FILE_GETNAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten name string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetName(int fd, uint32_t hFile, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_FileGetTag(int fd, uint32_t hFile, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETTAG packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten tag value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetTag(int fd, uint32_t hFile, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_FilePutTag(int fd, uint32_t hFile, VARIANT newVal)
   * @brief     Send the b-CAP ID_FILE_PUTTAG packet.
   * @param[in] fd File descriptor.
   * @param[in] hFile File handle.
   * @param[in] newVal The tag value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FilePutTag(int fd, uint32_t hFile, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_FileGetID(int fd, uint32_t hFile, VARIANT *pVal)
   * @brief      Send the b-CAP ID_FILE_GETID packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hFile File handle.
   * @param[out] pVal The gotten ID.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileGetID(int fd, uint32_t hFile, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_FilePutID(int fd, uint32_t hFile, VARIANT newVal)
   * @brief     Send the b-CAP ID_FILE_PUTID packet.
   * @param[in] fd File descriptor.
   * @param[in] hFile File handle.
   * @param[in] newVal The ID to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FilePutID(int fd, uint32_t hFile, VARIANT newVal);

  /**
   * @fn            HRESULT bCap_FileRelease(int fd, uint32_t *hFile)
   * @brief         Send the b-CAP ID_FILE_RELEASE packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hFile File handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_FileRelease(int fd, uint32_t *hFile);

  /**
   * @fn         HRESULT bCap_RobotGetVariable(int fd, uint32_t hRobot, BSTR bstrName, BSTR bstrOption, uint32_t *hVariable)
   * @brief      Send the b-CAP ID_ROBOT_GETVARIABLE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hController Robot handle.
   * @param[in]  bstrName File name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hVariable Variable handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGetVariable(int fd, uint32_t hRobot, BSTR bstrName, BSTR bstrOption,
      uint32_t *hVariable);

  /**
   * @fn         HRESULT bCap_RobotGetVariableNames(int fd, uint32_t hRobot, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_ROBOT_GETVARIABLENAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hRobot Robot handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGetVariableNames(int fd, uint32_t hRobot, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_RobotExecute(int fd, uint32_t hRobot, BSTR bstrCommand, VARIANT vntParam, VARIANT *pVal)
   * @brief      Send the b-CAP ID_ROBOT_EXECUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hRobot Robot handle.
   * @param[in]  bstrCommand Executing command name.
   * @param[in]  vntParam Executing parameters.
   * @param[out] pVal Result value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotExecute(int fd, uint32_t hRobot, BSTR bstrCommand, VARIANT vntParam,
      VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_RobotAccelerate(int fd, uint32_t hRobot, int32_t lAxis, float fAccel, float fDecel)
   * @brief     Send the b-CAP ID_ROBOT_ACCELERATE packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] lAxis Axis number.
   * @param[in] fAccel Acceleration value.
   * @param[in] fDecel Deceleration value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotAccelerate(int fd, uint32_t hRobot, int32_t lAxis, float fAccel,
      float fDecel);

  /**
   * @fn        HRESULT bCap_RobotChange(int fd, uint32_t hRobot, BSTR bstrName)
   * @brief     Send the b-CAP ID_ROBOT_CHANGE packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] bstrName Hand name.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotChange(int fd, uint32_t hRobot, BSTR bstrName);

  /**
   * @fn        HRESULT bCap_RobotChuck(int fd, uint32_t hRobot, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_CHUCK packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotChuck(int fd, uint32_t hRobot, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_RobotDrive(int fd, uint32_t hRobot, int32_t lNo, float fMov, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_DRIVE packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] lNo Axis number.
   * @param[in] fMov Movement value.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotDrive(int fd, uint32_t hRobot, int32_t lNo, float fMov,
      BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_RobotGoHome(int fd, uint32_t hRobot)
   * @brief     Send the b-CAP ID_ROBOT_GOHOME packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGoHome(int fd, uint32_t hRobot);

  /**
   * @fn        HRESULT bCap_RobotHalt(int fd, uint32_t hRobot, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_HALT packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotHalt(int fd, uint32_t hRobot, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_RobotHold(int fd, uint32_t hRobot, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_HOLD packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotHold(int fd, uint32_t hRobot, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_RobotMove(int fd, uint32_t hRobot, int32_t lComp, VARIANT vntPose, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_MOVE packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] lComp Move interpolation.
   * @param[in] vntPose Pose data.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotMove(int fd, uint32_t hRobot, int32_t lComp, VARIANT vntPose,
      BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_RobotRotate(int fd, uint32_t hRobot, VARIANT vntRotSuf, float fDeg, VARIANT vntPivot, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_ROTATE packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] vntRotSuf Rotate surface.
   * @param[in] fDeg Degree.
   * @param[in] vntPivot Center of rotation.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotRotate(int fd, uint32_t hRobot, VARIANT vntRotSuf, float fDeg,
      VARIANT vntPivot, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_RobotSpeed(int fd, uint32_t hRobot, int32_t lAxis, float fSpeed)
   * @brief     Send the b-CAP ID_ROBOT_SPEED packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] lAxis Axis number.
   * @param[in] fSpeed Sleep value to be set.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotSpeed(int fd, uint32_t hRobot, int32_t lAxis, float fSpeed);

  /**
   * @fn        HRESULT bCap_RobotUnchuck(int fd, uint32_t hRobot, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_UNCHUCK packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotUnchuck(int fd, uint32_t hRobot, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_RobotUnhold(int fd, uint32_t hRobot, BSTR bstrOption)
   * @brief     Send the b-CAP ID_ROBOT_UNHOLD packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotUnhold(int fd, uint32_t hRobot, BSTR bstrOption);

  /**
   * @fn         HRESULT bCap_RobotGetAttribute(int fd, uint32_t hRobot, int32_t *pVal)
   * @brief      Send the b-CAP ID_ROBOT_GETATTRIBUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hRobot Robot handle.
   * @param[out] pVal The gotten attribute value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGetAttribute(int fd, uint32_t hRobot, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_RobotGetHelp(int fd, uint32_t hRobot, BSTR *pVal)
   * @brief      Send the b-CAP ID_ROBOT_GETHELP packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hRobot Robot handle.
   * @param[out] pVal The gotten help string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGetHelp(int fd, uint32_t hRobot, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_RobotGetName(int fd, uint32_t hRobot, BSTR *pVal)
   * @brief      Send the b-CAP ID_ROBOT_GETNAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hRobot Robot handle.
   * @param[out] pVal The gotten name string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGetName(int fd, uint32_t hRobot, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_RobotGetTag(int fd, uint32_t hRobot, VARIANT *pVal)
   * @brief      Send the b-CAP ID_ROBOT_GETTAG packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hRobot Robot handle.
   * @param[out] pVal The gotten tag value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGetTag(int fd, uint32_t hRobot, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_RobotPutTag(int fd, uint32_t hRobot, VARIANT newVal)
   * @brief     Send the b-CAP ID_ROBOT_PUTTAG packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] newVal The tag value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotPutTag(int fd, uint32_t hRobot, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_RobotGetID(int fd, uint32_t hRobot, VARIANT *pVal)
   * @brief      Send the b-CAP ID_ROBOT_GETID packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hRobot Robot handle.
   * @param[out] pVal The gotten ID.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotGetID(int fd, uint32_t hRobot, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_RobotPutID(int fd, uint32_t hRobot, VARIANT newVal)
   * @brief     Send the b-CAP ID_ROBOT_PUTID packet.
   * @param[in] fd File descriptor.
   * @param[in] hRobot Robot handle.
   * @param[in] newVal The ID to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotPutID(int fd, uint32_t hRobot, VARIANT newVal);

  /**
   * @fn            HRESULT bCap_RobotRelease(int fd, uint32_t *hRobot)
   * @brief         Send the b-CAP ID_ROBOT_RELEASE packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hRobot Robot handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_RobotRelease(int fd, uint32_t *hRobot);

  /**
   * @fn         HRESULT bCap_TaskGetVariable(int fd, uint32_t hTask, BSTR bstrName, BSTR bstrOption, uint32_t *hVariable)
   * @brief      Send the b-CAP ID_TASK_GETVARIABLE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[in]  bstrName Variable name.
   * @param[in]  bstrOption Option strings.
   * @param[out] hVariable Variable handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetVariable(int fd, uint32_t hTask, BSTR bstrName, BSTR bstrOption,
      uint32_t *hVariable);

  /**
   * @fn         HRESULT bCap_TaskGetVariableNames(int fd, uint32_t hTask, BSTR bstrOption, VARIANT *pVal)
   * @brief      Send the b-CAP ID_TASK_GETVARIABLENAMES packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[in]  bstrOption Option strings.
   * @param[out] pVal The gotten names.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetVariableNames(int fd, uint32_t hTask, BSTR bstrOption,
      VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_TaskExecute(int fd, uint32_t hTask, BSTR bstrCommand, VARIANT vntParam, VARIANT *pVal)
   * @brief      Send the b-CAP ID_TASK_EXECUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[in]  bstrCommand Executing command name.
   * @param[in]  vntParam Executing parameters.
   * @param[out] pVal Result value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskExecute(int fd, uint32_t hTask, BSTR bstrCommand, VARIANT vntParam,
      VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_TaskStart(int fd, uint32_t hTask, int32_t lMode, BSTR bstrOption)
   * @brief     Send the b-CAP ID_TASK_START packet.
   * @param[in] fd File descriptor.
   * @param[in] hTask Task handle.
   * @param[in] lMode Start mode.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskStart(int fd, uint32_t hTask, int32_t lMode, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_TaskStop(int fd, uint32_t hTask, int32_t lMode, BSTR bstrOption)
   * @brief     Send the b-CAP ID_TASK_STOP packet.
   * @param[in] fd File descriptor.
   * @param[in] hTask Task handle.
   * @param[in] lMode Stop mode.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskStop(int fd, uint32_t hTask, int32_t lMode, BSTR bstrOption);

  /**
   * @fn        HRESULT bCap_TaskDelete(int fd, uint32_t hTask, BSTR bstrOption)
   * @brief     Send the b-CAP ID_TASK_DELETE packet.
   * @param[in] fd File descriptor.
   * @param[in] hTask Task handle.
   * @param[in] bstrOption Option strings.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskDelete(int fd, uint32_t hTask, BSTR bstrOption);

  /**
   * @fn         HRESULT bCap_TaskGetFileName(int fd, uint32_t hTask, BSTR *pVal)
   * @brief      Send the b-CAP ID_TASK_GETFILENAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[out] pVal The gotten file name.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetFileName(int fd, uint32_t hTask, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_TaskGetAttribute(int fd, uint32_t hTask, int32_t *pVal)
   * @brief      Send the b-CAP ID_TASK_GETATTRIBUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[out] pVal The gotten attribute value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetAttribute(int fd, uint32_t hTask, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_TaskGetHelp(int fd, uint32_t hTask, BSTR *pVal)
   * @brief      Send the b-CAP ID_TASK_GETHELP packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[out] pVal The gotten help string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetHelp(int fd, uint32_t hTask, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_TaskGetName(int fd, uint32_t hTask, BSTR *pVal)
   * @brief      Send the b-CAP ID_TASK_GETNAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[out] pVal The gotten name string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetName(int fd, uint32_t hTask, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_TaskGetTag(int fd, uint32_t hTask, VARIANT *pVal)
   * @brief      Send the b-CAP ID_TASK_GETTAG packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[out] pVal The gotten tag value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetTag(int fd, uint32_t hTask, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_TaskPutTag(int fd, uint32_t hTask, VARIANT newVal)
   * @brief     Send the b-CAP ID_TASK_PUTTAG packet.
   * @param[in] fd File descriptor.
   * @param[in] hTask Task handle.
   * @param[in] newVal The tag value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskPutTag(int fd, uint32_t hTask, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_TaskGetID(int fd, uint32_t hTask, VARIANT *pVal)
   * @brief      Send the b-CAP ID_TASK_GETID packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hTask Task handle.
   * @param[out] pVal The gotten ID.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskGetID(int fd, uint32_t hTask, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_TaskPutID(int fd, uint32_t hTask, VARIANT newVal)
   * @brief     Send the b-CAP ID_TASK_PUTID packet.
   * @param[in] fd File descriptor.
   * @param[in] hTask Task handle.
   * @param[in] newVal The ID to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskPutID(int fd, uint32_t hTask, VARIANT newVal);

  /**
   * @fn            HRESULT bCap_TaskRelease(int fd, uint32_t *hTask)
   * @brief         Send the b-CAP ID_TASK_RELEASE packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hTask Task handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_TaskRelease(int fd, uint32_t *hTask);

  /**
   * @fn         HRESULT bCap_VariableGetDateTime(int fd, uint32_t hVariable, VARIANT *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETDATETIME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten time value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetDateTime(int fd, uint32_t hVariable, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_VariableGetValue(int fd, uint32_t hVariable, VARIANT *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETVALUE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetValue(int fd, uint32_t hVariable, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_VariablePutValue(int fd, uint32_t hVariable, VARIANT newVal)
   * @brief     Send the b-CAP ID_VARIABLE_PUTVALUE packet.
   * @param[in] fd File descriptor.
   * @param[in] hVariable Variable handle.
   * @param[in] newVal The value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariablePutValue(int fd, uint32_t hVariable, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_VariableGetAttribute(int fd, uint32_t hVariable, int32_t *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETATTRIBUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten attribute value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetAttribute(int fd, uint32_t hVariable, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_VariableGetHelp(int fd, uint32_t hVariable, BSTR *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETHELP packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten help string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetHelp(int fd, uint32_t hVariable, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_VariableGetName(int fd, uint32_t hVariable, BSTR *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETNAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten name string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetName(int fd, uint32_t hVariable, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_VariableGetTag(int fd, uint32_t hVariable, VARIANT *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETTAG packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten tag value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetTag(int fd, uint32_t hVariable, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_VariablePutTag(int fd, uint32_t hVariable, VARIANT newVal)
   * @brief     Send the b-CAP ID_VARIABLE_PUTTAG packet.
   * @param[in] fd File descriptor.
   * @param[in] hVariable Variable handle.
   * @param[in] newVal The tag value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariablePutTag(int fd, uint32_t hVariable, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_VariableGetID(int fd, uint32_t hVariable, VARIANT *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETID packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten ID.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetID(int fd, uint32_t hVariable, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_VariablePutID(int fd, uint32_t hVariable, VARIANT newVal)
   * @brief     Send the b-CAP ID_VARIABLE_PUTID packet.
   * @param[in] fd File descriptor.
   * @param[in] hVariable Variable handle.
   * @param[in] newVal The ID to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariablePutID(int fd, uint32_t hVariable, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_VariableGetMicrosecond(int fd, uint32_t hVariable, int32_t *pVal)
   * @brief      Send the b-CAP ID_VARIABLE_GETMICROSECOND packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hVariable Variable handle.
   * @param[out] pVal The gotten time value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableGetMicrosecond(int fd, uint32_t hVariable, int32_t *pVal);

  /**
   * @fn            HRESULT bCap_VariableRelease(int fd, uint32_t *hVariable)
   * @brief         Send the b-CAP ID_VARIABLE_RELEASE packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hVariable Variable handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_VariableRelease(int fd, uint32_t *hVariable);

  /**
   * @fn         HRESULT bCap_CommandExecute(int fd, uint32_t hCommand, int32_t lMode, VARIANT *pVal)
   * @brief      Send the b-CAP ID_COMMAND_EXECUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[in]  lMode Execution mode.
   * @param[out] pVal Result value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandExecute(int fd, uint32_t hCommand, int32_t lMode, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_CommandCancel(int fd, uint32_t hCommand)
   * @brief     Send the b-CAP ID_COMMAND_CANCEL packet.
   * @param[in] fd File descriptor.
   * @param[in] hCommand Command handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandCancel(int fd, uint32_t hCommand);

  /**
   * @fn         HRESULT bCap_CommandGetTimeout(int fd, uint32_t hCommand, int32_t *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETTIMEOUT packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten timeout value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetTimeout(int fd, uint32_t hCommand, int32_t *pVal);

  /**
   * @fn        HRESULT bCap_CommandPutTimeout(int fd, uint32_t hCommand, int32_t newVal)
   * @brief     Send the b-CAP ID_COMMAND_PUTTIMEOUT packet.
   * @param[in] fd File descriptor.
   * @param[in] hCommand Command handle.
   * @param[in] newVal The timeout value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandPutTimeout(int fd, uint32_t hCommand, int32_t newVal);

  /**
   * @fn         HRESULT bCap_CommandGetState(int fd, uint32_t hCommand, int32_t *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETSTATE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten state value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetState(int fd, uint32_t hCommand, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_CommandGetParameters(int fd, uint32_t hCommand, VARIANT *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETPARAMETERS packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten parameters.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetParameters(int fd, uint32_t hCommand, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_CommandPutParameters(int fd, uint32_t hCommand, VARIANT newVal)
   * @brief     Send the b-CAP ID_VARIABLE_PUTPARAMETERS packet.
   * @param[in] fd File descriptor.
   * @param[in] hCommand Command handle.
   * @param[in] newVal The parameters to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandPutParameters(int fd, uint32_t hCommand, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_CommandGetResult(int fd, uint32_t hCommand, VARIANT *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETRESULT packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten result.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetResult(int fd, uint32_t hCommand, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_CommandGetAttribute(int fd, uint32_t hCommand, int32_t *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETATTRIBUTE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten attribute value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetAttribute(int fd, uint32_t hCommand, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_CommandGetHelp(int fd, uint32_t hCommand, BSTR *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETHELP packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten help string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetHelp(int fd, uint32_t hCommand, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_CommandGetName(int fd, uint32_t hCommand, BSTR *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETNAME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten name string.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetName(int fd, uint32_t hCommand, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_CommandGetTag(int fd, uint32_t hCommand, VARIANT *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETTAG packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten tag value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetTag(int fd, uint32_t hCommand, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_CommandPutTag(int fd, uint32_t hCommand, VARIANT newVal)
   * @brief     Send the b-CAP ID_COMMAND_PUTTAG packet.
   * @param[in] fd File descriptor.
   * @param[in] hCommand Command handle.
   * @param[in] newVal The tag value to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandPutTag(int fd, uint32_t hCommand, VARIANT newVal);

  /**
   * @fn         HRESULT bCap_CommandGetID(int fd, uint32_t hCommand, VARIANT *pVal)
   * @brief      Send the b-CAP ID_COMMAND_GETID packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hCommand Command handle.
   * @param[out] pVal The gotten ID.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandGetID(int fd, uint32_t hCommand, VARIANT *pVal);

  /**
   * @fn        HRESULT bCap_CommandPutID(int fd, uint32_t hCommand, VARIANT newVal)
   * @brief     Send the b-CAP ID_COMMAND_PUTID packet.
   * @param[in] fd File descriptor.
   * @param[in] hCommand Command handle.
   * @param[in] newVal The ID to be put.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandPutID(int fd, uint32_t hCommand, VARIANT newVal);

  /**
   * @fn            HRESULT bCap_CommandRelease(int fd, uint32_t *hCommand)
   * @brief         Send the b-CAP ID_COMMAND_RELEASE packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hCommand Command handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_CommandRelease(int fd, uint32_t *hCommand);

  /**
   * @fn        HRESULT bCap_MessageReply(int fd, uint32_t hMessage, VARIANT vntData)
   * @brief     Send the b-CAP ID_MESSAGE_REPLY packet.
   * @param[in] fd File descriptor.
   * @param[in] hMessage Message handle.
   * @param[in] vntData The reply data.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageReply(int fd, uint32_t hMessage, VARIANT vntData);

  /**
   * @fn        HRESULT bCap_MessageClear(int fd, uint32_t hMessage)
   * @brief     Send the b-CAP ID_MESSAGE_CLEAR packet.
   * @param[in] fd File descriptor.
   * @param[in] hMessage Message handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageClear(int fd, uint32_t hMessage);

  /**
   * @fn         HRESULT bCap_MessageGetDateTime(int fd, uint32_t hMessage, VARIANT *pVal)
   * @brief      Send the b-CAP ID_MESSAGE_GETDATETIME packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hMessage Message handle.
   * @param[out] pVal The gotten time value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageGetDateTime(int fd, uint32_t hMessage, VARIANT *pVal);

  /**
   * @fn         HRESULT bCap_MessageGetDescription(int fd, uint32_t hMessage, BSTR *pVal)
   * @brief      Send the b-CAP ID_MESSAGE_GETDESCRIPTION packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hMessage Message handle.
   * @param[out] pVal The gotten description.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageGetDescription(int fd, uint32_t hMessage, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_MessageGetDestination(int fd, uint32_t hMessage, BSTR *pVal)
   * @brief      Send the b-CAP ID_MESSAGE_GETDESTINATION packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hMessage Message handle.
   * @param[out] pVal The gotten destination.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageGetDestination(int fd, uint32_t hMessage, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_MessageGetNumber(int fd, uint32_t hMessage, int32_t *pVal)
   * @brief      Send the b-CAP ID_MESSAGE_GETNUMBER packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hMessage Message handle.
   * @param[out] pVal The gotten number.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageGetNumber(int fd, uint32_t hMessage, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_MessageGetSerialNumber(int fd, uint32_t hMessage, int32_t *pVal)
   * @brief      Send the b-CAP ID_MESSAGE_GETSERIALNUMBER packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hMessage Message handle.
   * @param[out] pVal The gotten serial number.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageGetSerialNumber(int fd, uint32_t hMessage, int32_t *pVal);

  /**
   * @fn         HRESULT bCap_MessageGetSource(int fd, uint32_t hMessage, BSTR *pVal)
   * @brief      Send the b-CAP ID_MESSAGE_GETSOURCE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hMessage Message handle.
   * @param[out] pVal The gotten source.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageGetSource(int fd, uint32_t hMessage, BSTR *pVal);

  /**
   * @fn         HRESULT bCap_MessageGetValue(int fd, uint32_t hMessage, VARIANT *pVal)
   * @brief      Send the b-CAP ID_MESSAGE_GETVALUE packet.
   * @param[in]  fd File descriptor.
   * @param[in]  hMessage Message handle.
   * @param[out] pVal The gotten value.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageGetValue(int fd, uint32_t hMessage, VARIANT *pVal);

  /**
   * @fn            HRESULT bCap_MessageRelease(int fd, uint32_t *hMessage)
   * @brief         Send the b-CAP ID_MESSAGE_RELEASE packet.
   * @param[in]     fd The file descriptor.
   * @param[in,out] hMessage Message handle.
   */
  _BCAP_EXP_CLIENT HRESULT
  bCap_MessageRelease(int fd, uint32_t *hMessage);

#ifdef __cplusplus
}
#endif

#endif /* BCAP_CLIENT_H_ */
