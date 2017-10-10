#ifndef DN_TPCOMM_H_
#define DN_TPCOMM_H_

/**
 * @file    dn_tpcomm.h
 * @brief   TP communication API file.
 * @details Defines TP communication APIs.
 *
 * @version 1.1
 * @date    2014/11/06
 * @date    2015/01/21 Adds Mini TP functions.
 * @author  DENSO WAVE
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

#ifndef _DN_EXP_TPCOMM
#define _DN_EXP_TPCOMM
#endif /* _DN_EXP_TPCOMM */

#include "../dn_common.h"

/**
 * @def   TP_CONN_MAX
 * @brief A definition for the maximum count of TP connections.
 * @note  You can change this parameter to 1 or more.
 */
#define TP_CONN_MAX (1)

/**
 * @def   TP_RETRY_NAK
 * @brief A definition for the retry count of sending and receiving NAK packet.
 */
#define TP_RETRY_NAK (2)

/**
 * @def   TP_RETRY_TIMEOUT
 * @brief A definition for the retry count of communication timeout.
 */
#define TP_RETRY_TIMEOUT (2)

/**
 * @def   TP_ID_CLIENT
 * @brief A definition for the client ID.
 */
#define TP_ID_CLIENT (0x11)

/**
 * @def   TP_ID_SERVER
 * @brief A definition for the server ID.
 */
#define TP_ID_SERVER (0x90)

/**
 * @def   TP_CMD_REQ_ID
 * @brief A definition for the ROBOTalk command which means requests ID.
 */
#define TP_CMD_REQ_ID (0x0EF0)

/**
 * @def   TP_CMD_KEYINFO
 * @brief A definition for the ROBOTalk command which means requests or raised key event.
 */
#define TP_CMD_KEYINFO (0x0C20)

/**
 * @def   TP_CMD_TOUTCHINFO
 * @brief A definition for the ROBOTalk command which means requests or raised touch event.
 */
#define TP_CMD_TOUTCHINFO (0x0C21)

/**
 * @def   TP_CMD_BEEP
 * @brief A definition for the ROBOTalk command which means requests BEEP.
 */
#define TP_CMD_BEEP (0x0C28)

/**
 * @def   TP_CMD_LED_ON
 * @brief A definition for the ROBOTalk command which means requests LED on.
 */
#define TP_CMD_LED_ON (0x0C29)

/**
 * @def   TP_CMD_LED_OFF
 * @brief A definition for the ROBOTalk command which means requests LED off.
 */
#define TP_CMD_LED_OFF (0x0C2A)

/**
 * @def   TP_CMD_LED_FLASH
 * @brief A definition for the ROBOTalk command which means requests LED flash.
 */
#define TP_CMD_LED_FLASH (0x0C2B)

/**
 * @def   TP_CMD_GET_KEYSTATE
 * @brief A definition for the ROBOTalk command which means requests key state.
 */
#define TP_CMD_GET_KEYSTATE (0x0C2D)

/**
 * @def   TP_CMD_BKL_ON
 * @brief A definition for the ROBOTalk command which means requests backlight on.
 */
#define TP_CMD_BKL_ON (0x0C33)

/**
 * @def   TP_CMD_LCD
 * @brief A definition for the ROBOTalk command which means requests change LCD.
 */
#define TP_CMD_LCD (0x0C49)

/**
 * @def   TP_CMD_MTP_DRAW
 * @brief A definition for the ROBOTalk command which means requests draw Mini TP.
 */
#define TP_CMD_MTP_DRAW (0x0C81)

/**
 * @def   TP_CMD_PING
 * @brief A definition for the ROBOTalk command which means requests check wire.
 */
#define TP_CMD_PING (0x0C2F)

/**
 * @enum  TP_STATE
 * @brief TP connection state.
 */
enum TP_STATE
{
  TP_DISCONNECT = 0, /**< Disconnected. */
  TP_CONNECT,        /**< Connected.    */
  TP_TPLESS,         /**< TP less.      */
  TP_TPERROR,        /**< Error.        */
};

/**
 * @enum  TP_KEY_ENUM
 * @brief TP key information.
 */
enum TP_KEY_ENUM
{
  DIAL_MODE_AUTO = 1,   /**< Auto mode.               */
  DIAL_MODE_MANUAL = 2, /**< Manual mode.             */
  DIAL_MODE_TEACH = 4,  /**< Teach check mode.        */
  BTN_DIR_UP = 1,       /**< Up button pressed.       */
  BTN_DIR_DOWN = 2,     /**< Down button pressed.     */
  BTN_DIR_LEFT = 4,     /**< Left button pressed.     */
  BTN_DIR_RIGHT = 8,    /**< Right button pressed.    */
  BTN_FUNC_F1 = 1,      /**< F1 button pressed.       */
  BTN_FUNC_F2 = 2,      /**< F2 button pressed.       */
  BTN_FUNC_F3 = 4,      /**< F3 button pressed.       */
  BTN_FUNC_F4 = 8,      /**< F4 button pressed.       */
  BTN_FUNC_F5 = 16,     /**< F5 button pressed.       */
  BTN_FUNC_F6 = 32,     /**< F6 button pressed.       */
  DIAL_DIR_MINUS = 1,   /**< Dial minus               */
  DIAL_DIR_PLUS = 2,    /**< Dial plus                */
  JOG_MINUS_J1 = 1,     /**< J1 minus button pressed. */
  JOG_MINUS_J2 = 2,     /**< J2 minus button pressed. */
  JOG_MINUS_J3 = 4,     /**< J3 minus button pressed. */
  JOG_MINUS_J4 = 8,     /**< J4 minus button pressed. */
  JOG_MINUS_J5 = 16,    /**< J5 minus button pressed. */
  JOG_MINUS_J6 = 32,    /**< J6 minus button pressed. */
  JOG_PLUS_J1 = 1,      /**< J1 plus button pressed.  */
  JOG_PLUS_J2 = 2,      /**< J2 plus button pressed.  */
  JOG_PLUS_J3 = 4,      /**< J3 plus button pressed.  */
  JOG_PLUS_J4 = 8,      /**< J4 plus button pressed.  */
  JOG_PLUS_J5 = 16,     /**< J5 plus button pressed.  */
  JOG_PLUS_J6 = 32,     /**< J6 plus button pressed.  */
};

/**
 * @enum  TP_TOUCH_ENUM
 * @brief TP touch information.
 */
enum TP_TOUCH_ENUM
{
  MODE_TOUCH = 0x54,   /**< TP touched.  */
  MODE_RELEASE = 0x52, /**< TP released. */
};

/**
 * @enum  LED_STATE
 * @brief LED state information.
 */
enum LED_STATE
{
  LED_OFF = 0, /**< LED off.   */
  LED_ON,      /**< LED on.    */
  LED_FLASH,   /**< LED flash. */
};

/**
 * @enum  MINITP_COMMAND
 * @brief Mini TP command information
 */
enum MINITP_COMMAND
{
  COLOR_FG = 0x10,    /**< Sets character color.  */
  COLOR_BG = 0x11,    /**< Sets background color. */
  COLOR_FILL = 0x12,  /**< Sets fill color.       */
  DRAW_STRING = 0x20, /**< Draws a string.        */
  DRAW_LINE = 0x30,   /**< Draws a line.          */
  DRAW_RECT = 0x31,   /**< Draws a rectangle.     */
};

/**
 *  @enum  MINITP_PARAM
 *  @brief Mini TP parameter information
 */
enum MINITP_PARAM
{
  COLOR_BLACK = 0,    /**< Color black.                                   */
  COLOR_WHITE = 0xF,  /**< Color white.                                   */
  COLOR_TRANS = 0xFF, /**< Color transparent.                             */
  SIZE_SMALL = 0,     /**< String size to small.                          */
  SIZE_MIDDLE = 1,    /**< String size to middle.                         */
  SIZE_LARGE = 2,     /**< String size to large.                          */
  SIZE_LARGE_H = 3,   /**< String size to large and half width character. */
  ATTR_NORMAL = 0,    /**< String attribute to normal.                    */
  ATTR_REVERSE = 1,   /**< String attribute to reverse.                   */
  ATTR_FLASH = 2,     /**< String attribute to flash.                     */
  ATTR_EMPHASIS = 4,  /**< String attribute to emphasis.                  */
  ATTR_UNDERLINE = 8, /**< String attribute to underline.                 */
};

/**
 * @struct TP_KEY_INFO
 * @brief  TP key bit mapping.
 */
struct TP_KEY_INFO
{
  uint64_t rsv1 :2;       /**< Not used.                   (Bit:  0 -  1) */
  uint64_t btn_stop :1;   /**< Stop button pressed.        (Bit:  2     ) */
  uint64_t dial_mode :3;  /**< Dial mode.                  (Bit:  3 -  5) */
  uint64_t btn_motor :1;  /**< Motor button pressed.       (Bit:  6     ) */
  uint64_t rsv2 :3;       /**< Not used.                   (Bit:  7 -  9) */
  uint64_t btn_dir :4;    /**< Direction buttons pressed.  (Bit: 10 - 13) */
  uint64_t btn_ok :1;     /**< OK button pressed.          (Bit: 14     ) */
  uint64_t btn_cancel :1; /**< Cancel button pressed.      (Bit: 15     ) */
  uint64_t btn_lock :1;   /**< Lock button pressed.        (Bit: 16     ) */
  uint64_t btn_r_sel :1;  /**< R-SEL button pressed.       (Bit: 17     ) */
  uint64_t btn_m_mod :1;  /**< M-MOD button pressed.       (Bit: 18     ) */
  uint64_t btn_speed :1;  /**< Speed button pressed.       (Bit: 19     ) */
  uint64_t btn_shift :1;  /**< Shift button pressed.       (Bit: 20     ) */
  uint64_t btn_func :6;   /**< Function buttons pressed.   (Bit: 21 - 26) */
  uint64_t rsv3 :4;       /**< Not used.                   (Bit: 27 - 30) */
  uint64_t dial_dir :2;   /**< Dial direction.             (Bit: 31 - 32) */
  uint64_t rsv4 :8;       /**< Not used.                   (Bit: 33 - 40) */
  uint64_t jog_minus :6;  /**< J[n] minus buttons pressed. (Bit: 41 - 46) */
  uint64_t rsv5 :4;       /**< Not used.                   (Bit: 47 - 50) */
  uint64_t jog_plus :6;   /**< J[n] plus buttons pressed.  (Bit: 51 - 56) */
  uint64_t rsv6 :7;       /**< Not used.                   (Bit: 57 - 63) */
};

/**
 * @struct TP_TOUCH_INFO
 * @brief  TP touch bit mapping.
 */
struct TP_TOUCH_INFO
{
  uint8_t mode;   /**< Event type: TOUCH_INFO.         */
  uint16_t pos_x; /**< X axis of the current position. */
  uint16_t pos_y; /**< Y axis of the current position. */
};

/**
 * @struct CALL_FUNC_TP
 * @brief  Callback functions of TP event.
 * @note   You should set these callback functions for catching TP event.
 */
struct CALL_FUNC_TP
{
  HRESULT
  (*Call_TPState)(int state); /**< Called when the TP connection state is changed. */
  HRESULT
  (*Call_TPKeyInfo)(struct TP_KEY_INFO key_info); /**< Called when the TP key state is changed. */
  HRESULT
  (*Call_TPTouchInfo)(struct TP_TOUCH_INFO touch_info); /**< Called when the TP touched or released. */
#if defined(_DEBUG)
  HRESULT (*Call_TPDefault)(uint16_t command, const uint8_t *data, uint8_t len_data);
#endif
};

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn        HRESULT TPComm_SetCallFunc(const struct CALL_FUNC_TP *func)
   * @brief     Sets callback functions.
   * @param[in] func Callback functions to be set.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_SetCallFunc(const struct CALL_FUNC_TP *func);

  /**
   * @fn         HRESULT TPComm_Open(const char *connect, uint32_t timeout, int client, int *pfd)
   * @brief      Starts TP communication.
   * @param[in]  connect Connection parameters.
   * @param[in]  timeout Timeout value.
   * @param[in]  client TP mode flag. (Client: = 1, Server: 0)
   * @param[out] pfd The pointer of File descriptor.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_Open(const char *connect, uint32_t timeout, int client, int *pfd);

  /**
   * @fn            HRESULT TPComm_Close(int *pfd)
   * @brief         Ends TP communication.
   * @param[in,out] pfd The pointer of File descriptor.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_Close(int *pfd);

  /**
   * @fn        HRESULT TPComm_SetTimeout(int fd, uint32_t timeout)
   * @brief     Sets timeout.
   * @param[in] fd File descriptor.
   * @param[in] timeout Timeout value to be set.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_SetTimeout(int fd, uint32_t timeout);

  /**
   * @fn         HRESULT TPComm_GetTimeout(int fd, uint32_t *timeout)
   * @brief      Gets timeout
   * @param[in]  fd File descriptor.
   * @param[out] timeout The gotten timeout value.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_GetTimeout(int fd, uint32_t *timeout);

  /**
   * @fn         HRESULT TPComm_GetTPState(int fd, int *state)
   * @brief      Gets TP state.
   * @param[in]  fd File descriptor.
   * @param[out] state The gotten TP state.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_GetTPState(int fd, int *state);

  /**
   * @fn        HRESULT TPComm_BEEP(int fd, int16_t time)
   * @brief     TP beeps.
   * @param[in] fd File descriptor.
   * @param[in] time Beeping time.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_BEEP(int fd, int16_t time);

  /**
   * @fn        HRESULT TPComm_LED(int fd, int16_t number, int16_t state)
   * @brief     Changes TP LED.
   * @param[in] fd File descriptor.
   * @param[in] number The LED number. (1: MORTOR, 2: LOCK, -1: Both)
   * @param[in] state State to be changed. (0: OFF, 1: ON, 2: FLASH)
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_LED(int fd, int16_t number, int16_t state);

  /**
   * @fn        HRESULT TPComm_LCD(int fd, int16_t contrast)
   * @brief     Changes TP LCD.
   * @param[in] fd File descriptor.
   * @param[in] contrast The LCD value. (0: The most darken, 7: The most brightest)
   * note       It will take 2 or more seconds to reflect the set LDC value completely.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_LCD(int fd, int16_t contrast);

  /**
   * @fn        HRESULT TPComm_LCD(int fd, int16_t contrast)
   * @brief     Changes TP LCD.
   * @param[in] fd File descriptor.
   * @param[in] contrast The LCD value. (0: The most darken, 7: The most brightest)
   * note       It will take 2 or more seconds to reflect the set LDC value completely.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_LCD(int fd, int16_t contrast);

  /**
   * @fn        HRESULT TPComm_DrawMiniTP(int fd, VARIANT commands)
   * @brief     Sends Draw Mini TP commands.
   * @param[in] fd File descriptor.
   * @param[in] commands Mini TP commands.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_DrawMiniTP(int fd, VARIANT commands);

  /**
   * @fn        HRESULT TPComm_DrawString(int fd, BSTR bstr, uint8_t pos_x, uint8_t pos_y, uint8_t size, uint8_t attr, uint8_t color_fg, uint8_t color_bg)
   * @brief     Draws a string to Mini TP.
   * @param[in] fd File descriptor.
   * @param[in] bstr The drawing string.
   * @param[in] pos_x The drawing position for axis X.
   * @param[in] pos_y The drawing position for axis Y.
   * @param[in] size The size of drawing string.
   * @param[in] attr The attribute of drawing string.
   * @param[in] color_fg The color of drawing string.
   * @param[in] color_bg The background color of drawing string.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_DrawString(int fd, BSTR bstr, uint8_t pos_x, uint8_t pos_y,
      uint8_t size, uint8_t attr, uint8_t color_fg, uint8_t color_bg);

  /**
   * @fn        HRESULT TPComm_DrawLine(int fd, uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y, uint8_t color_fg)
   * @brief     Draws a line to Mini TP.
   * @param[in] fd File descriptor.
   * @param[in] start_x The start position for axis X.
   * @param[in] start_y The start position for axis Y.
   * @param[in] end_x The end position for axis X.
   * @param[in] end_y The end position for axis Y.
   * @param[in] color_fg The color of drawing line.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_DrawLine(int fd, uint8_t start_x, uint8_t start_y, uint8_t end_x,
      uint8_t end_y, uint8_t color_fg);

  /**
   * @fn        HRESULT TPComm_DrawRectangle(int fd, uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y, uint8_t color_fg, uint8_t color_bg)
   * @brief     Draws a rectangle to Mini TP.
   * @param[in] fd File descriptor.
   * @param[in] start_x The start position for axis X.
   * @param[in] start_y The start position for axis Y.
   * @param[in] end_x The end position for axis X.
   * @param[in] end_y The end position for axis Y.
   * @param[in] color_fg The color of drawing rectangle.
   * @param[in] color_fg The background color of drawing rectangle.
   */
  _DN_EXP_TPCOMM HRESULT
  TPComm_DrawRectangle(int fd, uint8_t start_x, uint8_t start_y, uint8_t end_x,
      uint8_t end_y, uint8_t color_fg, uint8_t color_bg);

#ifdef __cplusplus
}
#endif

#endif /* DN_TPCOMM_H_ */
