#ifndef DN_ROBOTALK_H_
#define DN_ROBOTALK_H_

/**
 * @file    dn_robotalk.h
 * @brief   ROBOTalk API file.
 * @details Defines ROBOTalk APIs.
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

#ifndef _DN_EXP_ROBOTALK
#define _DN_EXP_ROBOTALK
#endif /* _DN_EXP_ROBOTALK */

/**
 * @def   RTK_CHECK_TYPE
 * @brief A definition for the check_conn_param function.
 * @note  ROBOTalk can communicate with UDP and COM.
 */
#define RTK_CHECK_TYPE (CHECK_TYPE_UDP | CHECK_TYPE_COM)

/**
 * @def   RTK_CHECK_SEND
 * @brief A definition for the check_conn_param function.
 */
#define RTK_CHECK_SEND (RTK_CHECK_TYPE | CHECK_FUNC_SEND)

/**
 * @def   RTK_CHECK_RECV
 * @brief A definition for the check_conn_param function.
 */
#define RTK_CHECK_RECV (RTK_CHECK_TYPE | CHECK_FUNC_SEND | CHECK_FUNC_RECV)

/**
 * @def   RTK_ENQ
 * @brief A definition for the ROBOTalk enquiry.
 */
#define RTK_ENQ (0x05)

/**
 * @def   RTK_CMD_REJ
 * @brief A definition for the ROBOTalk command which means reject.
 */
#define RTK_CMD_REJ (0x0E01)

/**
 * @def   RTK_CMD_ACK
 * @brief A definition for the ROBOTalk command which means acknowledge.
 */
#define RTK_CMD_ACK (0x0E02)

/**
 * @def   RTK_CMD_NAK
 * @brief A definition for the ROBOTalk command which means negative acknowledge.
 */
#define RTK_CMD_NAK (0x0E82)

/**
 * @def   RTK_SIZE_PACKET
 * @brief The maximum buffer size of a ROBOTalk packet.
 */
#define RTK_SIZE_PACKET (256)

/**
 * @def   RTK_SIZE_HEADER
 * @brief The header buffer size of a ROBOTalk packet.
 */
#define RTK_SIZE_HEADER (6)

/**
 * @def   RTK_SIZE_DATA
 * @brief The data buffer size of a ROBOTalk packet.
 */
#define RTK_SIZE_DATA (RTK_SIZE_PACKET - RTK_SIZE_HEADER)

/**
 * @def   RTK_SIZE_COMMAND
 * @brief The command buffer size of a ROBOTalk packet.
 */
#define RTK_SIZE_COMMAND (2)

/**
 * @def   RTK_SIZE_CRC
 * @brief The CRC buffer size of a ROBOTalk packet.
 */
#define RTK_SIZE_CRC (2)

/**
 * @def   RTK_RETRY_FLAG
 * @brief A definition for the ROBOTalk command. This bit means the retry packet.
 */
#define RTK_RETRY_FLAG (0x1000)

/**
 * @def   RTK_RETRY_COUNT
 * @brief A definition for the ROBOTalk command. The upper 3bits mean the retry count.
 */
#define RTK_RETRY_COUNT (0x2000)

/**
 * @def   RTK_RETRY_MASK
 * @brief A definition for the ROBOTalk command. The upper 4bits mean the retry packet.
 */
#define RTK_RETRY_MASK (0xF000)

/**
 * @def       NativeCommand(cmd)
 * @brief     A macro that removes the retry flag and count from the ROBOTalk command.
 * @param[in] cmd The ROBOTalk command.
 */
#define NativeCommand(cmd) ((cmd) & ~RTK_RETRY_MASK)

/**
 * @union  RTK_PACKET
 * @brief  A type definition for the ROBOTalk packet.
 * @note   The ROBOTalk packet's byte order is big endian.
 */
union RTK_PACKET
{
  struct
  {
    uint8_t enq;      /**< Enquiry.                                              */
    uint8_t len;      /**< The length of data. This does not contain CRC length. */
    uint16_t command; /**< Command.                                              */
    uint8_t from_id;  /**< Id of host.                                           */
    uint8_t to_id;    /**< Id of the target client.                              */
    uint8_t data[RTK_SIZE_DATA]; /**< Data. The tail 2bits is used for CRC.      */
  };
  uint8_t buf[RTK_SIZE_PACKET];  /**< Buffer. */
};

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn         HRESULT rtk_param2packet(uint16_t command, const uint8_t *data, uint8_t len_data, uint8_t from_id, uint8_t to_id, union RTK_PACKET *dst)
   * @brief      Creates a ROBOTalk packet.
   * @param[in]  command The ROBOTalk command.
   * @param[in]  data The ROBOTalk data.
   * @param[in]  len_data The length of data.
   * @param[in]  from_id The id of host.
   * @param[in]  to_id The id of the target client.
   * @param[out] packet The created ROBOTalk packet.
   */
  _DN_EXP_ROBOTALK HRESULT
  rtk_param2packet(uint16_t command, const uint8_t *data, uint8_t len_data,
      uint8_t from_id, uint8_t to_id, union RTK_PACKET *packet);

  /**
   * @fn        uint16_t rtk_calc_crc(const union RTK_PACKET *packet)
   * @brief     Calculates CRC of the ROBOTalk packet.
   * @param[in] packet The ROBOTalk packet to be calculated.
   */
  _DN_EXP_ROBOTALK uint16_t
  rtk_calc_crc(const union RTK_PACKET *packet);

  /**
   * @fn        HRESULT rtk_send(const struct CONN_PARAM_COMMON *device, union RTK_PACKET *packet_send)
   * @brief     Sends ROBOTalk packet.
   * @param[in] device The common communication parameters.
   * @param[in] packet_send The ROBOTalk packet to be sent.
   */
  _DN_EXP_ROBOTALK HRESULT
  rtk_send(const struct CONN_PARAM_COMMON *device,
      union RTK_PACKET *packet_send);

  /**
   * @fn         HRESULT rtk_recv(const struct CONN_PARAM_COMMON *device, union RTK_PACKET *packet_recv, int client, unsigned int retry_nak)
   * @brief      Receives ROBOTalk packet.
   * @param[in]  device The common communication parameters.
   * @param[out] packet_recv The ROBOTalk packet to be received.
   * @param[in]  client Flag that means the receiver is client(1) or not(0).
   * @param[in]  retry_nak The retry count when receiving the NAK packet.
   */
  _DN_EXP_ROBOTALK HRESULT
  rtk_recv(const struct CONN_PARAM_COMMON *device,
      union RTK_PACKET *packet_recv, int client, unsigned int retry_nak);

#ifdef __cplusplus
}
#endif

#endif /* DN_ROBOTALK_H_ */
