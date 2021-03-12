#ifndef BCAP_COMMON_H_
#define BCAP_COMMON_H_

/**
 * @file    bcap_common.h
 * @brief   b-CAP Common API file.
 * @details Defines b-CAP common APIs.
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

#ifndef _BCAP_EXP_COMMON
#define _BCAP_EXP_COMMON
#endif /* _BCAP_EXP_COMMON */

/**
 * @def   BCAP_CHECK_TYPE
 * @brief A definition for the check_conn_param function.
 * @note  b-CAP does self check communication type can communicate with UDP and COM.
 */
#define BCAP_CHECK_TYPE (0)

/**
 * @def   BCAP_CHECK_SEND
 * @brief A definition for the check_conn_param function.
 */
#define BCAP_CHECK_SEND (BCAP_CHECK_TYPE | CHECK_FUNC_SEND)

/**
 * @def   BCAP_CHECK_RECV
 * @brief A definition for the check_conn_param function.
 */
#define BCAP_CHECK_RECV (BCAP_CHECK_TYPE | CHECK_FUNC_RECV)

/**
 * @def   S_EXECUTING
 * @brief Succeeded but the server has been executing yet.
 */
#define S_EXECUTING _HRESULT_TYPEDEF_(0x00000900L)

/**
 * @def   BCAP_HEADER
 * @brief A definition for the b-CAP header.
 */
#define BCAP_HEADER (0x01)

/**
 * @def   BCAP_TERMINATOR
 * @brief A definition for the b-CAP terminator.
 */
#define BCAP_TERMINATOR (0x04)

/**
 * @def   BCAP_SIZE_HEADER
 * @brief A definition for the size of b-CAP header.
 */
#define BCAP_SIZE_HEADER (1)

/**
 * @def   BCAP_SIZE_TERMINATOR
 * @brief A definition for the size of b-CAP terminator.
 */
#define BCAP_SIZE_TERMINATOR (1)

/**
 * @def   BCAP_SIZE_LEN
 * @brief A definition for the size of b-CAP message length.
 */
#define BCAP_SIZE_LEN (4)

/**
 * @def   BCAP_SIZE_SERIAL
 * @brief A definition for the size of b-CAP serial number.
 */
#define BCAP_SIZE_SERIAL (2)

/**
 * @def   BCAP_SIZE_RESERVE
 * @brief A definition for the size of b-CAP version or retry count.
 */
#define BCAP_SIZE_RESERVE (2)

/**
 * @def   BCAP_SIZE_ID
 * @brief A definition for the size of b-CAP function ID or return code.
 */
#define BCAP_SIZE_ID (4)

/**
 * @def   BCAP_SIZE_ARGC
 * @brief A definition for the size of b-CAP number of args.
 */
#define BCAP_SIZE_ARGC (2)

/**
 * @def   BCAP_SIZE_DATA_LEN
 * @brief A definition for the size of b-CAP argument data length.
 */
#define BCAP_SIZE_DATA_LEN (4)

/**
 * @def   BCAP_POS_HEADER
 * @brief A definition for the buffer position of b-CAP header.
 */
#define BCAP_POS_HEADER (0)

/**
 * @def   BCAP_POS_LEN
 * @brief A definition for the buffer position of b-CAP message length.
 */
#define BCAP_POS_LEN (BCAP_POS_HEADER + BCAP_SIZE_HEADER)

/**
 * @def   BCAP_POS_SERIAL
 * @brief A definition for the buffer position of b-CAP serial number.
 */
#define BCAP_POS_SERIAL (BCAP_POS_LEN + BCAP_SIZE_LEN)

/**
 * @def   BCAP_POS_RESERVE
 * @brief A definition for the buffer position of b-CAP version or retry count.
 */
#define BCAP_POS_RESERVE (BCAP_POS_SERIAL + BCAP_SIZE_SERIAL)

/**
 * @def   BCAP_POS_ID
 * @brief A definition for the buffer position of b-CAP function ID or return code.
 */
#define BCAP_POS_ID (BCAP_POS_RESERVE + BCAP_SIZE_RESERVE)

/**
 * @def   BCAP_POS_ARGC
 * @brief A definition for the buffer position of b-CAP number of args.
 */
#define BCAP_POS_ARGC (BCAP_POS_ID + BCAP_SIZE_ID)

/**
 * @def   BCAP_POS_DATA_LEN
 * @brief A definition for the buffer position of b-CAP argument data length.
 */
#define BCAP_POS_DATA_LEN (BCAP_POS_ARGC + BCAP_SIZE_ARGC)

/**
 * @def   BCAP_SIZE_INFO_LEN
 * @brief A definition for the size of b-CAP information message length.
 * @note  BCAP_SIZE_LEN + BCAP_SIZE_SERIAL + BCAP_SIZE_RESERVE + BCAP_SIZE_ID + BCAP_SIZE_ARGC
 */
#define BCAP_SIZE_INFO_LEN (14)

/**
 * @def   BCAP_SIZE_MIN
 * @brief A definition for the minimum buffer size of a b-CAP packet.
 */
#define BCAP_SIZE_MIN (BCAP_SIZE_HEADER + BCAP_SIZE_INFO_LEN + BCAP_SIZE_TERMINATOR)

/**
 * @def   BCAP_SIZE_BSTR_LEN
 * @brief A definition for the size of BSTR string length.
 */
#define BCAP_SIZE_BSTR_LEN (4)

/**
 * @def   BCAP_SIZE_BSTR_BUFFER
 * @brief A definition for the size of a BSTR character.
 */
#define BCAP_SIZE_BSTR_BUFFER (2)

/**
 * @def   BCAP_SIZE_VARIANT_TYPE
 * @brief A definition for the size of VARIANT data type.
 */
#define BCAP_SIZE_VARIANT_TYPE (2)

/**
 * @def   BCAP_SIZE_VARIANT_NUM
 * @brief A definition for the size of VARIANT number of elements.
 */
#define BCAP_SIZE_VARIANT_NUM (4)

/**
 * @def   BCAP_SIZE_CRC
 * @brief A definition for the size of b-CAP CRC.
 */
#define BCAP_SIZE_CRC (2)

/**
 * @def   BCAP_POS_CRC(total_size)
 * @brief A macro for calculating the buffer position of b-CAP CRC.
 */
#define BCAP_POS_CRC(total_size) (total_size - BCAP_SIZE_CRC - BCAP_SIZE_TERMINATOR)

/**
 * @def   BCAP_SIZE_CALC_CRC(total_size)
 * @brief A macro for calculating a b-CAP CRC.
 */
#define BCAP_SIZE_CALC_CRC(total_size) (total_size - BCAP_SIZE_HEADER - BCAP_SIZE_CRC - BCAP_SIZE_TERMINATOR)

/**
 * @struct BCAP_PACKET
 * @brief  A type definition for the b-CAP packet.
 * @note   The b-CAP packet's byte order is little endian.
 */
struct BCAP_PACKET
{
  uint16_t serial; /**< Serial number.              */
  uint16_t reserv; /**< Version or retry count.     */
  int32_t id;      /**< Function ID or return code. */
  uint16_t argc;   /**< Number of args.             */
  VARIANT* args;   /**< Arguments.                  */
};

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn         HRESULT bcap_packet2bytary(const struct BCAP_PACKET *src, char *dst, uint32_t len_dst)
   * @brief      Converts the b-CAP packet to a byte array.
   * @param[in]  src The b-CAP packet to be converted.
   * @param[out] dst The converted byte array.
   * @param[in]  len_dst The length of allocated byte array.
   */
  _BCAP_EXP_COMMON HRESULT
  bcap_packet2bytary(const struct BCAP_PACKET *src, char *dst,
      uint32_t len_dst);

  /**
   * @fn         HRESULT bcap_bytary2packet(const char *src, uint32_t len_src, struct BCAP_PACKET *dst)
   * @brief      Converts the byte array to a b-CAP packet.
   * @param[in]  src The byte array to be converted.
   * @param[in]  len_src The length of allocated byte array.
   * @param[out] dst The converted b-CAP packet.
   * @note       If you want to be allocated args by system, then sets argc = -1 and args = NULL.
   */
  _BCAP_EXP_COMMON HRESULT
  bcap_bytary2packet(const char *src, uint32_t len_src,
      struct BCAP_PACKET *dst);

  /**
   * @fn         uint32_t bcap_calc_size_packet(const struct BCAP_PACKET *packet)
   * @brief      Calculates the converted buffer size of the b-CAP packet.
   * @param[in]  packet The b-CAP packet to be calculated.
   */
  _BCAP_EXP_COMMON uint32_t
  bcap_calc_size_packet(const struct BCAP_PACKET *packet);

  /**
   * @fn         uint16_t bcap_calc_crc(uint8_t *buf, uint32_t len_buf);
   * @brief      Calculates CRC of the b-CAP packet.
   * @param[in]  packet The b-CAP packet to be calculated.
   */
  _BCAP_EXP_COMMON uint16_t
  bcap_calc_crc(uint8_t *buf, uint32_t len_buf);

  /**
   * @fn         HRESULT bcap_send(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_send)
   * @brief      Sends b-CAP packet.
   * @param[in]  device The common communication parameters.
   * @param[in]  packet_send The b-CAP packet to be sent.
   */
  _BCAP_EXP_COMMON HRESULT
  bcap_send(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_send);

  /**
   * @fn         HRESULT bcap_recv(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_recv, int client)
   * @brief      Receives b-CAP packet.
   * @param[in]  device The common communication parameters.
   * @param[out] packet_recv The b-CAP packet to be received.
   * @param[in]  client Flag that means the receiver is client(1) or not(0).
   */
  _BCAP_EXP_COMMON HRESULT
  bcap_recv(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_recv,
      int client);

#ifdef __cplusplus
}
#endif

#endif /* BCAP_COMMON_H_ */
