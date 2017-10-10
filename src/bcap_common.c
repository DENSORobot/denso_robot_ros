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

#include "stdint.h"
#include <stdlib.h>
#include <string.h>

#if defined(_USE_WIN_API)
#include <winsock2.h>
#else
#if defined(_USE_LINUX_API)
#include <arpa/inet.h>
#else
#include "dn_additional.h"
#endif

/**
 * @def   _TIME_DIFFERENCE
 * @brief A definition for the time difference between time_t(0) and DATE(0).
 */
#define _TIME_DIFFERENCE (25569.0)

/**
 * @def   _SEC_ONEDAY
 * @brief A definition for the second of one day.
 */
#define _SEC_ONEDAY (24 * 60 * 60)

#endif

#include "dn_common.h"
#include "dn_device.h"
#include "dn_udp.h"
#include "bcap_common.h"

static HRESULT
bcap_vntary2bytary(const VARIANT *src, uint32_t argc, char *dst,
    uint32_t len_dst, uint32_t *offset, int flag);
static HRESULT
bcap_bytary2vntary(const char *src, uint32_t len_src, VARIANT *dst,
    uint32_t argc, uint32_t *offset, int flag);

/**
 * @fn        uint32_t bcap_calc_size_variant(const VARIANT *vnt)
 * @brief     Calculates the buffer length of the VARIANT.
 * @param[in] vnt The VARIANT value to be calculated.
 */
static uint32_t
bcap_calc_size_variant(const VARIANT *vnt)
{
  int32_t i, lbnd, ubnd, cnt;
  uint16_t vt;
  void *pdata;
  uint32_t ret = 0;

  if (vnt != NULL) {
    vt = vnt->vt;
    if ((vt & VT_ARRAY) != 0) {
      if (vnt->parray != NULL) {
        SafeArrayGetLBound(vnt->parray, 1, &lbnd);
        SafeArrayGetUBound(vnt->parray, 1, &ubnd);
        cnt = ubnd - lbnd + 1;

        switch (vt ^ VT_ARRAY) {
          case VT_UI1:
            ret = cnt;
            break;
          case VT_I2:
          case VT_UI2:
          case VT_BOOL:
            ret = cnt * 2;
            break;
          case VT_I4:
          case VT_UI4:
          case VT_R4:
            ret = cnt * 4;
            break;
          case VT_I8:
          case VT_UI8:
          case VT_R8:
          case VT_CY:
          case VT_DATE:
            ret = cnt * 8;
            break;
          case VT_BSTR:
            SafeArrayAccessData(vnt->parray, &pdata);
            for (i = 0; i < cnt; i++) {
              ret += BCAP_SIZE_BSTR_LEN;
              if (((BSTR*) pdata + i) != NULL) {
                  ret += BCAP_SIZE_BSTR_BUFFER
                      * SysStringLen(*((BSTR*) pdata + i));
              }
            }
            SafeArrayUnaccessData(vnt->parray);
            break;
          case VT_VARIANT:
            SafeArrayAccessData(vnt->parray, &pdata);
            for (i = 0; i < cnt; i++) {
              ret += BCAP_SIZE_VARIANT_TYPE + BCAP_SIZE_VARIANT_NUM;
              if (((VARIANT*) pdata + i) != NULL) {
                  ret += bcap_calc_size_variant(((VARIANT*) pdata + i));
              }
            }
            SafeArrayUnaccessData(vnt->parray);
            break;
          default:
            break;
        }
      }
    } else {
      switch (vt) {
        case VT_UI1:
          ret = 1;
          break;
        case VT_I2:
        case VT_UI2:
        case VT_BOOL:
          ret = 2;
          break;
        case VT_I4:
        case VT_UI4:
        case VT_R4:
        case VT_ERROR:
          ret = 4;
          break;
        case VT_I8:
        case VT_UI8:
        case VT_R8:
        case VT_CY:
        case VT_DATE:
          ret = 8;
          break;
        case VT_BSTR:
          ret = BCAP_SIZE_BSTR_LEN;
          if (vnt->bstrVal != NULL) {
            ret += BCAP_SIZE_BSTR_BUFFER * SysStringLen(vnt->bstrVal);
          }
          break;
        default:
          break;
      }
    }
  }

  return ret;
}

/**
 * @fn         void bcap_vntdate2bytary(const DATE *src, char *dst)
 * @brief      Converts the DATE value to a byte array.
 * @param[in]  src The DATE value to be converted.
 * @param[out] dst The converted byte array.
 */
static void
bcap_vntdate2bytary(const DATE *src, char *dst)
{
#if defined(_USE_WIN_API)
  memcpy_le(dst, src, 8);
#else
  double dbl = 0;
  if (*src > 0) {
    dbl = (double) (*src) / _SEC_ONEDAY + _TIME_DIFFERENCE;
  }
  memcpy_le(dst, &dbl, 8);
#endif
}

/**
 * @fn         void bcap_bytary2vntdate(const char *src, DATE *dst)
 * @brief      Converts the byte array to a DATE value.
 * @param[in]  src The byte array to be converted.
 * @param[out] dst The converted DATE value.
 */
static void
bcap_bytary2vntdate(const char *src, DATE *dst)
{
#if defined(_USE_WIN_API)
  memcpy_le(dst, src, 8);
#else
  double dbl = 0;
  memcpy_le(&dbl, src, 8);
  if (dbl > 0) {
    *dst = (DATE) ((dbl - _TIME_DIFFERENCE) * _SEC_ONEDAY);
  }
#endif
}

/**
 * @fn            HRESULT bcap_vnt2bytary(const VARIANT *src, uint32_t argc, char *dst, uint32_t *offset)
 * @brief         Converts the VARIANT value to a byte array.
 * @param[in]     src The VARIANT value to be converted.
 * @param[in]     argc The number of arrays.
 * @param[out]    dst The converted byte array.
 * @param[in]     len_dst The length of allocated byte array.
 * @param[in,out] offset The current buffer position to convert.
 */
static HRESULT
bcap_vnt2bytary(const VARIANT *src, uint32_t argc, char *dst, uint32_t len_dst,
    uint32_t *offset)
{
  uint16_t vt;
  uint32_t i, j, len_bstr, size = 0;
  void const *pdata;
  void *parray;
  HRESULT hr;

  vt = src->vt;
  if ((vt & VT_ARRAY) != 0) {
    if (src->parray != NULL) {
      SafeArrayAccessData(src->parray, &parray);
      switch (vt ^ VT_ARRAY) {
        case VT_UI1:
          if (*offset + argc > len_dst)
            return E_INVALIDARG;
          memcpy(&dst[*offset], parray, argc);
          *offset += argc;

          size = 0;
          break;
        case VT_I2:
        case VT_UI2:
        case VT_BOOL:
          size = 2;
          break;
        case VT_I4:
        case VT_UI4:
        case VT_R4:
          size = 4;
          break;
        case VT_I8:
        case VT_UI8:
        case VT_R8:
        case VT_CY:
          size = 8;
          break;
        case VT_DATE:
          size = 8;
          for (i = 0; i < argc; i++) {
            if (*offset + size > len_dst)
              return E_INVALIDARG;
            bcap_vntdate2bytary((DATE *) parray + i, &dst[*offset]);
            *offset += size;
          }

          size = 0;
          break;
        case VT_BSTR:
          for (i = 0; i < argc; i++) {
            size = BCAP_SIZE_BSTR_LEN;
            if (*offset + size > len_dst)
              return E_INVALIDARG;
            len_bstr = BCAP_SIZE_BSTR_BUFFER
                * SysStringLen(*((BSTR *) parray + i));
            memcpy_le(&dst[*offset], &len_bstr, size);
            *offset += size;

            size = BCAP_SIZE_BSTR_BUFFER;
            for (j = 0; j < len_bstr / size; j++) {
              if (*offset + size > len_dst)
                return E_INVALIDARG;
              memcpy_le(&dst[*offset], *((BSTR *) parray + i) + j, size);
              *offset += size;
            }
          }

          size = 0;
          break;
        case VT_VARIANT:
          hr = bcap_vntary2bytary((VARIANT *) parray, argc, dst, len_dst,
              offset, 0);
          if (FAILED(hr))
            return hr;

          size = 0;
          break;
        default:
          break;
      }

      if (size > 0) {
        for (i = 0; i < argc; i++) {
          if (*offset + size > len_dst)
            return E_INVALIDARG;
          memcpy_le(&dst[*offset], (char *) parray + i * size, size);
          *offset += size;
        }
      }

      SafeArrayUnaccessData(src->parray);
    }
  } else {
    switch (vt) {
      case VT_UI1:
        size = 1;
        pdata = &src->bVal;
        break;
      case VT_I2:
        size = 2;
        pdata = &src->iVal;
        break;
      case VT_UI2:
        size = 2;
        pdata = &src->uiVal;
        break;
      case VT_BOOL:
        size = 2;
        pdata = &src->boolVal;
        break;
      case VT_I4:
        size = 4;
        pdata = &src->lVal;
        break;
      case VT_UI4:
        size = 4;
        pdata = &src->ulVal;
        break;
      case VT_R4:
        size = 4;
        pdata = &src->fltVal;
        break;
      case VT_ERROR:
        size = 4;
        pdata = &src->scode;
        break;
      case VT_I8:
        size = 8;
        pdata = &src->llVal;
        break;
      case VT_UI8:
        size = 8;
        pdata = &src->ullVal;
        break;
      case VT_R8:
        size = 8;
        pdata = &src->dblVal;
        break;
      case VT_CY:
        size = 8;
        pdata = &src->cyVal;
        break;
      case VT_DATE:
        size = 8;
        if (*offset + size > len_dst)
          return E_INVALIDARG;
        bcap_vntdate2bytary(&src->date, &dst[*offset]);
        *offset += size;

        size = 0;
        break;
      case VT_BSTR:
        size = BCAP_SIZE_BSTR_LEN;
        if (*offset + size > len_dst)
          return E_INVALIDARG;
        len_bstr = BCAP_SIZE_BSTR_BUFFER * SysStringLen(src->bstrVal);
        memcpy_le(&dst[*offset], &len_bstr, size);
        *offset += size;

        size = BCAP_SIZE_BSTR_BUFFER;
        for (i = 0; i < len_bstr / size; i++) {
          if (*offset + size > len_dst)
            return E_INVALIDARG;
          memcpy_le(&dst[*offset], src->bstrVal + i, size);
          *offset += size;
        }

        size = 0;
        break;
      default:
        break;
    }

    if (size > 0) {
      if (*offset + size > len_dst)
        return E_INVALIDARG;
      memcpy_le(&dst[*offset], pdata, size);
      *offset += size;
    }
  }

  return S_OK;
}

/**
 * @fn            HRESULT bcap_vntary2bytary(const VARIANT *src, uint32_t argc, char *dst, uint32_t *offset, int flag)
 * @brief         Converts the VARIANT array to a byte array.
 * @param[in]     src The VARIANT array to be converted.
 * @param[in]     argc The number of arrays.
 * @param[out]    dst The converted byte array.
 * @param[in]     len_dst The length of allocated byte array.
 * @param[in,out] offset The current buffer position to convert.
 * @param[in]     flag Flag that means whether this function called from bcap_packet2bytary or not.
 */
static HRESULT
bcap_vntary2bytary(const VARIANT *src, uint32_t argc, char *dst,
    uint32_t len_dst, uint32_t *offset, int flag)
{
  int32_t lbnd, ubnd;
  uint32_t i, cnt, size, len_vnt, offset_tmp;
  const VARIANT *vnt;
  HRESULT hr = S_OK;

  for (i = 0; i < argc; i++) {
    vnt = &src[i];
    offset_tmp = *offset;

    if (flag != 0) {
      *offset += BCAP_SIZE_DATA_LEN;
    }

    size = BCAP_SIZE_VARIANT_TYPE;
    if (*offset + size > len_dst)
      return E_INVALIDARG;
    memcpy_le(&dst[*offset], &vnt->vt, size);
    *offset += size;

    if ((vnt->vt & VT_ARRAY) != 0) {
      if (vnt->parray != NULL) {
        SafeArrayGetLBound(vnt->parray, 1, &lbnd);
        SafeArrayGetUBound(vnt->parray, 1, &ubnd);
        cnt = ubnd - lbnd + 1;
      } else {
        return E_INVALIDARG;
      }
    } else {
      cnt = 1;
    }

    size = BCAP_SIZE_VARIANT_NUM;
    if (*offset + size > len_dst)
      return E_INVALIDARG;
    memcpy_le(&dst[*offset], &cnt, size);
    *offset += size;

    hr = bcap_vnt2bytary(vnt, cnt, dst, len_dst, offset);
    if (FAILED(hr))
      return hr;

    if (flag != 0) {
      len_vnt = *offset - offset_tmp - BCAP_SIZE_DATA_LEN;
      memcpy_le(&dst[offset_tmp], &len_vnt, BCAP_SIZE_DATA_LEN);
    }
  }

  return hr;
}

/**
 * @fn         HRESULT bcap_packet2bytary(const struct BCAP_PACKET *src, char *dst, uint32_t len_dst)
 * @brief      Converts the b-CAP packet to a byte array.
 * @param[in]  src The b-CAP packet to be converted.
 * @param[out] dst The converted byte array.
 * @param[in]  len_dst The length of allocated byte array.
 */
HRESULT
bcap_packet2bytary(const struct BCAP_PACKET *src, char *dst, uint32_t len_dst)
{
  uint32_t size, offset = 1;
  HRESULT hr;

  if (src == NULL || dst == NULL)
    return E_INVALIDARG;

  if (len_dst < BCAP_SIZE_MIN)
    return E_INVALIDARG;

  memset(dst, 0, len_dst);

  dst[BCAP_POS_HEADER] = BCAP_HEADER;

  size = BCAP_SIZE_LEN;
  memcpy_le(&dst[offset], &len_dst, size);
  offset += size;

  size = BCAP_SIZE_SERIAL;
  memcpy_le(&dst[offset], &src->serial, size);
  offset += size;

  size = BCAP_SIZE_RESERVE;
  memcpy_le(&dst[offset], &src->reserv, size);
  offset += size;

  size = BCAP_SIZE_ID;
  memcpy_le(&dst[offset], &src->id, size);
  offset += size;

  size = BCAP_SIZE_ARGC;
  memcpy_le(&dst[offset], &src->argc, size);
  offset += size;

  hr = bcap_vntary2bytary(src->args, src->argc, dst, len_dst - 1, &offset, 1);

  dst[len_dst - 1] = BCAP_TERMINATOR;

  return hr;
}

/**
 * @fn            HRESULT bcap_bytary2vnt(const char *src, uint32_t len_src, VARIANT *dst, uint32_t argc, uint32_t *offset)
 * @brief         Converts the byte array to a VARIANT value.
 * @param[in]     src The byte array to be converted.
 * @param[in]     len_src The length of allocated byte array.
 * @param[out]    dst The converted VARIANT value.
 * @param[in]     argc The number of arrays.
 * @param[in,out] offset The current buffer position to convert.
 */
static HRESULT
bcap_bytary2vnt(const char *src, uint32_t len_src, VARIANT *dst, uint32_t argc,
    uint32_t *offset)
{
  uint16_t vt;
  uint32_t i, j, len_bstr, size = 0;
  void *pdata, *parray;
  HRESULT hr;

  vt = dst->vt;
  if ((vt & VT_ARRAY) != 0) {
    dst->parray = SafeArrayCreateVector(vt ^ VT_ARRAY, 1, argc);
    if (dst->parray == NULL)
      return E_OUTOFMEMORY;

    SafeArrayAccessData(dst->parray, &parray);
    switch (vt ^ VT_ARRAY) {
      case VT_UI1:
        if (*offset + argc > len_src)
          return E_INVALIDARG;
        memcpy(parray, &src[*offset], argc);
        *offset += argc;

        size = 0;
        break;
      case VT_I2:
      case VT_UI2:
      case VT_BOOL:
        size = 2;
        break;
      case VT_I4:
      case VT_UI4:
      case VT_R4:
        size = 4;
        break;
      case VT_I8:
      case VT_UI8:
      case VT_R8:
      case VT_CY:
        size = 8;
        break;
      case VT_DATE:
        size = 8;
        for (i = 0; i < argc; i++) {
          if (*offset + size > len_src)
            return E_INVALIDPACKET;
          bcap_bytary2vntdate(&src[*offset], (DATE *) parray + i);
          *offset += size;
        }

        size = 0;
        break;
      case VT_BSTR:
        for (i = 0; i < argc; i++) {
          size = BCAP_SIZE_BSTR_LEN;
          if (*offset + size > len_src)
            return E_INVALIDPACKET;
          memcpy_le(&len_bstr, &src[*offset], size);
          len_bstr /= BCAP_SIZE_BSTR_BUFFER;
          *offset += size;

          *((BSTR*) parray + i) = SysAllocStringLen(L"", len_bstr);
          if (*((BSTR*) parray + i) == NULL)
            return E_OUTOFMEMORY;

          size = BCAP_SIZE_BSTR_BUFFER;
          for (j = 0; j < len_bstr; j++) {
            if (*offset + size > len_src)
              return E_INVALIDPACKET;
            memcpy_le(*((BSTR*) parray + i) + j, &src[*offset], size);
            *offset += size;
          }
        }

        size = 0;
        break;
      case VT_VARIANT:
        hr = bcap_bytary2vntary(src, len_src, (VARIANT *) parray, argc, offset,
            0);
        if (FAILED(hr))
          return hr;

        size = 0;
        break;
      default:
        break;
    }

    if (size > 0) {
      for (i = 0; i < argc; i++) {
        if (*offset + size > len_src)
          return E_INVALIDPACKET;
        memcpy_le((char *) parray + i * size, &src[*offset], size);
        *offset += size;
      }
    }

    SafeArrayUnaccessData(dst->parray);
  } else {
    switch (vt) {
      case VT_UI1:
        size = 1;
        pdata = &dst->bVal;
        break;
      case VT_I2:
        size = 2;
        pdata = &dst->iVal;
        break;
      case VT_UI2:
        size = 2;
        pdata = &dst->uiVal;
        break;
      case VT_BOOL:
        size = 2;
        pdata = &dst->boolVal;
        break;
      case VT_I4:
        size = 4;
        pdata = &dst->lVal;
        break;
      case VT_UI4:
        size = 4;
        pdata = &dst->ulVal;
        break;
      case VT_R4:
        size = 4;
        pdata = &dst->fltVal;
        break;
      case VT_ERROR:
        size = 4;
        pdata = &dst->scode;
        break;
      case VT_I8:
        size = 8;
        pdata = &dst->llVal;
        break;
      case VT_UI8:
        size = 8;
        pdata = &dst->ullVal;
        break;
      case VT_R8:
        size = 8;
        pdata = &dst->dblVal;
        break;
      case VT_CY:
        size = 8;
        pdata = &dst->cyVal;
        break;
      case VT_DATE:
        size = 8;
        if (*offset + size > len_src)
          return E_INVALIDPACKET;
        bcap_bytary2vntdate(&src[*offset], &dst->date);
        *offset += size;

        size = 0;
        break;
      case VT_BSTR:
        size = BCAP_SIZE_BSTR_LEN;
        if (*offset + size > len_src)
          return E_INVALIDPACKET;
        memcpy_le(&len_bstr, &src[*offset], size);
        len_bstr /= BCAP_SIZE_BSTR_BUFFER;
        *offset += size;

        dst->bstrVal = SysAllocStringLen(L"", len_bstr);
        if (dst->bstrVal == NULL)
          return E_OUTOFMEMORY;

        size = BCAP_SIZE_BSTR_BUFFER;
        for (i = 0; i < len_bstr; i++) {
          if (*offset + size > len_src)
            return E_INVALIDPACKET;
          memcpy_le(dst->bstrVal + i, &src[*offset], size);
          *offset += size;
        }

        size = 0;
        break;
      default:
        break;
    }

    if (size > 0) {
      if (*offset + size > len_src)
        return E_INVALIDPACKET;
      memcpy_le(pdata, &src[*offset], size);
      *offset += size;
    }
  }

  return S_OK;
}

/**
 * @fn            bcap_bytary2vntary(const char *src, uint32_t len_src, VARIANT *dst, uint32_t argc, uint32_t *offset, int flag)
 * @brief         Converts the byte array to a VARIANT array.
 * @param[in]     src The byte array to be converted.
 * @param[in]     len_src The length of allocated byte array.
 * @param[out]    dst The converted VARIANT array.
 * @param[in]     argc The number of arrays.
 * @param[in,out] offset The current buffer position to convert.
 * @param[in]     flag Flag that means whether this function called from bcap_bytary2packet or not.
 */
static HRESULT
bcap_bytary2vntary(const char *src, uint32_t len_src, VARIANT *dst,
    uint32_t argc, uint32_t *offset, int flag)
{
  uint32_t i, cnt, size;
  VARIANT *vnt;
  HRESULT hr = S_OK;

  for (i = 0; i < argc; i++) {
    vnt = &dst[i];
    VariantInit(vnt);

    if (flag != 0) {
      *offset += BCAP_SIZE_DATA_LEN;
    }

    size = BCAP_SIZE_VARIANT_TYPE;
    if (*offset + size > len_src)
      return E_INVALIDPACKET;
    memcpy_le(&vnt->vt, &src[*offset], size);
    *offset += size;

    size = BCAP_SIZE_VARIANT_NUM;
    if (*offset + size > len_src)
      return E_INVALIDPACKET;
    memcpy_le(&cnt, &src[*offset], size);
    *offset += size;

    hr = bcap_bytary2vnt(src, len_src, vnt, cnt, offset);
    if (FAILED(hr))
      return hr;
  }

  return hr;
}

/**
 * @fn         HRESULT bcap_bytary2packet(const char *src, uint32_t len_src, struct BCAP_PACKET *dst)
 * @brief      Converts the byte array to a b-CAP packet.
 * @param[in]  src The byte array to be converted.
 * @param[in]  len_src The length of allocated byte array.
 * @param[out] dst The converted b-CAP packet.
 * @note       If you want to be allocated args by system, then sets argc = -1 and args = NULL.
 */
HRESULT
bcap_bytary2packet(const char *src, uint32_t len_src, struct BCAP_PACKET *dst)
{
  uint16_t tmp_argc;
  uint32_t len_tmp, size, offset = 1;
  HRESULT hr = S_OK;

  if (src == NULL || dst == NULL)
    return E_INVALIDARG;

  if (len_src < BCAP_SIZE_MIN)
    return E_INVALIDPACKET;

  size = BCAP_SIZE_LEN;
  memcpy_le(&len_tmp, &src[offset], size);
  offset += size;

  if (len_tmp != len_src)
    return E_INVALIDPACKET;

  if ((src[BCAP_POS_HEADER] != BCAP_HEADER)
      || (src[len_src - 1] != BCAP_TERMINATOR))
    return E_INVALIDPACKET;

  size = BCAP_SIZE_SERIAL;
  memcpy_le(&dst->serial, &src[offset], size);
  offset += size;

  size = BCAP_SIZE_RESERVE;
  memcpy_le(&dst->reserv, &src[offset], size);
  offset += size;

  size = BCAP_SIZE_ID;
  memcpy_le(&dst->id, &src[offset], size);
  offset += size;

  size = BCAP_SIZE_ARGC;
  memcpy_le(&tmp_argc, &src[offset], size);
  offset += size;

  if (dst->argc == (uint16_t) -1) {
    if (dst->args != NULL)
      return E_INVALIDARG;
    dst->argc = tmp_argc;
  } else {
    if ((dst->argc > 0) && (dst->args == NULL))
      return E_INVALIDARG;
    if (tmp_argc > dst->argc)
      return E_INVALIDPACKET;
    dst->argc = tmp_argc;
  }

  if (dst->argc > 0) {
    if (dst->args == NULL) {
      dst->args = (VARIANT *) malloc(sizeof(VARIANT) * dst->argc);
      if (dst->args == NULL)
        return E_OUTOFMEMORY;
    }

    hr = bcap_bytary2vntary(src, len_src - 1, dst->args, dst->argc, &offset,
        1);
  }

  return hr;
}

/**
 * @fn         uint32_t bcap_calc_size_packet(const struct BCAP_PACKET *packet)
 * @brief      Calculates the converted buffer size of the b-CAP packet.
 * @param[in]  packet The b-CAP packet to be calculated.
 */
uint32_t
bcap_calc_size_packet(const struct BCAP_PACKET *packet)
{
  int i;
  uint32_t ret = 0;

  if (packet != NULL) {
    ret = BCAP_SIZE_MIN;
    for (i = 0; i < packet->argc; i++) {
      ret += BCAP_SIZE_DATA_LEN + BCAP_SIZE_VARIANT_TYPE
          + BCAP_SIZE_VARIANT_NUM
          + bcap_calc_size_variant(&packet->args[i]);
    }
  }

  return ret;
}

/**
 * @fn         uint16_t bcap_calc_crc(uint8_t *buf, uint32_t len_buf);
 * @brief      Calculates CRC of the b-CAP packet.
 * @param[in]  packet The b-CAP packet to be calculated.
 */
uint16_t
bcap_calc_crc(uint8_t *buf, uint32_t len_buf)
{
  int cnt;
  uint32_t pos;
  uint16_t crc = 0xFFFF;

  if (buf != NULL) {
    for (pos = 0; pos < len_buf; pos++) {
      crc ^= (buf[pos] << 8);
      for (cnt = 0; cnt < 8; cnt++) {
        if (crc & 0x8000) {
          crc = (crc << 1) ^ 0x1021;
        } else {
          crc <<= 1;
        }
      }
    }
  }

  return crc;
}

/**
 * @fn         HRESULT bcap_send(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_send)
 * @brief      Sends b-CAP packet.
 * @param[in]  device The common communication parameters.
 * @param[in]  packet_send The b-CAP packet to be sent.
 */
HRESULT
bcap_send(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_send)
{
  int opt_tcp;
  uint16_t crc;
  uint32_t len_send, len_max;
  char *buf_send = NULL;
  void *parg = NULL;
  struct udpaddr opt_udp;
  HRESULT hr;

  hr = check_conn_param(device, BCAP_CHECK_SEND);
  if (FAILED(hr))
    return hr;

  if (packet_send == NULL)
    return E_INVALIDARG;

  switch (device->type) {
    case CONN_TCP:
      len_max = (uint32_t) -1;
      opt_tcp = 0;
      parg = &opt_tcp;
      break;
    case CONN_UDP:
      if (device->arg == NULL)
        return E_INVALIDARG;
      len_max = UDP_MAX_PACKET;
      opt_udp.flag = 0;
      opt_udp.addr = *(struct sockaddr_in *) device->arg;
      parg = &opt_udp;
      break;
    case CONN_COM:
      len_max = UDP_MAX_PACKET;
      break;
    default:
      return E_INVALIDARG;
  }

  len_send = bcap_calc_size_packet(packet_send);
  if (device->type == CONN_COM) {
    len_send += BCAP_SIZE_CRC;
  }

  if (len_send > len_max) {
    hr = E_TOO_MUCH_DATA;
    goto send_end;
  }

  buf_send = (char *) malloc(len_send);
  if (buf_send == NULL) {
    hr = E_OUTOFMEMORY;
    goto send_end;
  }

  hr = bcap_packet2bytary(packet_send, buf_send, len_send);
  if (FAILED(hr))
    goto send_end;

  if (device->type == CONN_COM) {
    crc = bcap_calc_crc((uint8_t *) &buf_send[BCAP_POS_LEN],
        BCAP_SIZE_CALC_CRC(len_send));
    memcpy_le(&buf_send[BCAP_POS_CRC(len_send)], &crc, BCAP_SIZE_CRC);
  }

  hr = device->dn_send(device->sock, buf_send, len_send, parg);
  if (FAILED(hr))
    goto send_end;

send_end:
  if (buf_send != NULL) {
    free(buf_send);
    buf_send = NULL;
  }

  return hr;
}

/**
 * @fn         HRESULT bcap_recv(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_recv, int client)
 * @brief      Receives b-CAP packet.
 * @param[in]  device The common communication parameters.
 * @param[out] packet_recv The b-CAP packet to be received.
 * @param[in]  client Flag that means the receiver is client(1) or not(0).
 */
HRESULT
bcap_recv(struct CONN_PARAM_COMMON *device, struct BCAP_PACKET *packet_recv,
    int client)
{
  int opt_tcp, flag_init = 0, flag_crc = 1;
  uint16_t crc, crc_recv;
  uint32_t len_recv, len_recved, len_tmp, len_min, len_max;
  char *buf_recv = NULL, *pos, buf_tmp[UDP_MAX_PACKET + 1] =
    { 0, };
  void *parg = NULL;
  struct udpaddr opt_udp;
  HRESULT hr;

  hr = check_conn_param(device, BCAP_CHECK_RECV);
  if (FAILED(hr))
    return hr;

  if (packet_recv == NULL)
    return E_INVALIDARG;

  switch (device->type) {
    case CONN_TCP:
      len_min = BCAP_SIZE_MIN;
      len_max = (uint32_t) -1;
      opt_tcp = MSG_PEEK;
      parg = &opt_tcp;
      break;
    case CONN_UDP:
      if (device->arg == NULL)
        return E_INVALIDARG;
      len_min = BCAP_SIZE_MIN;
      len_max = UDP_MAX_PACKET;
      opt_udp.flag = 0;
      parg = &opt_udp;
      break;
    case CONN_COM:
      len_min = BCAP_SIZE_MIN + BCAP_SIZE_CRC;
      len_max = UDP_MAX_PACKET;
      break;
    default:
      return E_INVALIDARG;
  }

  while (1) {
    len_recved = 0;
    while (1) {
      pos = (char *) memchr(buf_tmp, BCAP_HEADER, len_recved);
      if (pos != NULL) {
        if (pos != buf_tmp) {
          if (device->type == CONN_TCP) {
            len_tmp = ((long) pos - (long) buf_tmp);
            device->dn_recv(device->sock, buf_tmp, len_tmp, &len_recv,
                device->timeout, NULL);
          }
          len_recved -= ((long) pos - (long) buf_tmp);
          memcpy(buf_tmp, pos, len_recved);
        }
      } else {
        if (device->type == CONN_TCP) {
          device->dn_recv(device->sock, buf_tmp, len_recved, &len_recv,
              device->timeout, NULL);
        }
        len_recved = 0;
      }

      if (len_recved >= BCAP_SIZE_HEADER + BCAP_SIZE_LEN) {
        break;
      }

      if (device->type == CONN_TCP) {
        len_recved = 0;
      }

      hr = device->dn_recv(device->sock, &buf_tmp[len_recved],
          UDP_MAX_PACKET - len_recved, &len_tmp, device->timeout, parg);

      if (device->type == CONN_UDP) {
        if (client || flag_init) {
          hr = SUCCEEDED(hr) ?
                  udp_check_sockaddr((struct sockaddr_in *) device->arg,
                      &opt_udp.addr) : hr;
        } else {
          flag_init = 1;
          *(struct sockaddr_in *) device->arg = opt_udp.addr;
        }
      }

      if (FAILED(hr))
        goto recv_end;

      len_recved += len_tmp;
    }

    memcpy_le(&len_recv, &buf_tmp[BCAP_POS_LEN], BCAP_SIZE_LEN);
    if ((len_recv < len_min) || (len_max < len_recv)) {
      if (device->type == CONN_TCP) {
        device->dn_recv(device->sock, buf_tmp, BCAP_SIZE_HEADER, &len_tmp,
            device->timeout, NULL);
      }
      len_recved -= BCAP_SIZE_HEADER;
      memcpy(buf_tmp, &buf_tmp[BCAP_POS_LEN], len_recved);
      continue;
    }

    buf_recv = (char *) malloc(len_recv);
    if (buf_recv == NULL) {
      hr = E_OUTOFMEMORY;
      goto recv_end;
    }

    if (device->type == CONN_TCP) {
      opt_tcp = 0;
      device->dn_recv(device->sock, buf_recv, len_recv, &len_recved,
          device->timeout, parg);
    } else {
      memcpy(buf_recv, buf_tmp,
          (len_recv < len_recved) ? len_recv : len_recved);
    }

    while (len_recv > len_recved) {
      hr = device->dn_recv(device->sock, &buf_recv[len_recved],
          len_recv - len_recved, &len_tmp, device->timeout, parg);

      if (device->type == CONN_UDP) {
        hr = SUCCEEDED(hr) ?
                udp_check_sockaddr((struct sockaddr_in *) device->arg,
                    &opt_udp.addr) : hr;
      }

      if (FAILED(hr))
        goto recv_end;

      len_recved += len_tmp;
    }

    hr = bcap_bytary2packet(buf_recv, len_recv, packet_recv);
    if (FAILED(hr))
      goto recv_end;

    if (device->type == CONN_COM) {
      crc = bcap_calc_crc((uint8_t *) &buf_recv[BCAP_POS_LEN],
          BCAP_SIZE_CALC_CRC(len_recv));
      memcpy_le(&crc_recv, &buf_recv[BCAP_POS_CRC(len_recv)],
          BCAP_SIZE_CRC);
      flag_crc = (crc == crc_recv);
    }

    if (flag_crc) {
      break;
    }
  }

recv_end:
  if (buf_recv != NULL) {
    free(buf_recv);
    buf_recv = NULL;
  }

  return hr;
}
