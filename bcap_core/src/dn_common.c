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
#include <windows.h>
#elif defined(_USE_LINUX_API)
#include <float.h>
#include <limits.h>
#include <locale.h>
#include <errno.h>
#ifndef _wcsicmp
#define _wcsicmp wcscasecmp
#endif
#ifndef __USE_XOPEN
#define __USE_XOPEN
#endif
#endif

#include "dn_common.h"

/**
 * @def   _LEN_VARIANT2BSTR
 * @brief A definition for the string length for converting VARIANT to BSTR.
 */
#define _LEN_VARIANT2BSTR (500)

#ifndef _OLEAUTO_H_

/**
 * @fn        BSTR SysAllocString(const wchar_t *sz)
 * @brief     Allocates and returns BSTR.
 * @param[in] sz Unicode string to be copied. This must be NULL terminated.
 * @note      If there is no enough memory space, then return NULL.
 */
BSTR
SysAllocString(const wchar_t *sz)
{
  if (sz == NULL)
    return NULL;

  return SysAllocStringLen(sz, wcslen(sz));
}

/**
 * @fn        BSTR SysAllocStringLen(const wchar_t *pch, uint16_t cch)
 * @brief     Allocates and returns BSTR.
 * @param[in] pch Unicode string to be copied. This must be NULL terminated.
 * @param[in] cch The number of characters to be copied.
 * @note      If there is no enough memory space, then return NULL.
 */
BSTR
SysAllocStringLen(const wchar_t *pch, uint16_t cch)
{
  uint16_t minlen;
  BSTR bstr = NULL;

  minlen = sizeof(wchar_t) * (cch + 1);
  bstr = (BSTR) malloc(minlen);

  if (bstr != NULL) {
    memset(bstr, 0, minlen);

    if (pch != NULL) {
      minlen = wcslen(pch);
      minlen = (cch < minlen ? cch : minlen);
      memcpy(bstr, pch, sizeof(wchar_t) * minlen);
    }
  }

  return bstr;
}

/**
 * @fn            void SysFreeString(BSTR bstr)
 * @brief         Releases the memory of BSTR.
 * @param[in,out] bstr Unicode string to be freed.
 */
void
SysFreeString(BSTR bstr)
{
  if (bstr != NULL) {
    free(bstr);
    bstr = NULL;
  }
}

/**
 * @fn        uint16 SysStringLen(BSTR bstr)
 * @brief     Gets and returns the number of characters of BSTR.
 * @param[in] bstr Unicode string to be gotten. This must be NULL terminated.
 */
uint16_t
SysStringLen(BSTR bstr)
{
  uint16_t len = 0;

  if (bstr != NULL) {
    len = wcslen(bstr);
  }

  return len;
}

/**
 * @fn        SAFEARRAY* SafeArrayCreateVector(uint16_t vt, int32_t lLbound, uint32_t cElements)
 * @brief     Allocates and returns SAFEARRAY.
 * @param[in] vt Variant type.
 * @param[in] lLbound The lower bound of array. This should be 0.
 * @param[in] cElements The number of elements.
 * @note      If there is no enough memory space, then return NULL.
 */
SAFEARRAY*
SafeArrayCreateVector(uint16_t vt, int32_t lLbound, uint32_t cElements)
{
  int sz;
  SAFEARRAY* psa = NULL;

  psa = (SAFEARRAY*) malloc(sizeof(SAFEARRAY));

  if (psa != NULL) {
    memset(psa, 0, sizeof(SAFEARRAY));

    psa->cDims = 1;
    psa->vt = vt;
    psa->rgsabound[0].lLbound = lLbound;
    psa->rgsabound[0].cElements = cElements;

    if (cElements > 0) {
      switch (vt) {
        case VT_UI1:
          sz = 1;
          break;
        case VT_I2:
        case VT_UI2:
        case VT_BOOL:
          sz = 2;
          break;
        case VT_I4:
        case VT_UI4:
        case VT_R4:
          sz = 4;
          break;
        case VT_I8:
        case VT_UI8:
        case VT_R8:
        case VT_CY:
          sz = 8;
          break;
        case VT_DATE:
          sz = sizeof(DATE);
          break;
        case VT_BSTR:
          sz = sizeof(BSTR);
          break;
        case VT_VARIANT:
          sz = sizeof(VARIANT);
          break;
        default:
          free(psa);
          psa = NULL;
          goto exit_proc;
      }

      psa->cbElements = sz;
      psa->pvData = malloc(cElements * sz);

      if (psa->pvData == NULL) {
        free(psa);
        psa = NULL;
        goto exit_proc;
      }

      memset(psa->pvData, 0, cElements * sz);
    }
  }

exit_proc:
  return psa;
}

/**
 * @fn            HRESULT SafeArrayDestroy(SAFEARRAY *psa)
 * @brief         Releases the memory of SAFEARRAY.
 * @param[in,out] psa SAFEARRAY to be freed.
 */
HRESULT
SafeArrayDestroy(SAFEARRAY *psa)
{
  int32_t i;

  if (psa != NULL) {
    if (psa->pvData != NULL) {
      switch (psa->vt) {
        case VT_BSTR:
          for (i = 0; i < psa->rgsabound[0].cElements; i++) {
            SysFreeString(*((BSTR*) psa->pvData + i));
          }
          break;
        case VT_VARIANT:
          for (i = 0; i < psa->rgsabound[0].cElements; i++) {
            VariantClear(((VARIANT*) psa->pvData + i));
          }
          break;
        default:
          break;
      }
      free(psa->pvData);
      psa->pvData = NULL;
    }
    free(psa);
    psa = NULL;
  }

  return S_OK;
}

/**
 * @fn        uint16_t SafeArrayGetDim(SAFEARRAY *psa)
 * @brief     Gets and returns the dimension of SAFEARRAY.
 * @param[in] psa SAFEARRAY to be gotten.
 */
uint16_t
SafeArrayGetDim(SAFEARRAY *psa)
{
  if (psa == NULL)
    return 0;

  return psa->cDims;
}

/**
 * @fn        uint32_t SafeArrayGetElemsize(SAFEARRAY *psa)
 * @brief     Gets and returns the size of an element.
 * @param[in] psa SAFEARRAY to be gotten.
 */
uint32_t
SafeArrayGetElemsize(SAFEARRAY *psa)
{
  if (psa == NULL)
    return 0;

  return psa->cbElements;
}

/**
 * @fn         HRESULT SafeArrayGetLBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plLbound)
 * @brief      Gets the lower bound of SAFEARRAY.
 * @param[in]  psa SAFEARRAY to be gotten.
 * @param[in]  nDim The target dimension.
 * @param[out] plLbound The gotten lower bound.
 */
HRESULT
SafeArrayGetLBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plLbound)
{
  if (psa == NULL || plLbound == NULL)
    return E_INVALIDARG;
  if (nDim <= 0 || psa->cDims < nDim)
    return DISP_E_BADINDEX;

  *plLbound = psa->rgsabound[nDim - 1].lLbound;

  return S_OK;
}

/**
 * @fn         HRESULT SafeArrayGetUBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plUbound)
 * @brief      Gets the upper bound of SAFEARRAY.
 * @param[in]  psa SAFEARRAY to be gotten.
 * @param[in]  nDim The target dimension.
 * @param[out] plUbound The gotten upper bound.
 */
HRESULT
SafeArrayGetUBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plUbound)
{
  if (psa == NULL || plUbound == NULL)
    return E_INVALIDARG;
  if (nDim <= 0 || psa->cDims < nDim)
    return DISP_E_BADINDEX;

  *plUbound = (psa->rgsabound[nDim - 1].cElements
      + psa->rgsabound[nDim - 1].lLbound - 1);

  return S_OK;
}

/**
 * @fn         HRESULT SafeArrayGetVartype(SAFEARRAY *psa, uint16_t *pvt)
 * @brief      Gets the variant type of SAFEARRAY.
 * @param[in]  psa SAFEARRAY to be gotten.
 * @param[out] pvt The gotten variant type.
 */
HRESULT
SafeArrayGetVartype(SAFEARRAY *psa, uint16_t *pvt)
{
  if (psa == NULL || pvt == NULL)
    return E_INVALIDARG;

  *pvt = psa->vt;

  return S_OK;
}

/**
 * @fn         HRESULT SafeArrayAccessData(SAFEARRAY *psa, void **ppvData)
 * @brief      Accesses the SAFEARRAY and gets the pointer of array data.
 * @param[in]  psa SAFEARRAY to be accessed.
 * @param[out] ppvData The gotten pointer.
 * @note  This function must be called before accessing SAFEARRAY data.
 */
HRESULT
SafeArrayAccessData(SAFEARRAY *psa, void **ppvData)
{
  if (psa == NULL || ppvData == NULL)
    return E_INVALIDARG;

  *ppvData = psa->pvData;

  return S_OK;
}

/**
 * @fn        HRESULT SafeArrayUnaccessData(SAFEARRAY *psa)
 * @brief     Unaccesses the SAFEARRAY.
 * @param[in] psa SAFEARRAY to be unaccessed.
 * @note      This function must be called after accessing SAFEARRAY data.
 */
HRESULT
SafeArrayUnaccessData(SAFEARRAY *psa)
{
  if (psa == NULL)
    return E_INVALIDARG;

  return S_OK;
}

/**
 * @fn            void VariantInit(VARIANT *pvarg)
 * @brief         Initializes the VARIANT.
 * @param[in,out] pvarg VARIANT to be initialized.
 * @note          This function must be called before accessing VARIANT.
 */
void
VariantInit(VARIANT *pvarg)
{
  if (pvarg != NULL) {
    memset(pvarg, 0, sizeof(VARIANT));
  }
}

/**
 * @fn            void VariantClear(VARIANT *pvarg)
 * @brief         Clears the VARIANT.
 * @param[in,out] pvarg VARIANT to be cleared.
 * @note          This function must be called before destructing VARIANT.
 */
void
VariantClear(VARIANT *pvarg)
{
  if (pvarg != NULL) {
    if (pvarg->vt & VT_ARRAY) {
      if (pvarg->parray != NULL) {
        SafeArrayDestroy(pvarg->parray);
        pvarg->parray = NULL;
      }
    }
    else if (pvarg->vt == VT_BSTR) {
      if (pvarg->bstrVal != NULL) {
        SysFreeString(pvarg->bstrVal);
        pvarg->bstrVal = NULL;
      }
    }
    memset(pvarg, 0, sizeof(VARIANT));
  }
}

#if (_DN_USE_VARIANT_API)
/**
 * @fn         HRESULT VariantCopy(VARIANT *pvargDest, const VARIANT *pvargSrc)
 * @brief      Copies the source variant to destination variant.
 * @param[out] pvargDest The destination variant to be changed.
 * @param[in]  pvarSrc The source variant.
 */
HRESULT
VariantCopy(VARIANT *pvargDest, const VARIANT *pvargSrc)
{
  if ((pvargDest == NULL) || (pvargSrc == NULL)) {
    return E_INVALIDARG;
  }

  if (pvargDest == pvargSrc) {
    return S_OK;
  }

  VariantClear(pvargDest);

  if (pvargSrc->vt & VT_ARRAY) {
    int32_t i, lLbound = 0;
    uint32_t cbElements = 0, cElements;

    lLbound = pvargSrc->parray->rgsabound[0].lLbound;
    cElements = pvargSrc->parray->rgsabound[0].cElements;
    cbElements = pvargSrc->parray->cbElements;

    switch (pvargSrc->vt ^ VT_ARRAY) {
      case VT_I2:
      case VT_I4:
      case VT_I8:
      case VT_R4:
      case VT_R8:
      case VT_CY:
      case VT_DATE:
      case VT_BOOL:
      case VT_UI1:
      case VT_UI2:
      case VT_UI4:
      case VT_UI8:
        pvargDest->vt = pvargSrc->vt;
        pvargDest->parray = SafeArrayCreateVector(pvargSrc->vt ^ VT_ARRAY,
            lLbound, cElements);
        memcpy(pvargDest->parray->pvData, pvargSrc->parray->pvData,
            cbElements * cElements);
        break;
      case VT_BSTR:
        pvargDest->vt = pvargSrc->vt;
        pvargDest->parray = SafeArrayCreateVector(VT_BSTR, lLbound, cElements);
        for (i = 0; i < cElements; i++) {
          *((BSTR*) pvargDest->parray->pvData + i) = SysAllocString(
              *((BSTR*) pvargSrc->parray->pvData + i));
        }
        break;
      case VT_VARIANT:
        pvargDest->vt = pvargSrc->vt;
        pvargDest->parray = SafeArrayCreateVector(VT_VARIANT, lLbound,
            cElements);
        for (i = 0; i < cElements; i++) {
          VariantCopy(((VARIANT*) pvargDest->parray->pvData + i),
              ((VARIANT*) pvargSrc->parray->pvData + i));
        }
        break;
      default:
        return DISP_E_BADVARTYPE;
    }
  } else {
    switch (pvargSrc->vt) {
      case VT_EMPTY:
      case VT_NULL:
      case VT_I2:
      case VT_I4:
      case VT_I8:
      case VT_R4:
      case VT_R8:
      case VT_CY:
      case VT_DATE:
      case VT_ERROR:
      case VT_BOOL:
      case VT_UI1:
      case VT_UI2:
      case VT_UI4:
      case VT_UI8:
        memcpy(pvargDest, pvargSrc, sizeof(VARIANT));
        break;
      case VT_BSTR:
        pvargDest->vt = VT_BSTR;
        pvargDest->bstrVal = SysAllocString(pvargSrc->bstrVal);
        break;
      default:
        return DISP_E_BADVARTYPE;
    }
  }

  return S_OK;
}

/**
 * @fn         HRESULT Variant2Bstr(BSTR *pbstr, VARIANT *pvarg)
 * @brief      Change VARIANT to BSTR.
 * @param[out] pbstr The destination BSTR.
 * @param[in]  pvarg The source VARIANT.
 */
static HRESULT
Variant2Bstr(BSTR *pbstr, VARIANT *pvarg)
{
  HRESULT hr = S_OK;
  char chStr[_LEN_VARIANT2BSTR];
  wchar_t wchStr[_LEN_VARIANT2BSTR];
  struct tm *tmVal;

  switch (pvarg->vt) {
    case VT_I2:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%d", pvarg->iVal);
      break;
    case VT_I4:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%ld", pvarg->lVal);
      break;
    case VT_I8:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%lld", pvarg->llVal);
      break;
    case VT_R4:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%.7G", pvarg->fltVal);
      break;
    case VT_R8:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%.15G", pvarg->dblVal);
      break;
    case VT_CY:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%lld", pvarg->cyVal.int64);
      break;
    case VT_DATE:
      tmVal = gmtime(&pvarg->date);
      strftime(chStr, _LEN_VARIANT2BSTR, FORMAT_DATE2BSTR, tmVal);
      mbstowcs(wchStr, chStr, _LEN_VARIANT2BSTR);
      break;
    case VT_BOOL:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%d", pvarg->boolVal);
      break;
    case VT_UI1:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%u", pvarg->bVal);
      break;
    case VT_UI2:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%u", pvarg->uiVal);
      break;
    case VT_UI4:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%lu", pvarg->ulVal);
      break;
    case VT_UI8:
      swprintf(wchStr, _LEN_VARIANT2BSTR, L"%llu", pvarg->ullVal);
      break;
    default:
      hr = DISP_E_BADVARTYPE;
      break;
  }

  if(SUCCEEDED(hr)) {
    *pbstr = SysAllocString(wchStr);
  }

  return hr;
}

/**
 * @fn         HRESULT Bstr2Variant(VARIANT *pvarg, int16_t vt, BSTR bstr)
 * @brief      Change BSTR to VARIANT.
 * @param[out] pvarg The destination VARIANT.
 * @param[in]  bstr The source BSTR.
 */
static HRESULT
Bstr2Variant(VARIANT *pvarg, int16_t vt, BSTR bstr)
{
  HRESULT hr = S_OK;
  int flag = 0, len;
  char *chRet, *chStr = NULL;
  wchar_t *wchRet;
  struct tm tmVal;
  VARIANT vntTmp;

  if(*bstr == L'\0') {
    return DISP_E_TYPEMISMATCH;
  }

  VariantInit(&vntTmp);

  if (pvarg->bstrVal == bstr) {
    flag = 1;
  }

  switch (vt) {
    case VT_EMPTY:
    case VT_NULL:
      break;
    case VT_ERROR:
      hr = DISP_E_TYPEMISMATCH;
      break;
    case VT_I2:
    case VT_I4:
    case VT_BOOL:
      errno = 0;
      vntTmp.lVal = (int32_t) wcstol(bstr, &wchRet, 0);
      if ((wchRet == NULL) || ((*wchRet != L'.') && (*wchRet != L'\0'))) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      else if ((vt == VT_I2) || (vt == VT_BOOL)) {
        vntTmp.vt = VT_I4;
        hr = VariantChangeType(&vntTmp, &vntTmp, 0, vt);
      }
      break;
    case VT_I8:
      errno = 0;
      vntTmp.llVal = (int64_t) wcstoll(bstr, &wchRet, 0);
      if ((wchRet == NULL) || ((*wchRet != L'.') && (*wchRet != L'\0'))) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_R4:
      errno = 0;
      vntTmp.fltVal = wcstof(bstr, &wchRet);
      if ((wchRet == NULL) || (*wchRet != L'\0')) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_R8:
      errno = 0;
      vntTmp.dblVal = wcstod(bstr, &wchRet);
      if ((wchRet == NULL) || (*wchRet != L'\0')) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_CY:
      errno = 0;
      vntTmp.cyVal.int64 = (int64_t) wcstoll(bstr, &wchRet, 0);
      if ((wchRet == NULL) || ((*wchRet != L'.') && (*wchRet != L'\0'))) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_UI1:
    case VT_UI2:
    case VT_UI4:
      errno = 0;
      vntTmp.ulVal = (uint32_t) wcstoul(bstr, &wchRet, 0);
      if ((wchRet == NULL) || ((*wchRet != L'.') && (*wchRet != L'\0'))) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      else if ((vt == VT_UI1) || (vt == VT_UI2)) {
        vntTmp.vt = VT_UI4;
        hr = VariantChangeType(&vntTmp, &vntTmp, 0, vt);
      }
      else if (*bstr == L'-') {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_UI8:
      errno = 0;
      vntTmp.ullVal = (uint64_t) wcstoull(bstr, &wchRet, 0);
      if ((wchRet == NULL) || ((*wchRet != L'.') && (*wchRet != L'\0'))) {
        hr = DISP_E_TYPEMISMATCH;
      }
      else if (errno == ERANGE) {
        hr = DISP_E_OVERFLOW;
      }
      else if (*bstr == L'-') {
        hr = DISP_E_OVERFLOW;
      }
      break;
    case VT_DATE:
      len = wcstombs(NULL, bstr, 0) + 1;
      if(len <= 0) {
        hr = DISP_E_TYPEMISMATCH;
        break;
      }

      chStr = (char *) malloc(len);
      if(chStr == NULL) {
        hr = E_OUTOFMEMORY;
        break;
      }

      wcstombs(chStr, bstr, len);
      memset(&tmVal, 0, sizeof(struct tm));
      chRet = strptime(chStr, FORMAT_DATE2BSTR, &tmVal);
      if ((chRet == NULL) || (*chRet != '\0')) {
        hr = DISP_E_TYPEMISMATCH;
      } else {
        vntTmp.date = mktime(&tmVal);
      }
      free(chStr);
      break;
    default:
      hr = DISP_E_BADVARTYPE;
      break;
  }
  errno = 0;

  if (SUCCEEDED(hr)) {
    *pvarg = vntTmp;
    if (flag) {
      SysFreeString(bstr);
    }
  }

  VariantClear(&vntTmp);

  return hr;
}

/**
 * @fn         HRESULT VariantChangeType(VARIANT *pvargDest, VARIANT *pvarSrc, uint16_t wFlags, uint16_t vt)
 * @brief      Changes the source variant to destination variant with the indicated type.
 * @param[out] pvargDest The destination variant to be changed.
 * @param[in]  pvarSrc The source variant.
 * @param[in]  wFlags Flags.
 * @param[in]  vt The variant type.
 * @note       This function is not sufficiently compatible with Windows.
 */
HRESULT
VariantChangeType(VARIANT *pvargDest, VARIANT *pvarSrc, uint16_t wFlags,
    uint16_t vt)
{
  HRESULT hr = S_OK;

  if ((pvargDest == NULL) || (pvarSrc == NULL)) {
    return E_INVALIDARG;
  }

  if (pvargDest != pvarSrc) {
    VariantClear(pvargDest);
    if (vt == pvarSrc->vt) {
      return VariantCopy(pvargDest, pvarSrc);
    }
  } else {
    if (vt == pvarSrc->vt) {
      return S_OK;
    }
  }

#define CheckOverflow(typeDst, typeSrc) \
  ((sizeof(typeDst) < sizeof(typeSrc)) \
      || (pvarSrc->vt == VT_R4) \
      || (pvarSrc->vt == VT_R8))

#define IsUnsigned \
  ((pvarSrc->vt == VT_UI1) \
      || (pvarSrc->vt == VT_UI2) \
      || (pvarSrc->vt == VT_UI4) \
      || (pvarSrc->vt == VT_UI8))

#define SubChangeType(type, val, chk) \
  switch (vt) { \
    case VT_EMPTY: \
    case VT_NULL: \
      memset(pvargDest, 0, sizeof(VARIANT)); \
      break; \
    case VT_ERROR: \
      return DISP_E_TYPEMISMATCH; \
      break; \
    case VT_I2: \
      if(chk) { if(CheckOverflow(int16_t, type) \
          && ((!IsUnsigned && ((val) < (type)SHRT_MIN)) \
              || ((type)SHRT_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->iVal = (int16_t)(val); \
      break; \
    case VT_I4: \
      if(chk) { if(CheckOverflow(int32_t, type) \
          && ((!IsUnsigned && ((val) < (type)LONG_MIN)) \
              || ((type)LONG_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->lVal = (int32_t)(val); \
      break; \
    case VT_I8: \
      if(chk) { if(CheckOverflow(int64_t, type) \
          && ((!IsUnsigned && ((val) < (type)LLONG_MIN)) \
              || ((type)LLONG_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->llVal = (int64_t)(val); \
      break; \
    case VT_R4: \
      if(chk) { if((pvarSrc->vt == VT_R8) \
          && (((val) < (double)-FLT_MAX) \
              || ((double)FLT_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->fltVal = (float)(val); \
      break; \
    case VT_R8: \
      pvargDest->dblVal = (double)(val); \
      break; \
    case VT_CY: \
      if(chk) { if(CheckOverflow(int64_t, type) \
          && ((!IsUnsigned && ((val) < (type)LLONG_MIN)) \
              || ((type)LLONG_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->cyVal.int64 = (int64_t)(val); \
      break; \
    case VT_DATE: \
      pvargDest->date = (DATE)(val); \
      break; \
    case VT_BSTR: \
      hr = Variant2Bstr(&pvargDest->bstrVal, pvarSrc); \
      break;\
    case VT_BOOL: \
      pvargDest->boolVal = ((val) ? VARIANT_TRUE : VARIANT_FALSE); \
      break; \
    case VT_UI1: \
      if(chk) { if(((val) < (type)0) || \
          (CheckOverflow(uint8_t, type) && ((type)USHRT_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->bVal = (uint8_t)(val); \
      break; \
    case VT_UI2: \
      if(chk) { if(((pvarSrc->vt != VT_I2) && ((val) < (type)0)) || \
          (CheckOverflow(uint16_t, type) && ((type)USHRT_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->uiVal = (uint16_t)(val); \
      break; \
    case VT_UI4: \
      if(chk) { if(((pvarSrc->vt != VT_I4) && ((val) < (type)0)) || \
          (CheckOverflow(uint32_t, type) && ((type)ULONG_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->ulVal = (uint32_t)(val); \
      break; \
    case VT_UI8: \
      if(chk) { if(((pvarSrc->vt != VT_I8) && ((val) < (type)0)) || \
          (CheckOverflow(uint64_t, type) && ((type)ULLONG_MAX < (val)))) { \
        return DISP_E_OVERFLOW; \
      } } \
      pvargDest->ullVal = (uint64_t)(val); \
      break; \
    default: \
      return DISP_E_BADVARTYPE; \
  }

  switch (pvarSrc->vt) {
    case VT_EMPTY:
      switch (vt) {
        case VT_BSTR:
          pvargDest->bstrVal = SysAllocString(L"");
          break;
        case VT_ERROR:
          return DISP_E_TYPEMISMATCH;
        default:
          memset(pvargDest, 0, sizeof(VARIANT));
          break;
      }
      break;
    case VT_NULL:
    case VT_ERROR:
      return DISP_E_TYPEMISMATCH;
    case VT_I2:
      SubChangeType(int16_t, pvarSrc->iVal, 1);
      break;
    case VT_I4:
      SubChangeType(int32_t, pvarSrc->lVal, 1);
      break;
    case VT_I8:
      SubChangeType(int64_t, pvarSrc->llVal, 1);
      break;
    case VT_R4:
      SubChangeType(float, pvarSrc->fltVal, 1);
      break;
    case VT_R8:
      SubChangeType(double, pvarSrc->dblVal, 1);
      break;
    case VT_CY:
      SubChangeType(int64_t, pvarSrc->cyVal.int64, 1);
      break;
    case VT_DATE:
      SubChangeType(DATE, pvarSrc->date, 1);
      break;
    case VT_BSTR:
      hr = Bstr2Variant(pvargDest, vt, pvarSrc->bstrVal);
      break;
    case VT_BOOL:
      SubChangeType(VARIANT_BOOL, pvarSrc->boolVal, 0);
      break;
    case VT_UI1:
      SubChangeType(uint8_t, pvarSrc->bVal, 1);
      break;
    case VT_UI2:
      SubChangeType(uint16_t, pvarSrc->uiVal, 1);
      break;
    case VT_UI4:
      SubChangeType(uint32_t, pvarSrc->ulVal, 1);
      break;
    case VT_UI8:
      SubChangeType(uint64_t, pvarSrc->ullVal, 1);
      break;
    default:
      return DISP_E_BADVARTYPE;
  }

  if (SUCCEEDED(hr)) {
    pvargDest->vt = vt;
  }

  return hr;
}
#endif /* _DN_USE_VARIANT_API */
#endif /* _OLEAUTO_H_ */

#if (_DN_USE_VARIANT_API)
/**
 * @fn         uint32_t ChangeVarType(VARIANT varSrc, uint16_t vt, void *pDest, uint32_t dwSize)
 * @brief      Changes the variant to destination value with the indicated type.
 * @param[in]  varSrc The source variant.
 * @param[in]  vt The variant type.
 * @param[out] pDest The pointer of the destination value.
 * @param[in]  dwSize The maximum number of changed values.
 */
uint32_t
ChangeVarType(VARIANT varSrc, uint16_t vt, void *pDest, uint32_t dwSize)
{
  HRESULT hr = S_OK;
  uint32_t dwRet, dwCnt = 0;
  VARIANT vntTmp;

  if ((pDest == NULL) || (dwSize == 0)) {
    return 0;
  }

  VariantInit(&vntTmp);

  if (varSrc.vt & VT_ARRAY) {
    int32_t i, lMax =
        (int32_t) varSrc.parray->rgsabound[0].cElements;
    void *pVal, *pPos = pDest;

    SafeArrayAccessData(varSrc.parray, &pVal);
    for (i = 0; i < lMax; i++) {
      VariantClear(&vntTmp);
      switch (varSrc.vt ^ VT_ARRAY) {
        case VT_I2:
          vntTmp.vt = VT_I2;
          vntTmp.iVal = *((int16_t *) pVal + i);
          break;
        case VT_I4:
          vntTmp.vt = VT_I4;
          vntTmp.lVal = *((int32_t *) pVal + i);
          break;
        case VT_I8:
          vntTmp.vt = VT_I8;
          vntTmp.llVal = *((int64_t *) pVal + i);
          break;
        case VT_R4:
          vntTmp.vt = VT_R4;
          vntTmp.fltVal = *((float *) pVal + i);
          break;
        case VT_R8:
          vntTmp.vt = VT_R8;
          vntTmp.dblVal = *((double *) pVal + i);
          break;
        case VT_CY:
          vntTmp.vt = VT_CY;
          vntTmp.cyVal = *((CY *) pVal + i);
          break;
        case VT_DATE:
          vntTmp.vt = VT_DATE;
          vntTmp.date = *((DATE *) pVal + i);
          break;
        case VT_BSTR:
          vntTmp.vt = VT_BSTR;
          vntTmp.bstrVal = SysAllocString(*((BSTR *) pVal + i));
          break;
        case VT_BOOL:
          vntTmp.vt = VT_BOOL;
          vntTmp.boolVal = *((VARIANT_BOOL *) pVal + i);
          break;
        case VT_VARIANT:
          VariantCopy(&vntTmp, (VARIANT *) pVal + i);
          break;
        case VT_UI1:
          vntTmp.vt = VT_UI1;
          vntTmp.bVal = *((uint8_t *) pVal + i);
          break;
        case VT_UI2:
          vntTmp.vt = VT_UI2;
          vntTmp.uiVal = *((uint16_t *) pVal + i);
          break;
        case VT_UI4:
          vntTmp.vt = VT_UI4;
          vntTmp.ulVal = *((uint32_t *) pVal + i);
          break;
        case VT_UI8:
          vntTmp.vt = VT_UI8;
          vntTmp.ullVal = *((uint64_t *) pVal + i);
          break;
        default:
          hr = E_INVALIDARG;
          break;
      }

      if (FAILED(hr)) {
        break;
      }

      if (vt != VT_VARIANT) {
        if (vntTmp.vt & VT_ARRAY) {
          break;
        }

        dwRet = ChangeVarType(vntTmp, vt, pPos, 1);
        if (dwRet == 0) {
          break;
        }
      } else {
        hr = VariantCopy((VARIANT*) pPos, &vntTmp);
        if (FAILED(hr)) {
          break;
        }
        dwRet = 1;
      }

      dwCnt += dwRet;
      pPos = ((char *) pPos + varSrc.parray->cbElements);

      if (dwCnt >= dwSize) {
        break;
      }
    }
    SafeArrayUnaccessData(varSrc.parray);
  } else {
    VariantCopy(&vntTmp, &varSrc);
    if (vt != VT_VARIANT) {
      hr = VariantChangeType(&vntTmp, &vntTmp, 0, vt);
      if (FAILED(hr)) {
        dwCnt = 0;
        goto exit_proc;
      }
    }

    dwCnt = 1;
    switch (vt) {
      case VT_I2:
        *(int16_t *) pDest = vntTmp.iVal;
        break;
      case VT_I4:
        *(int32_t *) pDest = vntTmp.lVal;
        break;
      case VT_I8:
        *(int64_t *) pDest = vntTmp.llVal;
        break;
      case VT_R4:
        *(float *) pDest = vntTmp.fltVal;
        break;
      case VT_R8:
        *(double *) pDest = vntTmp.dblVal;
        break;
      case VT_CY:
        *(CY *) pDest = vntTmp.cyVal;
        break;
      case VT_DATE:
        *(DATE *) pDest = vntTmp.date;
        break;
      case VT_BSTR:
        *(BSTR *) pDest = SysAllocString(vntTmp.bstrVal);
        break;
      case VT_BOOL:
        *(VARIANT_BOOL *) pDest = vntTmp.boolVal ? VARIANT_TRUE : VARIANT_FALSE;
        break;
      case VT_VARIANT:
        VariantCopy((VARIANT *) pDest, &vntTmp);
        break;
      case VT_UI1:
        *(uint8_t *) pDest = vntTmp.bVal;
        break;
      case VT_UI2:
        *(uint16_t *) pDest = vntTmp.uiVal;
        break;
      case VT_UI4:
        *(uint32_t *) pDest = vntTmp.ulVal;
        break;
      case VT_UI8:
        *(uint64_t *) pDest = vntTmp.ullVal;
        break;
      default:
        dwCnt = 0;
        break;
    }
  }

exit_proc:
  VariantClear(&vntTmp);

  return dwCnt;
}

/**
 * @fn         HRESULT GetOptionValue(BSTR bstrSrc, BSTR bstrKey, uint16_t vt, VARIANT *pvarDest)
 * @brief      Searchs the key string from source string and sets the value to the destination variant with the indicated type.
 * @param[in]  bstrSrc The source string.
 * @param[in]  bstrKey The key string.
 * @param[in]  vt The variant type.
 * @param[out] pvarDest The destination variant.
 */
HRESULT
GetOptionValue(BSTR bstrSrc, BSTR bstrKey, uint16_t vt, VARIANT *pvarDest)
{
  HRESULT hr = S_OK;
  int iLevelCount = 0, bGetTitle = 0;
  long lTitleTerm;
  uint16_t i, uiLenTarget = 0, uiLenOptTitle = 0, uiLenOneOpt = 0;
  wchar_t *wchTmp, *wchPos, wchStart = -1, wchEnd = -1;
  BSTR bstrTmp, bstrTarget, bstrOptTitle, bstrOneOpt;
  VARIANT vntTmp;

  if ((bstrKey == NULL) || (pvarDest == NULL)) {
    return E_INVALIDARG;
  }

  if (bstrSrc != NULL) {
    bstrTarget = SysAllocString(bstrSrc);
  } else {
    bstrTarget = SysAllocString(L"");
  }

  bstrOptTitle = SysAllocString(L"");
  bstrOneOpt = SysAllocString(L"");

  uiLenTarget = SysStringLen(bstrTarget);
  wchTmp = bstrTarget;

  for (i = 0; i <= uiLenTarget; i++, wchTmp++) {
    /* Sets the beginning and ending character for nest */
    if (!iLevelCount) {
      switch (*wchTmp) {
        case L'(':
        case L')':
          wchStart = L'(';
          wchEnd = L')';
          break;
        case L'[':
        case L']':
          wchStart = L'[';
          wchEnd = L']';
          break;
        case L'{':
        case L'}':
          wchStart = L'{';
          wchEnd = L'}';
          break;
        case L'<':
        case L'>':
          wchStart = L'<';
          wchEnd = L'>';
          break;
        default:
          break;
      }
    }

    /* Sets the nest level */
    if (*wchTmp == wchStart) {
      iLevelCount++;
      if (iLevelCount == 1) {
        memcpy(wchTmp, wchTmp + 1, sizeof(wchar_t) * (uiLenTarget - i));
        uiLenTarget--;
        i--;
        wchTmp--;
        continue;
      }
    }
    else if (*wchTmp == wchEnd) {
      iLevelCount--;

      if (iLevelCount < 0) {
        hr = E_FAIL;
        goto exit_proc;
      }

      if (!iLevelCount) {
        memcpy(wchTmp, wchTmp + 1, sizeof(wchar_t) * (uiLenTarget - i));
        uiLenTarget--;
        i--;
        wchTmp--;
        continue;
      }
    }

    /* Gets the option name */
    if (!iLevelCount) {
      if (*wchTmp == L'=') {
        if (bGetTitle) {
          hr = E_FAIL;
          goto exit_proc;
        }

        if (uiLenOptTitle > 0) {
          SysFreeString(bstrOptTitle);
          bstrOptTitle = SysAllocStringLen(wchTmp - uiLenOptTitle,
              uiLenOptTitle);
          uiLenOptTitle = 0;
        }

        bGetTitle = 1;
        continue;
      }

      if (*wchTmp == L',' || *wchTmp == L'\0') {
        if (uiLenOneOpt > 0) {
          SysFreeString(bstrOneOpt);
          bstrOneOpt = SysAllocStringLen(wchTmp - uiLenOneOpt,
              uiLenOneOpt);
          uiLenOneOpt = 0;
        }

        bstrTmp = bstrOptTitle;
        wchPos = bstrTmp;
        while (wchPos[0] == L' ') {
          wchPos++;
        }

        lTitleTerm = wcslen(wchPos) - 1;
        while (wchPos[lTitleTerm] == L' ') {
          wchPos[lTitleTerm] = L'\0';
          lTitleTerm--;
        }

        bstrOptTitle = SysAllocString(wchPos);
        SysFreeString(bstrTmp);

        if (!_wcsicmp(bstrOptTitle, bstrKey)) {
          if (!SysStringLen(bstrOneOpt)) {
            if (vt == VT_BOOL) {
              pvarDest->vt = VT_BOOL;
              pvarDest->boolVal = VARIANT_TRUE;
            } else {
              VariantClear(pvarDest);
            }
            hr = S_OK;
            goto exit_proc;
          }
          break;
        }

        if (*wchTmp == L',') {
          SysFreeString(bstrOptTitle);
          bstrOptTitle = SysAllocString(L"");
          uiLenOptTitle = 0;

          SysFreeString(bstrOneOpt);
          bstrOneOpt = SysAllocString(L"");
          uiLenOneOpt = 0;

          bGetTitle = 0;
          continue;
        } else {
          VariantClear(pvarDest);
          hr = S_OK;
          goto exit_proc;
        }
      }
    }

    if (bGetTitle) {
      uiLenOneOpt++;
    } else {
      uiLenOptTitle++;
    }
  }

  if (iLevelCount != 0) {
    hr = E_FAIL;
    goto exit_proc;
  }

  bstrTmp = bstrOneOpt;
  wchPos = bstrTmp;
  while (wchPos[0] == L' ') {
    wchPos++;
  }

  lTitleTerm = wcslen(wchPos) - 1;
  while (wchPos[lTitleTerm] == L' ') {
    wchPos[lTitleTerm] = L'\0';
    lTitleTerm--;
  }

  bstrOneOpt = SysAllocString(wchPos);
  SysFreeString(bstrTmp);

  vntTmp.vt = VT_BSTR;
  vntTmp.bstrVal = bstrOneOpt;

  hr = VariantChangeType(pvarDest, &vntTmp, 0, vt);

  exit_proc: SysFreeString(bstrTarget);
  SysFreeString(bstrOptTitle);
  SysFreeString(bstrOneOpt);

  return hr;
}
#endif /* _DN_USE_VARIANT_API */

#if (_DN_USE_BSTR_API)
/**
 * @fn         wchar_t* ConvertMultiByte2WideChar(const char* chSrc)
 * @brief      Converts string to wide string.
 * @param[in]  chSrc The source string.
 */
wchar_t*
ConvertMultiByte2WideChar(const char* chSrc)
{
  wchar_t* chDest = NULL;

  if(chSrc != NULL) {
#ifdef _USE_WIN_API
    int iLen = MultiByteToWideChar(CP_ACP, 0, chSrc, -1, NULL, 0);
    if(iLen > 0) {
      chDest = (wchar_t*)malloc(sizeof(wchar_t)*iLen);
      if(chDest == NULL) return NULL;
      MultiByteToWideChar(CP_ACP, 0, chSrc, -1, chDest, iLen);
    }
#else
    char* locale = setlocale(LC_ALL, setlocale(LC_CTYPE, ""));
    int iLen = mbstowcs(NULL, chSrc, 0) + 1;
    if(iLen > 0) {
      chDest = (wchar_t*)malloc(sizeof(wchar_t)*iLen);
      if(chDest == NULL) return NULL;
      mbstowcs(chDest, chSrc, iLen);
    }
    setlocale(LC_ALL, locale);
#endif
  }

  return chDest;  
}

/**
 * @fn         char* ConvertWideChar2MultiByte(const wchar_t* chSrc)
 * @brief      Converts wide string to string.
 * @param[in]  chSrc The source string.
 */
char*
ConvertWideChar2MultiByte(const wchar_t* chSrc)
{
  char* chDest = NULL;

  if(chSrc != NULL) {
#ifdef _USE_WIN_API
    int iLen = WideCharToMultiByte(CP_ACP, 0, chSrc, -1, NULL, 0, NULL, NULL);
    if(iLen > 0) {
      chDest = (char*)malloc(iLen);
      if(chDest == NULL) return NULL;
      WideCharToMultiByte(CP_ACP, 0, chSrc, -1, chDest, iLen, NULL, NULL);
    }
#else
    char* locale = setlocale(LC_ALL, setlocale(LC_CTYPE, ""));
    int iLen = wcstombs(NULL, chSrc, 0) + 1;
    if(iLen > 0) {
      chDest = (char*)malloc(iLen);
      if(chDest == NULL) return NULL;
      wcstombs(chDest, chSrc, iLen);
    }
    setlocale(LC_ALL, locale);
#endif
  }

  return chDest;
}
#endif /* _DN_USE_BSTR_API */
