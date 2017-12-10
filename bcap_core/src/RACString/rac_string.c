/**
 * Software License Agreement (MIT License)
 *
 * @copyright Copyright (c) 2016 DENSO WAVE INCORPORATED
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_USE_WIN_API)
#include <windows.h>
#endif

#include "dn_common.h"
#include "rac_string.h"

/**
 * @fn            int SplitRacStr(const char* chSrc, char** chSplit)
 * @brief         Splits RAC string.
 * @param[in]     chSrc The source string.
 * @param[in]     chSplit The splitted string.
 */
static int
SplitRacStr(const char* chSrc, char** chSplit)
{
  int iCnt = 1, iBracket = 0;
  char* chPos;

  *chSplit = (char*)malloc(strlen(chSrc) + 1);
  if(*chSplit == NULL) return -1;

  strcpy(*chSplit, chSrc);
  chPos = *chSplit;

  while(*chPos != '\0') {
    switch(*chPos) {
      case '(':
        iBracket++;
        break;
      case ')':
        iBracket--;
        if(iBracket < 0) {
          free(*chSplit);
          return -1;
        }
        break;
      case ',':
        if(iBracket == 0) {
          *chPos = '\0';
          iCnt++;
        }
        break;
      default:
        break;
    }

    chPos++;
  }

  if(iBracket != 0) {
    free(*chSplit);
    return -1;
  }

  return iCnt;
}

/**
 * @fn            HRESULT ConvertRacStr2Variant(uint16_t vt, const char* chSrc, VARIANT* pvargDest)
 * @brief         Converts RAC string to VARIANT.
 * @param[in]     vt The variant type.
 * @param[in]     chSrc The source string.
 * @param[out]    pvargDest The destination variant.
 */
HRESULT
ConvertRacStr2Variant(uint16_t vt, const char* chSrc, VARIANT* pvargDest)
{
  HRESULT  hr = S_OK;
  int      i, iCnt;
  char    *chSplit, *chPos, *chTmp, *chNext;
  wchar_t *wchTmp;
  VARIANT  vntTmp;
  void    *parray;

  if((chSrc == NULL) || (pvargDest == NULL)) {
    return E_INVALIDARG;
  }

  if(vt != VT_BSTR) {
    iCnt = SplitRacStr(chSrc, &chSplit);
    if(iCnt < 0) {
      return E_INVALIDARG;
    }
  } else {
    iCnt = 1;
    chSplit = (char*)malloc(strlen(chSrc) + 1);
    if(chSplit == NULL) return E_OUTOFMEMORY;
    strcpy(chSplit, chSrc);
  }

  VariantInit(&vntTmp);
  VariantClear(pvargDest);

  if(vt == (VT_VARIANT | VT_ARRAY)) {
    pvargDest->vt = VT_VARIANT | VT_ARRAY;
    pvargDest->parray = SafeArrayCreateVector(VT_VARIANT, 0, iCnt);
    SafeArrayAccessData(pvargDest->parray, &parray);
    for(i = 0, chPos = chSplit;
      (i < iCnt) && SUCCEEDED(hr);
      i++)
    {
      chNext = chPos + strlen(chPos) + 1;

      chTmp = strchr(chPos, '(');
      if(chTmp == NULL) { hr = E_INVALIDARG; break; }

      *chTmp = '\0';
      if(strspn(chPos, " ") != (chTmp - chPos)) {
        hr = E_INVALIDARG;
        break;
      }

      chPos = chTmp + 1;

      chTmp = strrchr(chPos, ')');
      if(chTmp == NULL) { hr = E_INVALIDARG; break; }

      *chTmp = '\0';
      if(*(chTmp + 1) != '\0') {
        if(strspn(chTmp + 1, " ") != strlen(chTmp + 1)) {
          hr = E_INVALIDARG;
          break;
        }
      }

      chTmp = strchr(chPos, ',');
      if(chTmp == NULL) { hr = E_INVALIDARG; break; }

      *chTmp = '\0';
      if(strspn(chPos, " 0123456789") != (chTmp - chPos)) {
        hr = E_INVALIDARG;
        break;
      }

      VariantInit((VARIANT*)parray + i);
      hr = ConvertRacStr2Variant(atoi(chPos), chTmp + 1, (VARIANT*)parray + i);

      chPos = chNext;
    }
    SafeArrayUnaccessData(pvargDest->parray);
  }
  else if(vt & VT_ARRAY) {
    pvargDest->vt = vt;
      pvargDest->parray = SafeArrayCreateVector(vt ^ VT_ARRAY, 0, iCnt);
    SafeArrayAccessData(pvargDest->parray, &parray);
    for(i = 0, chPos = chSplit;
      (i < iCnt) && SUCCEEDED(hr);
      i++, chPos += strlen(chPos) + 1)
    {
      wchTmp = ConvertMultiByte2WideChar(chPos);
      if(wchTmp == NULL) { hr = E_OUTOFMEMORY; break; }

      if(vt == (VT_BSTR | VT_ARRAY)) {
        *((BSTR*)parray + i) = SysAllocString(wchTmp);
      } else {
        vntTmp.vt      = VT_BSTR;
        vntTmp.bstrVal = SysAllocString(wchTmp);
        hr = VariantChangeType(&vntTmp, &vntTmp, 0, vt ^ VT_ARRAY);
        if(SUCCEEDED(hr)) {
          memcpy((char*)parray + i * pvargDest->parray->cbElements,
            &vntTmp.iVal, pvargDest->parray->cbElements);
        }
      }
      VariantClear(&vntTmp);
      free(wchTmp);
    }
    SafeArrayUnaccessData(pvargDest->parray);
  }
  else {
    if(iCnt > 1) {
      hr = E_INVALIDARG;
    } else {
      wchTmp = ConvertMultiByte2WideChar(chSplit);
      if(wchTmp == NULL) { hr = E_OUTOFMEMORY; }
      else {
        vntTmp.vt      = VT_BSTR;
        vntTmp.bstrVal = SysAllocString(wchTmp);
        hr = VariantChangeType(pvargDest, &vntTmp, 0, vt);
        VariantClear(&vntTmp);
        free(wchTmp);
      }
    }
  }

  VariantClear(&vntTmp);
  free(chSplit);

  return hr;
}

/**
 * @fn            HRESULT ConvertVariant2RacStr(VARIANT varSrc, char** chDest)
 * @brief         Converts VARIANT to RAC string.
 * @param[in]     varSrc The source variant.
 * @param[out]    chDest The destination string.
 */
HRESULT
ConvertVariant2RacStr(VARIANT varSrc, char** chDest)
{
  HRESULT hr = S_OK;

  if(chDest == NULL) {
    return E_INVALIDARG;
  }

  *chDest = NULL;

  if(varSrc.vt == (VT_VARIANT | VT_ARRAY)) {
    int32_t i, lbnd, ubnd, cnt;
    uint32_t len = 0;
    char chTmp[10], *chPoint;
    VARIANT *pvarTmp;

    SafeArrayGetLBound(varSrc.parray, 1, &lbnd);
    SafeArrayGetUBound(varSrc.parray, 1, &ubnd);
    cnt = ubnd - lbnd + 1;
  
    SafeArrayAccessData(varSrc.parray, (void**)&pvarTmp);
    for(i = 0; i < cnt; i++) {
      hr = ConvertVariant2RacStr(pvarTmp[i], &chPoint);
      if(FAILED(hr)) { if(*chDest != NULL) { free(*chDest); } break; }

      len += sprintf(chTmp, "%d,", pvarTmp[i].vt);
      len += strlen(chPoint) + ((i == cnt - 1) ? 3 : 4);
      *chDest = (char*)realloc(*chDest, len);
      if(*chDest == NULL) { hr = E_OUTOFMEMORY; free(chPoint); break; }

      if(i == 0) *chDest[0] = '\0';

      strcat(*chDest, "(");
      strcat(*chDest, chTmp);
      strcat(*chDest, chPoint);
      strcat(*chDest, (i == cnt - 1) ? ")" : "),");

      free(chPoint);
    }
    SafeArrayUnaccessData(varSrc.parray);
  }
  else if(varSrc.vt & VT_ARRAY) {
    int32_t i, lbnd, ubnd, cnt;
    uint32_t len = 0;
    char *chPoint;
    void *parray;
    VARIANT *pvarTmp;

    SafeArrayGetLBound(varSrc.parray, 1, &lbnd);
    SafeArrayGetUBound(varSrc.parray, 1, &ubnd);
    cnt = ubnd - lbnd + 1;

    SafeArrayAccessData(varSrc.parray, &parray);
    if(varSrc.vt == (VT_BSTR | VT_ARRAY)) {
      for(i = 0; i < cnt; i++) {
        chPoint = ConvertWideChar2MultiByte(*((BSTR*)parray + i));
        if(chPoint == NULL) { hr = E_OUTOFMEMORY; break; }

        len += strlen(chPoint) + ((i == cnt - 1) ? 1 : 2);
        *chDest = (char*)realloc(*chDest, len);
        if(*chDest == NULL) { hr = E_OUTOFMEMORY; free(chPoint); break; }

        if(i == 0) *chDest[0] = '\0';

        strcat(*chDest, chPoint);
        strcat(*chDest, (i == cnt - 1) ? "" : ",");

        free(chPoint);
      }
    } else {
      pvarTmp = (VARIANT*)malloc(sizeof(VARIANT) * cnt);
      if(pvarTmp == NULL) { hr = E_OUTOFMEMORY; }
      else {
        memset(pvarTmp, 0, sizeof(VARIANT) * cnt);
        for(i = 0; i < cnt; i++) {
          pvarTmp[i].vt = varSrc.vt ^ VT_ARRAY;
          memcpy(&pvarTmp[i].iVal, (char*)parray + i * varSrc.parray->cbElements,
            varSrc.parray->cbElements);
          hr = VariantChangeType(&pvarTmp[i], &pvarTmp[i], 0, VT_BSTR);
          if(FAILED(hr)) break;
        }

        if(SUCCEEDED(hr)) {
          for(i = 0; i < cnt; i++) {
            chPoint = ConvertWideChar2MultiByte(pvarTmp[i].bstrVal);
            if(chPoint == NULL) { hr = E_OUTOFMEMORY; break; }

            len += strlen(chPoint) + ((i == cnt - 1) ? 1 : 2);
            *chDest = (char*)realloc(*chDest, len);
            if(*chDest == NULL) { hr = E_OUTOFMEMORY; free(chPoint); break; }

            if(i == 0) *chDest[0] = '\0';

            strcat(*chDest, chPoint);
            strcat(*chDest, (i == cnt - 1) ? "" : ",");

            free(chPoint);
          }
        }

        for(i = 0; i < cnt; i++) {
          VariantClear(&pvarTmp[i]);
        }
        free(pvarTmp);
      }
    }
    SafeArrayUnaccessData(varSrc.parray);
  }
  else {
    VARIANT varTmp;
    VariantInit(&varTmp);
    hr = VariantChangeType(&varTmp, &varSrc, 0, VT_BSTR);
    if(SUCCEEDED(hr)) {
      *chDest = ConvertWideChar2MultiByte(varTmp.bstrVal);
      if(*chDest == NULL) hr = E_OUTOFMEMORY;
    }
    VariantClear(&varTmp);
  }

  return hr;
}
