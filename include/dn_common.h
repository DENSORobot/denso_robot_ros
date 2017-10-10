#ifndef DN_COMMON_H_
#define DN_COMMON_H_

/**
 * @file    dn_common.h
 * @brief   Common API file.
 * @details Defines some types, macros and functions to be compatible with Windows.
 *
 * @version 1.2
 * @date    2014/11/06
 * @date    2015/01/20 Adds VariantCopy, VariantChangeType, ChangeVarType, and GetOptionValue functions.
 * @date    2016/09/15 Adds ConvertMultiByte2WideChar and ConvertWideChar2MultiByte functions.
 * @date    2017/01/18 Adds VT_I8 and VT_UI8.
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

#ifndef _DN_EXP_COMMON
#define _DN_EXP_COMMON
#endif /* _DN_EXP_COMMON */

#ifndef _DN_USE_VARIANT_API
#define _DN_USE_VARIANT_API (1)
#endif

#ifndef _DN_USE_BSTR_API
#define _DN_USE_BSTR_API (1)
#endif

/**
 * @def   FORMAT_DATE2BSTR
 * @brief A definition for the format string converting DATE to BSTR.
 * @note  You can change this parameter.
 */
#define FORMAT_DATE2BSTR "%Y/%m/%d %H:%M:%S"

#ifndef _HRESULT_DEFINED
#define _HRESULT_DEFINED

typedef int32_t HRESULT;

#endif /* _HRESULT_DEFINED */

#ifndef _WINERROR_

/**
 * @def   SUCCEEDED(hr)
 * @brief A macro that returns TRUE/FALSE. If hr is zero or more, then returns TRUE.
 */
#define SUCCEEDED(hr) ((HRESULT)hr >= 0)

/**
 * @def   FAILED(hr)
 * @brief A macro that returns TRUE/FALSE. If hr is less than zero, then returns TRUE.
 */
#define FAILED(hr) ((HRESULT)hr < 0)

/**
 * @def   _HRESULT_TYPEDEF_(_sc)
 * @brief A macro that defines a new HRESULT.
 */
#define _HRESULT_TYPEDEF_(_sc) ((HRESULT)_sc)

/**
 * @def   S_OK
 * @brief Succeeded.
 */
#define S_OK ((HRESULT)0L)

/**
 * @def   S_FALSE
 * @brief Succeeded but some processes may remain.
 */
#define S_FALSE ((HRESULT)1L)

/**
 * @def   E_NOTIMPL
 * @brief Failed because the function is not implemented.
 */
#define E_NOTIMPL _HRESULT_TYPEDEF_(0x80004001L)

/**
 * @def   E_FAIL
 * @brief Failed because unspecified error occurs.
 */
#define E_FAIL _HRESULT_TYPEDEF_(0x80004005L)

/**
 * @def   E_ACCESSDENIED
 * @brief Failed because the resource is not ready.
 */
#define E_ACCESSDENIED _HRESULT_TYPEDEF_(0x80070005L)

/**
 * @def   E_HANDLE
 * @brief Failed because the handle is invalid.
 */
#define E_HANDLE _HRESULT_TYPEDEF_(0x80070006L)

/**
 * @def   E_OUTOFMEMORY
 * @brief Failed because there is no enough memory space.
 */
#define E_OUTOFMEMORY _HRESULT_TYPEDEF_(0x8007000EL)

/**
 * @def   E_INVALIDARG
 * @brief Failed because some arguments are invalid.
 */
#define E_INVALIDARG _HRESULT_TYPEDEF_(0x80070057L)

/**
 * @def   E_UNEXPECTED
 * @brief Failed because unexpected error happens.
 */
#define E_UNEXPECTED _HRESULT_TYPEDEF_(0x8000FFFFL)

/**
 * @def   DISP_E_TYPEMISMATCH
 * @brief Failed because the type mismatch.
 */
#define DISP_E_TYPEMISMATCH _HRESULT_TYPEDEF_(0x80020005L)

/**
 * @def   DISP_E_BADVARTYPE
 * @brief Failed because bad variable type.
 */
#define DISP_E_BADVARTYPE _HRESULT_TYPEDEF_(0x80020008L)

/**
 * @def   DISP_E_OVERFLOW
 * @brief Failed because out of range.
 */
#define DISP_E_OVERFLOW _HRESULT_TYPEDEF_(0x8002000AL)

/**
 * @def   DISP_E_BADINDEX
 * @brief Failed because the index is invalid.
 */
#define DISP_E_BADINDEX _HRESULT_TYPEDEF_(0x8002000BL)

#endif /* _WINERROR_ */

/**
 * @def   E_TIMEOUT
 * @brief Failed because the communication timed out.
 */
#define E_TIMEOUT _HRESULT_TYPEDEF_(0x80000900L)

/**
 * @def   E_NOT_CONNECTED
 * @brief Failed because the connection is not established.
 */
#define E_NOT_CONNECTED _HRESULT_TYPEDEF_(0x80000902L)

/**
 * @def   E_MAX_OBJECT
 * @brief Failed because the packet is too much.
 */
#define E_MAX_OBJECT _HRESULT_TYPEDEF_(0x80000905L)

/**
 * @def   E_ALREADY_REGISTER
 * @brief Failed because the packet is too much.
 */
#define E_ALREADY_REGISTER _HRESULT_TYPEDEF_(0x80000906L)

/**
 * @def   E_TOO_MUCH_DATA
 * @brief Failed because the packet is too much.
 */
#define E_TOO_MUCH_DATA _HRESULT_TYPEDEF_(0x80000909L)

/**
 * @def   E_MAX_CONNECT
 * @brief Failed because the number of connection is too much.
 */
#define E_MAX_CONNECT _HRESULT_TYPEDEF_(0x8000090BL)

/**
 * @def   OSERR2HRESULT(err)
 * @brief A macro that returns HREUSLT(0x8091) which means OS error.
 */
#define OSERR2HRESULT(err) (((err) & 0x0000FFFF) | 0x80910000)

#ifndef __wtypes_h__

/**
 * @enum  VARENUM
 * @brief Variant type
 */
enum VARENUM
{
  VT_EMPTY = 0,      /**< No argument  */
  VT_NULL = 1,       /**< No argument  */
  VT_I2 = 2,         /**< int16_t      */
  VT_I4 = 3,         /**< int32_t      */
  VT_R4 = 4,         /**< float        */
  VT_R8 = 5,         /**< double       */
  VT_CY = 6,         /**< CY           */
  VT_DATE = 7,       /**< DATE         */
  VT_BSTR = 8,       /**< BSTR         */
  VT_ERROR = 10,     /**< ERROR        */
  VT_BOOL = 11,      /**< VARIANT_BOOL */
  VT_VARIANT = 12,   /**< VARIANT      */
  VT_UI1 = 17,       /**< uint8_t      */
  VT_UI2 = 18,       /**< uint16_t     */
  VT_UI4 = 19,       /**< uint32_t     */
  VT_I8 = 20,        /**< int64_t      */
  VT_UI8 = 21,       /**< uint64_t     */
  VT_ARRAY = 0x2000, /**< SAFEARRAY    */
};

#if (!defined(_WCHAR_H) && (!defined(_INC_WCHAR)))
#include <wchar.h>
#endif

typedef wchar_t *BSTR;

/**
 * @union tagCY
 * @brief A type definition for the signed 64bit integer.
 */
typedef union tagCY
{
  struct
  {
    unsigned long Lo; /**< The lower value */
    long Hi;          /**< The upper value */
  } DUMMYSTRUCTNAME;
  int64_t int64; /**< The signed 64bit integer */
} CY;

#if (!defined(_TIME_H) && (!defined(_INC_WCHAR)))
#include <time.h>
#endif

typedef time_t DATE;

typedef int16_t VARIANT_BOOL;

/**
 * @def   VARIANT_TRUE
 * @brief TRUE for VARIANT.
 */
#define VARIANT_TRUE ((VARIANT_BOOL)-1)

/**
 * @def   VARIANT_FALSE
 * @brief FALSE for VARIANT.
 */
#define VARIANT_FALSE ((VARIANT_BOOL) 0)

#endif /* __wtypes_h__ */

#ifndef __oaidl_h__

/**
 * @struct SAFEARRAYBOUND
 * @brief  A type definition for the bound of SAFEARRAY.
 */
typedef struct SAFEARRAYBOUND
{
  uint32_t cElements; /**< The number of elements       */
  int32_t lLbound;    /**< The lower bound of SAFEARRAY */
} SAFEARRAYBOUND;

/**
 * @struct SAFEARRAY
 * @brief  A type definition for the array.
 */
typedef struct SAFEARRAY
{
  uint16_t cDims;              /**< The number of dimensions  */
  uint16_t vt;                 /**< Variant type              */
  uint32_t cbElements;         /**< The size of an element    */
  void* pvData;                /**< The pointer of array data */
  SAFEARRAYBOUND rgsabound[1]; /**< The parameters of bounds  */
} SAFEARRAY;

/**
 * @struct VARIANT
 * @brief  A type definition for the multi type variable.
 */
typedef struct VARIANT
{
  uint16_t vt; /**< Variant type */
  union
  {
    int16_t iVal;         /**< VT_I2    */
    int32_t lVal;         /**< VT_I4    */
    int64_t llVal;        /**< VT_I8    */
    float fltVal;         /**< VT_R4    */
    double dblVal;        /**< VT_R8    */
    CY cyVal;             /**< VT_CY    */
    DATE date;            /**< VT_DATE  */
    BSTR bstrVal;         /**< VT_BSTR  */
    int32_t scode;        /**< VT_ERROR */
    VARIANT_BOOL boolVal; /**< VT_BOOL  */
    uint8_t bVal;         /**< VT_UI1   */
    uint16_t uiVal;       /**< VT_UI2   */
    uint32_t ulVal;       /**< VT_UI4   */
    uint64_t ullVal;      /**< VT_UI8   */
    SAFEARRAY* parray;    /**< VT_ARRAY */
  };
} VARIANT;

#endif /* __oaidl_h__ */

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef _OLEAUTO_H_

  /**
   * @fn        BSTR SysAllocString(const wchar_t *sz)
   * @brief     Allocates and returns BSTR.
   * @param[in] sz Unicode string to be copied. This must be NULL terminated.
   * @note      If there is no enough memory space, then return NULL.
   */
  _DN_EXP_COMMON BSTR
  SysAllocString(const wchar_t *sz);

  /**
   * @fn        BSTR SysAllocStringLen(const wchar_t *pch, uint16_t cch)
   * @brief     Allocates and returns BSTR.
   * @param[in] pch Unicode string to be copied. This must be NULL terminated.
   * @param[in] cch The number of characters to be copied.
   * @note      If there is no enough memory space, then return NULL.
   */
  _DN_EXP_COMMON BSTR
  SysAllocStringLen(const wchar_t *pch, uint16_t cch);

  /**
   * @fn            void SysFreeString(BSTR bstr)
   * @brief         Releases the memory of BSTR.
   * @param[in,out] bstr Unicode string to be freed.
   */
  _DN_EXP_COMMON void
  SysFreeString(BSTR bstr);

  /**
   * @fn        uint16 SysStringLen(BSTR bstr)
   * @brief     Gets and returns the number of characters of BSTR.
   * @param[in] bstr Unicode string to be gotten. This must be NULL terminated.
   */
  _DN_EXP_COMMON uint16_t
  SysStringLen(BSTR bstr);

  /**
   * @fn        SAFEARRAY* SafeArrayCreateVector(uint16_t vt, int32_t lLbound, uint32_t cElements)
   * @brief     Allocates and returns SAFEARRAY.
   * @param[in] vt Variant type.
   * @param[in] lLbound The lower bound of array. This should be 0.
   * @param[in] cElements The number of elements.
   * @note      If there is no enough memory space, then return NULL.
   */
  _DN_EXP_COMMON SAFEARRAY*
  SafeArrayCreateVector(uint16_t vt, int32_t lLbound, uint32_t cElements);

  /**
   * @fn            HRESULT SafeArrayDestroy(SAFEARRAY *psa)
   * @brief         Releases the memory of SAFEARRAY.
   * @param[in,out] psa SAFEARRAY to be freed.
   */
  _DN_EXP_COMMON HRESULT
  SafeArrayDestroy(SAFEARRAY *psa);

  /**
   * @fn        uint16_t SafeArrayGetDim(SAFEARRAY *psa)
   * @brief     Gets and returns the dimension of SAFEARRAY.
   * @param[in] psa SAFEARRAY to be gotten.
   */
  _DN_EXP_COMMON uint16_t
  SafeArrayGetDim(SAFEARRAY *psa);

  /**
   * @fn        uint32_t SafeArrayGetElemsize(SAFEARRAY *psa)
   * @brief     Gets and returns the size of an element.
   * @param[in] psa SAFEARRAY to be gotten.
   */
  _DN_EXP_COMMON uint32_t
  SafeArrayGetElemsize(SAFEARRAY *psa);

  /**
   * @fn         HRESULT SafeArrayGetLBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plLbound)
   * @brief      Gets the lower bound of SAFEARRAY.
   * @param[in]  psa SAFEARRAY to be gotten.
   * @param[in]  nDim The target dimension.
   * @param[out] plLbound The gotten lower bound.
   */
  _DN_EXP_COMMON HRESULT
  SafeArrayGetLBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plLbound);

  /**
   * @fn         HRESULT SafeArrayGetUBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plUbound)
   * @brief      Gets the upper bound of SAFEARRAY.
   * @param[in]  psa SAFEARRAY to be gotten.
   * @param[in]  nDim The target dimension.
   * @param[out] plUbound The gotten upper bound.
   */
  _DN_EXP_COMMON HRESULT
  SafeArrayGetUBound(SAFEARRAY *psa, uint16_t nDim, int32_t *plUbound);

  /**
   * @fn         HRESULT SafeArrayGetVartype(SAFEARRAY *psa, uint16_t *pvt)
   * @brief      Gets the variant type of SAFEARRAY.
   * @param[in]  psa SAFEARRAY to be gotten.
   * @param[out] pvt The gotten variant type.
   */
  _DN_EXP_COMMON HRESULT
  SafeArrayGetVartype(SAFEARRAY *psa, uint16_t *pvt);

  /**
   * @fn         HRESULT SafeArrayAccessData(SAFEARRAY *psa, void **ppvData)
   * @brief      Accesses the SAFEARRAY and gets the pointer of array data.
   * @param[in]  psa SAFEARRAY to be accessed.
   * @param[out] ppvData The gotten pointer.
   * @note  This function must be called before accessing SAFEARRAY data.
   */
  _DN_EXP_COMMON HRESULT
  SafeArrayAccessData(SAFEARRAY *psa, void **ppvData);

  /**
   * @fn        HRESULT SafeArrayUnaccessData(SAFEARRAY *psa)
   * @brief     Unaccesses the SAFEARRAY.
   * @param[in] psa SAFEARRAY to be unaccessed.
   * @note      This function must be called after accessing SAFEARRAY data.
   */
  _DN_EXP_COMMON HRESULT
  SafeArrayUnaccessData(SAFEARRAY *psa);

  /**
   * @fn            void VariantInit(VARIANT *pvarg)
   * @brief         Initializes the VARIANT.
   * @param[in,out] pvarg VARIANT to be initialized.
   * @note          This function must be called before accessing VARIANT.
   */
  _DN_EXP_COMMON void
  VariantInit(VARIANT *pvarg);

  /**
   * @fn            void VariantClear(VARIANT *pvarg)
   * @brief         Clears the VARIANT.
   * @param[in,out] pvarg VARIANT to be cleared.
   * @note          This function must be called before destructing VARIANT.
   */
  _DN_EXP_COMMON void
  VariantClear(VARIANT *pvarg);

#if (_DN_USE_VARIANT_API)
  /**
   * @fn         HRESULT VariantCopy(VARIANT *pvargDest, const VARIANT *pvargSrc)
   * @brief      Copies the source variant to destination variant.
   * @param[out] pvargDest The destination variant to be changed.
   * @param[in]  pvarSrc The source variant.
   */
  _DN_EXP_COMMON HRESULT
  VariantCopy(VARIANT *pvargDest, const VARIANT *pvargSrc);

  /**
   * @fn         HRESULT VariantChangeType(VARIANT *pvargDest, VARIANT *pvarSrc, uint16_t wFlags, uint16_t vt)
   * @brief      Changes the source variant to destination variant with the indicated type.
   * @param[out] pvargDest The destination variant to be changed.
   * @param[in]  pvarSrc The source variant.
   * @param[in]  wFlags Flags.
   * @param[in]  vt The variant type.
   * @note       This function is not sufficiently compatible with Windows.
   */
  _DN_EXP_COMMON HRESULT
  VariantChangeType(VARIANT *pvargDest, VARIANT *pvarSrc, uint16_t wFlags,
      uint16_t vt);
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
  _DN_EXP_COMMON uint32_t
  ChangeVarType(VARIANT varSrc, uint16_t vt, void *pDest, uint32_t dwSize);

  /**
   * @fn         HRESULT GetOptionValue(BSTR bstrSrc, BSTR bstrKey, uint16_t vt, VARIANT *pvarDest)
   * @brief      Searchs the key string from source string and sets the value to the destination variant with the indicated type.
   * @param[in]  bstrSrc The source string.
   * @param[in]  bstrKey The key string.
   * @param[in]  vt The variant type.
   * @param[out] pvarDest The destination variant.
   */
  _DN_EXP_COMMON HRESULT
  GetOptionValue(BSTR bstrSrc, BSTR bstrKey, uint16_t vt, VARIANT *pvarDest);
#endif /* _DN_USE_VARIANT_API */

#if (_DN_USE_BSTR_API)
  /**
   * @fn         wchar_t* ConvertMultiByte2WideChar(const char* chSrc)
   * @brief      Converts string to wide string.
   * @param[in]  chSrc The source string.
   */
  _DN_EXP_COMMON wchar_t*
  ConvertMultiByte2WideChar(const char* chSrc);

  /**
   * @fn         char* ConvertWideChar2MultiByte(const wchar_t* chSrc)
   * @brief      Converts wide string to string.
   * @param[in]  chSrc The source string.
   */
  _DN_EXP_COMMON char*
  ConvertWideChar2MultiByte(const wchar_t* chSrc);
#endif /* _DN_USE_BSTR_API */

#ifdef __cplusplus
}
#endif

#endif /* DN_COMMON_H_ */
