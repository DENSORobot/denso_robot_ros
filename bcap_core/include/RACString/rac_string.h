#ifndef RAC_STRING_H_
#define RAC_STRING_H_

/**
 * @file    rac_string.h
 * @brief   RAC(Robot Action Command) String API file.
 * @details Defines RAC String convertion APIs.
 *
 * @version 1.0
 * @date    2016/09/15
 * @author  DENSO WAVE
 *
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

#ifndef _DN_EXP_RACSTR
#define _DN_EXP_RACSTR
#endif /* _DN_EXP_RACSTR */

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @fn            HRESULT ConvertRacStr2Variant(uint16_t vt, const char* chSrc, VARIANT* pvargDest)
   * @brief         Converts RAC string to VARIANT.
   * @param[in]     vt The variant type.
   * @param[in]     chSrc The source string.
   * @param[out]    pvargDest The destination variant.
   */
  _DN_EXP_RACSTR HRESULT
  ConvertRacStr2Variant(uint16_t vt, const char* chSrc, VARIANT* pvargDest);

  /**
   * @fn            HRESULT ConvertVariant2RacStr(VARIANT varSrc, char** chDest)
   * @brief         Converts VARIANT to RAC string.
   * @param[in]     varSrc The source variant.
   * @param[out]    chDest The destination string.
   */
  _DN_EXP_RACSTR HRESULT
  ConvertVariant2RacStr(VARIANT varSrc, char** chDest);

#ifdef __cplusplus
}
#endif

#endif /* RAC_STRING_H_ */
