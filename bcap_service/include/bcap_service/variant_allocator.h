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

#ifndef _VARIANT_ALLOCATOR_H_
#define _VARIANT_ALLOCATOR_H_

#include <limits>
#include "bcap_core/dn_common.h"

template<class T> class VariantAllocator
{
public:
  typedef size_t  size_type;
  typedef ptrdiff_t difference_type;
  typedef T* pointer;
  typedef const T* const_pointer;
  typedef T& reference;
  typedef const T& const_reference;
  typedef T value_type;

  template <class U>
  struct rebind
  {
    typedef VariantAllocator<U> other;
  };

  pointer allocate(size_type num, const void* hint = 0)
  {
    return (pointer)( ::operator new(num * sizeof(T)));
  }

  void construct(pointer p, const T& value)
  {
    VariantInit(p);
    VariantCopy(p, &value);
  }

  pointer address(reference value) const
  { 
    return &value; 
  }

  const_pointer address(const_reference value) const
  { 
    return &value;
  }

  void destroy(pointer p)
  {
    VariantClear(p);
  }

  void deallocate(pointer p, size_type n)
  {
    ::operator delete((void*)p);
  }

  size_type max_size() const throw()
  {
    return std::numeric_limits<size_t>::max() / sizeof(T);
  }
};

#endif
