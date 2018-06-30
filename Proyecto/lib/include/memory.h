//#if defined(_MSC_VER)
//#define NOMINMAX
//#pragma once
//#endif

#ifndef __MEMORY_H_
#define __MEMORY_H_

void *AllocAligned(size_t size);
template <typename T>
T *AllocAligned(size_t count) {
    return (T *)AllocAligned(count * sizeof(T));
}

void FreeAligned(void *);

#endif // __MEMORY_H_