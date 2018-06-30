#ifndef __CORE_H__
#define __CORE_H__

#include <gmtl/gmtl.h>
#include <glog/logging.h>
#define NODE_NAME_SUPPORT

#ifdef M_PI
    #undef M_PI
#endif
#define M_PI       3.14159265358979323846f
#define INV_PI     0.31830988618379067154f
#define INV_TWOPI  0.15915494309189533577f
#define INV_FOURPI 0.07957747154594766788f
#ifndef INFINITY
    #define INFINITY FLT_MAX
#endif
#define _EPSILON 0.0001f

#ifdef _MSC_VER
#define MachineEpsilon (std::numeric_limits<float>::epsilon() * 0.5)
#else
static constexpr float MachineEpsilon =
std::numeric_limits<float>::epsilon() * 0.5;
#endif

#include <float.h>

#define isnan _isnan
//#define isinf(f) (!_finite((f)))
#if _MSC_VER >= 1900
    #define int8_t __int8
    #define uint8_t unsigned __int8
    #define int16_t __int16
    #define uint16_t unsigned __int16
    #define int32_t __int32
    #define uint32_t unsigned __int32
    #define int64_t __int64
    #define uint64_t unsigned __int64
#else
    #include <stdint.h>
#endif

#ifdef _WIN32
    #pragma warning (disable : 4305) // double constant assigned to float
    #pragma warning (disable : 4244) // int -> float conversion
    #pragma warning (disable : 4267) // size_t -> unsigned int conversion
    #pragma warning (disable : 4996) // size_t -> unsigned int conversion
#endif // _WIN32

// Global Include Files
#include <math.h>
#include <stdlib.h>
#define _GNU_SOURCE 1
#include <stdio.h>
#include <string.h>
#include <string>
using std::string;
#include <vector>
using std::vector;
#include "error.h"
#include <assert.h>
#include <algorithm>
using std::min;
using std::max;
using std::swap;
using std::sort;

#ifndef _WIN32
    typedef unsigned int* uintptr_t;
#endif

#if defined(_MSC_VER)
#define THREAD_LOCAL __declspec(thread)
#else
#define THREAD_LOCAL __thread
#endif

// Global Inline Functions
inline float Lerp(float t, float v1, float v2) {
    return (1.f - t) * v1 + t * v2;
}


inline float Clamp(float val, float low, float high) {
    if (val < low) return low;
    else if (val > high) return high;
    else return val;
}


inline int Clamp(int val, int low, int high) {
    if (val < low) return low;
    else if (val > high) return high;
    else return val;
}


inline int Mod(int a, int b) {
    int n = int(a/b);
    a -= n*b;
    if (a < 0) a += b;
    return a;
}


inline float Radians(float deg) {
    return ((float)M_PI/180.f) * deg;
}


inline float Degrees(float rad) {
    return (180.f/(float)M_PI) * rad;
}


inline float Log2(float x) {
    static float invLog2 = 1.f / logf(2.f);
    return logf(x) * invLog2;
}


inline int Floor2Int(float val);
inline int Log2Int(float v) {
    return Floor2Int(Log2(v));
}


inline bool IsPowerOf2(int v) {
    return (v & (v - 1)) == 0;
}


inline unsigned int RoundUpPow2(unsigned int v) {
    v--;
    v |= v >> 1;    v |= v >> 2;
    v |= v >> 4;    v |= v >> 8;
    v |= v >> 16;
    return v+1;
}


inline int Floor2Int(float val) {
    return (int)floorf(val);
}


inline int Round2Int(float val) {
    return Floor2Int(val + 0.5f);
}


inline int Float2Int(float val) {
    return (int)val;
}


inline int Ceil2Int(float val) {
    return (int)ceilf(val);
}

inline bool Quadratic(float A, float B, float C, float *t0, float *t1) {
    // Find quadratic discriminant
    float discrim = B * B - 4.f * A * C;
    if (discrim <= 0.) return false;
    float rootDiscrim = sqrtf(discrim);

    // Compute quadratic _t_ values
    float q;
    if (B < 0) q = -.5f * (B - rootDiscrim);
    else       q = -.5f * (B + rootDiscrim);
    *t0 = q / A;
    *t1 = C / q;
    if (*t0 > *t1) swap(*t0, *t1);
    return true;
}

inline float gamma(int n) {
    return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
}

inline bool SameHemisphere(const gmtl::Vec3f &w, const gmtl::Vec3f &wp) {
    return w[2] * wp[2] > 0.f;
}

// Global Inline Functions
inline uint32_t FloatToBits(float f) {
    uint32_t ui;
    memcpy(&ui, &f, sizeof(float));
    return ui;
}

inline float BitsToFloat(uint32_t ui) {
    float f;
    memcpy(&f, &ui, sizeof(uint32_t));
    return f;
}

inline uint64_t FloatToBits(double f) {
    uint64_t ui;
    memcpy(&ui, &f, sizeof(double));
    return ui;
}

inline double BitsToFloat(uint64_t ui) {
    double f;
    memcpy(&f, &ui, sizeof(uint64_t));
    return f;
}

struct UniversalOptions
{
    UniversalOptions(): quiet(false), nThreads(0) {}
    bool quiet;
    int nThreads;
};

extern UniversalOptions g_UniversalOptions;
#endif
