// half.h - IEEE 754 half-precision floating-point (float16) C header
// 兼容 ARM64 (__fp16) 与 x86 (uint16_t)，支持 union/POD/序列化
// 参考 half.hpp (Christian Rau) 实现，纯 C，无 C++ 特性
// Copyright (c) 2024

#ifndef HALF_H
#define HALF_H

#include <stdint.h>
#include <string.h>

#if defined(__aarch64__) || defined(__ARM_FP16_FORMAT_IEEE) || defined(__ARM_FP16_ARGS)
    typedef __fp16 float16;
    #define HALF_ARM 1
#else
    typedef uint16_t float16;
    #define HALF_ARM 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

// float16 -> float (C float)
static inline float half_to_float(float16 h)
{
#if HALF_ARM
    // ARM: __fp16 可直接转 float
    return (float)h;
#else
    // x86: uint16_t 按 IEEE 754 half 转 float
    uint16_t x = h;
    uint32_t sign = (x & 0x8000) << 16;
    uint32_t exp  = (x & 0x7C00) >> 10;
    uint32_t mant = (x & 0x03FF);
    uint32_t f;
    if (exp == 0) {
        if (mant == 0) {
            f = sign;
        } else {
            // subnormal
            exp = 1;
            while ((mant & 0x0400) == 0) {
                mant <<= 1;
                exp--;
            }
            mant &= 0x03FF;
            exp = exp + (127 - 15);
            f = sign | (exp << 23) | (mant << 13);
        }
    } else if (exp == 0x1F) {
        // Inf/NaN
        f = sign | 0x7F800000 | (mant << 13);
    } else {
        // normalized
        exp = exp + (127 - 15);
        f = sign | (exp << 23) | (mant << 13);
    }
    float out;
    memcpy(&out, &f, sizeof(out));
    return out;
#endif
}

// float (C float) -> float16
static inline float16 float_to_half(float f)
{
#if HALF_ARM
    // ARM: 直接转 __fp16
    return (__fp16)f;
#else
    uint32_t x;
    memcpy(&x, &f, sizeof(x));
    uint32_t sign = (x >> 16) & 0x8000;
    int32_t exp  = ((x >> 23) & 0xFF) - 127 + 15;
    uint32_t mant = x & 0x7FFFFF;
    uint16_t h;
    if (exp <= 0) {
        if (exp < -10) {
            h = sign;
        } else {
            mant = (mant | 0x800000) >> (1 - exp);
            h = sign | (mant >> 13);
        }
    } else if (exp == 0xFF - 127 + 15) {
        if (mant == 0) {
            h = sign | 0x7C00; // Inf
        } else {
            h = sign | 0x7C00 | (mant >> 13); // NaN
        }
    } else if (exp > 30) {
        h = sign | 0x7C00; // overflow -> Inf
    } else {
        h = sign | (exp << 10) | (mant >> 13);
    }
    return h;
#endif
}

#ifdef __cplusplus
}
#endif

#endif // HALF_H
