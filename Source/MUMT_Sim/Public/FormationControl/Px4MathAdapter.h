/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE.
 *
 * PX4-Autopilot v1.17.0, commit
 * d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
 ****************************************************************************/
#pragma once

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <limits>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif
#ifndef M_PI_2_F
#define M_PI_2_F 1.57079632679489661923f
#endif
#ifndef CONSTANTS_ONE_G
#define CONSTANTS_ONE_G 9.80665f
#endif
#ifndef M_TWOPI_F
#define M_TWOPI_F 6.28318530717958647692f
#endif
#ifndef PX4_ISFINITE
#define PX4_ISFINITE(x) std::isfinite(x)
#endif
#ifndef PX4_WARN
#define PX4_WARN(...) ((void)0)
#endif
template<typename T> constexpr T lerp(T a,T b,T t){return a+(b-a)*t;}

namespace math {
template<typename T> constexpr T min(T a, T b) { return a < b ? a : b; }
template<typename T> constexpr T max(T a, T b) { return a > b ? a : b; }
template<typename T> constexpr T constrain(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }
template<typename T> constexpr T sq(T x) { return x * x; }
template<typename T> constexpr T sign(T x) { return (T(0) < x) - (x < T(0)); }
template<typename T> constexpr int signNoZero(T x) { return (T(0) <= x) - (x < T(0)); }
template<typename T> constexpr T abs_t(T x) { return x < T(0) ? -x : x; }
template<typename T> inline bool isFinite(T x) { return std::isfinite(x); }
namespace trajectory { inline float computeMaxSpeedFromDistance(float jerk,float accel,float distance,float final_speed) {
 const auto sqr=[](float x){return x*x;}; const float b=4.f*sqr(accel)/jerk;
 const float c=-2.f*accel*distance-sqr(final_speed);
 return fmaxf(.5f*(-b+sqrtf(sqr(b)-4.f*c)),final_speed);
} }
}

namespace matrix {
template<typename T> class Vector2 {
public:
    constexpr Vector2() = default;
    constexpr Vector2(T x, T y) : v_{x, y} {}
    template<typename U> explicit constexpr Vector2(const Vector2<U>& o) : v_{T(o(0)), T(o(1))} {}
    constexpr T &operator()(int i) { return v_[i]; }
    constexpr T operator()(int i) const { return v_[i]; }
    constexpr Vector2 operator+(const Vector2& o) const { return {v_[0]+o.v_[0], v_[1]+o.v_[1]}; }
    constexpr Vector2 operator-(const Vector2& o) const { return {v_[0]-o.v_[0], v_[1]-o.v_[1]}; }
    constexpr Vector2 operator-() const { return {-v_[0],-v_[1]}; }
    constexpr Vector2 operator*(T s) const { return {v_[0]*s,v_[1]*s}; }
    constexpr Vector2 operator/(T s) const { return {v_[0]/s,v_[1]/s}; }
    constexpr T dot(const Vector2& o) const { return v_[0]*o.v_[0]+v_[1]*o.v_[1]; }
    constexpr T cross(const Vector2& o) const { return v_[0]*o.v_[1]-v_[1]*o.v_[0]; }
    T norm() const { return std::sqrt(dot(*this)); }
    Vector2 normalized() const { return *this / norm(); }
private: T v_[2]{};
};
template<typename T> constexpr Vector2<T> operator*(T s,const Vector2<T>& v){return v*s;}
template<typename T,int R,int C> class Matrix { public:
 T &operator()(int r,int c){return v_[r*C+c];} T operator()(int r,int c)const{return v_[r*C+c];}
 Vector2<T> operator*(const Vector2<T>& x)const { static_assert(R==2&&C==2); return {v_[0]*x(0)+v_[1]*x(1),v_[2]*x(0)+v_[3]*x(1)}; }
private:T v_[R*C]{}; };
using Vector2f=Vector2<float>;
using Vector2d=Vector2<double>;
template<typename T> constexpr T sign(T x) { return math::sign(x); }
inline float wrap_pi(float a) { while (a > M_PI_F) a -= 2.f*M_PI_F; while (a < -M_PI_F) a += 2.f*M_PI_F; return a; }
}
namespace time_literals { constexpr uint64_t operator"" _s(unsigned long long seconds){return seconds*1000000ULL;} }
