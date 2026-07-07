/*
    BT_Geometry::Vector3 — Controller_CY 이식용 최소 벡터 수학.

    원본 프로젝트(AIP)의 Vector3.h는 제공되지 않아, Controller_CY.cpp가
    사용하는 연산만 동일 시그니처로 재구성했다:
      생성자(x,y,z), +, -, 스칼라 *(양측), 스칼라 /, dot(), length()

    좌표 규약 (Controller_CY 입력 기준):
      X = North, Y = East, Z = Up(고도 양수)  — "FNED" 표기지만 Z는 위가 양수인
      평면 좌표계다 (원본 주석: 고도를 양수로 가지는 평면 좌표계).
*/
#pragma once

#include <cmath>

namespace BT_Geometry
{

struct Vector3
{
    float X = 0.f;
    float Y = 0.f;
    float Z = 0.f;

    Vector3() = default;
    Vector3(float InX, float InY, float InZ) : X(InX), Y(InY), Z(InZ) {}

    Vector3 operator+(const Vector3& V) const { return Vector3(X + V.X, Y + V.Y, Z + V.Z); }
    Vector3 operator-(const Vector3& V) const { return Vector3(X - V.X, Y - V.Y, Z - V.Z); }
    Vector3 operator*(float S) const          { return Vector3(X * S, Y * S, Z * S); }
    Vector3 operator/(float S) const          { return Vector3(X / S, Y / S, Z / S); }

    float dot(const Vector3& V) const { return X * V.X + Y * V.Y + Z * V.Z; }
    float length() const              { return std::sqrt(X * X + Y * Y + Z * Z); }
};

inline Vector3 operator*(float S, const Vector3& V) { return V * S; }

} // namespace BT_Geometry
