/*
    BT_Geometry::Quaternion — Controller_CY 이식용 최소 쿼터니언.

    Controller_CY.cpp는 X/Y/Z/W 성분만 직접 읽어 Forward/Up/Right 벡터를
    추출하므로, 성분 저장과 Hamilton 곱만 제공한다.
*/
#pragma once

namespace BT_Geometry
{

struct Quaternion
{
    float X = 0.f;
    float Y = 0.f;
    float Z = 0.f;
    float W = 1.f;

    Quaternion() = default;
    Quaternion(float InX, float InY, float InZ, float InW) : X(InX), Y(InY), Z(InZ), W(InW) {}

    // Hamilton 곱 (this ∘ Q): 회전 합성 — this가 나중에 적용되는 회전.
    Quaternion operator*(const Quaternion& Q) const
    {
        return Quaternion(
            W * Q.X + X * Q.W + Y * Q.Z - Z * Q.Y,
            W * Q.Y + Y * Q.W + Z * Q.X - X * Q.Z,
            W * Q.Z + Z * Q.W + X * Q.Y - Y * Q.X,
            W * Q.W - X * Q.X - Y * Q.Y - Z * Q.Z);
    }
};

} // namespace BT_Geometry
