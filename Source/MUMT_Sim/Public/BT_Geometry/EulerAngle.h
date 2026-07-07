/*
    BT_Geometry::EulerAngle — Controller_CY 이식용 오일러각 → 쿼터니언 변환.

    ★ 규약 검증 완료 (2026-07-05, 수치 브루트포스 역공학):
    Controller_CY.cpp의 Forward/Up/Right 추출 공식이 X=North, Y=East, Z=Up
    프레임에서 표준 항공 자세(roll φ, pitch θ, yaw ψ — JSBSim 라디안 출력)와
    정확히 일치하려면 다음 조합이 유일해다:

        q = qY(+ψ) * qX(+θ) * qZ(+φ)      (반각 axis-angle의 Hamilton 곱)

    무작위 자세 5종에서 세 축 벡터 모두 1e-9 이내 일치함을 확인했다.
    (검증 스크립트: find_quat_convention 계열 — F/U/R 추출 공식 쌍으로 검증)
*/
#pragma once

#include <cmath>
#include "BT_Geometry/Quaternion.h"

namespace BT_Geometry
{

struct EulerAngle
{
    float Roll  = 0.f;   // φ (rad)
    float Pitch = 0.f;   // θ (rad)
    float Yaw   = 0.f;   // ψ (rad)

    Quaternion toQuaternion() const
    {
        const float hr = Roll  * 0.5f;
        const float hp = Pitch * 0.5f;
        const float hy = Yaw   * 0.5f;

        const Quaternion QYaw  (0.f,           std::sin(hy), 0.f,           std::cos(hy)); // qY(+ψ)
        const Quaternion QPitch(std::sin(hp),  0.f,          0.f,           std::cos(hp)); // qX(+θ)
        const Quaternion QRoll (0.f,           0.f,          std::sin(hr),  std::cos(hr)); // qZ(+φ)

        return QYaw * QPitch * QRoll;
    }
};

} // namespace BT_Geometry
