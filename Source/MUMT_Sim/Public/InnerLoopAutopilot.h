#pragma once
//
// InnerLoopAutopilot — (heading, altitude, speed, roll_ff) 명령 → 조종면+스로틀 PID 내루프.
// BVRGym AircraftPIDAutopilot 골격 개선판 + Controller_CY의 선회 보상 아이디어를 결합해
// JSBSim F-16 3기 폐루프로 검증된 설계(inner_loop.py + pid.py)를 그대로 C++ 포팅.
// GetStick(조준 제어기)을 대체한다.
//
// 의존성 없음(std 수학만) — UE 빌드와 JSBSim 단독 검증 하네스 양쪽에서 컴파일된다.
//
#include <cmath>
#include <algorithm>

namespace MumtCtl
{
    inline double DegToRad(double d) { return d * 3.14159265358979323846 / 180.0; }
    inline double RadToDeg(double r) { return r * 180.0 / 3.14159265358979323846; }
    // 최단 회전각 [-180,180] (BVRGym delta_heading)
    inline double DeltaHeading(double target, double current) { return std::remainder(target - current, 360.0); }
}

// pid.py 포팅: P + 조건부-anti-windup I + (측정값 미분 + 1차 필터) D.
struct FInnerPID
{
    double Kp = 0, Ki = 0, Kd = 0, OutMin = -1, OutMax = 1, Tau = 0.05;
    double Integral = 0, PrevMeas = 0, PrevDeriv = 0;
    bool   bHasPrev = false;

    void Configure(double kp, double ki, double kd, double lo, double hi, double tau = 0.05)
    {
        Kp = kp; Ki = ki; Kd = kd; OutMin = lo; OutMax = hi; Tau = tau;
    }

    double Update(double error, double measForDeriv, double dt)
    {
        const double p = Kp * error;

        double dMeas = bHasPrev ? (measForDeriv - PrevMeas) / dt : 0.0;
        if (Tau > 0.0)
            dMeas = (Tau * PrevDeriv + dt * dMeas) / (Tau + dt);
        const double d = -Kd * dMeas;                       // derivative on measurement

        Integral += error * dt;
        const double unclamped = p + Ki * Integral + d;
        const double out = std::min(OutMax, std::max(OutMin, unclamped));

        // 조건부 anti-windup: 포화 방향으로 더 밀고 있으면 이번 적분을 되돌림
        const bool satHi = unclamped > OutMax, satLo = unclamped < OutMin;
        if ((satHi && error > 0.0) || (satLo && error < 0.0))
            Integral -= error * dt;

        PrevMeas = measForDeriv; PrevDeriv = dMeas; bHasPrev = true;
        return out;
    }

    void Reset() { Integral = 0; PrevMeas = 0; PrevDeriv = 0; bHasPrev = false; }
};

struct FInnerLoopOutput { float Aileron; float Elevator; float Rudder; float Throttle; float SpeedBrake; };

// inner_loop.py::InnerLoopAutopilot 포팅. 상태 영속(per-UAV 인스턴스) 필수.
struct FInnerLoopAutopilot
{
    // 게인 (inner_loop.py 기본값, JSBSim F-16 검증)
    double BankLimitDeg = 60.0;   // 뱅크 제한 (리더는 외부에서 40으로 낮춰 마진 확보)
    // 84° 오버슛(결함 B)의 원인은 게인이 아니라 rollRef 계단 급변이었다. 슬루 제한이
    // 그걸 막으므로 게인은 되살린다: 1.5는 유도가 명령한 컷각(20°+)을 내루프가 못 따라가
    // 횡오차 수렴을 3배 느리게 만들었다(인엔진 시험 2026-07-09: 컷 23° 명령 → 실측 4°).
    double KpHdg2Roll   = 2.5;    // 헤딩오차 1° → roll_ref 2.5° (25°에서 뱅크한계 도달)
    double RollSlewDps  = 40.0;   // rollRef 슬루 제한(°/s) — 레퍼런스 계단 급변 방지 [B]
    double TanRefM      = 1500.0; // atan2 유도 기준거리
    double ThetaMax     = 25.0, ThetaMin = -20.0;

    // 조종면 부호 미세조정 (PIE 튜닝, 기본 1). elevator: F16 음수=기수 up.
    double AilSign = 1.0, ElvSign = 1.0;

    FInnerPID Roll, Pitch, Speed;
    double PrevRollRef = 1e9;     // 슬루 제한 상태 (1e9 = 미초기화 → 첫 스텝은 현재 φ에서 시작)
    bool bInit = false;

    void Init()
    {
        Roll.Configure (0.04, 0.0,   0.012, -1.0, 1.0);
        Pitch.Configure(0.06, 0.008, 0.03,  -1.0, 1.0);
        Speed.Configure(0.08, 0.020, 0.0,    0.05, 1.0);
        bInit = true;
    }

    // 각도 단위: deg. 속도: m/s. p/q: deg/s (EulerRates). climb: 상승률 m/s(+위).
    // throttleNorm: speedCmd<=0 일 때 쓰는 개루프 스로틀 폴백.
    FInnerLoopOutput Step(double headingCmd, double altCmd, double speedCmd, double rollFfDeg,
                          double phiDeg, double thetaDeg, double psiDeg,
                          double altM, double tasMps, double climbMps,
                          double pDps, double qDps, double throttleNorm, double dt)
    {
        if (!bInit) Init();

        // ── 횡: heading → roll_ref(피드포워드 뱅크 포함) → aileron ──
        const double dPsi = MumtCtl::DeltaHeading(headingCmd, psiDeg);
        double rollRef = rollFfDeg + KpHdg2Roll * dPsi;
        rollRef = std::min(BankLimitDeg, std::max(-BankLimitDeg, rollRef));
        // 슬루 제한 [B]: 레퍼런스가 계단으로 튀면 에일러론 포화→롤 관성→오버슛(84° 실측).
        // 40°/s 램프면 레이트 항이 선형 영역에 머물러 ζ≈0.9 감쇠가 살아난다.
        if (PrevRollRef > 1e8)
            PrevRollRef = phiDeg;                        // 재engage 범프리스: 현재 뱅크에서 출발
        const double slew = RollSlewDps * dt;
        rollRef = std::min(PrevRollRef + slew, std::max(PrevRollRef - slew, rollRef));
        PrevRollRef = rollRef;
        double aileron = Roll.Update(rollRef - phiDeg, phiDeg, dt) - 0.004 * pDps;
        aileron *= AilSign;

        // ── 종: alt → gamma_ref → theta_ref(받음각 보상) → elevator + 선회보상 ──
        const double v = std::max(tasMps, 30.0);
        const double gamma    = MumtCtl::RadToDeg(std::asin(std::min(1.0, std::max(-1.0, climbMps / v))));
        const double gammaRef = MumtCtl::RadToDeg(std::atan2(altCmd - altM, TanRefM));
        double thetaRef = gammaRef + (thetaDeg - gamma);                 // 경로각 보상
        thetaRef = std::min(ThetaMax, std::max(ThetaMin, thetaRef));
        double elevator = -Pitch.Update(thetaRef - thetaDeg, thetaDeg, dt) + 0.006 * qDps;
        const double phiR = MumtCtl::DegToRad(std::min(std::abs(phiDeg), 80.0));
        elevator -= 0.15 * (1.0 / std::cos(phiR) - 1.0);                 // 뱅크 시 양력 손실 보상
        elevator *= ElvSign;

        // ── 속도: TAS → throttle (speedCmd<=0 이면 개루프 폴백) ──
        const double throttle = (speedCmd > 0.0)
            ? Speed.Update(speedCmd - tasMps, tasMps, dt)
            : throttleNorm;

        // 과속 시 스피드브레이크 — 아이들 감속(F-16 클린 ≈ -1.5 m/s²)만으로는 리더
        // 급감속(-5 m/s²)을 못 따라가 슬롯을 지나침(V1 실측 -80m). +8 m/s 초과부터 비례 전개.
        const double overspeed = (speedCmd > 0.0) ? (tasMps - speedCmd) : 0.0;
        const double speedbrake = std::min(1.0, std::max(0.0, (overspeed - 5.0) / 15.0));

        FInnerLoopOutput o;
        o.Aileron    = (float)std::min(1.0, std::max(-1.0, aileron));
        o.Elevator   = (float)std::min(1.0, std::max(-1.0, elevator));
        o.Rudder     = 0.f;                                              // bank-to-turn: 러더 미사용
        o.Throttle   = (float)std::min(1.0, std::max(0.0, throttle));
        o.SpeedBrake = (float)speedbrake;
        return o;
    }
};
