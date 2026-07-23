// F16CommandController — 비행 Reference(roll/pitch/airspeed) → JSBSim FCS 명령(cmd-norm).
// 새 3계층 무인기 제어 구조의 최하위 계층:
//   FormationGuidance(슬롯·오차) → FixedWingGuidance(Reference) → [F16CommandController]
//
// 출력은 JSBSim FCS "명령" 수준만 쓴다:
//   fcs/aileron-cmd-norm / fcs/elevator-cmd-norm / fcs/rudder-cmd-norm
//   fcs/throttle-cmd-norm / fcs/speedbrake-cmd-norm
// 조종면 '위치'(fcs/*-pos-rad)는 f16.xml <flight_control>이 계산한다 — 외부에서 쓰지 않는다.
// f16.xml FCS에 내장된 롤레이트 루프·G/받음각 리미터·요-SAS를 여기서 중복 구현하지 않는다.
//
// 이 제어기는 PX4 Attitude/Rate Controller가 아니다 — Stick-equivalent 외곽 자세 루프
// (자세 오차 → cmd-norm)이며, 레이트 루프는 f16.xml FCS 안에 있다.
//
// 조종면 PID 게인은 구 InnerLoopAutopilot(JSBSim F-16 3기 폐루프 검증)에서 계승.
// heading→roll 외루프는 FixedWingGuidance로 분리됐다.
//
// 의존성 없음(std 수학만) — UE 빌드와 JSBSim 단독 검증 하네스 양쪽에서 컴파일된다.
#pragma once

#include <cmath>
#include <algorithm>

namespace MumtCtl
{
    inline double DegToRad(double d) { return d * 3.14159265358979323846 / 180.0; }
    inline double RadToDeg(double r) { return r * 180.0 / 3.14159265358979323846; }
    // 최단 회전각 [-180,180]
    inline double DeltaHeading(double target, double current) { return std::remainder(target - current, 360.0); }
}

// pid.py 포팅: P + 조건부-anti-windup I + (측정값 미분 + 1차 필터) D.
struct FInnerPID
{
    double Kp = 0, Ki = 0, Kd = 0, OutMin = -1, OutMax = 1, Tau = 0.05;
    double MaxDtS = 0.25;   // 호출측과 독립적인 적분/미분 점프 최종 방어
    double Integral = 0, PrevMeas = 0, PrevDeriv = 0;
    double LastOutput = 0;
    bool   bHasPrev = false;

    void Configure(double kp, double ki, double kd, double lo, double hi, double tau = 0.05)
    {
        Kp = kp; Ki = ki; Kd = kd; OutMin = lo; OutMax = hi; Tau = tau;
    }

    double Update(double error, double measForDeriv, double dt)
    {
        // 유효성 [§2]: 잘못된 dt/NaN/Inf는 적분·미분 상태를 갱신하지 않고
        // 마지막 정상 출력을 유지한다 (dt 상한은 호출측 정책 — 여기선 발산 방지만).
        if (!std::isfinite(error) || !std::isfinite(measForDeriv) ||
            !std::isfinite(dt) || dt <= 0.0 || dt > MaxDtS)
            return LastOutput;

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
        LastOutput = out;
        return out;
    }

    void Reset() { Integral = 0; PrevMeas = 0; PrevDeriv = 0; LastOutput = 0; bHasPrev = false; }
};

// JSBSim FCS 입력 명령 — 이 구조체만이 무인기→JSBSim의 유일한 명령 인터페이스다.
struct FF16ControlCommand
{
    float AileronCmdNorm    = 0.f;   // → Commands.Aileron    (fcs/aileron-cmd-norm)  [-1,1]
    float ElevatorCmdNorm   = 0.f;   // → Commands.Elevator   (fcs/elevator-cmd-norm) [-1,1] (F16: 음수=기수 up)
    float RudderCmdNorm     = 0.f;   // → Commands.Rudder     (fcs/rudder-cmd-norm)   [-1,1]
    float ThrottleCmdNorm   = 0.f;   // → EngineCommands[0]   (fcs/throttle-cmd-norm) [0,1]
    float SpeedbrakeCmdNorm = 0.f;   // → Commands.SpeedBrake (fcs/speedbrake-cmd-norm) [0,1]
};

// 자세 감쇠 방식 [§5] — 두 방식을 동시에 기본 활성화하지 않는다.
enum class EDampingMode
{
    DerivOnMeas,   // A(기본): PID의 측정값-미분 D항 사용, 별도 레이트 항 없음.
                   //   근거: UE 인엔진 경로는 EulerRates가 rad/s로 전달되던 기간(수정 전)
                   //   레이트 항이 사실상 0이었는데도 FormationTest 전 게이트 PASS —
                   //   실검증 이력과 동일 동작이 A다. (2026-07-11 A/B 하네스 비교로 확정)
    BodyRate,      // B(옵션): PID Kd=0, 명시적 레이트 감쇠 사용. 주의 — UE 경로가 공급하는
                   //   각속도는 body p,q,r가 아니라 오일러각 변화율(φ̇,θ̇,ψ̇)이다 (아래 Step 주석).
};

// Reference 추종 제어기. 상태 영속(per-UAV 인스턴스) 필수 — PID 적분기 보유.
struct FF16CommandController
{
    // 조종면 부호 미세조정 (PIE 튜닝, 기본 1). elevator: F16 음수=기수 up.
    double AilSign = 1.0, ElvSign = 1.0;

    // 요 채널: bank-to-turn 운용 + f16.xml FCS 내장 요-SAS(사이드슬립/요레이트 댐핑)가
    // 이미 루프를 닫고 있어 외부 중복 댐핑은 금지 — 기본 0. (비례 훅만 남겨둠;
    // FCS SAS를 끄는 실험에서만 사용. 러더는 단순 ψ̇ 비례 감쇠이며 β 피드백은 없다.)
    double YawDamperGain = 0.0;

    // ── 자세 감쇠 [§5] ──
    EDampingMode DampingMode = EDampingMode::DerivOnMeas;
    double RollRateDampGain  = 0.004;   // 방식 B에서만 사용 (per deg/s)
    double PitchRateDampGain = 0.006;   // 방식 B에서만 사용 (per deg/s)

    // ── dt/입력 유효성 [§2] — 제어주기 60Hz(1/60s 고정 전달) 기준 ──
    double NominalDtS = 1.0 / 60.0;   // 명목 주기 (invalid 지속시간 적산용)
    double MinDtS     = 1.0 / 1000.0; // 이보다 짧으면 미분 폭주 위험 → invalid 처리
    double MaxDtS     = 0.25;         // 15프레임 결손 초과 → invalid 처리
    double InvalidInputTimeoutS = 1.0; // invalid 지속 → Failsafe 전환

    // Failsafe [§2-6]: 상위 계층(UDP 수신부)의 소실 홀드(<5s 마지막 명령, >5s 수평유지)가
    // 1차 정책이고, 이 명령은 '컨트롤러 입력 자체가 오염된' 최후 단계용이다.
    // 기본값 근거: 중립 조종면(f16.xml FCS가 SAS로 대체로 유지) + 순항 트림 스로틀.
    // Init()에서 ThrottleCmdNorm이 ThrottleTrimNorm으로 동기화된다(직접 지정 시 그 값 유지).
    FF16ControlCommand FailsafeCmd;
    bool bFailsafeCmdCustom = false;   // true면 Init()이 FailsafeCmd를 건드리지 않음

    // ── Throttle Trim + Feedforward [§4] ──
    // 기본 0.26 근거: V1 하네스 실측 정상상태 스로틀 — 220 m/s 직선 0.289, 150 m/s 0.225
    // (formation_run.csv, JSBSim f16 클린 형상). 비행 조건별로 조정 가능한 설정값.
    double ThrottleTrimNorm = 0.26;
    // 속도 PI 보정량 한계(±). 기본 1.0 = 실효 [−ff, 1−ff] → 스로틀 전 구간 [0,1] 권한.
    // ※ 0.5로 줄이면 최대 스로틀이 ff+0.5(≈0.76)로 잘려 이륙 가속·Vmax 추격이 불가 —
    //   인엔진 지상이륙 회귀에서 리더 상승 지연·팔로워 11.6km 낙오로 실측(2026-07-11).
    //   트림 FF의 이점(오차 0에서 트림 유지, 범프리스 시작)은 1.0에서도 동일하다.
    double MaxThrottleCorr  = 1.0;
    // (참고) 스로틀 명령 슬루는 시험 후 채택하지 않음 — 대칭 슬루는 감속 컷 지연으로
    // D창 120m 발산, 상승 전용 슬루도 회복 지연으로 D 17.7→19.9m 악화 실측(2026-07-11).

    // ── Speedbrake [§6] — 과속 시 감속 보조 (아이들 감속 ≈ -1.5 m/s²만으론
    //    리더 급감속 -5 m/s²를 못 따라가 슬롯 지나침, V1 실측 -80m) ──
    double SpeedbrakeStartOverspeedMps = 5.0;   // 이 초과부터 전개 시작 (기존 코드 동작 유지)
    double SpeedbrakeFullOverspeedMps  = 20.0;  // 이 이상 완전 전개

    FInnerPID Roll, Pitch, Speed;
    bool bInit = false;

    // 상태
    FF16ControlCommand LastCmd;        // 일시적 invalid 입력 시 유지용 [§2-3]
    bool   bHasLastCmd = false;
    double InvalidInputTimer = 0.0;

    // 진단 [§13]
    double LastThrottleFF   = 0.0;     // 이번 스텝의 feedforward(트림) 성분
    double LastThrottleCorr = 0.0;     // 이번 스텝의 속도 PI 보정 성분
    bool   bAilSaturated = false, bElvSaturated = false;

    void Init()
    {
        // 게인: inner_loop.py 기본값 (JSBSim F-16 검증) — 변경 금지 수치.
        // DampingMode에 따라 D항/레이트항을 상호배타로 구성한다 [§5].
        const bool bA = (DampingMode == EDampingMode::DerivOnMeas);
        Roll.Configure (0.04, 0.0,   bA ? 0.012 : 0.0, -1.0, 1.0);
        Pitch.Configure(0.06, 0.008, bA ? 0.03  : 0.0, -1.0, 1.0);
        // Speed PI는 '전체 스로틀'이 아니라 '트림 대비 보정량'을 출력 [§4].
        // 출력 한계는 Step에서 실효 범위(0..1 - ff)로 매 스텝 갱신 → anti-windup 정확.
        Speed.Configure(0.08, 0.020, 0.0, -MaxThrottleCorr, MaxThrottleCorr);
        if (!bFailsafeCmdCustom)
            FailsafeCmd.ThrottleCmdNorm = (float)ThrottleTrimNorm;
        bInit = true;
    }

    // 재engage용 [§9]: PID 상태를 비우고 트림 FF에서 범프리스로 시작한다
    // (Integral=0 → throttle = trim + Kp·err ≈ trim; roll/pitch는 슬루가 FWG에 있음).
    // LastCmd도 무효화 — 낡은 명령에서 슬루하지 않고 트림에서 새로 시작.
    void Reset()
    {
        Roll.Reset(); Pitch.Reset(); Speed.Reset();
        InvalidInputTimer = 0.0;
        bHasLastCmd = false;
    }

    // 각도 deg / 속도 m/s.
    // phiDotDps/thetaDotDps/psiDotDps: 오일러각 변화율 φ̇,θ̇,ψ̇ (deg/s).
    //   ※ body angular rate p,q,r가 아니다 — UE 경로는 플러그인 AircraftState.EulerRates
    //   (JSBSim FGAuxiliary::GetEulerRates, rad/s → 수신부에서 deg/s 변환)를 공급한다.
    //   방식 A(기본)에서는 미사용(요 훅 제외). 방식 B에서 감쇠 근사로 사용
    //   (소각 근사 φ̇≈p, θ̇≈q — 고뱅크에서 오차 있음을 감안).
    // throttleRefNorm: FixedWingGuidance의 ThrottleRefNorm. >=0이면 feedforward로 사용,
    //   <0이면 ThrottleTrimNorm 사용 [§4-2,3].
    // throttleFallback: airspeedRef<=0(속도루프 미사용) 시의 개루프 스로틀 [0,1].
    FF16ControlCommand Step(double rollRefDeg, double pitchRefDeg, double airspeedRefMps,
                            double throttleRefNorm,
                            double phiDeg, double thetaDeg,
                            double phiDotDps, double thetaDotDps, double psiDotDps,
                            double tasMps, double throttleFallback, double dt)
    {
        if (!bInit) Init();

        // ── 입력 유효성 [§2] ──
        const bool bFinite =
            std::isfinite(rollRefDeg) && std::isfinite(pitchRefDeg) &&
            std::isfinite(airspeedRefMps) && std::isfinite(throttleRefNorm) &&
            std::isfinite(phiDeg) && std::isfinite(thetaDeg) &&
            std::isfinite(phiDotDps) && std::isfinite(thetaDotDps) && std::isfinite(psiDotDps) &&
            std::isfinite(tasMps) && std::isfinite(throttleFallback);
        const bool bDtOk = std::isfinite(dt) && dt >= MinDtS && dt <= MaxDtS;
        if (!bFinite || !bDtOk)
        {
            // 한 프레임 결함 → 마지막 정상 명령 유지; 지속되면 Failsafe [§2-3,4].
            InvalidInputTimer += NominalDtS;
            if (InvalidInputTimer >= InvalidInputTimeoutS || !bHasLastCmd)
                return FailsafeCmd;
            return LastCmd;
        }
        InvalidInputTimer = 0.0;

        // ── Roll: (roll_ref − φ) → aileron ──
        double aileron = Roll.Update(rollRefDeg - phiDeg, phiDeg, dt);
        if (DampingMode == EDampingMode::BodyRate)
            aileron -= RollRateDampGain * phiDotDps;
        aileron *= AilSign;

        // ── Pitch: (pitch_ref − θ) → elevator (+뱅크 양력손실 보상) ──
        double elevator = -Pitch.Update(pitchRefDeg - thetaDeg, thetaDeg, dt);
        if (DampingMode == EDampingMode::BodyRate)
            elevator += PitchRateDampGain * thetaDotDps;
        const double phiR = MumtCtl::DegToRad(std::min(std::abs(phiDeg), 80.0));
        elevator -= 0.15 * (1.0 / std::cos(phiR) - 1.0);                 // 뱅크 시 양력 손실 보상
        elevator *= ElvSign;

        // ── Speed → Throttle = Feedforward(트림) + PI 보정 [§4] ──
        double throttle;
        if (airspeedRefMps > 0.0)
        {
            const double ff = (throttleRefNorm >= 0.0) ? throttleRefNorm : ThrottleTrimNorm;
            // PI 출력한계를 실효 범위로 갱신: 최종 스로틀 [0,1]과 ±MaxThrottleCorr의 교집합.
            // 한계가 실제 포화 지점과 일치해야 조건부 anti-windup이 정확히 동작한다 [§4-7].
            Speed.OutMin = std::max(-MaxThrottleCorr, 0.0 - ff);
            Speed.OutMax = std::min( MaxThrottleCorr, 1.0 - ff);
            const double corr = Speed.Update(airspeedRefMps - tasMps, tasMps, dt);
            throttle = ff + corr;
            LastThrottleFF = ff; LastThrottleCorr = corr;
        }
        else
        {
            throttle = throttleFallback;   // 개루프 폴백 (속도루프 미사용)
            LastThrottleFF = throttleFallback; LastThrottleCorr = 0.0;
        }

        // ── Speedbrake [§6]: Start 초과부터 비례 전개, Full 이상 완전 전개 ──
        const double overspeed = (airspeedRefMps > 0.0) ? (tasMps - airspeedRefMps) : 0.0;
        double speedbrake = 0.0;
        if (SpeedbrakeFullOverspeedMps > SpeedbrakeStartOverspeedMps)
            speedbrake = std::min(1.0, std::max(0.0,
                (overspeed - SpeedbrakeStartOverspeedMps)
                / (SpeedbrakeFullOverspeedMps - SpeedbrakeStartOverspeedMps)));
        else   // 설정 역전/동일 → 안전한 계단형으로 강등 [§6-3]
            speedbrake = (overspeed > SpeedbrakeStartOverspeedMps) ? 1.0 : 0.0;

        FF16ControlCommand o;
        const double ailClamped = std::min(1.0, std::max(-1.0, aileron));
        const double elvClamped = std::min(1.0, std::max(-1.0, elevator));
        bAilSaturated = (ailClamped != aileron);
        bElvSaturated = (elvClamped != elevator);
        o.AileronCmdNorm    = (float)ailClamped;
        o.ElevatorCmdNorm   = (float)elvClamped;
        o.RudderCmdNorm     = (float)std::min(1.0, std::max(-1.0, -YawDamperGain * psiDotDps / 180.0));
        o.ThrottleCmdNorm   = (float)std::min(1.0, std::max(0.0, throttle));
        o.SpeedbrakeCmdNorm = (float)speedbrake;
        LastCmd = o; bHasLastCmd = true;
        return o;
    }
};
