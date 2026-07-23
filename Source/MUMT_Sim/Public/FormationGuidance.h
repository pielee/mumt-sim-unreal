// FormationGuidance — 유인기(리더) 기준 무인기 목표 편대 슬롯 계산 계층.
// 새 3계층 무인기 제어 구조의 최상위 계층:
//   [FormationGuidance] → FixedWingGuidance → F16CommandController → JSBSim FCS
//
// 이 계층은 '기하'만 담당한다:
//   1. 리더 track/선회율 ω 추정 (참 속도벡터 기반 — yaw 크랩 무관)
//   2. 리더 track 프레임 슬롯 오프셋 → 지역 NED 변환 (선회 시 슬롯이 같이 회전)
//   3. 슬롯 속도 v_slot = v_leader + ω×r, 구심가속도 a_slot = ω×(ω×r)
//   4. Along-track / Cross-track / Vertical / 상대속도 오차 분해
//   5. 슬롯 캡처·유지 판정 (디바운스), 리더와의 최소 안전거리 검사
//
// 비행 reference(course/roll/alt/speed)는 FixedWingGuidance가, JSBSim 명령은
// F16CommandController가 만든다. 이 파일은 그 둘을 모른다.
//
// 의존성 없음(<cmath>,<algorithm>만) — UE 밖 JSBSim 스탠드얼론 하네스에서 단독 검증 가능.
//
// 법칙 출처(검증 이력 보존): 리더 track = atan2(vE,vN) [결함 E 해소],
// ω = track 미분 + 1차 LPF τ=0.25s + 데드밴드 0.3°/s [결함 F 해소, V1 실측 튜닝],
// 저속(<50m/s) track/ω 홀드 (지상활주 조향 지터 → Rff 슬램 방지, PIE 2026-07-09 실측).
#pragma once

#include <cmath>
#include <algorithm>

// FormationGuidance → FixedWingGuidance 인터페이스.
// 전부 지역 NED (m, m/s, m/s²), 고도/상승률은 +위, 각도는 rad.
struct FFormationTarget
{
    // 목표 슬롯
    double SlotN = 0.0, SlotE = 0.0, SlotAltM = 0.0;   // Target Slot Position
    double SlotVn = 0.0, SlotVe = 0.0;                 // Target Slot Velocity (ω×r 포함)
    double SlotClimbMps = 0.0;                         //   수직 성분 (= 리더 상승률)
    // 슬롯 가속도의 '구심 성분만' ω×(ω×r) [§8] — 완전한 슬롯 가속도가 아니다:
    // 리더 선형가속도·각가속도 α×r 미포함 (신뢰성 있게 얻을 수 없어 추가하지 않음).
    // 현재 제어에는 미사용 — 진단·확장용 (TECS·곡률FF 승급 대비).
    double SlotCentripetalAn = 0.0, SlotCentripetalAe = 0.0;
    // 리더 기준 정보 (FixedWingGuidance의 벡터필드·roll_ff가 사용)
    double TrackRad   = 0.0;                           // 리더 track (atan2(vE,vN))
    double OmegaRadps = 0.0;                           // 필터+데드밴드 적용 선회율 (+우선회)

    // 오차 (리더 track 프레임, slot − own)
    double EAlongM = 0.0;                              // Along-Track Error (+: 슬롯이 앞)
    double ECrossM = 0.0;                              // Cross-Track Error (+: 슬롯이 우측)
    double EVertM  = 0.0;                              // Vertical Error    (+: 슬롯이 위)
    double DVAlongMps = 0.0, DVCrossMps = 0.0;         // Relative Velocity Error (v_slot − v_own)
    double ClosingSpeedMps = 0.0;                      // 슬롯 접근 폐속도 (+: 접근 중)
    double SlotDistM   = 0.0;                          // 수평 슬롯 거리
    double SlotDist3M  = 0.0;                          // 3D 슬롯 거리 (캡처/유지 판정 기준)
    double SeparationM = 0.0;                          // 리더와의 3D 거리 (안전거리 검사)
    double SeparationRateMps  = 0.0;                   // d(Separation)/dt LPF (−: 접근 중)
                                                       //   — 감속 보조 한계 경고 판정용 [§7]
    double SeparationDeficitM = 0.0;                   // max(0, minimum_separation − Separation)
                                                       //   — 최소 안전거리 '감속 보조' 입력
                                                       //   (완전한 충돌 회피 아님 [§7])

    // 판정
    bool bLeaderValid      = false;                    // 리더 track 신뢰 가능 (비행 중)
    bool bCaptured         = false;                    // capture_tolerance 이내 CaptureHoldS 지속 (래치)
    bool bMaintained       = false;                    // 캡처 상태 && maintain_tolerance 이내
    bool bSeparationBreach = false;                    // minimum_separation 미만 → 감속 요구
};

struct FFormationGuidance
{
    // ── 리더 상태 추정 (V1 하네스/PIE 검증 수치 — 변경 금지) ──
    double OmegaLpfTau      = 0.25;    // 리더 선회율 필터 [결함 F]
    double OmegaMaxDps      = 15.0;    // track 미분 스파이크 가드
    double OmegaDeadDps     = 0.3;     // 직선비행 track 노이즈(±0.1°/s) → Rff ±3° 증폭 차단
    double MinTrackSpeedMps = 50.0;    // 리더 저속 → track/ω 홀드 (지상활주 지터 가드)

    // ── 캡처/유지/안전거리 판정 (BT가 ROS 명령으로 지정) ──
    double CaptureTolM     = 30.0;     // 슬롯 3D 오차 < 이 값 지속 → 캡처
    double MaintainTolM    = 50.0;     // 캡처 후 3D 오차 < 이 값 → 유지 중
    double MinSeparationM  = 0.0;      // 리더와의 3D 최소 안전거리 (0 = 검사 없음)
    double CaptureHoldS    = 2.0;      // 캡처 확정 지속시간
    double CaptureLossS    = 2.0;      // 2×CaptureTol 초과 지속 → 캡처 해제

    // ── dt 위생 [§2] — 제어주기 60Hz(고정 1/60 전달) 기준 ──
    double NominalDtS = 1.0 / 60.0;    // 명목 주기(진단/설정 호환; invalid dt는 홀드)
    double MaxDtS     = 0.25;          // dt 상한 (필터·타이머 점프 방지)
    double SepRateLpfTau = 0.5;        // 분리거리 변화율 LPF (경고 판정용 [§7])

    // ── 상태 (per-UAV 유지 — FUavControl에 저장) ──
    double PrevTrackRad = 1e9;         // 1e9 = 미초기화 센티널
    double OmegaFilt    = 0.0;         // rad/s (+우선회)
    double CaptureTimer = 0.0;
    double BreachTimer  = 0.0;
    double PrevSeparationM = -1.0;     // <0 = 미초기화
    double SepRateFilt  = 0.0;
    bool   bCapturedState = false;
    FFormationTarget LastTarget;         // invalid 프레임 홀드용 (내부 상태 오염 방지)
    bool   bHasLastTarget = false;

    // 진단 (마지막 Step 오차 — UE 로그/하네스 게이트용, 제어에 미사용)
    double LastEAlongM = 0.0;
    double LastECrossM = 0.0;

    void Reset()
    {
        PrevTrackRad    = 1e9;
        OmegaFilt       = 0.0;
        CaptureTimer    = 0.0;
        BreachTimer     = 0.0;
        PrevSeparationM = -1.0;
        SepRateFilt     = 0.0;
        bCapturedState  = false;
        bHasLastTarget  = false;
    }

    static double WrapPi(double A)
    {
        return std::remainder(A, 2.0 * 3.14159265358979323846);
    }

    bool IsValidDt(double Dt) const
    {
        return std::isfinite(Dt) && Dt > 0.0 && Dt <= MaxDtS;
    }

    // 입력 전부 지역 NED(m, m/s), 고도/상승률 +위, dt는 호출 주기(초).
    // 리더 속도는 참벡터(VelocityNEDfps 변환) — yaw 근사 금지 [결함 E].
    FFormationTarget Step(
        double LdrN, double LdrE, double LdrAltM,
        double LdrVn, double LdrVe, double LdrClimbMps,
        double OwnN, double OwnE, double OwnAltM,
        double OwnVn, double OwnVe,
        double FrontM, double RightM, double UpM,
        double Dt)
    {
        constexpr double kPi = 3.14159265358979323846;   // UE PI 매크로 충돌 방지
        // 비정상 프레임은 추정기/래치/미분 상태를 전혀 갱신하지 않는다. 마지막 정상
        // target을 홀드하면 최하위 Controller의 invalid-timeout 정책과도 일관된다.
        const bool bFinite =
            std::isfinite(LdrN) && std::isfinite(LdrE) && std::isfinite(LdrAltM) &&
            std::isfinite(LdrVn) && std::isfinite(LdrVe) && std::isfinite(LdrClimbMps) &&
            std::isfinite(OwnN) && std::isfinite(OwnE) && std::isfinite(OwnAltM) &&
            std::isfinite(OwnVn) && std::isfinite(OwnVe) &&
            std::isfinite(FrontM) && std::isfinite(RightM) && std::isfinite(UpM);
        if (!bFinite || !IsValidDt(Dt))
            return bHasLastTarget ? LastTarget : FFormationTarget{};

        FFormationTarget T;

        // ── 1. 리더 track + 선회율 ω ──
        const double LdrGs = std::sqrt(LdrVn * LdrVn + LdrVe * LdrVe);
        const bool bTrackValid = LdrGs >= MinTrackSpeedMps;
        double TrackRad;
        if (bTrackValid)
            TrackRad = std::atan2(LdrVe, LdrVn);
        else
            TrackRad = (PrevTrackRad < 1e8) ? PrevTrackRad : 0.0;

        if (bTrackValid && PrevTrackRad < 1e8 && Dt > 1e-6)
        {
            double Raw = WrapPi(TrackRad - PrevTrackRad) / Dt;
            const double Lim = OmegaMaxDps * kPi / 180.0;
            Raw = std::max(-Lim, std::min(Lim, Raw));
            const double Alpha = Dt / (OmegaLpfTau + Dt);
            OmegaFilt += Alpha * (Raw - OmegaFilt);
        }
        if (bTrackValid)
            PrevTrackRad = TrackRad;
        // 하드 게이트: |ω|<dead → 0, 이상이면 원값 그대로. (소프트 shift형은 선회 전
        // 구간에 -dead 바이어스를 넣어 3°/s 슬롯오차를 7→19m로 키움 — V1 실측)
        const double Dead = OmegaDeadDps * kPi / 180.0;
        const double W = (std::abs(OmegaFilt) >= Dead) ? OmegaFilt : 0.0;

        // ── 2~3. 슬롯 위치·속도·가속도 (트랙 프레임: fwd=(c,s), right=(−s,c) — NED) ──
        const double C = std::cos(TrackRad), S = std::sin(TrackRad);
        const double Rn = FrontM * C - RightM * S;      // r = slot − leader
        const double Re = FrontM * S + RightM * C;
        T.SlotN    = LdrN + Rn;
        T.SlotE    = LdrE + Re;
        T.SlotAltM = LdrAltM + UpM;
        T.SlotVn   = LdrVn - W * Re;                    // v_slot = v_leader + ω×r
        T.SlotVe   = LdrVe + W * Rn;
        T.SlotClimbMps = LdrClimbMps;
        T.SlotCentripetalAn = -W * W * Rn;              // 구심 성분만 (완전한 가속도 아님 [§8])
        T.SlotCentripetalAe = -W * W * Re;
        T.TrackRad   = TrackRad;
        T.OmegaRadps = W;

        // ── 4. 오차 분해 (리더 트랙 프레임) ──
        const double En = T.SlotN - OwnN, Ee = T.SlotE - OwnE;
        T.EAlongM =  En * C + Ee * S;                   // +: 슬롯이 앞 → 가속
        T.ECrossM = -En * S + Ee * C;                   // +: 슬롯이 우측 → 우로 컷
        T.EVertM  = T.SlotAltM - OwnAltM;
        LastEAlongM = T.EAlongM;
        LastECrossM = T.ECrossM;

        const double DVn = T.SlotVn - OwnVn, DVe = T.SlotVe - OwnVe;
        T.DVAlongMps =  DVn * C + DVe * S;
        T.DVCrossMps = -DVn * S + DVe * C;

        T.SlotDistM  = std::sqrt(En * En + Ee * Ee);
        T.SlotDist3M = std::sqrt(T.SlotDistM * T.SlotDistM + T.EVertM * T.EVertM);
        // 폐속도 (+: own이 슬롯으로 접근 중) = −d|e|/dt = ê·(v_own − v_slot)
        if (T.SlotDistM > 1.0)
            T.ClosingSpeedMps = (En * (OwnVn - T.SlotVn) + Ee * (OwnVe - T.SlotVe)) / T.SlotDistM;

        // ── 5. 안전거리 (리더와의 3D 거리) — '감속 보조' 판정 입력 [§7] ──
        const double LdN = LdrN - OwnN, LdE = LdrE - OwnE, LdU = LdrAltM - OwnAltM;
        T.SeparationM = std::sqrt(LdN * LdN + LdE * LdE + LdU * LdU);
        if (PrevSeparationM >= 0.0 && Dt > 1e-6)
        {
            const double Raw = (T.SeparationM - PrevSeparationM) / Dt;
            const double Alpha = Dt / (SepRateLpfTau + Dt);
            SepRateFilt += Alpha * (Raw - SepRateFilt);
        }
        PrevSeparationM = T.SeparationM;
        T.SeparationRateMps   = SepRateFilt;
        T.bSeparationBreach   = (MinSeparationM > 0.0) && (T.SeparationM < MinSeparationM);
        T.SeparationDeficitM  = T.bSeparationBreach ? (MinSeparationM - T.SeparationM) : 0.0;

        // ── 캡처/유지 판정 (디바운스 래치) ──
        if (!bCapturedState)
        {
            if (T.SlotDist3M < CaptureTolM) CaptureTimer += Dt;
            else                            CaptureTimer = 0.0;
            if (CaptureTimer >= CaptureHoldS) { bCapturedState = true; BreachTimer = 0.0; }
        }
        else
        {
            if (T.SlotDist3M > 2.0 * CaptureTolM) BreachTimer += Dt;
            else                                  BreachTimer = 0.0;
            if (BreachTimer >= CaptureLossS) { bCapturedState = false; CaptureTimer = 0.0; }
        }
        T.bLeaderValid = bTrackValid;
        T.bCaptured    = bCapturedState;
        T.bMaintained  = bCapturedState && (T.SlotDist3M <= MaintainTolM);

        LastTarget = T;
        bHasLastTarget = true;
        return T;
    }
};
