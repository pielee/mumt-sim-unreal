// 편대/추격 유도 (인엔진 60Hz) — BT FormationGuidance의 계산부를 UE로 이관.
// BT는 "누구를 어떻게"(모드·슬롯·표적)만 지정하고, 숫자 계산은 여기서 리더/표적
// pawn을 직독(지연 0)하며 수행한다. 출력(heading/alt/speed/roll_ff)은 그대로
// FInnerLoopAutopilot::Step에 들어간다.
//
// 의존성 없음(<cmath>,<algorithm>만): InnerLoopAutopilot.h와 같은 이유로,
// UE 밖 JSBSim 스탠드얼론 하네스에서 단독 폐루프 검증 가능해야 한다.
//
// 법칙 출처: JSBSim 3기 검증 설계(formation_guidance.py) + 영상검증 결함 수정:
//  [E] 리더 track = atan2(vE,vN) — 참 속도벡터(VelocityNEDfps) 기반, yaw 크랩 무관
//  [F] ω = track 미분 + 1차 LPF τ=0.25s (구 BT 10Hz 필터 τ=2.0s → 8배 빠른 감지)
//      v_slot = v_leader + ω×r — 선회 시 슬롯 이동 예측 (안쪽 감속/바깥 가속)
//  [A] 횡: 벡터필드 χc = χ_slot + χ∞·(2/π)·atan(k·e_cross) — 멀수록 큰 컷각(χ∞ 포화).
//      구 pure-pursuit lookahead(멀수록 작은 컷각→평행비행)의 교과서 대체
//      (Beard&McLain / rosplane path_follower 계열, BSD)
//  [C] 종: V = |v_slot| + clamp(Kp·e_along + Kd·ė, ±VCorrMax=150) — 구 45 굶김 해소
//  [D] roll_ff = atan(ω·V/g)·scale, scale=clamp(1−|e_cross|/2000, 0.25, 1)
//      — 구 400m 하드컷(선회 중 피드포워드 소실) 해소. 최종 ±RollFfLimit 클램프.

#pragma once

#include <cmath>
#include <algorithm>

struct FGuidanceCmd
{
    double HeadingDeg = 0.0;   // 나침반(0=북) [0,360)
    double AltM       = 0.0;   // UE-Z 고도(m)
    double SpeedMps   = 0.0;
    double RollFfDeg  = 0.0;
};

// ── 편대 슬롯 유도 ────────────────────────────────────────────────────────────
struct FFormationGuidance
{
    // 파라미터 (V1 하네스에서 튜닝·확정)
    double ChiInfDeg        = 70.0;    // 벡터필드 최대 컷각(deg)
    double KPath            = 0.004;   // [1/m] — 고속(200+m/s)용, TdCross 리드와 세트.
                                       // (0.008+리드無=±100m 진동, 0.0025=꼬리 정착 지연 — V1 실측)
    double TdCross          = 2.5;     // [s] cross-rate 리드 — 헤딩루프 지연(롤업~선회 ~2.5s) 선보상.
                                       // 없으면 220m/s에서 주기 40s·±100m 위빙 (V1 실측)
    double KpAlong          = 0.25;    // 전후오차 → 속도 보정 (0.30은 스로틀 지연과 겹쳐 ±110m 진동 — V1)
    double KdAlong          = 0.80;    // 참 속도벡터 기반 감쇠 (스로틀 응답 지연 선보상)
    double VCorrMax         = 150.0;   // 속도 보정 상한 [C]
    double RollFfFloor      = 0.25;    // roll_ff 게이팅 바닥값 [D]
    double RollFfCrossScale = 2000.0;  // [m]
    double RffTurnCal       = 1.2;     // f16 요-SAS(워시아웃 없음)가 정상선회 요레이트를 억제 →
                                       // 이론 atan(ωV/g)보다 실측 +9° 필요 (V1: 3°/s@220 = 58.8° vs 이론 49.6°)
    double RollFfLimitDeg   = 54.0;    // 팔로워 뱅크캡(62) 이하 — 피드백 여유 8°
    double OmegaLpfTau      = 0.25;    // 리더 선회율 필터 [F]
    double OmegaMaxDps      = 15.0;    // track 미분 스파이크 가드
    double OmegaDeadDps     = 0.3;     // 직선비행 track 노이즈(±0.1°/s)가 rff ±3°로 증폭돼
                                       // ±15m 위빙 유발(V1) → 소프트 데드밴드로 차단
    double ClimbLeadS       = 8.0;     // 수직 선행보상(초) — 내루프 수직 시정수 TanRefM/V(≈7~9s)와
                                       // 정합해야 지속상승 정상오차가 소거됨 (2.0은 +88m 지연 — V1 실측)
    double MinTrackSpeedMps = 50.0;    // 리더 저속 → track/ω 홀드 (20은 지상활주 조향 지터가
                                       // ω±4°/s로 증폭돼 Rff 슬램 — PIE 2026-07-09 실측)

    // ── 원거리 REJOIN (F-15/T-33의 MPC 궤적계획에 해당하는 고전 유도 대체) ──
    // 벡터필드는 근거리 슬롯유지 법칙: km급 변위에선 컷각 포화(±70°)+고속 선회반경(km급)
    // 조합이 ±4km S-위빙을 만든다(2026-07-10 PIE 실측). 멀면 요격점(슬롯의 t_lead초 후
    // 예측 위치)을 조준하는 부드러운 추격 곡선으로 접근, 가까워지면 벡터필드 복귀.
    double RejoinEnterM   = 800.0;     // 슬롯거리 초과 → REJOIN 진입
    double RejoinExitM    = 400.0;     // 미만 → 벡터필드 복귀 (히스테리시스)
    double RejoinLeadMaxS = 10.0;      // 요격 리드타임 상한(초)
    double RejoinTauS     = 8.0;       // 접근 감속 테이퍼: 폐속도 ≤ 거리/τ (time-to-go 제동).
                                       // 없으면 60m/s 과속 도착 → 제동거리 ~900m → +824m 바운스
                                       // 로 ~35s 낭비 (인엔진 시험 2026-07-10 실측)
    double RejoinMinClose = 15.0;      // 폐속도 하한 — 지수 꼬리 방지

    // 상태 (per-UAV 유지 — FUavControl에 저장)
    double PrevTrackRad = 1e9;         // 1e9 = 미초기화 센티널
    double OmegaFilt    = 0.0;         // rad/s (+우선회)
    bool   bRejoin      = false;       // 원거리 재합류 모드 (히스테리시스 상태)

    // 진단 (마지막 Step의 슬롯 오차 — UE 로그/하네스 게이트용, 제어에 미사용)
    double LastEAlongM = 0.0;
    double LastECrossM = 0.0;

    void Reset()
    {
        PrevTrackRad = 1e9;
        OmegaFilt    = 0.0;
        bRejoin      = false;
    }

    static double WrapPi(double A)
    {
        return std::remainder(A, 2.0 * 3.14159265358979323846);
    }

    // 전부 NED(m, m/s), 고도/상승률은 +위. dt는 호출 주기(초).
    FGuidanceCmd Step(
        double LdrN, double LdrE, double LdrAltM,
        double LdrVn, double LdrVe, double LdrClimbMps,
        double OwnN, double OwnE,
        double OwnVn, double OwnVe,
        double FrontM, double RightM, double UpM,
        double MinSpd, double MaxSpd, double MinAltM,
        double Dt)
    {
        constexpr double kPi = 3.14159265358979323846;   // UE의 PI 매크로(UE_PI deprecation)와 충돌 방지
        constexpr double G  = 9.80665;

        // ── 리더 track + 선회율 ω ──
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

        // ── 슬롯 위치·속도 (트랙 프레임: fwd=(c,s), right=(−s,c) — NED) ──
        const double C = std::cos(TrackRad), S = std::sin(TrackRad);
        const double Rn = FrontM * C - RightM * S;      // r = slot − leader
        const double Re = FrontM * S + RightM * C;
        const double SlotN = LdrN + Rn, SlotE = LdrE + Re;
        const double SlotAlt = LdrAltM + UpM;
        const double VsN = LdrVn - W * Re;              // v_slot = v_leader + ω×r
        const double VsE = LdrVe + W * Rn;
        const double VsMag = std::sqrt(VsN * VsN + VsE * VsE);

        // ── 오차 분해 (리더 트랙 프레임) ──
        const double En = SlotN - OwnN, Ee = SlotE - OwnE;
        const double EAlong =  En * C + Ee * S;         // +: 슬롯이 앞 → 가속
        const double ECross = -En * S + Ee * C;         // +: 슬롯이 우측 → 우로 컷
        LastEAlongM = EAlong;
        LastECrossM = ECross;

        // ── REJOIN 판정 (히스테리시스) ──
        const double SlotDist = std::sqrt(En * En + Ee * Ee);
        if (!bRejoin && SlotDist > RejoinEnterM)      bRejoin = true;
        else if (bRejoin && SlotDist < RejoinExitM)   bRejoin = false;

        // ── 횡 ──
        double ChiCmd;
        if (bRejoin)
        {
            // 원거리: 요격점 조준 (tail-chase 추격 곡선 — 컷각 포화·S-위빙 없음)
            const double OwnSpd = std::max(std::sqrt(OwnVn * OwnVn + OwnVe * OwnVe), 100.0);
            const double TLead  = std::min(RejoinLeadMaxS, SlotDist / OwnSpd);
            const double AimN   = SlotN + VsN * TLead;
            const double AimE   = SlotE + VsE * TLead;
            ChiCmd = std::atan2(AimE - OwnE, AimN - OwnN);
        }
        else
        {
            // 근거리: 벡터필드 [A] — 슬롯 이동방향 기준선에 수렴 (+cross-rate 리드)
            const double ChiSlot = (VsMag > 1.0) ? std::atan2(VsE, VsN) : TrackRad;
            const double ChiInf  = ChiInfDeg * kPi / 180.0;
            const double ECrossDot = -(VsN - OwnVn) * S + (VsE - OwnVe) * C;
            ChiCmd = ChiSlot
                + ChiInf * (2.0 / kPi) * std::atan(KPath * (ECross + TdCross * ECrossDot));
        }

        // ── 종: 속도 [C] (ė는 참 속도벡터 차 [E]) ──
        const double DeAlong = (VsN - OwnVn) * C + (VsE - OwnVe) * S;
        double VCorr = KpAlong * EAlong + KdAlong * DeAlong;
        VCorr = std::max(-VCorrMax, std::min(VCorrMax, VCorr));
        double SpeedCmd = std::max(MinSpd, std::min(MaxSpd, VsMag + VCorr));
        if (bRejoin)
        {
            // 접근 감속 테이퍼: 폐속도가 거리/τ를 넘지 않게 — 과속 도착 바운스 방지
            const double MaxClose = std::max(RejoinMinClose, SlotDist / RejoinTauS);
            SpeedCmd = std::max(MinSpd, std::min(SpeedCmd, VsMag + MaxClose));
        }

        // ── 수직: 슬롯 고도 + 상승 선행보상, 하한 가드 ──
        double AltCmd = SlotAlt + LdrClimbMps * ClimbLeadS;
        if (MinAltM > 0.0)
            AltCmd = std::max(AltCmd, MinAltM);

        // ── roll_ff [D] (RffTurnCal: 요-SAS 선회효율 손실 보정) ──
        // REJOIN 중엔 0: 추격 곡선의 필요 뱅크는 리더 ω와 무관 — 잔여 ff는 헤딩 바이어스만 만든다.
        double RollFf = 0.0;
        if (!bRejoin)
        {
            const double PhiFf = std::atan2(W * std::max(SpeedCmd, 50.0), G) * 180.0 / kPi * RffTurnCal;
            double Scale = 1.0 - std::abs(ECross) / RollFfCrossScale;
            Scale = std::max(RollFfFloor, std::min(1.0, Scale));
            RollFf = PhiFf * Scale;
            RollFf = std::max(-RollFfLimitDeg, std::min(RollFfLimitDeg, RollFf));
        }

        FGuidanceCmd Cmd;
        double H = ChiCmd * 180.0 / kPi;
        H = std::fmod(H, 360.0);
        if (H < 0.0) H += 360.0;
        Cmd.HeadingDeg = H;
        Cmd.AltM       = AltCmd;
        Cmd.SpeedMps   = SpeedCmd;
        Cmd.RollFfDeg  = RollFf;
        return Cmd;
    }
};

// ── 추격(공격) 유도 — pure pursuit + standoff 속도법칙 ─────────────────────────
// InterceptTarget의 tail-chase 법칙 이관: 표적 방위로 기수, 표적 고도, 속도는
// 표적속도 + Kp×(거리−standoff). WEZ 방아쇠 판단은 BT에 남는다(10Hz로 충분).
struct FPursuitGuidance
{
    double StandoffM = 200.0;
    double KpSpeed   = 0.15;

    FGuidanceCmd Step(
        double TgtN, double TgtE, double TgtAltM,
        double TgtVn, double TgtVe,
        double OwnN, double OwnE,
        double MinSpd, double MaxSpd, double MinAltM) const
    {
        constexpr double kPi = 3.14159265358979323846;   // UE의 PI 매크로(UE_PI deprecation)와 충돌 방지
        const double Dn = TgtN - OwnN, De = TgtE - OwnE;
        const double Dist = std::sqrt(Dn * Dn + De * De);

        double H = std::atan2(De, Dn) * 180.0 / kPi;
        if (H < 0.0) H += 360.0;

        const double TgtSpd = std::sqrt(TgtVn * TgtVn + TgtVe * TgtVe);
        const double Spd = std::max(MinSpd, std::min(MaxSpd, TgtSpd + KpSpeed * (Dist - StandoffM)));

        double Alt = TgtAltM;
        if (MinAltM > 0.0)
            Alt = std::max(Alt, MinAltM);

        FGuidanceCmd Cmd;
        Cmd.HeadingDeg = H;
        Cmd.AltM       = Alt;
        Cmd.SpeedMps   = Spd;
        Cmd.RollFfDeg  = 0.0;   // 추격은 피드백만 (내루프 슬루가 급캡처 담당)
        return Cmd;
    }
};
