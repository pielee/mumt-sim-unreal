// FixedWingGuidance — 목표(편대 슬롯 target 또는 direct course/alt/speed)를 F-16이
// 추종 가능한 비행 Reference로 변환하는 중간 계층.
//   FormationGuidance(슬롯·오차) → [FixedWingGuidance] → F16CommandController → JSBSim FCS
//
// 횡방향: 벡터필드 (Beard&McLain 교과서 계열의 자체 구현 — PX4 NPFG가 아님) + cross-rate
//         리드, 원거리는 REJOIN 요격 추격 곡선 (히스테리시스 전환) — 검증 법칙 수치 그대로.
//         course 오차 → roll reference (비례 + 뱅크 피드포워드, 슬루/뱅크 제한).
//         Course 피드백은 ground course(atan2(Ve,Vn)) 기준 — yaw(기수 방위)가 아니다 [§3].
//         저속(course 불안정)에서만 yaw로 폴백한다.
// 종방향: 최소 구조 (PX4 TECS가 아님 — 에너지 결합·보호기능 없는 단순 PD/PID.
//         TECS 승급 시 FFlightReference 인터페이스 뒤만 교체):
//         고도 오차 → FPA/pitch reference (상승·강하율 제한),
//         속도법칙(슬롯속도 + along-track PD) → airspeed reference.
// 제한값: 뱅크각·피치각·상승/강하율·속도·roll reference 슬루를 전부 이 계층에서 적용.
//
// 의존성: FormationGuidance.h(FFormationTarget)만. UE 밖 JSBSim 하네스에서 단독 검증 가능.
#pragma once

#include <cmath>
#include <algorithm>
#include "FormationGuidance.h"

// FixedWingGuidance → F16CommandController 인터페이스.
struct FFlightReference
{
    double CourseRefDeg    = 0.0;    // 나침반 course reference [0,360)
    double RollRefDeg      = 0.0;    // 뱅크 reference (피드포워드 포함, 제한·슬루 적용 후)
    double RollFfDeg       = 0.0;    // (진단) roll feedforward 성분
    double AltRefM         = 0.0;    // 고도 reference (UE-Z m)
    double FpaRefDeg       = 0.0;    // 비행경로각 reference (상승·강하율 제한 반영)
    double PitchRefDeg     = 0.0;    // FPA + 경로각 보상, ThetaMax/Min 클램프 후
    double AirspeedRefMps  = 0.0;    // <=0 이면 개루프 스로틀 폴백
    double ThrottleRefNorm = -1.0;   // <0 = 없음 (속도 PI가 닫음) — TECS류 FF 확장 자리
};

// Direct/Attack 모드의 고수준 명령 (BT direct 명령·추격 유도 출력·소실 홀드 공용)
struct FDirectCmd
{
    double CourseDeg = 0.0;   // 나침반(0=북) [0,360)
    double AltM      = 0.0;   // UE-Z 고도(m)
    double SpeedMps  = 0.0;   // <=0 = 개루프 스로틀 폴백
    double RollFfDeg = 0.0;
};

struct FFixedWingGuidance
{
    // ── 횡: 벡터필드 (V1 하네스/PIE 검증 수치 — 변경 금지) ──
    double ChiInfDeg = 70.0;    // 최대 컷각(deg)
    double KPath     = 0.004;   // [1/m] — 고속(200+m/s)용, TdCross 리드와 세트.
                                // (0.008+리드無=±100m 진동, 0.0025=꼬리 정착 지연 — V1 실측)
    double TdCross   = 2.5;     // [s] cross-rate 리드 — 헤딩루프 지연(롤업~선회 ~2.5s) 선보상.
                                // 없으면 220m/s에서 주기 40s·±100m 위빙 (V1 실측)

    // ── 종 속도: along-track PD (V1 검증 수치) ──
    double KpAlong  = 0.25;     // 전후오차 → 속도 보정 (0.30은 스로틀 지연과 겹쳐 ±110m 진동 — V1)
    double KdAlong  = 0.80;     // 참 속도벡터 기반 감쇠 (스로틀 응답 지연 선보상)
    double VCorrMax = 150.0;    // 속도 보정 상한 [결함 C 해소]
    double SepBrakeGain = 2.0;  // 안전거리 부족 1m당 감속(m/s) — minimum_separation 침범 시

    // ── roll feedforward (V1/PIE 검증 수치) ──
    double RollFfFloor      = 0.25;    // 게이팅 바닥값 [결함 D 해소]
    double RollFfCrossScale = 2000.0;  // [m]
    double RffTurnCal       = 1.2;     // f16 요-SAS(워시아웃 없음)가 정상선회 요레이트를 억제 →
                                       // 이론 atan(ωV/g)보다 실측 +9° 필요 (V1: 3°/s@220 = 58.8° vs 49.6°)
    double RollFfLimitDeg   = 54.0;    // 팔로워 뱅크캡(62) 이하 — 피드백 여유 8°

    // ── 원거리 REJOIN (히스테리시스 상태기계 — 2026-07-10 인엔진 시험 수치) ──
    // 벡터필드는 근거리 슬롯유지 법칙: km급 변위에선 컷각 포화+선회반경 조합이 S-위빙을
    // 만든다(PIE 실측). 멀면 요격점(슬롯의 t_lead초 후 예측 위치) 조준 추격, 가까우면 복귀.
    double RejoinEnterM   = 800.0;
    double RejoinExitM    = 400.0;
    double RejoinLeadMaxS = 10.0;
    double RejoinTauS     = 8.0;       // 접근 감속 테이퍼: 폐속도 ≤ 거리/τ (과속 도착 바운스 방지)
    double RejoinMinClose = 15.0;

    // ── course → roll reference (구 내루프 외곽 — 검증 수치) ──
    double BankLimitDeg = 60.0;   // 뱅크 제한 (편대/추격 모드는 수신부가 62로 설정)
    double KpHdg2Roll   = 2.5;    // course 오차 1° → roll_ref 2.5° (25°에서 뱅크한계 도달)
    double RollSlewDps  = 40.0;   // roll reference 슬루 제한 — 계단 급변→84° 오버슛 방지 [결함 B]

    // ── 고도 → FPA/pitch reference (검증 수치 + 신규 상승·강하율 제한) ──
    double TanRefM     = 1500.0;  // atan2 유도 기준거리 (수직 시정수 TanRefM/V ≈ 7~9s)
    double ThetaMaxDeg = 25.0, ThetaMinDeg = -20.0;
    double MaxClimbMps = 80.0;    // 상승률 제한 (0 = 없음). F-16 운용 envelope 내 — 게이트 무영향
    double MaxSinkMps  = 60.0;    // 강하율 제한 (0 = 없음)
    double ClimbLeadS  = 8.0;     // 수직 선행보상(초) — 수직 시정수와 정합해야 지속상승 정상오차
                                  // 소거 (2.0은 +88m 지연 — V1 실측)

    // ── 속도 reference 슬루 (신규, 기본 꺼짐 — 검증 수치 보존) ──
    double SpeedRefSlewMps2 = 0.0;   // 0 = 제한 없음

    // ── Course 피드백 [§3] ──
    double CourseMinGroundSpeedMps = 30.0;  // 이상이면 ground course = atan2(Ve,Vn) 사용;
                                            // 미만(활주 초기 등 course 불안정)이면 마지막
                                            // 유효 course, 그것도 없으면 yaw 폴백.

    // ── dt 위생 [§2] — 제어주기 60Hz(고정 1/60 전달) 기준 ──
    double NominalDtS = 1.0 / 60.0;  // 비유한/0/음수 dt 대체값 (슬루·리드 항 보호)
    double MaxDtS     = 0.25;        // dt 상한 (긴 프레임 결손이 슬루 점프가 되는 것 방지)

    // ── 상태 (per-UAV 유지) ──
    bool   bRejoin       = false;
    double PrevRollRef   = 1e9;   // 1e9 = 미초기화 → 첫 스텝은 현재 φ에서 시작 (범프리스)
    double PrevSpeedRef  = 1e9;
    double LastCourseDeg = 1e9;   // 마지막 유효 ground course (저속 폴백용)
    FFlightReference LastReference; // invalid 프레임 홀드용
    bool   bHasLastReference = false;

    // 진단
    double LastCourseRefDeg     = 0.0;
    double LastCurrentCourseDeg = 0.0;   // 이번 스텝이 사용한 현재 course (로그용)
    bool   bSepAssistActive = false;     // 최소 안전거리 감속 보조가 속도를 실제로 깎았음
    bool   bSepWarning      = false;     // 속도가 이미 하한인데 거리가 계속 감소 [§7-3]

    void Reset()
    {
        bRejoin       = false;
        PrevRollRef   = 1e9;
        PrevSpeedRef  = 1e9;
        LastCourseDeg = 1e9;
        bHasLastReference = false;
    }

    static double DeltaHeadingDeg(double target, double current)
    {
        return std::remainder(target - current, 360.0);   // 최단 회전각 [-180,180]
    }

    bool IsValidDt(double Dt) const
    {
        return std::isfinite(Dt) && Dt > 0.0 && Dt <= MaxDtS;
    }

    // 현재 ground course [§3]: 충분한 지상속도면 속도벡터로, 아니면 마지막 유효 course,
    // 그것도 없으면(초기 활주) yaw. wrap [0,360).
    double ComputeCurrentCourseDeg(double OwnVn, double OwnVe, double YawDeg)
    {
        if (!std::isfinite(OwnVn) || !std::isfinite(OwnVe) || !std::isfinite(YawDeg))
            return (LastCourseDeg < 1e8) ? LastCourseDeg : 0.0;
        const double Gs = std::sqrt(OwnVn * OwnVn + OwnVe * OwnVe);
        if (Gs >= CourseMinGroundSpeedMps)
        {
            double C = std::atan2(OwnVe, OwnVn) * 180.0 / 3.14159265358979323846;
            if (C < 0.0) C += 360.0;
            LastCourseDeg = C;
            return C;
        }
        return (LastCourseDeg < 1e8) ? LastCourseDeg : YawDeg;
    }

    // ── 편대 모드: FFormationTarget → FFlightReference ──
    // Own 상태: NED m / m/s / deg / 고도·상승 +위 (UE-Z m). PsiDeg = 기수 방위(yaw) —
    // course 피드백은 내부에서 OwnVn/OwnVe로 계산하고 yaw는 저속 폴백에만 쓴다 [§3].
    FFlightReference StepFormation(
        const FFormationTarget& T,
        double OwnN, double OwnE, double OwnVn, double OwnVe,
        double PhiDeg, double ThetaDeg, double PsiDeg,
        double AltM, double TasMps, double ClimbMps,
        double MinSpd, double MaxSpd, double MinAltM,
        double MaxClosingMps, double Dt)
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double G   = 9.80665;
        const bool bFinite =
            std::isfinite(T.SlotN) && std::isfinite(T.SlotE) &&
            std::isfinite(T.SlotAltM) && std::isfinite(T.SlotVn) && std::isfinite(T.SlotVe) &&
            std::isfinite(T.SlotClimbMps) && std::isfinite(T.TrackRad) && std::isfinite(T.OmegaRadps) &&
            std::isfinite(T.EAlongM) && std::isfinite(T.ECrossM) &&
            std::isfinite(T.DVAlongMps) && std::isfinite(T.DVCrossMps) &&
            std::isfinite(T.SlotDistM) && std::isfinite(T.SeparationDeficitM) &&
            std::isfinite(T.SeparationRateMps) &&
            std::isfinite(OwnN) && std::isfinite(OwnE) && std::isfinite(OwnVn) && std::isfinite(OwnVe) &&
            std::isfinite(PhiDeg) && std::isfinite(ThetaDeg) && std::isfinite(PsiDeg) &&
            std::isfinite(AltM) && std::isfinite(TasMps) && std::isfinite(ClimbMps) &&
            std::isfinite(MinSpd) && std::isfinite(MaxSpd) && std::isfinite(MinAltM) &&
            std::isfinite(MaxClosingMps);
        if (!bFinite || !IsValidDt(Dt))
            return bHasLastReference ? LastReference : FFlightReference{};

        // ── REJOIN 판정 (수평 슬롯거리 히스테리시스) ──
        if (!bRejoin && T.SlotDistM > RejoinEnterM)      bRejoin = true;
        else if (bRejoin && T.SlotDistM < RejoinExitM)   bRejoin = false;

        const double VsMag = std::sqrt(T.SlotVn * T.SlotVn + T.SlotVe * T.SlotVe);

        // ── 횡: course reference ──
        double ChiCmd;
        if (bRejoin)
        {
            // 원거리: 요격점 조준 (tail-chase 추격 곡선 — 컷각 포화·S-위빙 없음)
            const double OwnSpd = std::max(std::sqrt(OwnVn * OwnVn + OwnVe * OwnVe), 100.0);
            const double TLead  = std::min(RejoinLeadMaxS, T.SlotDistM / OwnSpd);
            const double AimN   = T.SlotN + T.SlotVn * TLead;
            const double AimE   = T.SlotE + T.SlotVe * TLead;
            ChiCmd = std::atan2(AimE - OwnE, AimN - OwnN);
        }
        else
        {
            // 근거리: 벡터필드 [결함 A 해소] — 슬롯 이동방향 기준선에 수렴 (+cross-rate 리드)
            const double ChiSlot = (VsMag > 1.0) ? std::atan2(T.SlotVe, T.SlotVn) : T.TrackRad;
            const double ChiInf  = ChiInfDeg * kPi / 180.0;
            ChiCmd = ChiSlot
                + ChiInf * (2.0 / kPi) * std::atan(KPath * (T.ECrossM + TdCross * T.DVCrossMps));
        }

        // ── 종: airspeed reference [결함 C 해소] (ė는 참 속도벡터 차 [결함 E]) ──
        double VCorr = KpAlong * T.EAlongM + KdAlong * T.DVAlongMps;
        VCorr = std::max(-VCorrMax, std::min(VCorrMax, VCorr));
        double SpeedCmd = std::max(MinSpd, std::min(MaxSpd, VsMag + VCorr));
        if (bRejoin)
        {
            // 접근 감속 테이퍼: 폐속도가 거리/τ를 넘지 않게 — 과속 도착 바운스 방지
            const double MaxClose = std::max(RejoinMinClose, T.SlotDistM / RejoinTauS);
            SpeedCmd = std::max(MinSpd, std::min(SpeedCmd, VsMag + MaxClose));
        }
        // 최대 폐속도 제한 (maximum_closing_speed_mps, 0 = 없음)
        if (MaxClosingMps > 0.0)
            SpeedCmd = std::max(MinSpd, std::min(SpeedCmd, VsMag + MaxClosingMps));
        // ── 최소 안전거리 '감속 보조' [§7] — 완전한 충돌 회피가 아니다 ──
        // 부족량 비례로 airspeed reference만 낮춘다 (Minimum-Separation Speed-Reduction
        // Assist). 측면·정면 기하를 바꾸는 Roll/Path 회피법칙은 별도 Collision Avoidance
        // 모듈의 몫 — 이 계층에 없다. 속도가 이미 하한인데 거리가 계속 줄면 경고만 세운다.
        bSepAssistActive = false;
        bSepWarning      = false;
        if (T.SeparationDeficitM > 0.0)
        {
            const double Capped = std::max(MinSpd, std::min(SpeedCmd, VsMag - SepBrakeGain * T.SeparationDeficitM));
            bSepAssistActive = (Capped < SpeedCmd - 1e-9);
            SpeedCmd = Capped;
            bSepWarning = (SpeedCmd <= MinSpd + 0.1) && (T.SeparationRateMps < 0.0);
        }

        // ── 수직: 슬롯 고도 + 상승 선행보상, 하한 가드 ──
        double AltCmd = T.SlotAltM + T.SlotClimbMps * ClimbLeadS;
        if (MinAltM > 0.0)
            AltCmd = std::max(AltCmd, MinAltM);

        // ── roll feedforward [결함 D 해소] (RffTurnCal: 요-SAS 선회효율 손실 보정) ──
        // REJOIN 중엔 0: 추격 곡선의 필요 뱅크는 리더 ω와 무관 — 잔여 ff는 헤딩 바이어스만 만든다.
        double RollFf = 0.0;
        if (!bRejoin)
        {
            const double PhiFf = std::atan2(T.OmegaRadps * std::max(SpeedCmd, 50.0), G) * 180.0 / kPi * RffTurnCal;
            double Scale = 1.0 - std::abs(T.ECrossM) / RollFfCrossScale;
            Scale = std::max(RollFfFloor, std::min(1.0, Scale));
            RollFf = PhiFf * Scale;
            RollFf = std::max(-RollFfLimitDeg, std::min(RollFfLimitDeg, RollFf));
        }

        double CourseDeg = ChiCmd * 180.0 / kPi;
        CourseDeg = std::fmod(CourseDeg, 360.0);
        if (CourseDeg < 0.0) CourseDeg += 360.0;

        // 현재 ground course (yaw 아님 [§3]) — StepFormation/StepDirect 동일 규칙.
        const double CurCourseDeg = ComputeCurrentCourseDeg(OwnVn, OwnVe, PsiDeg);

        return MakeReference(CourseDeg, CurCourseDeg, RollFf, AltCmd, SpeedCmd,
                             PhiDeg, ThetaDeg, AltM, TasMps, ClimbMps, Dt);
    }

    // ── Direct/Attack 모드: 고수준 명령 → FFlightReference ──
    // OwnVn/OwnVe: ground velocity (NED m/s) — course 피드백용 [§3]. PsiDeg는 저속 폴백.
    FFlightReference StepDirect(
        const FDirectCmd& Cmd,
        double OwnVn, double OwnVe,
        double PhiDeg, double ThetaDeg, double PsiDeg,
        double AltM, double TasMps, double ClimbMps, double Dt)
    {
        const bool bFinite =
            std::isfinite(Cmd.CourseDeg) && std::isfinite(Cmd.AltM) &&
            std::isfinite(Cmd.SpeedMps) && std::isfinite(Cmd.RollFfDeg) &&
            std::isfinite(OwnVn) && std::isfinite(OwnVe) &&
            std::isfinite(PhiDeg) && std::isfinite(ThetaDeg) && std::isfinite(PsiDeg) &&
            std::isfinite(AltM) && std::isfinite(TasMps) && std::isfinite(ClimbMps);
        if (!bFinite || !IsValidDt(Dt))
            return bHasLastReference ? LastReference : FFlightReference{};
        const double CurCourseDeg = ComputeCurrentCourseDeg(OwnVn, OwnVe, PsiDeg);
        return MakeReference(Cmd.CourseDeg, CurCourseDeg, Cmd.RollFfDeg, Cmd.AltM, Cmd.SpeedMps,
                             PhiDeg, ThetaDeg, AltM, TasMps, ClimbMps, Dt);
    }

private:
    // 공통 마무리: course→roll reference(뱅크 제한+슬루), alt→FPA→pitch reference(율 제한),
    // airspeed reference 슬루. — 구 내루프의 외곽 법칙 수치 그대로.
    // CurrentCourseDeg: 현재 ground course (ComputeCurrentCourseDeg 결과) — yaw 아님 [§3].
    FFlightReference MakeReference(
        double CourseDeg, double CurrentCourseDeg, double RollFfDeg, double AltRefM, double SpeedRefMps,
        double PhiDeg, double ThetaDeg,
        double AltM, double TasMps, double ClimbMps, double Dt)
    {
        FFlightReference R;
        R.CourseRefDeg = CourseDeg;
        R.RollFfDeg    = RollFfDeg;
        R.AltRefM      = AltRefM;
        LastCourseRefDeg     = CourseDeg;
        LastCurrentCourseDeg = CurrentCourseDeg;

        // ── course 오차(목표 ground course − 현재 ground course) → roll reference ──
        const double dCrs = DeltaHeadingDeg(CourseDeg, CurrentCourseDeg);
        double RollRef = RollFfDeg + KpHdg2Roll * dCrs;
        RollRef = std::min(BankLimitDeg, std::max(-BankLimitDeg, RollRef));
        // 슬루 제한 [결함 B]: reference 계단 급변 → 에일러론 포화 → 롤 관성 → 84° 오버슛(실측).
        // 40°/s 램프면 레이트 항이 선형 영역에 머물러 ζ≈0.9 감쇠가 살아난다.
        if (PrevRollRef > 1e8)
            PrevRollRef = PhiDeg;                        // 재engage 범프리스: 현재 뱅크에서 출발
        const double Slew = RollSlewDps * Dt;
        RollRef = std::min(PrevRollRef + Slew, std::max(PrevRollRef - Slew, RollRef));
        PrevRollRef = RollRef;
        R.RollRefDeg = RollRef;

        // ── 고도 오차 → FPA reference → pitch reference (경로각 보상) ──
        const double V = std::max(TasMps, 30.0);
        const double Gamma    = std::asin(std::min(1.0, std::max(-1.0, ClimbMps / V))) * 180.0 / 3.14159265358979323846;
        double GammaRef = std::atan2(AltRefM - AltM, TanRefM) * 180.0 / 3.14159265358979323846;
        // 상승·강하율 제한 (신규): γ 한계 = asin(rate/V)
        if (MaxClimbMps > 0.0)
            GammaRef = std::min(GammaRef, std::asin(std::min(1.0, MaxClimbMps / V)) * 180.0 / 3.14159265358979323846);
        if (MaxSinkMps > 0.0)
            GammaRef = std::max(GammaRef, -std::asin(std::min(1.0, MaxSinkMps / V)) * 180.0 / 3.14159265358979323846);
        R.FpaRefDeg = GammaRef;
        double ThetaRef = GammaRef + (ThetaDeg - Gamma);                 // 경로각 보상
        ThetaRef = std::min(ThetaMaxDeg, std::max(ThetaMinDeg, ThetaRef));
        R.PitchRefDeg = ThetaRef;

        // ── airspeed reference (+선택적 슬루) ──
        double SpeedRef = SpeedRefMps;
        if (SpeedRefSlewMps2 > 0.0 && SpeedRef > 0.0)
        {
            if (PrevSpeedRef > 1e8) PrevSpeedRef = TasMps;
            const double SSlew = SpeedRefSlewMps2 * Dt;
            SpeedRef = std::min(PrevSpeedRef + SSlew, std::max(PrevSpeedRef - SSlew, SpeedRef));
            PrevSpeedRef = SpeedRef;
        }
        R.AirspeedRefMps = SpeedRef;
        LastReference = R;
        bHasLastReference = true;
        return R;
    }
};

// ── 추격(공격) 유도 — pure pursuit + standoff 속도법칙 (기존 검증 법칙 그대로) ────────
// InterceptTarget의 tail-chase 법칙: 표적 방위로 기수, 표적 고도, 속도는
// 표적속도 + Kp×(거리−standoff). WEZ 방아쇠 판단은 BT에 남는다(10Hz로 충분).
// 출력은 FDirectCmd — FixedWingGuidance::StepDirect가 소비한다.
struct FPursuitGuidance
{
    double StandoffM = 200.0;
    double KpSpeed   = 0.15;

    FDirectCmd Step(
        double TgtN, double TgtE, double TgtAltM,
        double TgtVn, double TgtVe,
        double OwnN, double OwnE,
        double MinSpd, double MaxSpd, double MinAltM) const
    {
        constexpr double kPi = 3.14159265358979323846;
        const double Dn = TgtN - OwnN, De = TgtE - OwnE;
        const double Dist = std::sqrt(Dn * Dn + De * De);

        double H = std::atan2(De, Dn) * 180.0 / kPi;
        if (H < 0.0) H += 360.0;

        const double TgtSpd = std::sqrt(TgtVn * TgtVn + TgtVe * TgtVe);
        const double Spd = std::max(MinSpd, std::min(MaxSpd, TgtSpd + KpSpeed * (Dist - StandoffM)));

        double Alt = TgtAltM;
        if (MinAltM > 0.0)
            Alt = std::max(Alt, MinAltM);

        FDirectCmd Cmd;
        Cmd.CourseDeg = H;
        Cmd.AltM      = Alt;
        Cmd.SpeedMps  = Spd;
        Cmd.RollFfDeg = 0.0;   // 추격은 피드백만 (roll reference 슬루가 급캡처 담당)
        return Cmd;
    }
};
