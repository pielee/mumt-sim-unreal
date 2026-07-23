// 3계층 제어기 단위 테스트 [§11] — JSBSim/UE 불필요 (헤더가 std 전용).
// 빌드·실행: bash Tools/build_verify_units.sh   (또는 g++ -std=c++17 -I<Public> 직접)
// 판정: 실패 0이면 exit 0.
#include "FormationGuidance.h"
#include "FixedWingGuidance.h"
#include "F16CommandController.h"

#include <cstdio>
#include <cmath>
#include <limits>

static int gFail = 0, gPass = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++gPass; } \
    else { ++gFail; printf("  ★FAIL %s:%d  %s\n", __FILE__, __LINE__, msg); } \
} while (0)

static const double NaN = std::numeric_limits<double>::quiet_NaN();
static const double Inf = std::numeric_limits<double>::infinity();
static const double DT  = 1.0 / 60.0;

static bool CmdFinite(const FF16ControlCommand& c)
{
    return std::isfinite(c.AileronCmdNorm) && std::isfinite(c.ElevatorCmdNorm) &&
           std::isfinite(c.RudderCmdNorm) && std::isfinite(c.ThrottleCmdNorm) &&
           std::isfinite(c.SpeedbrakeCmdNorm);
}

// 정상 순항 상태로 한 스텝
static FF16ControlCommand GoodStep(FF16CommandController& c, double dt = DT)
{
    return c.Step(/*rollRef*/0, /*pitchRef*/2, /*vRef*/220, /*thrRef*/-1,
                  /*phi*/0, /*theta*/2, /*φ̇*/0, /*θ̇*/0, /*ψ̇*/0,
                  /*tas*/220, /*fallback*/0.6, dt);
}

// ─── [§11] DeltaTime / NaN ───────────────────────────────────────────────────
static void TestDt()
{
    printf("[1] DeltaTime·NaN 유효성\n");
    FF16CommandController c;
    const FF16ControlCommand good = GoodStep(c);
    CHECK(CmdFinite(good), "정상 스텝 출력 유한");
    const double integ = c.Speed.Integral;

    const double badDt[] = { 0.0, -0.5, NaN, Inf, 10.0 * DT * 10.0 };  // 0/음수/NaN/Inf/정상의 100배
    for (double dt : badDt)
    {
        const FF16ControlCommand o = c.Step(0, 2, 220, -1, 0, 2, 0, 0, 0, 220, 0.6, dt);
        CHECK(CmdFinite(o), "비정상 dt에서 NaN 출력 없음");
        CHECK(std::fabs(o.AileronCmdNorm)  < 0.99, "비정상 dt에서 에일러론 즉시 포화 없음");
        CHECK(std::fabs(o.ElevatorCmdNorm) < 0.99, "비정상 dt에서 엘리베이터 즉시 포화 없음");
        CHECK(o.ThrottleCmdNorm == good.ThrottleCmdNorm, "비정상 dt → 마지막 정상 명령 유지");
        CHECK(c.Speed.Integral == integ, "비정상 dt에서 적분기 미갱신(상태 오염 없음)");
    }
    // NaN 입력 (reference/측정값)
    const FF16ControlCommand o1 = c.Step(NaN, 2, 220, -1, 0, 2, 0, 0, 0, 220, 0.6, DT);
    CHECK(CmdFinite(o1) && o1.ThrottleCmdNorm == good.ThrottleCmdNorm, "NaN reference → 명령 유지");
    const FF16ControlCommand o2 = c.Step(0, 2, 220, -1, NaN, 2, 0, 0, 0, 220, 0.6, DT);
    CHECK(CmdFinite(o2), "NaN 자세 → 유한 출력");
    CHECK(std::isfinite(c.Roll.Integral) && std::isfinite(c.Roll.PrevMeas), "PID 내부 상태에 NaN 미저장");

    // 지속 invalid → Failsafe (기본: 중립 조종면 + 트림 스로틀)
    FF16CommandController c2; GoodStep(c2);
    FF16ControlCommand last{};
    for (int i = 0; i < 70; ++i)   // 70/60 s > InvalidInputTimeoutS(1.0)
        last = c2.Step(NaN, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0.6, DT);
    CHECK(last.AileronCmdNorm == 0.f && last.ElevatorCmdNorm == 0.f, "지속 invalid → Failsafe 중립 조종면");
    CHECK(std::fabs(last.ThrottleCmdNorm - c2.ThrottleTrimNorm) < 1e-6, "Failsafe 스로틀 = 트림");
    // 회복
    const FF16ControlCommand rec = GoodStep(c2);
    CHECK(CmdFinite(rec) && c2.InvalidInputTimer == 0.0, "정상 입력 복귀 시 타이머 리셋");

    // FInnerPID 단독
    FInnerPID pid; pid.Configure(1, 0.1, 0.1, -1, 1);
    pid.Update(0.5, 0.0, DT);
    const double i0 = pid.Integral, lo = pid.LastOutput;
    CHECK(pid.Update(0.5, 0.0, 0.0) == lo && pid.Integral == i0, "PID dt=0 → 상태 미갱신·출력 유지");
    CHECK(pid.Update(NaN, 0.0, DT) == lo && pid.Integral == i0, "PID NaN error → 상태 미갱신");
    CHECK(pid.Update(0.5, 0.0, 1.0) == lo && pid.Integral == i0, "PID 과대 dt → 상태 미갱신");

    // Guidance 계층도 invalid 프레임에서 필터/슬루/래치를 갱신하지 않고 마지막 정상값 유지
    FFixedWingGuidance g;
    FDirectCmd dc; dc.CourseDeg = 35; dc.AltM = 1000; dc.SpeedMps = 220;
    const double cr = 35.0 * 3.14159265358979323846 / 180.0;
    const FFlightReference gr = g.StepDirect(dc, 220*std::cos(cr), 220*std::sin(cr),
                                              0, 0, 20, 1000, 220, 0, DT);
    const double prevRoll = g.PrevRollRef, prevCourse = g.LastCourseDeg;
    const FFlightReference badGr = g.StepDirect(dc, NaN, 0, 0, 0, 20, 1000, 220, 0, DT);
    CHECK(badGr.RollRefDeg == gr.RollRefDeg && g.PrevRollRef == prevRoll && g.LastCourseDeg == prevCourse,
          "FixedWingGuidance NaN → 마지막 reference 유지·상태 미갱신");
    CHECK(g.StepDirect(dc, 220, 0, 0, 0, 0, 1000, 220, 0, Inf).RollRefDeg == gr.RollRefDeg,
          "FixedWingGuidance invalid dt → 마지막 reference 유지");

    FFormationGuidance fg;
    const FFormationTarget ft = fg.Step(0,0,1000, 220,0,0, -80,100,1000, 220,0, -80,100,0, DT);
    const double prevTrack = fg.PrevTrackRad, prevSep = fg.PrevSeparationM;
    const FFormationTarget badFt = fg.Step(NaN,0,1000, 220,0,0, -80,100,1000, 220,0, -80,100,0, DT);
    CHECK(badFt.SlotN == ft.SlotN && fg.PrevTrackRad == prevTrack && fg.PrevSeparationM == prevSep,
          "FormationGuidance NaN → 마지막 target 유지·상태 미갱신");
}

// ─── [§11] Course/Yaw 분리 ───────────────────────────────────────────────────
static void TestCourseYaw()
{
    printf("[2] Course/Yaw 분리\n");
    // Yaw=20°, Ground Course=35°(속도벡터), Target Course=35° → Course 오차≈0 → 롤 명령≈0
    FFixedWingGuidance g;
    FDirectCmd cmd; cmd.CourseDeg = 35.0; cmd.AltM = 1000.0; cmd.SpeedMps = 220.0;
    const double crs = 35.0 * 3.14159265358979323846 / 180.0;
    const double Vn = 220.0 * std::cos(crs), Ve = 220.0 * std::sin(crs);
    FFlightReference r{};
    for (int i = 0; i < 10; ++i)
        r = g.StepDirect(cmd, Vn, Ve, /*phi*/0, /*theta*/0, /*yaw*/20.0, 1000, 220, 0, DT);
    CHECK(std::fabs(g.LastCurrentCourseDeg - 35.0) < 0.01, "현재 course = 속도벡터 기준 35°");
    CHECK(std::fabs(r.RollRefDeg) < 0.5, "course 오차 0 → 불필요한 롤 명령 없음 (yaw 20° 무시)");

    // 저속 폴백: gs < 임계 → 마지막 유효 course, 최초면 yaw
    FFixedWingGuidance g2;
    g2.StepDirect(cmd, 3.0, 2.0, 0, 0, /*yaw*/20.0, 1000, 10, 0, DT);
    CHECK(std::fabs(g2.LastCurrentCourseDeg - 20.0) < 0.01, "저속+이력없음 → yaw 폴백");
    g2.StepDirect(cmd, Vn, Ve, 0, 0, 20.0, 1000, 220, 0, DT);      // 유효 course 확보
    g2.StepDirect(cmd, 3.0, 2.0, 0, 0, /*yaw*/77.0, 1000, 10, 0, DT);
    CHECK(std::fabs(g2.LastCurrentCourseDeg - 35.0) < 0.01, "저속 → 마지막 유효 course 유지(yaw 아님)");
}

// ─── [§11] Throttle Trim ─────────────────────────────────────────────────────
static void TestThrottleTrim()
{
    printf("[3] Throttle Trim/Feedforward\n");
    FF16CommandController c;
    FF16ControlCommand o{};
    for (int i = 0; i < 120; ++i) o = GoodStep(c);   // 2s, 오차 0 유지
    CHECK(std::fabs(o.ThrottleCmdNorm - c.ThrottleTrimNorm) < 0.05,
          "속도오차 0 → 스로틀 ≈ 트림(0.26), Idle로 안 떨어짐");
    CHECK(o.ThrottleCmdNorm > 0.15, "스로틀 하한(구 0.05) 증상 없음");
    // ThrottleRefNorm >= 0 → FF로 사용
    const FF16ControlCommand o2 = c.Step(0, 2, 220, /*thrRef*/0.40, 0, 2, 0, 0, 0, 220, 0.6, DT);
    CHECK(std::fabs(c.LastThrottleFF - 0.40) < 1e-9, "ThrottleRefNorm>=0 → feedforward 채택");
    (void)o2;
    // 개루프 폴백 (vRef<=0)
    const FF16ControlCommand o3 = c.Step(0, 2, /*vRef*/0, -1, 0, 2, 0, 0, 0, 220, /*fallback*/0.6, DT);
    CHECK(std::fabs(o3.ThrottleCmdNorm - 0.6) < 1e-6, "vRef<=0 → 개루프 폴백 유지");
    // 전체 권한: 큰 속도 오차 → 스로틀 1.0/0.0 도달 (이륙 가속·Vmax 추격 — 인엔진 회귀 원인)
    FF16CommandController c4;
    FF16ControlCommand o4{};
    for (int i = 0; i < 5; ++i) o4 = c4.Step(0, 2, /*vRef*/220, -1, 0, 2, 0, 0, 0, /*tas*/60, 0.6, DT);
    CHECK(o4.ThrottleCmdNorm >= 0.999f, "큰 감속오차 → 풀스로틀 1.0 (0.76 캡 재발 방지)");
    for (int i = 0; i < 5; ++i) o4 = c4.Step(0, 2, /*vRef*/120, -1, 0, 2, 0, 0, 0, /*tas*/335, 0.6, DT);
    CHECK(o4.ThrottleCmdNorm <= 0.001f, "큰 과속오차 → 스로틀 0 도달 (감속 컷 즉시)");
}

// ─── [§11] Mode Re-engage (범프리스) ─────────────────────────────────────────
static void TestReengage()
{
    printf("[4] Re-engage 범프리스\n");
    FF16CommandController c;
    FFixedWingGuidance g;
    FDirectCmd cmd; cmd.CourseDeg = 90.0; cmd.AltM = 1000.0; cmd.SpeedMps = 220.0;
    const double Vn = 0.0, Ve = 220.0;   // course 90
    // 큰 course 오차로 얼마간 비행 (적분기·슬루 채움)
    for (int i = 0; i < 200; ++i) {
        FDirectCmd off = cmd; off.CourseDeg = 180.0;
        FFlightReference r = g.StepDirect(off, Vn, Ve, 30.0, 2, 90.0, 900, 200, 0, DT);
        c.Step(r.RollRefDeg, r.PitchRefDeg, r.AirspeedRefMps, r.ThrottleRefNorm,
               30.0, 2, 0, 0, 0, 200, 0.6, DT);
    }
    // Disable→Enable 모사: 리셋 후 현재 φ=10°에서 재개
    c.Reset(); g.Reset();
    FFlightReference r1 = g.StepDirect(cmd, Vn, Ve, /*phi*/10.0, 2, 90.0, 1000, 220, 0, DT);
    CHECK(std::fabs(r1.RollRefDeg - 10.0) <= 40.0 * DT + 1e-9,
          "재engage 첫 스텝 RollRef가 현재 φ에서 슬루 시작 (계단 없음)");
    FF16ControlCommand o1 = c.Step(r1.RollRefDeg, r1.PitchRefDeg, r1.AirspeedRefMps, r1.ThrottleRefNorm,
                                   10.0, 2, 0, 0, 0, 220, 0.6, DT);
    CHECK(std::fabs(o1.ThrottleCmdNorm - c.ThrottleTrimNorm) < 0.05, "재engage 스로틀 ≈ 트림 (급변 없음)");
    CHECK(c.Speed.Integral == 0.0 ? true : std::fabs(c.Speed.Integral) < 1e-3, "적분기 잔류 없음");
    (void)o1;
}

// ─── [§11] Speedbrake ────────────────────────────────────────────────────────
static void TestSpeedbrake()
{
    printf("[5] Speedbrake 설정 곡선\n");
    FF16CommandController c;
    auto Sb = [&](double over) {
        FF16ControlCommand o = c.Step(0, 2, 220, -1, 0, 2, 0, 0, 0, 220 + over, 0.6, DT);
        return (double)o.SpeedbrakeCmdNorm;
    };
    CHECK(Sb(4.0)  == 0.0, "overspeed < Start(5) → 0");
    CHECK(Sb(5.0)  == 0.0, "overspeed = Start → 0");
    const double mid = Sb(12.5);
    CHECK(mid > 0.45 && mid < 0.55, "중간(12.5) → ≈0.5");
    CHECK(Sb(20.0) == 1.0, "overspeed = Full(20) → 1");
    CHECK(Sb(30.0) == 1.0, "overspeed > Full → 1");
    // 설정 역전 → 안전한 계단형
    FF16CommandController c2; c2.SpeedbrakeStartOverspeedMps = 10.0; c2.SpeedbrakeFullOverspeedMps = 5.0;
    FF16ControlCommand a = c2.Step(0, 2, 220, -1, 0, 2, 0, 0, 0, 228, 0.6, DT);   // over 8 < Start10
    FF16ControlCommand b = c2.Step(0, 2, 220, -1, 0, 2, 0, 0, 0, 232, 0.6, DT);   // over 12 > Start10
    CHECK(a.SpeedbrakeCmdNorm == 0.f && b.SpeedbrakeCmdNorm == 1.f, "설정 역전 → 계단형 강등(안전)");
}

// ─── [§11] Formation Guidance 기하·판정 ──────────────────────────────────────
static void TestFormation()
{
    printf("[6] FormationGuidance\n");
    const double kPi = 3.14159265358979323846;

    // 직선 리더 (동쪽 220): 슬롯에 정확히 위치 → 오차 0, ω→0, 2s 후 캡처
    {
        FFormationGuidance f;
        FFormationTarget T{};
        // track=90°: slot = leader + (front=-80)·fwd + (right=100)·right, fwd=(0,1), right=(-1,0)
        const double SlotN = -100.0, SlotE = -80.0;
        for (int i = 0; i < 140; ++i)   // 2.33s > CaptureHoldS(2)
        {
            const double LdrE = 220.0 * i * DT;
            T = f.Step(0, LdrE, 1000, 0, 220, 0, SlotN, LdrE + SlotE, 1000, 0, 220,
                       -80, 100, 0, DT);
        }
        CHECK(std::fabs(T.EAlongM) < 0.5 && std::fabs(T.ECrossM) < 0.5, "직선: 슬롯 오차 ≈ 0");
        CHECK(std::fabs(T.OmegaRadps) < 1e-6, "직선: ω 데드밴드로 0");
        CHECK(T.bCaptured && T.bMaintained, "직선 2s 유지 → 캡처+유지");
        CHECK(std::fabs(T.SlotCentripetalAn) < 1e-9, "직선: 구심 성분 0");
    }

    // 일정 선회 리더 (3°/s): ω 추정 수렴 + 슬롯속도 = 유한차분 검증
    {
        FFormationGuidance f;
        FFormationTarget T{}, Tprev{};
        double th = kPi / 2.0;   // track 90°에서 시작
        const double W = 3.0 * kPi / 180.0, V = 220.0, R = V / W;
        for (int i = 0; i < 300; ++i)   // 5s
        {
            th += W * DT;
            const double LdrN = R * std::sin(th) - R, LdrE = -R * std::cos(th) + R; // 원 위 (임의 기준)
            const double Vn = V * std::cos(th), Ve = V * std::sin(th);
            Tprev = T;
            T = f.Step(LdrN, LdrE, 1000, Vn, Ve, 0, /*own 멀리*/-500, -500, 1000, 0, 220,
                       -80, 100, 0, DT);
        }
        CHECK(std::fabs(T.OmegaRadps - W) < 0.5 * kPi / 180.0, "선회: ω 추정 3°/s ±0.5");
        const double fdVn = (T.SlotN - Tprev.SlotN) / DT, fdVe = (T.SlotE - Tprev.SlotE) / DT;
        CHECK(std::fabs(fdVn - T.SlotVn) < 3.0 && std::fabs(fdVe - T.SlotVe) < 3.0,
              "선회: 슬롯속도(v+ω×r) ≈ 슬롯위치 유한차분");
        CHECK(std::fabs(std::sqrt(T.SlotCentripetalAn * T.SlotCentripetalAn +
                                  T.SlotCentripetalAe * T.SlotCentripetalAe)
                        - W * W * std::sqrt(80.0 * 80.0 + 100.0 * 100.0)) < 0.05,
              "선회: 구심 성분 크기 = ω²·|r|");
    }

    // 저속 리더: track/ω 홀드 (bLeaderValid false)
    {
        FFormationGuidance f;
        f.Step(0, 0, 1000, 0, 220, 0, -100, -80, 1000, 0, 220, -80, 100, 0, DT);   // 유효 track 90°
        FFormationTarget T = f.Step(0, 10, 1000, 5, 5, 0, -100, -70, 1000, 0, 0, -80, 100, 0, DT);
        CHECK(!T.bLeaderValid, "저속 리더 → track 신뢰 불가 플래그");
        CHECK(std::fabs(T.TrackRad - kPi / 2.0) < 1e-6, "저속 리더 → 마지막 유효 track 홀드");
    }

    // Track 360° wrap: 북 통과 (358°→2°, +2°/s) — ω 스파이크 없음
    {
        FFormationGuidance f;
        FFormationTarget T{};
        double trk = 358.0 * kPi / 180.0;
        for (int i = 0; i < 240; ++i)   // 4s → 358+8 = 6°
        {
            trk += (2.0 * kPi / 180.0) * DT;
            const double Vn = 220.0 * std::cos(trk), Ve = 220.0 * std::sin(trk);
            T = f.Step(0, 0, 1000, Vn, Ve, 0, -500, -500, 1000, 0, 220, -80, 100, 0, DT);
            if (std::fabs(T.OmegaRadps) >= 15.0 * kPi / 180.0)
            {
                CHECK(false, "wrap 중 ω 스파이크 없음");
                break;
            }
        }
        ++gPass;   // 루프 전체 무스파이크 통과 1건으로 계수
        CHECK(std::fabs(T.OmegaRadps - 2.0 * kPi / 180.0) < 0.7 * kPi / 180.0, "wrap 후 ω ≈ +2°/s");
    }

    // 캡처/해제 디바운스 + 리더 변경(Reset)
    {
        FFormationGuidance f;
        FFormationTarget T{};
        auto AtSlot  = [&](int n) { for (int i = 0; i < n; ++i) T = f.Step(0,0,1000, 0,220,0, -100,-80,1000, 0,220, -80,100,0, DT); };
        auto OffSlot = [&](int n) { for (int i = 0; i < n; ++i) T = f.Step(0,0,1000, 0,220,0, -170,-80,1000, 0,220, -80,100,0, DT); }; // 70m > 2×30
        AtSlot(110);  CHECK(!T.bCaptured, "1.83s < 2s → 아직 캡처 아님");
        AtSlot(20);   CHECK(T.bCaptured,  "2.17s → 캡처");
        OffSlot(110); CHECK(T.bCaptured,  "이탈 1.83s < 2s → 캡처 유지(디바운스)");
        OffSlot(20);  CHECK(!T.bCaptured, "이탈 2.17s → 캡처 해제");
        f.Reset();    CHECK(f.PrevTrackRad > 1e8 && !f.bCapturedState, "리더 변경 Reset → 추정기·래치 초기화");
    }

    // 최소 안전거리: breach/deficit/접근율 + FWG 감속 보조·경고
    {
        FFormationGuidance f; f.MinSeparationM = 200.0;
        FFormationTarget T{};
        for (int i = 0; i < 60; ++i)   // own이 리더로 접근 (150→90m)
            T = f.Step(0, 0, 1000, 0, 220, 0, -150.0 + i, 0, 1000, 1.0 * 60, 220, -80, 100, 0, DT);
        CHECK(T.bSeparationBreach, "분리 < 최소 → breach");
        CHECK(T.SeparationDeficitM > 0.0, "deficit > 0");
        CHECK(T.SeparationRateMps < 0.0, "접근 중 → 분리 변화율 음수");

        FFixedWingGuidance g;
        FFormationTarget Tm{};   // 수동 구성: 감속 보조 강제
        Tm.SlotVn = 0; Tm.SlotVe = 220; Tm.TrackRad = kPi / 2.0;
        Tm.SeparationDeficitM = 50.0; Tm.SeparationRateMps = -3.0;
        Tm.EAlongM = 200.0;   // 가속 요구 상황에서도
        FFlightReference r = g.StepFormation(Tm, 0, 0, 0, 220, 0, 0, 90.0, 1000, 220, 0,
                                             120, 335, 0, 0, DT);
        CHECK(r.AirspeedRefMps <= 220.0 - 2.0 * 50.0 + 1e-6, "감속 보조: Vref ≤ 슬롯속도 − 2·deficit");
        CHECK(g.bSepAssistActive, "감속 보조 발동 플래그");
        Tm.SeparationDeficitM = 300.0;   // 하한까지 깎임 → 경고
        r = g.StepFormation(Tm, 0, 0, 0, 220, 0, 0, 90.0, 1000, 220, 0, 120, 335, 0, 0, DT);
        CHECK(r.AirspeedRefMps >= 120.0 - 1e-6 && g.bSepWarning,
              "속도 하한 + 접근 지속 → separation warning");
    }
}

int main()
{
    printf("══ 3계층 제어기 단위 테스트 ══\n");
    TestDt();
    TestCourseYaw();
    TestThrottleTrim();
    TestReengage();
    TestSpeedbrake();
    TestFormation();
    printf("══ 결과: PASS=%d FAIL=%d ══\n", gPass, gFail);
    return gFail ? 1 : 0;
}
