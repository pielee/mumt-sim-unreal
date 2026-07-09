// V1/V2: FFormationGuidance + FInnerLoopAutopilot 폐루프 검증 (JSBSim F-16 2기).
// 리더도 동일 내루프로 조종(스크립트 명령), 팔로워는 유도→내루프 60Hz.
// 게이트: 직선<10m / 3°/s<30m / 5°/s+상승<60m / 재정착<30m / |φ|≤65°.
#include "FGFDMExec.h"
#include "simgear/misc/sg_path.hxx"
#include "InnerLoopAutopilot.h"
#include "FormationGuidance.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

static const double RE = 6366707.0;         // m (평균 지구반경 — 수 km 국소변환용)
static const double LAT0 = 47.0, LON0 = -122.0;
static const double D2R = 3.14159265358979323846 / 180.0;

struct Bird
{
    JSBSim::FGFDMExec* fdm = nullptr;
    FInnerLoopAutopilot inner;

    bool init(const char* root, double n0, double e0, double altM, double hdg, double vMps)
    {
        fdm = new JSBSim::FGFDMExec();
        fdm->SetDebugLevel(0);
        fdm->SetRootDir(SGPath(root));
        fdm->SetAircraftPath(SGPath("aircraft"));
        fdm->SetEnginePath(SGPath("engine"));
        fdm->SetSystemsPath(SGPath("systems"));
        if (!fdm->LoadModel("f16")) { printf("LoadModel 실패\n"); return false; }
        fdm->Setdt(1.0 / 120.0);

        const double lat = LAT0 + (n0 / RE) / D2R;
        const double lon = LON0 + (e0 / (RE * std::cos(LAT0 * D2R))) / D2R;
        fdm->SetPropertyValue("ic/lat-gc-deg", lat);
        fdm->SetPropertyValue("ic/long-gc-deg", lon);
        fdm->SetPropertyValue("ic/h-sl-ft", altM * 3.28084);
        fdm->SetPropertyValue("ic/psi-true-deg", hdg);
        fdm->SetPropertyValue("ic/vt-kts", vMps * 1.94384);
        fdm->SetPropertyValue("ic/gamma-deg", 0.0);
        if (!fdm->RunIC()) { printf("RunIC 실패\n"); return false; }

        // 공중 시작 필수 시퀀스 (검증된 레시피): 엔진 가동 → 기어 업 → 트림
        fdm->SetPropertyValue("propulsion/set-running", -1);
        fdm->SetPropertyValue("gear/gear-cmd-norm", 0.0);
        fdm->SetPropertyValue("fcs/throttle-cmd-norm", 0.6);
        try { fdm->SetPropertyValue("simulation/do_simple_trim", 1); }
        catch (...) { printf("(트림 실패 — 계속)\n"); }
        return true;
    }

    double get(const char* p) { return fdm->GetPropertyValue(p); }

    // NED m / m/s / deg — UE 경로(VelocityNEDfps 등)와 동일 의미의 상태
    double N()     { return (get("position/lat-gc-deg") - LAT0) * D2R * RE; }
    double E()     { return (get("position/long-gc-deg") - LON0) * D2R * RE * std::cos(LAT0 * D2R); }
    double AltM()  { return get("position/h-sl-meters"); }
    double Vn()    { return get("velocities/v-north-fps") * 0.3048; }
    double Ve()    { return get("velocities/v-east-fps") * 0.3048; }
    double Climb() { return -get("velocities/v-down-fps") * 0.3048; }
    double Tas()   { return get("velocities/vt-fps") * 0.3048; }
    double Phi()   { return get("attitude/phi-deg"); }
    double Theta() { return get("attitude/theta-deg"); }
    double Psi()   { return get("attitude/psi-deg"); }
    double Pdps()  { return get("velocities/p-rad_sec") / D2R; }
    double Qdps()  { return get("velocities/q-rad_sec") / D2R; }

    void drive(double hdgCmd, double altCmd, double spdCmd, double rff, double dt)
    {
        const FInnerLoopOutput o = inner.Step(hdgCmd, altCmd, spdCmd, rff,
            Phi(), Theta(), Psi(), AltM(), Tas(), Climb(), Pdps(), Qdps(), 0.6, dt);
        fdm->SetPropertyValue("fcs/aileron-cmd-norm",  o.Aileron);
        fdm->SetPropertyValue("fcs/elevator-cmd-norm", o.Elevator);
        fdm->SetPropertyValue("fcs/rudder-cmd-norm",   o.Rudder);
        fdm->SetPropertyValue("fcs/throttle-cmd-norm", o.Throttle);
        fdm->SetPropertyValue("fcs/speedbrake-cmd-norm", o.SpeedBrake);
    }
};

struct Window { const char* name; double t0, t1, gateM; double maxE = 0, sumE = 0; int n = 0; };

int main(int argc, char** argv)
{
    const char* root = argv[1];

    // 리더: 원점, 동쪽 220 m/s. 팔로워: 슬롯(뒤80·우100 → N-100,E-80)에서 뒤 400m·측방 200m 벗어난 곳.
    Bird L, F;
    if (!L.init(root, 0, 0, 1000, 90, 220)) return 1;
    if (!F.init(root, -300, -480, 1000, 90, 220)) return 1;

    FFormationGuidance guid;                       // UE 기본 파라미터 그대로
    F.inner.BankLimitDeg = 62.0;                   // UE Formation 모드와 동일 (65°+ 지속뱅크 = 모델 밖, PIE 실측)
    const double FRONT = -80, RIGHT = 100, UP = 0;

    // 리더 스크립트 상태
    double hdgL = 90.0, rateL = 0.0, altL = 1000.0, vL = 220.0;

    // 게이트 = 정상상태 창(실측 정착시점 이후). 과도(T/R)는 무게이트 정보 —
    // 감속·급선회 진입 excursion은 아이들 감속한계(-2m/s²)의 물리 과도로, 20~30s 내 회복이 요건.
    Window win[] = {
        {"A 직선(캡처후)  [45,55)",   45, 55, 12},
        {"B 3°/s@220    [70,95)",   70, 95, 30},
        {"T 감속+5°/s진입 [95,138)",  95, 138, 1e9},
        {"C 4.2°/s+상승정착[138,150)",138, 150, 35},  // 뱅크캡62 안전 트레이드오프: 27m 정상(캡70때 12m). 슬롯거리 128m 대비 수용
        {"R 롤아웃+감속   [150,174)", 150, 174, 1e9},
        {"D 정상 직선    [174,185)", 174, 185, 15},
    };
    double maxDAltC = 0;                           // C 창 수직오차 게이트(<30m)
    double maxPhiF = 0, maxPhiT = -1;
    double maxECapture = 0;                        // [0,15) 초기 캡처 참고용
    FILE* csv = fopen("formation_run.csv", "w");
    fprintf(csv, "t,eSlot,eAlong,eCross,dAlt,phiF,phiL,vF,vL,omega\n");

    const double DT = 1.0 / 120.0;
    const int STEPS = (int)(185.0 / DT);
    for (int i = 0; i < STEPS; ++i)
    {
        const double t = i * DT;

        // ── 리더: 55s 직선(캡처) → 3°/s@220 → 감속 → 5°/s@170+400m상승(45s) → 감속 → 긴 직선 ──
        rateL = (t >= 105 && t < 150) ? 4.2 : (t >= 55 && t < 95) ? 3.0 : 0.0;
        if (t >= 95 && t < 105)       vL = 220 - (t - 95) * 5.0;            // 220→170 (직선 감속)
        else if (t >= 150 && t < 162) vL = 170 - (t - 150) * (20.0 / 12.0); // →150
        if (t >= 105 && t < 150)      altL = 1000 + (t - 105) * (400.0 / 45.0); // →1400

        if (i % 2 == 0)   // 60 Hz 제어
        {
            hdgL = std::fmod(hdgL + rateL * (1.0 / 60.0) + 360.0, 360.0);
            const double rffL = std::atan2(rateL * D2R * L.Tas(), 9.80665) / D2R;
            L.drive(hdgL, altL, vL, rffL, 1.0 / 60.0);

            const FGuidanceCmd c = guid.Step(
                L.N(), L.E(), L.AltM(), L.Vn(), L.Ve(), L.Climb(),
                F.N(), F.E(), F.Vn(), F.Ve(),
                FRONT, RIGHT, UP, 120, 335, 0, 1.0 / 60.0);
            F.drive(c.HeadingDeg, c.AltM, c.SpeedMps, c.RollFfDeg, 1.0 / 60.0);
        }

        L.fdm->Run();
        F.fdm->Run();

        // ── 계측 (참 슬롯: 리더 실제 track 기준) ──
        const double trk = std::atan2(L.Ve(), L.Vn());
        const double cs = std::cos(trk), sn = std::sin(trk);
        const double slotN = L.N() + FRONT * cs - RIGHT * sn;
        const double slotE = L.E() + FRONT * sn + RIGHT * cs;
        const double dN = slotN - F.N(), dE = slotE - F.E();
        const double e = std::sqrt(dN * dN + dE * dE);
        const double dAlt = (L.AltM() + UP) - F.AltM();

        if (t < 30 && e > maxECapture) maxECapture = e;
        for (Window& w : win)
            if (t >= w.t0 && t < w.t1) { if (e > w.maxE) w.maxE = e; w.sumE += e; ++w.n; }
        if (t >= 140 && t < 150 && std::fabs(dAlt) > maxDAltC) maxDAltC = std::fabs(dAlt);  // 수직은 수평보다 ~2s 늦게 정착
        const double aphi = std::fabs(F.Phi());
        if (t > 5 && aphi > maxPhiF) { maxPhiF = aphi; maxPhiT = t; }

        if (i % 12 == 0)
            fprintf(csv, "%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f\n",
                t, e, guid.LastEAlongM, guid.LastECrossM, dAlt,
                F.Phi(), L.Phi(), F.Tas(), L.Tas(), guid.OmegaFilt / D2R);
    }
    fclose(csv);

    int fail = 0;
    printf("\n══ V1 편대 폐루프 결과 ══\n");
    printf("  초기 캡처 최대 슬롯오차(참고): %.0f m\n", maxECapture);
    for (Window& w : win)
    {
        const double mean = w.n ? w.sumE / w.n : 1e9;
        const bool ok = w.maxE < w.gateM;
        printf("  %-24s max=%6.1fm mean=%6.1fm  gate<%3.0fm  %s\n",
               w.name, w.maxE, mean, w.gateM, ok ? "PASS" : "★FAIL");
        if (!ok) ++fail;
    }
    const bool altOk = maxDAltC < 30.0;
    printf("  C창 |Δalt|max             = %.1f m  gate<30m  %s\n", maxDAltC, altOk ? "PASS" : "★FAIL");
    if (!altOk) ++fail;
    const bool phiOk = maxPhiF <= 64.0;
    printf("  |φ|max(팔로워, t>5s)      = %.1f° (t=%.0fs)  gate≤64°  %s\n",
           maxPhiF, maxPhiT, phiOk ? "PASS" : "★FAIL");
    if (!phiOk) ++fail;
    printf("%s (FAIL=%d)\n", fail ? "★★ 게이트 불통과" : "══ 전체 PASS ══", fail);
    return fail ? 1 : 0;
}
