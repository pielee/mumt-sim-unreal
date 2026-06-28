// BVRGymAutopilot.cpp
#include "BVRGymAutopilot.h"
#include "Math/UnrealMathUtility.h"

// ─── FPID ─────────────────────────────────────────────────────────────────

float FPID::Update(float CurrentValue)
{
    const float Error = 0.f - CurrentValue;   // SetPoint always 0
    const float P     = Kp * Error;
    const float D     = Kd * (Error - Derivator);
    Derivator         = Error;
    Integrator        = FMath::Clamp(Integrator + Error, IntegMin, IntegMax);
    const float I     = Ki * Integrator;
    return P + I + D;
}

void FPID::Reset()
{
    Derivator  = 0.f;
    Integrator = 0.f;
}

void FPID::SetGains(const FPID& Cfg)
{
    Kp = Cfg.Kp; Ki = Cfg.Ki; Kd = Cfg.Kd;
    IntegMin = Cfg.IntegMin; IntegMax = Cfg.IntegMax;
    Derivator = 0.f;   // matches prior per-tick reset (roll/pitch D unchanged)
    // Integrator intentionally preserved → autothrottle trim accumulates.
}

// ─── FAircraftAutopilot ────────────────────────────────────────────────────

FAircraftAutopilot::FAircraftAutopilot()
{
    // Defaults from f16_config.py
    RollPID    = {0.01f, 0.f, 0.9f, -0.2f,  0.2f};
    RollSecPID = {0.2f,  0.f, 0.2f, -1.0f,  1.0f};
    PitchPID   = {0.3f,  0.f, 1.0f, -1.0f,  1.0f};
    // Speed-hold (autothrottle): PI on airspeed error. Integrator provides the
    // steady-state trim throttle (Ki*IntegMax ≈ 1). Output clamped to [0,1].
    ThrottlePID = {0.02f, 0.004f, 0.f, 0.f, 250.f};

    AltActSpace   = NavParams.AltActSpaceMin;
    HeadActSpace  = NavParams.HeadActSpaceMin;
    ThetaActSpace = NavParams.ThetaActSpaceMin;
}

FAutopilotOutput FAircraftAutopilot::GetControlInput(
    float DiffHeadDeg,
    float DiffAltM,
    float CurrentPhiDeg,
    float CurrentThetaDeg,
    float CurrentSpeedMps,
    float TargetSpeedMps)
{
    const float AbsHead = FMath::Abs(DiffHeadDeg);
    const float AbsAlt  = FMath::Abs(DiffAltM);

    static int32 ApLogCnt = 0;
    const bool bLog = (++ApLogCnt % 10 == 0);

    if (AbsHead >= HeadActSpace && AbsAlt <= AltActSpace)
    {
        // ── Hard-turn mode ────────────────────────────────────────────────
        AltActSpace = NavParams.AltActSpaceMax;

        const float BankRef = (DiffHeadDeg >= 0.f ? 1.f : -1.f) * NavParams.RollMax;
        SetRollPID(BankRef, false, CurrentPhiDeg);

        SetPitchPID(FMath::RadiansToDegrees(FMath::Atan2(DiffAltM, NavParams.TanRef)),
                    CurrentThetaDeg);

        // Stay-in-turn threshold must be <= the enter threshold (HeadActSpaceMax).
        // A level's NavParams override that sets HeadActSpaceMin > Max would invert
        // the hysteresis → hard-turn flickers off every tick → aircraft never turns.
        // Clamp defensively so turning works regardless of the configured value.
        HeadActSpace = FMath::Min(NavParams.HeadActSpaceMin, NavParams.HeadActSpaceMax);

        if (bLog) UE_LOG(LogTemp, Warning,
            TEXT("[AP-MODE] HARDTURN  DiffHead=%.1f DiffAlt=%.0f Phi=%.1f | Ail=%.3f Elv=%.3f"),
            DiffHeadDeg, DiffAltM, CurrentPhiDeg, AileronCmd, ElevatorCmd);
    }
    else
    {
        // ── Precision mode ────────────────────────────────────────────────
        AltActSpace = NavParams.AltActSpaceMin;

        float ThetaRef = FMath::RadiansToDegrees(
            FMath::Atan2(DiffAltM, NavParams.TanRef));
        ThetaRef = FMath::Clamp(ThetaRef,
            NavParams.DiveThetaMax, NavParams.ClimbThetaMax);

        if (AbsAlt > 1500.f)
        {
            ThetaActSpace = NavParams.ThetaActSpaceMax;
            SetRollPID(0.f, false, CurrentPhiDeg);
            if (bLog) UE_LOG(LogTemp, Warning,
                TEXT("[AP-MODE] PREC-CLIMB  ThetaRef=%.1f Theta=%.1f DiffAlt=%.0f | Ail=%.3f Elv=%.3f"),
                ThetaRef, CurrentThetaDeg, DiffAltM, AileronCmd, ElevatorCmd);
        }
        else if (AbsAlt < 1500.f && FMath::Abs(CurrentThetaDeg) > ThetaActSpace)
        {
            ThetaActSpace = NavParams.ThetaActSpaceMin;
            SetRollPID(0.f, false, CurrentPhiDeg);
            if (bLog) UE_LOG(LogTemp, Warning,
                TEXT("[AP-MODE] PREC-SETTLE ThetaRef=%.1f Theta=%.1f DiffAlt=%.0f | Ail=%.3f Elv=%.3f"),
                ThetaRef, CurrentThetaDeg, DiffAltM, AileronCmd, ElevatorCmd);
        }
        else
        {
            ThetaActSpace = NavParams.ThetaActSpaceMax;
            SetRollPID(0.f, false, CurrentPhiDeg);
            if (bLog) UE_LOG(LogTemp, Warning,
                TEXT("[AP-MODE] PREC-LEVEL  ThetaRef=%.1f Theta=%.1f DiffAlt=%.0f | Ail=%.3f Elv=%.3f"),
                ThetaRef, CurrentThetaDeg, DiffAltM, AileronCmd, ElevatorCmd);
        }

        SetPitchPID(ThetaRef, CurrentThetaDeg);
        HeadActSpace = NavParams.HeadActSpaceMax;
    }

    // Speed-hold runs regardless of turn/precision mode (energy management).
    SetThrottlePID(CurrentSpeedMps, TargetSpeedMps);

    return {AileronCmd, ElevatorCmd, 0.f, ThrottleCmd};
}

void FAircraftAutopilot::SetRollPID(float RollRef, bool bUseSecondary,
                                     float CurrentPhiDeg)
{
    RollRef = FMath::Clamp(RollRef, -180.f, 180.f);
    const float Diff = RollCircleClip(RollRef - CurrentPhiDeg);
    FPID& Pid = bUseSecondary ? RollSecPID : RollPID;
    AileronCmd = FMath::Clamp(-Pid.Update(Diff), -1.f, 1.f);
}

void FAircraftAutopilot::SetPitchPID(float ThetaRef, float CurrentThetaDeg)
{
    ThetaRef = FMath::Clamp(ThetaRef, -90.f, 90.f);
    ElevatorCmd = FMath::Clamp(PitchPID.Update(ThetaRef - CurrentThetaDeg),
                               -1.f, 1.f);
}

void FAircraftAutopilot::SetThrottlePID(float CurrentSpeedMps, float TargetSpeedMps)
{
    if (TargetSpeedMps <= 0.f)
    {
        ThrottleCmd = -1.f;   // speed-hold disabled → caller uses external throttle
        return;
    }
    // FPID error = 0 - CurrentValue, so pass (V - V_target) → error = (V_target - V):
    // slower than target → positive error → throttle up.
    ThrottleCmd = FMath::Clamp(ThrottlePID.Update(CurrentSpeedMps - TargetSpeedMps),
                               0.f, 1.f);
}

float FAircraftAutopilot::DeltaHeading(float TargetDeg, float CurrentDeg)
{
    float D = FMath::Fmod(TargetDeg - CurrentDeg + 180.f, 360.f);
    if (D < 0.f) D += 360.f;
    return D - 180.f;
}

float FAircraftAutopilot::RollCircleClip(float D)
{
    if (D >  180.f) return D - 360.f;
    if (D <= -180.f) return D + 360.f;
    return D;
}