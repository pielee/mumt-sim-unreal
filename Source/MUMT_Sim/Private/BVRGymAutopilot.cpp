// BVRGymAutopilot.cpp
// Surface control = faithful port of BVRGym jsb_gym/utils/autopilot.py
// (AircraftPIDAutopilot) + control.py (PID). Speed-hold (autothrottle) is added
// on top — BVRGym left throttle as a "todo: velocity control" (fixed 0.49).
#include "BVRGymAutopilot.h"
#include "Math/UnrealMathUtility.h"

// ─── FPID (control.py PID) ──────────────────────────────────────────────────

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
    // Runtime state (Derivator, Integrator) intentionally preserved → the PID
    // persists across ticks (BVRGym). Keeps D = Kd*(error - prev_error) and the
    // autothrottle's trim integrator intact.
}

// ─── FAircraftAutopilot ────────────────────────────────────────────────────

FAircraftAutopilot::FAircraftAutopilot()
{
    // Surface gains from f16_config.py (aircraft_PID_Gains).
    RollPID    = {0.01f, 0.f, 0.9f, -0.2f,  0.2f};
    RollSecPID = {0.2f,  0.f, 0.2f, -1.0f,  1.0f};
    PitchPID   = {0.3f,  0.f, 1.0f, -1.0f,  1.0f};
    // Speed-hold (autothrottle): PI on airspeed error. The integrator provides the
    // steady-state trim throttle (Ki*IntegMax = 0.004*250 = 1.0). Output [0,1].
    ThrottlePID = {0.02f, 0.004f, 0.f, 0.f, 250.f};

    AltActSpace   = NavParams.AltActSpaceMin;
    HeadActSpace  = NavParams.HeadActSpaceMin;
    ThetaActSpace = NavParams.ThetaActSpaceMin;
}

// Surface control = direct port of AircraftPIDAutopilot.get_control_input.
// Throttle = added speed-hold (SetThrottlePID).
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

    if (AbsHead >= HeadActSpace && AbsAlt <= AltActSpace)
    {
        // ── Hard-turn mode ────────────────────────────────────────────────
        // Larger altitude action space while turning.
        AltActSpace = NavParams.AltActSpaceMax;

        // Bank hard toward the heading.
        const float RollRotDir = (DiffHeadDeg >= 0.f) ? 1.f : -1.f;
        SetRollPID(RollRotDir * NavParams.RollMax, /*bUseSecondary=*/false, CurrentPhiDeg);

        // DIRECT elevator pull through the turn (ease off near the roll limit).
        if (NavParams.RollMax - FMath::Abs(CurrentPhiDeg) < 30.f)
            ElevatorCmd = -0.3f;
        else
            ElevatorCmd = -0.9f;

        // Stay-in-turn threshold = Head_act_space_min. Clamp <= Max so a level's
        // NavParams override with Min > Max can't invert the hysteresis (which
        // would flicker hard-turn off every tick → aircraft never turns).
        HeadActSpace = FMath::Min(NavParams.HeadActSpaceMin, NavParams.HeadActSpaceMax);
    }
    else
    {
        // ── Precision mode ────────────────────────────────────────────────
        AltActSpace = NavParams.AltActSpaceMin;

        float ThetaRef = FMath::RadiansToDegrees(FMath::Atan2(DiffAltM, NavParams.TanRef));
        ThetaRef = FMath::Clamp(ThetaRef, NavParams.DiveThetaMax, NavParams.ClimbThetaMax);

        if (AbsAlt > 1500.f)
        {
            // Big altitude error → climb/dive wings-level.
            ThetaActSpace = NavParams.ThetaActSpaceMax;
            SetRollPID(0.f, false, CurrentPhiDeg);
        }
        else if (AbsAlt < 1500.f && FMath::Abs(CurrentThetaDeg) > ThetaActSpace)
        {
            // Settle pitch back toward level before correcting heading.
            ThetaActSpace = NavParams.ThetaActSpaceMin;
            SetRollPID(0.f, false, CurrentPhiDeg);
        }
        else
        {
            // Fine heading hold: roll toward the target heading
            // (proportional ×3, or the secondary PID for small errors).
            ThetaActSpace = NavParams.ThetaActSpaceMax;
            if (AbsHead < 10.f)
                SetRollPID(DiffHeadDeg,        /*bUseSecondary=*/true,  CurrentPhiDeg);
            else
                SetRollPID(DiffHeadDeg * 3.f,  /*bUseSecondary=*/false, CurrentPhiDeg);
        }

        SetPitchPID(ThetaRef, CurrentThetaDeg);
        HeadActSpace = NavParams.HeadActSpaceMax;
    }

    // Speed-hold runs regardless of turn/precision mode (energy management add-on).
    SetThrottlePID(CurrentSpeedMps, TargetSpeedMps);

    return { AileronCmd, ElevatorCmd, 0.f, ThrottleCmd };
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
    // FPID error = 0 - CurrentValue, so passing (V - V_target) gives
    // error = (V_target - V): slower than target → positive error → throttle up.
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
