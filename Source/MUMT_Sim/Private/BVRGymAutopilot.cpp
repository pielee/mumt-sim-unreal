// BVRGymAutopilot.cpp
// NOTE: The BVRGym surface autopilot (FAircraftAutopilot) was removed on
// 2026-07-07 when the UAV surface controller was swapped for Controller_CY
// (StickController). This file now holds ONLY the generic FPID, which the
// autothrottle (speed-hold) in UDPControlReceiver still uses.
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
    // persists across ticks. Keeps D = Kd*(error - prev_error) and the
    // autothrottle's trim integrator intact.
}
