// BVRGymAutopilot.h
// NOTE: The BVRGym surface autopilot (FAircraftAutopilot / FAutopilotOutput /
// FAutopilotNavParams) was removed on 2026-07-07 when the UAV surface controller
// was swapped for Controller_CY (StickController). This header now exposes ONLY
// the generic FPID, still used by the autothrottle (speed-hold) in
// UDPControlReceiver::ApplyAutopilotToPawn. Filename kept to avoid module churn.
#pragma once

#include "CoreMinimal.h"
#include "BVRGymAutopilot.generated.h"

// ─── PID ─────────────────────────────────────────────────────────────────────
// control.py PID semantics (SetPoint always 0):
//   error      = 0 - current_value
//   P          = Kp * error
//   D          = Kd * (error - Derivator);  Derivator = error
//   Integrator = clamp(Integrator + error, IntegMin, IntegMax)
//   I          = Ki * Integrator
//   return P + I + D
// The PID object persists across ticks (Derivator/Integrator accumulate), so the
// derivative term is Kd*(error - previous_error) and the autothrottle integrator
// carries the steady-state trim throttle.

USTRUCT(BlueprintType)
struct MUMT_SIM_API FPID
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PID")
    float Kp = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PID")
    float Ki = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PID")
    float Kd = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PID")
    float IntegMin = -1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="PID")
    float IntegMax =  1.f;

    // Runtime state — not persisted, reset via Reset()
    float Derivator  = 0.f;
    float Integrator = 0.f;

    float Update(float CurrentValue);
    void  Reset();
    // Copy gains only (Kp/Ki/Kd/IntegMin/IntegMax) and KEEP the runtime state
    // (Derivator AND Integrator) — the PID persists across ticks, so this syncs
    // live-tuned gains WITHOUT breaking the derivative term or wiping the
    // autothrottle's trim integrator.
    void  SetGains(const FPID& Cfg);
};
