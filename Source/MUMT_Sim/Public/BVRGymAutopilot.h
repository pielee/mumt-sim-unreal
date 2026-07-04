// BVRGymAutopilot.h
// Outer-loop PID autopilot ported from BVRGym (autopilot.py, control.py,
// navigation.py, f16_config.py). Exposed as USTRUCT so gains can be edited
// in the Details panel without recompilation.
//
// Surface control is a faithful port of BVRGym's AircraftPIDAutopilot.get_control_input:
//   - hard-turn  : bank to RollMax + a DIRECT elevator pull (-0.9, or -0.3 near
//                  the roll limit) — NOT a pitch-PID to altitude.
//   - precision  : pitch-PID to a climb/dive angle, and roll toward the target
//                  heading (proportional ×3, or the secondary PID for |Δhead|<10).
// Speed control (autothrottle) is ADDED on top of the original (BVRGym left it as
// a "todo: velocity control", using a fixed 0.49): a speed-hold PI drives throttle
// to TargetSpeedMps. TargetSpeedMps<=0 disables it (Throttle output = -1 → caller
// uses the open-loop setpoint throttle).
#pragma once

#include "CoreMinimal.h"
#include "BVRGymAutopilot.generated.h"

// ─── PID ─────────────────────────────────────────────────────────────────────
// BVRGym control.py PID semantics (SetPoint always 0):
//   error      = 0 - current_value
//   P          = Kp * error
//   D          = Kd * (error - Derivator);  Derivator = error
//   Integrator = clamp(Integrator + error, IntegMin, IntegMax)
//   I          = Ki * Integrator
//   return P + I + D
// The PID object persists across ticks (Derivator/Integrator accumulate) exactly
// as in BVRGym, so the derivative term is Kd*(error - previous_error) and the
// autothrottle integrator carries the steady-state trim throttle.

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
    // (Derivator AND Integrator) — the BVRGym PID persists across ticks, so this
    // syncs live-tuned gains WITHOUT breaking the derivative term or wiping the
    // autothrottle's trim integrator.
    void  SetGains(const FPID& Cfg);
};

// ─── Navigation parameters ─────────────────────────────────────────────────

USTRUCT(BlueprintType)
struct MUMT_SIM_API FAutopilotNavParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float AltActSpaceMin   = 1000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float AltActSpaceMax   = 2000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float HeadActSpaceMin  =   10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float HeadActSpaceMax  =   35.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float ThetaActSpaceMin =   10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float ThetaActSpaceMax =   30.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float RollMax  =   80.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float TanRef   = 2000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float DiveThetaMax  = -45.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float ClimbThetaMax =  45.f;

};

// ─── Autopilot output ──────────────────────────────────────────────────────

struct FAutopilotOutput
{
    float Aileron  = 0.f;  // [-1, 1]
    float Elevator = 0.f;  // [-1, 1]
    float Rudder   = 0.f;  // always 0 (BVRGym aircraft autopilot leaves rudder at 0)
    float Throttle = -1.f; // [0,1] from speed-hold; <0 = speed control off → use external throttle
};

// ─── FAircraftAutopilot ────────────────────────────────────────────────────

class MUMT_SIM_API FAircraftAutopilot
{
public:
    FAircraftAutopilot();

    // Call at fixed 60 Hz. Surface control = faithful BVRGym get_control_input;
    // throttle = added speed-hold (autothrottle).
    //   DiffHeadDeg     = DeltaHeading(TargetHeading, CurrentPsiDeg)  → [-180,180]
    //   DiffAltM        = TargetAltM - CurrentAltM
    //   CurrentPhiDeg   = AircraftState.LocalEulerAngles.Roll
    //   CurrentThetaDeg = AircraftState.LocalEulerAngles.Pitch
    //   CurrentSpeedMps = current airspeed (m/s)
    //   TargetSpeedMps  = desired airspeed; <=0 disables speed-hold (Throttle output = -1)
    FAutopilotOutput GetControlInput(
        float DiffHeadDeg,
        float DiffAltM,
        float CurrentPhiDeg,
        float CurrentThetaDeg,
        float CurrentSpeedMps = 0.f,
        float TargetSpeedMps  = 0.f);

    // ((target - current + 180) % 360) - 180  →  [-180, 180]
    static float DeltaHeading(float TargetDeg, float CurrentDeg);

    // Synced from UPROPERTY structs before each tick
    FPID RollPID;
    FPID RollSecPID;
    FPID PitchPID;
    FPID ThrottlePID;              // speed-hold (autothrottle): drives throttle to hold TargetSpeed
    FAutopilotNavParams NavParams;

private:
    float AltActSpace;
    float HeadActSpace;
    float ThetaActSpace;
    float AileronCmd  = 0.f;
    float ElevatorCmd = 0.f;
    float ThrottleCmd = -1.f;

    void SetRollPID (float RollRef,  bool bUseSecondary, float CurrentPhiDeg);
    void SetPitchPID(float ThetaRef, float CurrentThetaDeg);
    void SetThrottlePID(float CurrentSpeedMps, float TargetSpeedMps);
    static float RollCircleClip(float D);
};
