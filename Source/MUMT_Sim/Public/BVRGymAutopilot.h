// BVRGymAutopilot.h
// Outer-loop PID autopilot ported from BVRGym (autopilot.py, control.py,
// navigation.py, f16_config.py). Exposed as USTRUCT so gains can be edited
// in the Details panel without recompilation.
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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Autopilot|Nav")
    float BankGain = 0.8f;

};

// ─── Autopilot output ──────────────────────────────────────────────────────

struct FAutopilotOutput
{
    float Aileron  = 0.f;  // [-1, 1]
    float Elevator = 0.f;  // [-1, 1]
    float Rudder   = 0.f;  // always 0
};

// ─── FAircraftAutopilot ────────────────────────────────────────────────────

class MUMT_SIM_API FAircraftAutopilot
{
public:
    FAircraftAutopilot();

    // Call at fixed 60 Hz.
    //   DiffHeadDeg     = DeltaHeading(TargetHeading, CurrentPsiDeg)  → [-180,180]
    //   DiffAltM        = TargetAltM - CurrentAltM
    //   CurrentPhiDeg   = AircraftState.LocalEulerAngles.Roll
    //   CurrentThetaDeg = AircraftState.LocalEulerAngles.Pitch
    FAutopilotOutput GetControlInput(
        float DiffHeadDeg,
        float DiffAltM,
        float CurrentPhiDeg,
        float CurrentThetaDeg);

    // ((target - current + 180) % 360) - 180  →  [-180, 180]
    static float DeltaHeading(float TargetDeg, float CurrentDeg);

    // Synced from UPROPERTY structs before each tick
    FPID RollPID;
    FPID RollSecPID;
    FPID PitchPID;
    FAutopilotNavParams NavParams;

private:
    float AltActSpace;
    float HeadActSpace;
    float ThetaActSpace;
    float AileronCmd  = 0.f;
    float ElevatorCmd = 0.f;

    void SetRollPID (float RollRef,  bool bUseSecondary, float CurrentPhiDeg);
    void SetPitchPID(float ThetaRef, float CurrentThetaDeg);
    static float RollCircleClip(float D);
};