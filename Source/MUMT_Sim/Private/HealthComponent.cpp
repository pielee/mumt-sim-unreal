#include "HealthComponent.h"
#include "JSBSimMovementComponent.h"

UHealthComponent::UHealthComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UHealthComponent::BeginPlay()
{
    Super::BeginPlay();
    CurrentHP = MaxHP;
    LifeState = EAircraftLifeState::Alive;
    JSBSim = GetOwner()->FindComponentByClass<UJSBSimMovementComponent>();
}

void UHealthComponent::ApplyDamage(float Amount, AActor* DamageInstigator)
{
    if (LifeState != EAircraftLifeState::Alive || Amount <= 0.f)
    {
        return;
    }

    CurrentHP = FMath::Max(0.f, CurrentHP - Amount);

    if (CurrentHP <= 0.f)
    {
        EnterFalling();
    }
}

void UHealthComponent::EnterFalling()
{
    LifeState = EAircraftLifeState::Falling;

    const double Sign = FMath::RandBool() ? 1.0 : -1.0;
    HardoverAileron  = Sign * FMath::RandRange(0.5, 0.9);
    HardoverElevator = FMath::RandRange(-0.4, -0.2);
    HardoverRudder   = Sign * FMath::RandRange(0.3, 0.5);

    if (JSBSim)
    {
        for (FEngineCommand& Engine : JSBSim->EngineCommands)
        {
            Engine.CutOff   = true;
            Engine.Throttle = 0.0;
        }
    }

    OnStartFalling.Broadcast();
}

void UHealthComponent::EnterCrashed()
{
    LifeState = EAircraftLifeState::Crashed;
    OnCrashed.Broadcast();
}

void UHealthComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                     FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (LifeState != EAircraftLifeState::Falling || !JSBSim)
    {
        return;
    }

    JSBSim->Commands.Aileron  = HardoverAileron;
    JSBSim->Commands.Elevator = HardoverElevator;
    JSBSim->Commands.Rudder   = HardoverRudder;

    if (JSBSim->AircraftState.AltitudeAGLFt < CrashAGLThresholdFt)
    {
        EnterCrashed();
    }
}
