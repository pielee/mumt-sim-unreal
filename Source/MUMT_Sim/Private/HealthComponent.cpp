#include "HealthComponent.h"
#include "DrawDebugHelpers.h"
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

    // 아군 사격 면제: 가해자 팀을 알 수 있으면 적대 관계일 때만 대미지.
    // 가해자 정보가 없는 대미지(환경 요인 등)는 그대로 적용.
    if (DamageInstigator)
    {
        if (const UHealthComponent* InstigatorHealth =
                DamageInstigator->FindComponentByClass<UHealthComponent>())
        {
            if (!AreHostile(Team, InstigatorHealth->Team))
            {
                return;
            }
        }
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

    // 기체 머리 위 HP 표시 — 매 틱 1프레임짜리 텍스트를 다시 그려 따라다니게 함.
    if (bShowHPDebug)
    {
        FColor Color = FColor::Green;
        FString Suffix;
        if (LifeState == EAircraftLifeState::Falling)
        {
            Color = FColor::Orange;  Suffix = TEXT("  FALLING");
        }
        else if (LifeState == EAircraftLifeState::Crashed)
        {
            Color = FColor::Red;     Suffix = TEXT("  CRASHED");
        }
        else if (CurrentHP <= MaxHP * 0.5f)
        {
            Color = FColor::Yellow;  // 피격 누적 경고
        }
        // 기체명 접두: 라벨이 2개로 보일 때 "다른 기체의 라벨"인지 "같은 기체에 컴포넌트가
        // 중복 부착"인지 즉시 판별하기 위함 (2026-07-10 HP 이중상 진단).
        const FString Label = FString::Printf(TEXT("[%s] HP %.0f / %.0f%s"),
            GetOwner() ? *GetOwner()->GetActorNameOrLabel() : TEXT("?"), CurrentHP, MaxHP, *Suffix);
        DrawDebugString(GetWorld(), FVector(0.f, 0.f, DebugTextHeightCm), Label,
                        GetOwner(), Color, /*Duration=*/0.f, /*bDrawShadow=*/true, DebugTextScale);
    }

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
