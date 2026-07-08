#include "MissileActor.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Pawn.h"
#include "HealthComponent.h"

AMissileActor::AMissileActor()
{
    PrimaryActorTick.bCanEverTick = true;

    Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
    Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);   // 판정은 근접신관이 함
    SetRootComponent(Mesh);
}

void AMissileActor::BeginPlay()
{
    Super::BeginPlay();
    LifeLeft  = LifetimeS;
    FlightDir = GetActorForwardVector();

    // 기존 로켓 메시를 런타임 로드해 부착 (없어도 동작 — 디버그 궤적으로 가시화)
    if (UStaticMesh* Rocket = LoadObject<UStaticMesh>(
            nullptr, TEXT("/Game/F16Control/meshes/sm_rocket.sm_rocket")))
    {
        Mesh->SetStaticMesh(Rocket);
    }
}

void AMissileActor::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    LifeLeft -= DeltaSeconds;
    if (LifeLeft <= 0.f)
    {
        Detonate(false);          // 수명 만료 — 자폭(빗나감)
        return;
    }

    // 유도: 표적 방향으로 회전율 제한 순수추종
    if (TargetPawn.IsValid())
    {
        const FVector ToTarget = TargetPawn->GetActorLocation() - GetActorLocation();
        const float DistM = ToTarget.Size() / 100.f;
        if (DistM < ProximityFuseM)
        {
            Detonate(true);
            return;
        }
        FlightDir = FMath::VInterpNormalRotationTo(
            FlightDir, ToTarget.GetSafeNormal(), DeltaSeconds, TurnRateDps);
    }

    const FVector OldLoc = GetActorLocation();
    const FVector NewLoc = OldLoc + FlightDir * (SpeedMps * 100.f * DeltaSeconds);
    SetActorLocationAndRotation(NewLoc, FlightDir.Rotation());

    if (bDrawDebugTrail)
        DrawDebugLine(GetWorld(), OldLoc, NewLoc, FColor::Orange, false, 1.5f, 0, 6.f);
}

void AMissileActor::Detonate(bool bHit)
{
    if (bHit && TargetPawn.IsValid())
    {
        if (UHealthComponent* Health = TargetPawn->FindComponentByClass<UHealthComponent>())
        {
            Health->ApplyDamage(Damage, Shooter.Get());
        }
        if (bDrawDebugTrail)
            DrawDebugSphere(GetWorld(), GetActorLocation(), 1500.f, 12, FColor::Red, false, 2.f);
    }
    OnDetonated.Broadcast();
    Destroy();
}
