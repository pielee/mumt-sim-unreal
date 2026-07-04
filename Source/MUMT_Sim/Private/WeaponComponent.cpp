#include "WeaponComponent.h"
#include "HealthComponent.h"
#include "GameFramework/Pawn.h"
#include "EngineUtils.h"

UWeaponComponent::UWeaponComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UWeaponComponent::SetGunFiring(bool bFiring)
{
    const bool bNewState = bFiring && (bUnlimitedGunAmmo || GunAmmo > 0);
    if (bNewState == bGunFiring)
    {
        return;
    }
    bGunFiring = bNewState;
    OnGunFiringChanged(bGunFiring);
}

bool UWeaponComponent::FireMissile()
{
    if (MissileCount <= 0)
    {
        return false;
    }
    --MissileCount;

    // 발사 순간 1회 WEZ 판정: 넓은 원추, 가장 가까운 적기 1대
    float Dist = 0.f;
    if (UHealthComponent* Target = FindNearestTargetInCone(MissileRangeM, MissileConeHalfAngleDeg, Dist))
    {
        Target->ApplyDamage(MissileDamage, GetOwner());
    }
    // 명중 여부와 무관하게 연출은 발사 (허공 발사 = 미사일만 소모)
    OnMissileFired();
    return true;
}

void UWeaponComponent::ConsumeMissileFireId(int64 FireId)
{
    if (FireId >= 0 && FireId != LastConsumedFireId)
    {
        LastConsumedFireId = FireId;
        FireMissile();
    }
}

void UWeaponComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                     FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!bGunFiring)
    {
        return;
    }

    // 기총: 매 틱, 좁은 원추 안 가장 가까운 적기 1대에 거리감쇠 대미지
    float Dist = 0.f;
    if (UHealthComponent* Target = FindNearestTargetInCone(GunRangeM, GunConeHalfAngleDeg, Dist))
    {
        const float RangeFactor = 1.f - (Dist / (GunRangeM * 100.f));
        Target->ApplyDamage(GunDamagePerSec * DeltaTime * RangeFactor, GetOwner());
    }

    // 탄약 소모 (무제한 스위치가 꺼져 있을 때만)
    if (!bUnlimitedGunAmmo)
    {
        AmmoSpentAccumulator += GunRoundsPerSec * DeltaTime;
        const int32 RoundsSpent = FMath::FloorToInt32(AmmoSpentAccumulator);
        if (RoundsSpent > 0)
        {
            AmmoSpentAccumulator -= RoundsSpent;
            GunAmmo = FMath::Max(0, GunAmmo - RoundsSpent);
            if (GunAmmo == 0)
            {
                SetGunFiring(false);
            }
        }
    }
}

UHealthComponent* UWeaponComponent::FindNearestTargetInCone(float RangeM, float ConeDeg, float& OutDist) const
{
    AActor* OwnerActor = GetOwner();
    UHealthComponent* MyHealth = OwnerActor->FindComponentByClass<UHealthComponent>();
    if (!MyHealth || !MyHealth->IsAlive())
    {
        return nullptr;   // 내 팀을 모르거나 내가 죽었으면 판정 불가
    }

    const FVector MuzzleLoc = OwnerActor->GetActorLocation();
    const FVector Forward   = OwnerActor->GetActorForwardVector();
    const float RangeUE     = RangeM * 100.f;
    const float CosCone     = FMath::Cos(FMath::DegreesToRadians(ConeDeg));

    UHealthComponent* Nearest = nullptr;
    float NearestDist = RangeUE + 1.f;

    for (TActorIterator<APawn> It(GetWorld()); It; ++It)
    {
        APawn* Target = *It;
        if (Target == OwnerActor)
        {
            continue;
        }

        UHealthComponent* TargetHealth = Target->FindComponentByClass<UHealthComponent>();
        if (!TargetHealth || !TargetHealth->IsAlive())
        {
            continue;
        }
        if (TargetHealth->Team == MyHealth->Team)
        {
            continue;
        }

        const FVector ToTarget = Target->GetActorLocation() - MuzzleLoc;
        const float Dist = ToTarget.Size();
        if (Dist > RangeUE || Dist < KINDA_SMALL_NUMBER)
        {
            continue;
        }

        const float CosAngle = FVector::DotProduct(Forward, ToTarget / Dist);
        if (CosAngle < CosCone)
        {
            continue;
        }

        if (Dist < NearestDist)   // 원추 통과자 중 최소 거리 갱신
        {
            NearestDist = Dist;
            Nearest = TargetHealth;
        }
    }

    OutDist = NearestDist;
    return Nearest;
}
