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
    OnGunFiringChanged.Broadcast(bGunFiring);
}

bool UWeaponComponent::FireMissile()
{
    if (MissileCount <= 0)
    {
        return false;
    }
    --MissileCount;

    // WEZ 원추 = 발사 허가·표적 지정 조건. 원추 안 최근접 적기를 이번 발사의
    // 호밍 표적으로 저장하고 OnMissileFired를 발화한다. 실제 미사일 비행(호밍
    // 애니메이션)은 BP_rocket이 담당 — BP는 GetLastMissileTarget()으로 이 표적을
    // 받아 추적하고, 명중 시 ReportMissileHit(적기)로 C++에 통보한다(대미지는 C++).
    // 원추 안 표적 없으면 무유도(BP가 직진 연출, 허공 발사 = 미사일만 소모).
    float Dist = 0.f;
    if (UHealthComponent* Target = FindNearestTargetInCone(MissileRangeM, MissileConeHalfAngleDeg, Dist))
    {
        LastMissileTarget = Cast<APawn>(Target->GetOwner());
    }
    else
    {
        LastMissileTarget = nullptr;
    }

    OnMissileFired.Broadcast();   // BP: 로켓 스폰 + 속도상속 + (표적 있으면) 호밍
    return true;
}

void UWeaponComponent::ReportMissileHit(AActor* HitActor)
{
    // BP_rocket이 표적에 명중했을 때 호출 — 대미지 판정은 C++이 소유.
    if (!HitActor)
    {
        return;
    }
    if (UHealthComponent* Health = HitActor->FindComponentByClass<UHealthComponent>())
    {
        Health->ApplyDamage(MissileDamage, GetOwner());
    }
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

    // 기체 월드 속도 (위치 미분) — 발사 여부와 무관하게 항상 갱신.
    // JSBSim이 폰 트랜스폼을 직접 세팅해 GetVelocity()가 0이므로 여기서 계산한다.
    const FVector Now = GetOwner()->GetActorLocation();
    if (bHasPrevLocation && DeltaTime > KINDA_SMALL_NUMBER)
    {
        OwnerWorldVelocity = (Now - PrevOwnerLocation) / DeltaTime;
    }
    PrevOwnerLocation = Now;
    bHasPrevLocation = true;

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
