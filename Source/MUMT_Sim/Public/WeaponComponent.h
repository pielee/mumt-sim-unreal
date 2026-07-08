#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "WeaponComponent.generated.h"

class UHealthComponent;

// 소유자 폰 BP가 바인딩하는 연출 이벤트 (컴포넌트 디테일 패널의 이벤트 섹션 / Assign 노드).
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnGunFiringChanged, bool, bFiring);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnMissileFired);

UCLASS(ClassGroup=(MUMT), meta=(BlueprintSpawnableComponent))
class MUMT_SIM_API UWeaponComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UWeaponComponent();

    // --- 기총 ---
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Gun")
    float GunRangeM = 1500.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Gun")
    float GunConeHalfAngleDeg = 2.5f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Gun")
    float GunDamagePerSec = 20.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Gun")
    bool bUnlimitedGunAmmo = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Gun")
    int32 GunAmmo = 511;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Gun")
    float GunRoundsPerSec = 100.f;

    // --- 미사일 ---
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Missile")
    float MissileRangeM = 8000.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Missile")
    float MissileConeHalfAngleDeg = 15.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Missile")
    float MissileDamage = 40.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Missile")
    int32 MissileCount = 3;

    // --- 런타임 상태 ---
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MUMT|Gun")
    bool bGunFiring = false;

    // --- 외부 인터페이스 ---
    UFUNCTION(BlueprintCallable, Category = "MUMT|Weapon")
    void SetGunFiring(bool bFiring);

    UFUNCTION(BlueprintCallable, Category = "MUMT|Weapon")
    bool FireMissile();

    UFUNCTION(BlueprintCallable, Category = "MUMT|Weapon")
    void ConsumeMissileFireId(int64 FireId);

    /** 이번 발사(OnMissileFired)의 호밍 표적. BP_rocket이 이 폰을 추적하면 된다.
        null이면 표적 미획득(허공 발사) → BP는 직진 연출. */
    UFUNCTION(BlueprintPure, Category = "MUMT|Weapon")
    APawn* GetLastMissileTarget() const { return LastMissileTarget.Get(); }

    /** BP_rocket이 표적에 명중했을 때 호출 — C++이 대미지(MissileDamage)를 적용한다.
        연출·타이밍은 BP, 판정은 C++이라는 분리를 유지한다. */
    UFUNCTION(BlueprintCallable, Category = "MUMT|Weapon")
    void ReportMissileHit(AActor* HitActor);

    /** 기체 월드 속도 (cm/s). JSBSim 플러그인이 폰 트랜스폼을 직접 세팅해
        GetVelocity()가 0을 반환하므로, 위치 미분으로 계산한 속도를 제공한다.
        BP가 트레이서/로켓 스폰 시 InheritedVelocity로 전달하는 용도. */
    UFUNCTION(BlueprintPure, Category = "MUMT|Weapon")
    FVector GetOwnerWorldVelocity() const { return OwnerWorldVelocity; }

    // --- BP 연출 훅 (소유자 폰 BP에서 바인딩) ---
    UPROPERTY(BlueprintAssignable, Category = "MUMT|Weapon")
    FOnGunFiringChanged OnGunFiringChanged;

    UPROPERTY(BlueprintAssignable, Category = "MUMT|Weapon")
    FOnMissileFired OnMissileFired;

protected:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;

private:
    /** 원추(사거리 RangeM, 반각 ConeDeg) 안에서 가장 가까운 살아있는 적 팀 폰의
        HealthComponent를 반환. 없으면 nullptr. OutDist에 그 거리(cm) 반환 */
    UHealthComponent* FindNearestTargetInCone(float RangeM, float ConeDeg, float& OutDist) const;

    float AmmoSpentAccumulator = 0.f;
    int64 LastConsumedFireId = -1;

    TWeakObjectPtr<APawn> LastMissileTarget;   // 이번 발사의 호밍 표적 (BP가 조회)

    FVector PrevOwnerLocation = FVector::ZeroVector;
    FVector OwnerWorldVelocity = FVector::ZeroVector;   // cm/s
    bool bHasPrevLocation = false;
};
