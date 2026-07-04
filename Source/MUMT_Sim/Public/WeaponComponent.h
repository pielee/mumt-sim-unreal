#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "WeaponComponent.generated.h"

class UHealthComponent;

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

    // --- BP 연출 훅 ---
    UFUNCTION(BlueprintImplementableEvent, Category = "MUMT|Weapon")
    void OnGunFiringChanged(bool bFiring);

    UFUNCTION(BlueprintImplementableEvent, Category = "MUMT|Weapon")
    void OnMissileFired();

protected:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;

private:
    /** 원추(사거리 RangeM, 반각 ConeDeg) 안에서 가장 가까운 살아있는 적 팀 폰의
        HealthComponent를 반환. 없으면 nullptr. OutDist에 그 거리(cm) 반환 */
    UHealthComponent* FindNearestTargetInCone(float RangeM, float ConeDeg, float& OutDist) const;

    float AmmoSpentAccumulator = 0.f;
    int64 LastConsumedFireId = -1;
};
