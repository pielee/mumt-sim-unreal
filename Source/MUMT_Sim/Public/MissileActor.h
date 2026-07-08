#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MissileActor.generated.h"

class UHealthComponent;
class UStaticMeshComponent;

// 폭발(명중/자폭) 시 BP 연출용 이벤트
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnMissileDetonated);

/**
 * 호밍 미사일 — FireMissile()의 "발사 순간 즉시 판정"을 대체하는 실제 발사체.
 *
 * 발사 시 UWeaponComponent가 스폰하고 표적 폰을 넘긴다. 매 틱 표적을 향해
 * 선회(회전율 제한 순수추종)하며 비행하고, ProximityFuseM 이내로 접근하면
 * 표적의 UHealthComponent에 Damage를 가하고 폭발한다. 표적이 회피 기동으로
 * 회전율 한계를 이겨내면 실제로 빗나가며, 수명이 다하면 자폭(소멸)한다.
 * 표적이 없거나(허공 발사) 도중에 파괴되면 직진 후 소멸.
 *
 * 시각화: sm_rocket 스태틱 메시를 런타임 로드해 부착(있으면), 및
 * bDrawDebugTrail 디버그 궤적 — BP 배선 없이도 눈에 보이게.
 */
UCLASS()
class MUMT_SIM_API AMissileActor : public AActor
{
    GENERATED_BODY()

public:
    AMissileActor();

    // ── 비행 특성 (스폰 후에도 Details에서 튜닝 가능) ──
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MUMT|Missile")
    float SpeedMps = 650.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MUMT|Missile")
    float TurnRateDps = 25.f;          // 회전율 한계 — 낮을수록 회피가 쉬움

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MUMT|Missile")
    float ProximityFuseM = 20.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MUMT|Missile")
    float Damage = 40.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MUMT|Missile")
    float LifetimeS = 20.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MUMT|Missile")
    bool bDrawDebugTrail = true;

    // ── 발사자가 설정 ──
    TWeakObjectPtr<APawn>  TargetPawn;     // null = 무유도(직진)
    TWeakObjectPtr<AActor> Shooter;        // ApplyDamage의 instigator

    UPROPERTY(BlueprintAssignable, Category = "MUMT|Missile")
    FOnMissileDetonated OnDetonated;

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;

private:
    void Detonate(bool bHit);

    UPROPERTY()
    TObjectPtr<UStaticMeshComponent> Mesh;

    FVector FlightDir = FVector::ForwardVector;   // 단위 비행 방향 (월드)
    float   LifeLeft  = 0.f;
};
