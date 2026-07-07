#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HealthComponent.generated.h"

class UJSBSimMovementComponent;

// 소유자 폰 BP가 바인딩하는 상태 전이 이벤트 (컴포넌트 디테일 패널의 이벤트 섹션 / Assign 노드).
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnStartFalling);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnCrashed);

UENUM(BlueprintType)
enum class ETeam : uint8
{
    Manned      UMETA(DisplayName = "Manned"),
    FriendlyUAV UMETA(DisplayName = "Friendly UAV"),
    Enemy       UMETA(DisplayName = "Enemy")
};

UENUM(BlueprintType)
enum class EAircraftLifeState : uint8
{
    Alive,
    Falling,
    Crashed
};

UCLASS(ClassGroup=(MUMT), meta=(BlueprintSpawnableComponent))
class MUMT_SIM_API UHealthComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UHealthComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Health")
    float MaxHP = 100.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Health")
    ETeam Team = ETeam::Enemy;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Health")
    float CrashAGLThresholdFt = 10.f;

    // 기체 머리 위 HP 디버그 텍스트 (PIE 화면 표시, uasset 불필요)
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Health|Debug")
    bool bShowHPDebug = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Health|Debug")
    float DebugTextHeightCm = 400.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MUMT|Health|Debug")
    float DebugTextScale = 1.5f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MUMT|Health")
    float CurrentHP = 0.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MUMT|Health")
    EAircraftLifeState LifeState = EAircraftLifeState::Alive;

    UFUNCTION(BlueprintCallable, Category = "MUMT|Health")
    void ApplyDamage(float Amount, AActor* DamageInstigator);

    UFUNCTION(BlueprintPure, Category = "MUMT|Health")
    bool IsAlive() const { return LifeState == EAircraftLifeState::Alive; }

    UPROPERTY(BlueprintAssignable, Category = "MUMT|Health")
    FOnStartFalling OnStartFalling;

    UPROPERTY(BlueprintAssignable, Category = "MUMT|Health")
    FOnCrashed OnCrashed;

protected:
    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;

private:
    void EnterFalling();
    void EnterCrashed();

    UPROPERTY()
    TObjectPtr<UJSBSimMovementComponent> JSBSim;

    double HardoverAileron  = 0.0;
    double HardoverElevator = 0.0;
    double HardoverRudder   = 0.0;
};
