#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "HealthComponent.generated.h"

class UJSBSimMovementComponent;

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

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MUMT|Health")
    float CurrentHP = 0.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MUMT|Health")
    EAircraftLifeState LifeState = EAircraftLifeState::Alive;

    UFUNCTION(BlueprintCallable, Category = "MUMT|Health")
    void ApplyDamage(float Amount, AActor* DamageInstigator);

    UFUNCTION(BlueprintPure, Category = "MUMT|Health")
    bool IsAlive() const { return LifeState == EAircraftLifeState::Alive; }

    UFUNCTION(BlueprintImplementableEvent, Category = "MUMT|Health")
    void OnStartFalling();

    UFUNCTION(BlueprintImplementableEvent, Category = "MUMT|Health")
    void OnCrashed();

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
