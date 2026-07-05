# 무장/HP/격추 시스템 — 변경 파일 전체 수집

문서 작성용 원본. Phase 1(컴포넌트) + Phase 3(무장 프로토콜) + 델리게이트 수정 + HP 디버그 표시 + dogfight 시나리오.
`.uasset`(블루프린트)은 제외 — 사용자가 에디터에서 별도 배선.

---

## 1. 커밋 목록

### mumt-sim-unreal (MUMT_Sim) — origin: pielee/MUMT_Sim (미푸시)
| 해시 | 메시지 |
|---|---|
| `40278ee` | Phase 1: UHealthComponent + UWeaponComponent (WEZ gun/missile, falling state machine) |
| `30778fc` | Phase 3: weapon command parsing, combat state broadcast, owner velocity provider |
| `580d501` | Fix: expose weapon/health events as BlueprintAssignable delegates |
| `271c45e` | Debug: on-screen HP display above aircraft (DrawDebugString, no uasset needed) |

### mumt-ros-bridge — origin: pielee/mumt-ros-bridge
| 해시 | 메시지 |
|---|---|
| `f0ff10d` | Phase 3: gun_firing + missile_fire_id in AircraftSetpoint |

### mumt-bt (py_bt_ros) — origin: pielee/mumt-bt
| 해시 | 메시지 |
|---|---|
| `eef0545` | Scenario: 1v1 UAV dogfight (engage node, pure pursuit + WEZ trigger) |

> 참고: 같은 세션에서 만든 관련 시나리오 2종(`45772d8` weapons_test, `d0fbef5` target_practice)은
> 이 문서 범위(dogfight)에 미포함. 필요 시 별도 수집 가능.

---

## 2. 파일 인벤토리

| 리포 | 경로 | 상태 | 변경 요약 |
|---|---|---|---|
| sim-unreal | Source/MUMT_Sim/Public/HealthComponent.h | 신규 | HP/팀/수명상태 enum, ApplyDamage, 델리게이트(OnStartFalling/OnCrashed), HP 디버그 프로퍼티 |
| sim-unreal | Source/MUMT_Sim/Private/HealthComponent.cpp | 신규 | 대미지→Falling→Crashed 상태머신, 엔진 CutOff+조종면 하드오버, AGL 추락판정, 머리위 HP 텍스트 |
| sim-unreal | Source/MUMT_Sim/Public/WeaponComponent.h | 신규 | 기총/미사일 파라미터, SetGunFiring/FireMissile/ConsumeMissileFireId, GetOwnerWorldVelocity, 연출 델리게이트 |
| sim-unreal | Source/MUMT_Sim/Private/WeaponComponent.cpp | 신규 | WEZ 원추 판정(최근접 적기 1대), 거리감쇠 기총, 미사일 에지발사, 위치미분 속도 |
| sim-unreal | Source/MUMT_Sim/Public/UDPControlReceiver.h | 수정 | FRemoteControlCommand·FUavSetpoint에 bGunFiring/MissileFireId 추가 |
| sim-unreal | Source/MUMT_Sim/Private/UDPControlReceiver.cpp | 수정 | 5010/5005 무장 파싱, 사망 가드, 무장 호출, 5006 hp/team/destroyed/missile_count, TeamToString |
| ros-bridge | src/custom_msgs/msg/AircraftSetpoint.msg | 수정 | gun_firing(bool), missile_fire_id(uint32) 추가, launch_missile deprecated |
| ros-bridge | src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py | 수정 | _on_setpoint 직렬화에 2필드 추가 |
| bt | scenarios/mumt_dogfight_1v1/bt_nodes.py | 신규 | GatherCombatState, ConditionEnemyAlive, EngageTarget |
| bt | scenarios/mumt_dogfight_1v1/uav1_bt.xml | 신규 | 적기(Enemy) 트리 |
| bt | scenarios/mumt_dogfight_1v1/uav2_bt.xml | 신규 | 아군(FriendlyUAV) 트리 |
| bt | scenarios/mumt_dogfight_1v1/configs/dogfight_uav1.yaml | 신규 | UAV1 러너 설정 |
| bt | scenarios/mumt_dogfight_1v1/configs/dogfight_uav2.yaml | 신규 | UAV2 러너 설정 |
| bt | scenarios/mumt_dogfight_1v1/README.md | 신규 | 실행법·트리구조·파라미터 문서 |
| bt | scenarios/mumt_dogfight_1v1/__init__.py | 신규 | (빈 파일) |

---

## 3. mumt-sim-unreal 파일 내용

### 3.1 Source/MUMT_Sim/Public/HealthComponent.h  [신규]

```cpp
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
```

### 3.2 Source/MUMT_Sim/Private/HealthComponent.cpp  [신규]

```cpp
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
        const FString Label = FString::Printf(TEXT("HP %.0f / %.0f%s"), CurrentHP, MaxHP, *Suffix);
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
```

### 3.3 Source/MUMT_Sim/Public/WeaponComponent.h  [신규]

```cpp
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

    FVector PrevOwnerLocation = FVector::ZeroVector;
    FVector OwnerWorldVelocity = FVector::ZeroVector;   // cm/s
    bool bHasPrevLocation = false;
};
```

### 3.4 Source/MUMT_Sim/Private/WeaponComponent.cpp  [신규]

```cpp
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

    // 발사 순간 1회 WEZ 판정: 넓은 원추, 가장 가까운 적기 1대
    float Dist = 0.f;
    if (UHealthComponent* Target = FindNearestTargetInCone(MissileRangeM, MissileConeHalfAngleDeg, Dist))
    {
        Target->ApplyDamage(MissileDamage, GetOwner());
    }
    // 명중 여부와 무관하게 연출은 발사 (허공 발사 = 미사일만 소모)
    OnMissileFired.Broadcast();
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
```

### 3.5 Source/MUMT_Sim/Public/UDPControlReceiver.h  [수정 — 변경 부분만]

전체 227줄 중 두 구조체에 무장 필드 추가. 나머지(소켓/포트/오토파일럿 프로퍼티)는 기존과 동일.

```cpp
// FRemoteControlCommand (5005 유인기 명령) — 무장 필드 추가
struct FRemoteControlCommand
{
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 0.0;
    double Throttle = 0.0;
    bool bValid = false;
    // Weapon triggers (Phase 3) — optional fields, keep defaults for old senders.
    bool  bGunFiring = false;
    int64 MissileFireId = 0;   // 0 = never fired (msg default); fire on change of id > 0
};

// FUavSetpoint (5010 무인기 setpoint) — 무장 필드 추가
struct FUavSetpoint
{
    float HeadingDeg     = 0.f;
    float AltitudeM      = 0.f;
    float Throttle       = 0.8f;  // used only when TargetSpeedMps <= 0 (open-loop)
    float TargetSpeedMps = 0.f;   // >0 → autothrottle holds this airspeed
    bool  LaunchMissile  = false; // deprecated — use MissileFireId
    // Weapon triggers (Phase 3) — optional fields, keep defaults for old senders.
    bool  bGunFiring     = false; // level-triggered: fire while true
    int64 MissileFireId  = 0;     // edge-triggered: one shot per new id (0 = never fired)
};
```

### 3.6 Source/MUMT_Sim/Private/UDPControlReceiver.cpp  [수정 — 변경 부분만]

전체 975줄 중 전투 관련 추가만 발췌 (나머지 소켓 수신/오토파일럿 PID/상태 송신 인프라는 기존과 동일).

**(a) include 2줄 추가**
```cpp
#include "HealthComponent.h"   // (알파벳순 위치)
#include "WeaponComponent.h"
```

**(b) 익명 namespace에 TeamToString 추가 (PawnIdName 아래)**
```cpp
    // ETeam → 5006 broadcast string ("manned" / "friendly_uav" / "enemy").
    FString TeamToString(ETeam Team)
    {
        switch (Team)
        {
        case ETeam::Manned:      return TEXT("manned");
        case ETeam::FriendlyUAV: return TEXT("friendly_uav");
        default:                 return TEXT("enemy");
        }
    }
```

**(c) ReceiveSetpointData() — 5010 setpoint JSON 파싱에 2필드 (StoreOne 람다 내)**
```cpp
            if (O->TryGetNumberField(TEXT("target_speed_mps"),V)) SP.TargetSpeedMps = (float)V;
            O->TryGetBoolField(TEXT("launch_missile"), SP.LaunchMissile);
            O->TryGetBoolField(TEXT("gun_firing"), SP.bGunFiring);                       // 추가
            if (O->TryGetNumberField(TEXT("missile_fire_id"), V)) SP.MissileFireId = (int64)V; // 추가

            Setpoints.Add(Name, SP);   // latest-wins per aircraft
```

**(d) ApplyAutopilotToPawn() — 사망 가드 (JSBSim null 체크 직후)**
```cpp
    // Dead/falling aircraft keep the hardover surfaces from UHealthComponent —
    // don't fight the crash. Drop the stale controller so any respawn starts fresh.
    if (const UHealthComponent* Health = Pawn->FindComponentByClass<UHealthComponent>())
    {
        if (!Health->IsAlive())
        {
            Autopilots.Remove(Key);
            return;
        }
    }
```

**(e) ApplyAutopilotToPawn() — 조종면 적용 후 무장 호출**
```cpp
    // Weapon triggers ride along with the setpoint (Phase 3). Missile edge
    // detection lives in UWeaponComponent; only forward ids > 0 so the msg
    // default (0 = never fired) can't be mistaken for a first shot.
    if (UWeaponComponent* Weapon = Pawn->FindComponentByClass<UWeaponComponent>())
    {
        Weapon->SetGunFiring(Setpoint.bGunFiring);
        if (Setpoint.MissileFireId > 0)
            Weapon->ConsumeMissileFireId(Setpoint.MissileFireId);
    }
```

**(f) ParseJsonCommand() — 5005 명령 JSON 파싱에 2필드 (FillCommand 람다 내)**
```cpp
        OutCommand.Throttle = JsonObject->GetNumberField(TEXT("throttle"));
        // Optional weapon triggers (Phase 3) — old senders simply omit them.
        JsonObject->TryGetBoolField(TEXT("gun_firing"), OutCommand.bGunFiring);
        double FireId = 0.0;
        if (JsonObject->TryGetNumberField(TEXT("missile_fire_id"), FireId))
        {
            OutCommand.MissileFireId = static_cast<int64>(FireId);
        }
        OutCommand.bValid = true;
```

**(g) ApplyControlCommandToPawn() — 5005 경로 사망 가드 (IsValid 직후)**
```cpp
    // Dead/falling aircraft ignore manned commands too (see ApplyAutopilotToPawn).
    if (const UHealthComponent* Health = Pawn->FindComponentByClass<UHealthComponent>())
    {
        if (!Health->IsAlive())
        {
            Autopilots.Remove(PawnIdName(Pawn));
            return false;
        }
    }
```

**(h) ApplyControlCommandToPawn() — 조종면 직접적용 후 무장 호출 (return 직전)**
```cpp
    // Weapon triggers (Phase 3) — same semantics as the autopilot path.
    if (UWeaponComponent* Weapon = Pawn->FindComponentByClass<UWeaponComponent>())
    {
        Weapon->SetGunFiring(Command.bGunFiring);
        if (Command.MissileFireId > 0)
        {
            Weapon->ConsumeMissileFireId(Command.MissileFireId);
        }
    }

    return bRollOk && bPitchOk && bYawOk && bThrottleOk;
```

**(i) BuildPawnState() — 5006 상태 JSON에 전투 필드 (team 문자열 필드 뒤)**
```cpp
    AddOptionalStringField(PawnJson, TEXT("team"), Pawn, TeamVarName);

    // Combat state (Phase 3) — omitted when the pawn has no combat components,
    // matching the optional-field convention above. When UHealthComponent exists
    // its ETeam overrides the legacy Blueprint "Team" string set just above.
    if (const UHealthComponent* Health = Pawn->FindComponentByClass<UHealthComponent>())
    {
        PawnJson->SetNumberField(TEXT("hp"), Health->CurrentHP);
        PawnJson->SetNumberField(TEXT("max_hp"), Health->MaxHP);
        PawnJson->SetStringField(TEXT("team"), TeamToString(Health->Team));
        PawnJson->SetBoolField(TEXT("destroyed"), !Health->IsAlive());
    }
    if (const UWeaponComponent* Weapon = Pawn->FindComponentByClass<UWeaponComponent>())
    {
        PawnJson->SetNumberField(TEXT("missile_count"), Weapon->MissileCount);
    }
```

---

## 4. mumt-ros-bridge 파일 내용

### 4.1 src/custom_msgs/msg/AircraftSetpoint.msg  [수정 — 전체]

```
string aircraft_name
float32 heading_deg
float32 altitude_m
float32 throttle_norm
float32 target_speed_mps
bool launch_missile      # deprecated — use missile_fire_id
bool gun_firing          # 상태: true인 동안 연사
uint32 missile_fire_id   # 이벤트: 값이 바뀔 때마다 1발 (0=미발사 초기값)
```
> 마지막 3줄이 이번 변경 (launch_missile 주석 추가 + gun_firing, missile_fire_id 신규).
> **주의**: 이 변경 후 `colcon build --packages-select custom_msgs mumt_ros_bridge` 재빌드 필수.

### 4.2 src/mumt_ros_bridge/mumt_ros_bridge/bridge_node.py  [수정 — _on_setpoint 함수]

```python
    def _on_setpoint(self, msg: AircraftSetpoint):
        # Per-UAV setpoint as JSON (carries aircraft_name so UE routes it to the
        # right UAV). Multiple BTs (one per UAV) each publish their own name.
        payload = json.dumps({
            "aircraft_name":    str(msg.aircraft_name),
            "heading_deg":      float(msg.heading_deg),
            "altitude_m":       float(msg.altitude_m),
            "throttle_norm":    float(max(0.0, min(1.0, msg.throttle_norm))),
            "target_speed_mps": float(msg.target_speed_mps),
            "launch_missile":   bool(msg.launch_missile),
            "gun_firing":       bool(msg.gun_firing),        # 추가
            "missile_fire_id":  int(msg.missile_fire_id),    # 추가
        })
        try:
            self._sp_sock.sendto(payload.encode("utf-8"),
                                 (self._unreal_ip, self._setpoint_port))
        except OSError as e:
            self.get_logger().warn(f"Setpoint UDP send error: {e}")
```
> 마지막 2개 payload 필드가 이번 변경. 나머지 노드 로직(소켓 3개, 상태 수신 타이머 등)은 기존과 동일.

---

## 5. mumt-bt 파일 내용

### 5.1 scenarios/mumt_dogfight_1v1/bt_nodes.py  [신규]

```python
"""
MUM-T 1v1 dogfight BT — 이륙 후 적기 교전 (pure pursuit + WEZ 방아쇠).

UAV1(Enemy) vs UAV2(FriendlyUAV). 팀은 **에디터에서** UHealthComponent.Team으로
지정하고, BT는 5006 상태의 Phase 3 확장 필드(team/destroyed/hp/missile_count)로
적을 식별한다. 명중 판정(WEZ 원추)은 UE UWeaponComponent가 하므로, BT는
"대략 겨눠졌을 때" 방아쇠만 당긴다 (기총 방아쇠 5° > WEZ 원추 2.5°).

발사 프로토콜 (Phase 3, AircraftSetpoint):
  gun_firing      : 상태(레벨) — true인 동안 UE가 연사
  missile_fire_id : 이벤트(에지) — 값이 바뀔 때마다 1발. 1부터 시작 (0=미발사 초기값)

기존 노드 재사용 (scenarios.mumt — 파일 수정 없이 import만):
  Takeoff     : 스폰 대비 상대 상승 판정 이륙
  OrbitLeader : 적 전멸 후 선회 대기 (loiter)
"""

import json
import math
import time

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Node, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic

# scenarios.mumt 노드/헬퍼 재사용 (import 부수효과로 Takeoff/OrbitLeader 등도 등록됨)
from scenarios.mumt.bt_nodes import (
    Takeoff, OrbitLeader, HoldSetpoint,
    unit_xy_to_heading, _name_matches, _alt_m, _setpoint, _own_routing_name,
    CM_TO_M, _STATE_TOPIC, _SETPOINT_TOPIC,
)

# ── 노드 등록 ──────────────────────────────────────────────────────────────────
BTNodeList.CONDITION_NODES.extend(["GatherCombatState", "ConditionEnemyAlive"])
BTNodeList.ACTION_NODES.extend(["EngageTarget"])


# ══════════════════════════════════════════════════════════════════════════════
# GatherCombatState — own + 적 팀 생존 기체 목록 인지
# ══════════════════════════════════════════════════════════════════════════════
class GatherCombatState(ConditionWithROSTopics):
    """상태 배치에서 own과 '적 팀 생존 기체 목록'을 blackboard에 기록. own을 찾으면 SUCCESS.
       (scenarios.mumt.GatherState의 전투 확장 — leader 대신 enemies를 채운다)

       blackboard 출력:
         own_state : own 기체 dict (없으면 None)
         all_states: {aircraft_name: dict}
         init_alt  : 기체별 스폰(최초관측) 고도 래칭 — Takeoff의 상대 판정용
         own_team  : own의 team 문자열 ("enemy"/"friendly_uav"/"manned", 컴포넌트 없으면 None)
         enemies   : team이 존재하고 own_team과 다르며 destroyed가 아닌 기체 목록"""

    def __init__(self, name, agent, own_name=""):
        super().__init__(name, agent, [(String, _STATE_TOPIC, "states")])
        self._own_name = own_name or (getattr(agent, "agent_id", "") or "").strip("/")

    def _predicate(self, agent, blackboard) -> bool:
        raw = self._cache.get("states")
        if not raw:
            return False
        try:
            payload = json.loads(raw.data)
        except (json.JSONDecodeError, AttributeError):
            return False
        if isinstance(payload, dict) and "aircraft" in payload:
            aircraft = payload["aircraft"]
        elif isinstance(payload, list):
            aircraft = payload
        else:
            return False

        blackboard["all_states"] = {a.get("aircraft_name", ""): a for a in aircraft}
        init = blackboard.setdefault("init_alt", {})
        for a in aircraft:
            init.setdefault(a.get("aircraft_name", ""), _alt_m(a))

        own = next((a for a in aircraft
                    if _name_matches(a.get("aircraft_name", ""), self._own_name)), None)
        own_team = (own or {}).get("team")
        blackboard["own_state"] = own
        blackboard["own_team"]  = own_team
        blackboard["enemies"]   = [
            a for a in aircraft
            if own_team and a.get("team") and a.get("team") != own_team
            and not a.get("destroyed", False) and a is not own
        ]

        if own is None:
            return False
        agent.ros_bridge.node.get_logger().info(
            f"[GatherCombatState] own={own.get('aircraft_name')} team={own_team} "
            f"hp={own.get('hp', '?')} msl={own.get('missile_count', '?')} "
            f"| 생존 적={len(blackboard['enemies'])}")
        return True


# ══════════════════════════════════════════════════════════════════════════════
# ConditionEnemyAlive — 적 팀 생존 기체 존재 여부
# ══════════════════════════════════════════════════════════════════════════════
class ConditionEnemyAlive(Node):
    """GatherCombatState가 채운 enemies가 비어있지 않으면 SUCCESS."""

    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Condition"
        self.is_expanded = False

    async def run(self, agent, blackboard):
        self.status = Status.SUCCESS if blackboard.get("enemies") else Status.FAILURE
        blackboard[self.name] = {"status": self.status, "is_expanded": self.is_expanded}
        return self.status


# ══════════════════════════════════════════════════════════════════════════════
# EngageTarget — 최근접 적 pure pursuit + WEZ 방아쇠
# ══════════════════════════════════════════════════════════════════════════════
class EngageTarget(ActionWithROSTopic):
    """적 팀 생존 기체 중 최근접 1대를 추적: 매 틱 표적 방위→heading, 표적 고도→altitude.
       발사 판정(5006의 자기 위치·헤딩 기준, wrap-around 처리):
         미사일: 거리<missile_range_m ∧ |방위오차|<missile_bearing_deg
                 → fire_id += 1 (쿨다운 missile_cooldown_sec, 잔탄>0일 때만)
         기총  : 거리<gun_range_m ∧ |방위오차|<gun_bearing_deg 인 동안 gun_firing=True
       생존 적이 없어지면(표적 destroyed) 기총을 끄고 SUCCESS.
       own 순간 결손 시 마지막 setpoint 재발행(래칭)."""

    def __init__(self, name, agent, own_name="",
                 engage_speed_mps=250.0,
                 missile_range_m=8000.0, missile_bearing_deg=15.0,
                 missile_cooldown_sec=10.0,
                 gun_range_m=1500.0, gun_bearing_deg=5.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name  = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._speed     = float(engage_speed_mps)
        self._msl_range = float(missile_range_m)
        self._msl_brg   = float(missile_bearing_deg)
        self._msl_cd    = float(missile_cooldown_sec)
        self._gun_range = float(gun_range_m)
        self._gun_brg   = float(gun_bearing_deg)
        self._fire_id   = 0        # 0=미발사 초기값, 첫 발사에 1
        self._last_fire = None     # time.monotonic() of last missile
        self._last_msg  = None
        self._done      = False

    def _build_message(self, agent, blackboard):
        own     = blackboard.get("own_state")
        enemies = blackboard.get("enemies") or []

        if not enemies:
            # 표적 destroyed(적 전멸) → 기총 끄고 SUCCESS
            self._done = True
            if self._last_msg is not None:
                self._last_msg.gun_firing = False
                return self._last_msg
            hdg = (own or {}).get("yaw", 0.0)
            alt = _alt_m(own) if own else 1000.0
            msg = _setpoint(_own_routing_name(own, self._own_name), hdg, alt,
                            target_speed=self._speed)
            msg.missile_fire_id = self._fire_id
            return msg

        self._done = False
        if not own:
            return self._last_msg   # 순간 결손 → 래칭 (None이면 FAILURE)

        ox, oy = own.get("x", 0.0) * CM_TO_M, own.get("y", 0.0) * CM_TO_M
        oyaw   = own.get("yaw", 0.0)

        def dist_m(a):
            return math.hypot(a.get("x", 0.0) * CM_TO_M - ox,
                              a.get("y", 0.0) * CM_TO_M - oy)

        target  = min(enemies, key=dist_m)          # 최근접 1대
        dist    = dist_m(target)
        bearing = unit_xy_to_heading(target.get("x", 0.0) * CM_TO_M - ox,
                                     target.get("y", 0.0) * CM_TO_M - oy)
        brg_err = abs(((bearing - oyaw + 180.0) % 360.0) - 180.0)   # wrap-around
        alt     = _alt_m(target)

        # 미사일: WEZ + 쿨다운 + 잔탄 (missile_count 필드 없으면 발사 허용)
        now = time.monotonic()
        fired = False
        if (dist < self._msl_range and brg_err < self._msl_brg
                and own.get("missile_count", 1) > 0
                and (self._last_fire is None or now - self._last_fire >= self._msl_cd)):
            self._fire_id += 1
            self._last_fire = now
            fired = True

        gun = dist < self._gun_range and brg_err < self._gun_brg

        agent.ros_bridge.node.get_logger().info(
            f"[EngageTarget] tgt={target.get('aircraft_name')} 거리={dist:.0f}m "
            f"방위오차={brg_err:.1f}° hp={target.get('hp', '?')} | "
            f"gun={'ON' if gun else 'off'} msl_id={self._fire_id}"
            f"{' ★발사' if fired else ''}")

        msg = _setpoint(_own_routing_name(own, self._own_name), bearing, alt,
                        target_speed=self._speed)
        msg.gun_firing      = bool(gun)
        msg.missile_fire_id = self._fire_id
        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.SUCCESS if self._done else Status.RUNNING
```

### 5.2 scenarios/mumt_dogfight_1v1/uav1_bt.xml  [신규]

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="mumt-dogfight-1v1">

    <!-- ===== UAV1 = 적기(Enemy) : 이륙 → 적 팀 생존 시 교전, 전멸 시 선회 대기 =====
         전제: 에디터에서 F16_UAV1 폰에 UHealthComponent(Team=Enemy) + UWeaponComponent 부착.
         BT는 5006의 team/destroyed로 적(=friendly_uav/manned)을 식별해 pure pursuit + WEZ 방아쇠. -->
    <ReactiveSequence>
      <GatherCombatState own_name="F16_UAV1"/>
      <Sequence>
        <Takeoff runway_heading_deg="90" climb_target_m="1000" uav_airborne_climb_m="120" own_name="F16_UAV1"/>
        <Fallback>
          <Sequence>
            <ConditionEnemyAlive/>
            <EngageTarget own_name="F16_UAV1" engage_speed_mps="250"
                          missile_range_m="8000" missile_bearing_deg="15" missile_cooldown_sec="10"
                          gun_range_m="1500" gun_bearing_deg="5"/>
          </Sequence>
          <!-- 적 전멸 → 고도 1000m 선회 대기 (기존 노드 재사용) -->
          <OrbitLeader own_name="F16_UAV1" altitude_m="1000" target_speed_mps="200"
                       start_heading_deg="90" turn_rate_dps="5" straight_time_s="0"/>
        </Fallback>
      </Sequence>
    </ReactiveSequence>

  </BehaviorTree>
</root>
```

### 5.3 scenarios/mumt_dogfight_1v1/uav2_bt.xml  [신규]

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="mumt-dogfight-1v1">

    <!-- ===== UAV2 = 아군(FriendlyUAV) : 이륙 → 적 팀 생존 시 교전, 전멸 시 선회 대기 =====
         전제: 에디터에서 F16_UAV2 폰에 UHealthComponent(Team=FriendlyUAV) + UWeaponComponent 부착.
         BT는 5006의 team/destroyed로 적(=enemy)을 식별해 pure pursuit + WEZ 방아쇠. -->
    <ReactiveSequence>
      <GatherCombatState own_name="F16_UAV2"/>
      <Sequence>
        <Takeoff runway_heading_deg="90" climb_target_m="1000" uav_airborne_climb_m="120" own_name="F16_UAV2"/>
        <Fallback>
          <Sequence>
            <ConditionEnemyAlive/>
            <EngageTarget own_name="F16_UAV2" engage_speed_mps="250"
                          missile_range_m="8000" missile_bearing_deg="15" missile_cooldown_sec="10"
                          gun_range_m="1500" gun_bearing_deg="5"/>
          </Sequence>
          <!-- 적 전멸 → 고도 1000m 선회 대기 (기존 노드 재사용) -->
          <OrbitLeader own_name="F16_UAV2" altitude_m="1000" target_speed_mps="200"
                       start_heading_deg="90" turn_rate_dps="5" straight_time_s="0"/>
        </Fallback>
      </Sequence>
    </ReactiveSequence>

  </BehaviorTree>
</root>
```

### 5.4 scenarios/mumt_dogfight_1v1/configs/dogfight_uav1.yaml  [신규]

```yaml
scenario:
  environment: scenarios.mumt_dogfight_1v1

agent:
  namespaces: "/F16_UAV1"            # 적기 (에디터에서 Team=Enemy 지정)
  behavior_tree_xml: "uav1_bt.xml"

bt_runner:
  bt_tick_rate: 10.0
  bt_visualiser:
    enabled: True
    screen_width: 600
    screen_height: 600
  profiling_mode: False
```

### 5.5 scenarios/mumt_dogfight_1v1/configs/dogfight_uav2.yaml  [신규]

```yaml
scenario:
  environment: scenarios.mumt_dogfight_1v1

agent:
  namespaces: "/F16_UAV2"            # 아군 (에디터에서 Team=FriendlyUAV 지정)
  behavior_tree_xml: "uav2_bt.xml"

bt_runner:
  bt_tick_rate: 10.0
  bt_visualiser:
    enabled: True
    screen_width: 600
    screen_height: 600
  profiling_mode: False
```

### 5.6 scenarios/mumt_dogfight_1v1/README.md  [신규]

````markdown
# mumt_dogfight_1v1 — 1v1 UAV 교전 시나리오

F16_UAV1(적, Enemy) vs F16_UAV2(아군, FriendlyUAV). 양쪽이 이륙 후 서로를
pure pursuit로 추적하며 WEZ 안에서 기총·미사일을 발사한다. 명중 판정(원추/거리/대미지)은
UE의 `UWeaponComponent`가 하고, BT는 방아쇠 조건만 판단한다.

## 전제 (에디터에서 1회 설정)

- UE 레벨의 두 UAV 폰에 **UHealthComponent + UWeaponComponent 부착** 완료 상태
  - F16_UAV1 → `Team = Enemy`
  - F16_UAV2 → `Team = Friendly UAV`
- Phase 3 빌드 적용 (5010 무장 파싱 + 5006 hp/team/destroyed/missile_count 브로드캐스트)
- `custom_msgs` 재빌드 완료 (gun_firing/missile_fire_id 추가 후 colcon build 필수)

## 실행 커맨드

```bash
# 0) (msg 변경 후 1회) ROS 워크스페이스 재빌드
cd ~/dev/mumt_ros_ws && colcon build --packages-select custom_msgs mumt_ros_bridge
source install/setup.bash

# 1) UE 에디터에서 PIE 실행 (RL_30 등)

# 2) 브리지
ros2 run mumt_ros_bridge bridge_node

# 3) UAV1 (적) BT — 터미널 1 (py_bt_ros 루트에서)
python main.py --config scenarios/mumt_dogfight_1v1/configs/dogfight_uav1.yaml

# 4) UAV2 (아군) BT — 터미널 2
python main.py --config scenarios/mumt_dogfight_1v1/configs/dogfight_uav2.yaml
```

## 트리 구조 (양쪽 동일, own_name만 다름)

```
ReactiveSequence
├─ GatherCombatState          # own + 적 팀 생존 목록 (5006 team/destroyed 파싱)
└─ Sequence
   ├─ Takeoff                 # 기존 노드 재사용 (스폰 대비 상대 상승 판정)
   └─ Fallback
      ├─ Sequence
      │  ├─ ConditionEnemyAlive
      │  └─ EngageTarget      # pure pursuit + WEZ 방아쇠, 표적 destroyed → SUCCESS
      └─ OrbitLeader          # 적 전멸 → 선회 대기 (기존 노드 재사용)
```

## EngageTarget 튜닝 파라미터 (XML 속성)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `engage_speed_mps` | 250 | 추적 목표속도 (UE 오토스로틀) |
| `missile_range_m` | 8000 | 미사일 방아쇠 거리 |
| `missile_bearing_deg` | 15 | 미사일 방아쇠 방위오차 (UE WEZ 원추 15°와 동일) |
| `missile_cooldown_sec` | 10 | 미사일 발사 간 최소 간격 (3발 순간 소진 방지) |
| `gun_range_m` | 1500 | 기총 방아쇠 거리 |
| `gun_bearing_deg` | 5 | 기총 방아쇠 방위오차 (UE WEZ 원추 2.5°보다 넓게 — 명중 판정은 UE가 함) |

발사 프로토콜: `gun_firing`은 레벨(true인 동안 연사), `missile_fire_id`는 에지
(값이 바뀔 때마다 1발, **1부터 시작** — 0은 미발사 초기값이라 UE가 무시).

## 동작 확인 포인트

- 양쪽 BT 로그에 `[EngageTarget] tgt=... 거리=... 방위오차=...`가 뜨고,
  WEZ 진입 시 `gun=ON` / `★발사`가 표시된다
- `ros2 topic echo /mumt/aircraft_states`에서 피격 기체의 `hp` 감소,
  격추 시 `destroyed: true` 확인
- 격추된 기체는 UE에서 엔진 정지 + 하드오버로 추락(Falling), 승자는 OrbitLeader로 선회 대기
````

### 5.7 scenarios/mumt_dogfight_1v1/__init__.py  [신규]

(빈 파일 — 패키지 마커)

---

## 6. 데이터 흐름 요약 (문서 작성 참고용)

### 발사 명령 (BT/조이스틱 → UE)
```
[무인기] EngageTarget → AircraftSetpoint(gun_firing, missile_fire_id)
         → /aircraft/setpoint → bridge._on_setpoint → UDP 5010(JSON)
         → ReceiveSetpointData → ApplyAutopilotToPawn → Weapon->SetGunFiring / ConsumeMissileFireId

[유인기] joystick → commands JSON(gun_firing, missile_fire_id) → UDP 5005
         → ParseJsonCommand → ApplyControlCommandToPawn → 동일 Weapon 호출
```

### 판정·대미지 (UE 내부)
```
Weapon TickComponent(기총) / FireMissile(미사일)
  → FindNearestTargetInCone(원추 판정) → Health->ApplyDamage
  → HP 0 → EnterFalling(엔진 CutOff + 조종면 하드오버) → AGL<임계 → EnterCrashed
```

### 전투 상태 (UE → BT)
```
BuildPawnState → 5006 JSON { hp, max_hp, team, destroyed, missile_count }
  → /mumt/aircraft_states → GatherCombatState → enemies 목록 → EngageTarget
```

### 5006 확장 필드 샘플 JSON
```json
{
  "aircraft_name": "F16_UAV1",
  "x": 500000.0, "y": 0.0, "z": 100000.0,
  "yaw": 90.0, "pitch": 2.0, "roll": 0.0,
  "speed_mps": 200.0, "throttle": 0.8,
  "hp": 60.0, "max_hp": 100.0,
  "team": "enemy", "destroyed": false,
  "missile_count": 2,
  "weapons": { "bullet_ammo": null, "rocket_ammo": null }
}
```

### 무장 파라미터 (UWeaponComponent 기본값)
| | 기총 | 미사일 |
|---|---|---|
| 판정 시점 | 발사 중 매 틱 | FireMissile 호출 순간 1회 |
| 원추 반각 | 2.5° | 15° |
| 사거리 | 1500 m | 8000 m |
| 대미지 | 20/s × (1 − 거리/사거리) | 40 고정 |
| 탄약 | 무제한(기본) | 3발 |
