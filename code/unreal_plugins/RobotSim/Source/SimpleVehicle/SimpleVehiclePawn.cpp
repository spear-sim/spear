
PRAGMA_DISABLE_DEPRECATION_WARNINGS

#include "SimpleVehiclePawn.h"
#include "SimpleWheel.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"

FName ASimpleVehiclePawn::VehicleMovementComponentName(
    TEXT("SimpleWheeledVehicleMovement"));
FName ASimpleVehiclePawn::VehicleMeshComponentName(TEXT("VehicleMesh'"));

ASimpleVehiclePawn::ASimpleVehiclePawn(
    const FObjectInitializer& ObjectInitializer)
    : APawn(ObjectInitializer)
{
    Mesh = CreateDefaultSubobject<USkeletalMeshComponent>(
        VehicleMeshComponentName);
    // setup skeletal mesh
    static ConstructorHelpers::FObjectFinder<USkeletalMesh> CarMesh(
        TEXT("/RobotSim/SimpleVehicle/freight/freight.freight"));
    Mesh->SetSkeletalMesh(CarMesh.Object);
    // setup animation
    static ConstructorHelpers::FObjectFinder<UAnimBlueprint> CarMeshAnimation(
        TEXT("/RobotSim/SimpleVehicle/freight/freight_Animation.freight_Animation"));
    Mesh->SetAnimClass(
        CarMeshAnimation.Object->GetAnimBlueprintGeneratedClass());

    Mesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
    Mesh->BodyInstance.bSimulatePhysics = true;
    Mesh->BodyInstance.bNotifyRigidBodyCollision = true;
    Mesh->BodyInstance.bUseCCD = true;
    Mesh->bBlendPhysics = true;
    Mesh->SetGenerateOverlapEvents(true);
    Mesh->SetCanEverAffectNavigation(false);
    // example for adding user-defined collision callback
    // Mesh->OnComponentHit.AddDynamic(this,
    //                                &ASimpleVehiclePawn::OnComponentCollision);
    RootComponent = Mesh;

    VehicleMovement =
        CreateDefaultSubobject<UWheeledVehicleMovementComponent,
                               USimpleWheeledVehicleMovementComponent>(
            VehicleMovementComponentName);

    // setup wheels
    VehicleMovement->WheelSetups.SetNum(4);
    // TODO dynamic tire?
    // https://answers.unrealengine.com/questions/325623/view.html
    VehicleMovement->WheelSetups[0].WheelClass = USimpleWheel::StaticClass();
    VehicleMovement->WheelSetups[0].BoneName = FName("FL");
    VehicleMovement->WheelSetups[0].AdditionalOffset = FVector(0.f, -12.f, 0.f);

    VehicleMovement->WheelSetups[1].WheelClass = USimpleWheel::StaticClass();
    VehicleMovement->WheelSetups[1].BoneName = FName("FR");
    VehicleMovement->WheelSetups[1].AdditionalOffset = FVector(0.f, 12.f, 0.f);

    VehicleMovement->WheelSetups[2].WheelClass = USimpleWheel::StaticClass();
    VehicleMovement->WheelSetups[2].BoneName = FName("RL");
    VehicleMovement->WheelSetups[2].AdditionalOffset = FVector(0.f, -12.f, 0.f);

    VehicleMovement->WheelSetups[3].WheelClass = USimpleWheel::StaticClass();
    VehicleMovement->WheelSetups[3].BoneName = FName("RR");
    VehicleMovement->WheelSetups[3].AdditionalOffset = FVector(0.f, 12.f, 0.f);

    VehicleMovement->SetIsReplicated(true); // Enable replication by default
    VehicleMovement->UpdatedComponent = Mesh;

    // Create default camera component
    FVector CameraPos(-200.0f, -00.0f, 80.0f);
    FRotator CameraOri(-10.0f, 0.0f, 0.0f);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera0"));
    Camera->SetRelativeLocationAndRotation(CameraPos, CameraOri);
    Camera->SetupAttachment(Mesh);
    Camera->bUsePawnControlRotation = false;
    Camera->FieldOfView = 90.f;
}

ASimpleVehiclePawn::~ASimpleVehiclePawn()
{
    // Set this pawn to call Tick() every frame.  You can turn this off to
    // improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

void ASimpleVehiclePawn::SetupPlayerInputComponent(
    class UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    // set up gameplay key bindings in RobotSimVehicleGameMode
    check(PlayerInputComponent);

    PlayerInputComponent->BindAxis("MoveForward", this,
                                   &ASimpleVehiclePawn::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this,
                                   &ASimpleVehiclePawn::MoveRight);
}

void ASimpleVehiclePawn::SetupInputBindings()
{
    URobotBlueprintLib::EnableInput(this);

    // keyboard control in RobotSimGameMode
    APlayerController* controller =
        this->GetWorld()->GetFirstPlayerController();

    controller->InputComponent->BindAxis("MoveForward", this,
                                         &ASimpleVehiclePawn::MoveForward);
    controller->InputComponent->BindAxis("MoveRight", this,
                                         &ASimpleVehiclePawn::MoveRight);
}

void ASimpleVehiclePawn::MoveForward(float Val)
{
    USimpleWheeledVehicleMovementComponent* vehicle_pawn =
        static_cast<USimpleWheeledVehicleMovementComponent*>(
            GetVehicleMovementComponent());

    float torque = 50 * Val;
    vehicle_pawn->SetDriveTorque(torque, 0);
    vehicle_pawn->SetDriveTorque(torque, 1);
    vehicle_pawn->SetDriveTorque(torque, 2);
    vehicle_pawn->SetDriveTorque(torque, 3);
    URobotBlueprintLib::LogMessage(FString("MoveFoward"),
                                   FString::SanitizeFloat(torque),
                                   LogDebugLevel::Informational, 30);
}

void ASimpleVehiclePawn::MoveRight(float Val)
{
    if (Val != 0)
    {
        USimpleWheeledVehicleMovementComponent* vehicle_pawn =
            static_cast<USimpleWheeledVehicleMovementComponent*>(
                GetVehicleMovementComponent());

        float torque = 50 * Val;
        vehicle_pawn->SetDriveTorque(torque, 0);
        vehicle_pawn->SetDriveTorque(-torque, 1);
        vehicle_pawn->SetDriveTorque(torque, 2);
        vehicle_pawn->SetDriveTorque(-torque, 3);
    }
    URobotBlueprintLib::LogMessage(FString("MoveRight"),
                                   FString::SanitizeFloat(Val),
                                   LogDebugLevel::Informational, 30);
}

// Called every frame
void ASimpleVehiclePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void ASimpleVehiclePawn::NotifyHit(class UPrimitiveComponent* HitComponent,
                                   class AActor* OtherActor,
                                   class UPrimitiveComponent* otherComp,
                                   bool bSelfMoved,
                                   FVector hitLocation,
                                   FVector hitNormal,
                                   FVector normalImpulse,
                                   const FHitResult& hit)
{
    URobotBlueprintLib::LogMessage(
        FString("NotifyHit: ") + HitComponent->GetName() + " with " +
            OtherActor->GetName(),
        " location: " + hitLocation.ToString() +
            " normal: " + normalImpulse.ToString(),
        LogDebugLevel::Informational, 30);
    this->mPawnEvents.getCollisionSignal().emit(
        HitComponent, OtherActor, otherComp, bSelfMoved, hitLocation, hitNormal,
        normalImpulse, hit);
}

void ASimpleVehiclePawn::OnComponentCollision(UPrimitiveComponent* HitComponent,
                                              AActor* OtherActor,
                                              UPrimitiveComponent* OtherComp,
                                              FVector NormalImpulse,
                                              const FHitResult& Hit)
{
    URobotBlueprintLib::LogMessage(
        FString("OnComponentCollision: ") + HitComponent->GetName() + " with " +
            OtherActor->GetName(),
        " location: " + Hit.Location.ToString() +
            " normal: " + Hit.Normal.ToString(),
        LogDebugLevel::Informational, 30);
}

// Called when the game starts or when spawned
void ASimpleVehiclePawn::BeginPlay()
{
    Super::BeginPlay();
}

USceneComponent* ASimpleVehiclePawn::GetComponent(FString componentName)
{
    throw std::runtime_error("Requested component named " +
                             std::string(TCHAR_TO_UTF8(*componentName)) +
                             " in GetComponent(), which does not exist.");
}

void ASimpleVehiclePawn::GetComponentReferenceTransform(FString componentName,
                                                        FVector& translation,
                                                        FRotator& rotation)
{
    throw std::runtime_error("Requested component named " +
                             std::string(TCHAR_TO_UTF8(*componentName)) +
                             " in GetComponent(), which does not exist.");
}

void ASimpleVehiclePawn::TeleportToLocation(FVector position,
                                            FQuat orientation,
                                            bool teleport)
{
    FVector translation =
        (position * URobotBlueprintLib::GetWorldToMetersScale(this)) -
        this->GetActorLocation();
    FRotator rotation =
        (orientation * this->GetActorQuat().Inverse()).Rotator();
    RobotBase::TeleportToLocation(
        position * URobotBlueprintLib::GetWorldToMetersScale(this), orientation,
        teleport);
}
PawnEvents* ASimpleVehiclePawn::GetPawnEvents()
{
    return &(this->mPawnEvents);
}

PRAGMA_ENABLE_DEPRECATION_WARNINGS