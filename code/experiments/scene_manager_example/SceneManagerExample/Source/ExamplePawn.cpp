#include "ExamplePawn.h"

#include <iostream>
#include <vector>

#include <Camera/CameraComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/SphereComponent.h>
#include <EngineUtils.h>
#include <Engine/SceneCapture2D.h>
#include <InputCoreTypes.h>
#include <Kismet/KismetSystemLibrary.h>
#include <Kismet/GameplayStatics.h>

#include "PhysicsManager.h"

AExamplePawn::AExamplePawn()
{
}

void AExamplePawn::BeginPlay()
{
    Super::BeginPlay();

    // ignore its collision
    this->GetCollisionComponent()->SetCollisionObjectType(ECollisionChannel::ECC_Camera);
    this->GetCollisionComponent()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);

    // test key binders
    GetWorld()->GetFirstPlayerController()->InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &AExamplePawn::test);

    GetWorld()->GetFirstPlayerController()->InputComponent->BindKey(EKeys::One, IE_Pressed, this, &AExamplePawn::test2);
    GetWorld()->GetFirstPlayerController()->InputComponent->BindKey(EKeys::Two, IE_Pressed, this, &AExamplePawn::switchPhysicalMaterial);
    GetWorld()->GetFirstPlayerController()->InputComponent->BindKey(EKeys::Three, IE_Pressed, this, &AExamplePawn::switchNewPhysicalMaterial);
}

void AExamplePawn::test()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::test"));
    // reload current level
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}

void AExamplePawn::test2()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::test2"));
    // reload current level
    PhysicsManager::terminate();
    PhysicsManager::initialize();
}

void AExamplePawn::switchPhysicalMaterial()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchPhysicalMaterial"));
    // get all actor tagged floor
    TArray<AActor*> actors_tagged_floor;
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), TEXT("floor"), actors_tagged_floor);
    std::vector<AActor*> actors;
    for (auto& actor : actors_tagged_floor)
    {
        actors.emplace_back(actor);
    }
    // update physical material
    PhysicsManager::setActorPhysicalMaterials(actors, physical_material_stat_ % 2 == 0 ? 2001 : 2000);
    physical_material_stat_++;
}

void AExamplePawn::switchNewPhysicalMaterial()
{
    // create a random material
    int physical_material_id = PhysicsManager::createPhysicalMaterial(0.1f, 1.0f);
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchNewPhysicalMaterial createPhysicalMaterial: %d"), physical_material_id);

    // get all actor tagged floor
    TArray<AActor*> actors_tagged_floor;
    UGameplayStatics::GetAllActorsWithTag(GetWorld(), TEXT("floor"), actors_tagged_floor);
    std::vector<AActor*> actors;
    for (auto& actor : actors_tagged_floor)
    {
        actors.emplace_back(actor);
    }

    PhysicsManager::setActorPhysicalMaterials(actors, physical_material_id);

    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchNewPhysicalMaterial setActorPhysicalMaterials: %d"), physical_material_id);
    physical_material_stat_++;
}
