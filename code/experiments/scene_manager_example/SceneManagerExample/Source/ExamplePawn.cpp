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
    PhysicsManager::setActorPhysicalMaterials(actors, physical_material_stat_ % 2 == 0 ? 1074 : 1000);
    physical_material_stat_++;
}

