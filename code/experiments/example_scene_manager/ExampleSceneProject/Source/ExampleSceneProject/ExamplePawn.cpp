#include "ExamplePawn.h"

#include <iostream>

#include "EngineUtils.h"
#include "Engine/SceneCapture2D.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SphereComponent.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"

#include "VWPhysicsManager.h"
#include "VWLightManager.h"
#include "VWLevelManager.h"
#include "VWCustomRenderingManager.h"


AExamplePawn::AExamplePawn()
{
    doorManager = CreateDefaultSubobject<UVWDoorManager>(TEXT("AVWLevelManager"));
    if (doorManager->loadData(GetWorld())) {
        doorManager->matchDoorActor(GetWorld());
    }
}

void AExamplePawn::BeginPlay()
{
    Super::BeginPlay();

    // ignore its collision
    this->GetCollisionComponent()->SetCollisionObjectType(ECollisionChannel::ECC_Camera);
    this->GetCollisionComponent()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);


    // test key binders
    GetWorld()->GetFirstPlayerController()->InputComponent->BindAction("Handbrake", IE_Pressed, this,
                                                                       &AExamplePawn::test);

	GetWorld()->GetFirstPlayerController()->InputComponent->BindAction("ActionOne", IE_Pressed, this,
		&AExamplePawn::switchDoor);
	GetWorld()->GetFirstPlayerController()->InputComponent->BindAction("ActionTwo", IE_Pressed, this,
		&AExamplePawn::switchRenderingMode);


}

void AExamplePawn::test()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::test"));
}

void AExamplePawn::switchRenderingMode()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchRenderingMode"));
    std::vector<AActor*> actors;
    for (TActorIterator<AActor> actor_itr(GetWorld(), AActor::StaticClass()); actor_itr; ++actor_itr) {
        actors.emplace_back(*actor_itr);
    }
    UVWCustomRenderingManager::setLambertianRendering(actors,rendering_mode_%2==1);
    rendering_mode_++;
}

void AExamplePawn::switchDoor(){
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchDoor"));

    doorManager->moveAllDoor(door_stat_);

    door_stat_ = !door_stat_;
}