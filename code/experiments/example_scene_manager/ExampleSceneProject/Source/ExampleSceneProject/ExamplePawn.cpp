#include "ExamplePawn.h"

#include <iostream>

#include "EngineUtils.h"
#include "Engine/SceneCapture2D.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SphereComponent.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"

#include "PhysicsManager.h"
#include "LightManager.h"
#include "LevelManager.h"
#include "CustomRenderingManager.h"


AExamplePawn::AExamplePawn()
{
}

void AExamplePawn::BeginPlay()
{
    Super::BeginPlay();

    // ignore its collision
    this->GetCollisionComponent()->SetCollisionObjectType(ECollisionChannel::ECC_Camera);
    this->GetCollisionComponent()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);

    // initialize UVWDoorManager
    //UVWDoorManager::initLevelDoorInfo(GetWorld());

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

//    std::vector<AActor*> actors;
//    for (TActorIterator<AActor> actor_itr(GetWorld(), AActor::StaticClass()); actor_itr; ++actor_itr) {
//        actors.emplace_back(*actor_itr);
//    }
//    UE_LOG(LogTemp, Warning, TEXT("ACapturePawn::test"));
//
//    VWPhysicsManager::updatePhysicalMaterial(actors,stat?1065:1074);
//    std::vector<std::string> maps;
//    VWLevelManager::getAllMapsInPak(maps);
//    std::string local_pak_file_path = "/home/xichen/Downloads/Map_235551809_linux.pak";
//    VWLevelManager::mountPakFromPath(local_pak_file_path);
//    for (auto& map:maps){
//        std::cout<<"map - "<<map<<std::endl;
//    }
}

void AExamplePawn::switchRenderingMode()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchRenderingMode"));
    std::vector<AActor*> actors;
    for (TActorIterator<AActor> actor_itr(GetWorld(), AActor::StaticClass()); actor_itr; ++actor_itr) {
        actors.emplace_back(*actor_itr);
    }
    CustomRenderingManager::setLambertianRendering(actors,rendering_mode_%2==1);
    rendering_mode_++;
}

void AExamplePawn::switchDoor(){
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchDoor"));

    //UVWDoorManager::moveAllDoor(door_stat_);

    door_stat_ = !door_stat_;
}
