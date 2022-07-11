#include "ExamplePawn.h"

#include <iostream>

#include "EngineUtils.h"
#include "Engine/SceneCapture2D.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SphereComponent.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Kismet/GameplayStatics.h"

//#include "VWPhysicsManager.h"
//#include "VWLightManager.h"
//#include "VWLevelManager.h"

AExamplePawn::AExamplePawn()
{
}

void AExamplePawn::BeginPlay()
{
    Super::BeginPlay();

    // ignore its collision
    this->GetCollisionComponent()->SetCollisionObjectType(ECollisionChannel::ECC_Camera);
    this->GetCollisionComponent()->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);


    // test only
    GetWorld()->GetFirstPlayerController()->InputComponent->BindAction("Handbrake", IE_Pressed, this,
                                                                       &AExamplePawn::test);

	GetWorld()->GetFirstPlayerController()->InputComponent->BindAction("ActionOne", IE_Pressed, this,
		&AExamplePawn::test);
}

void AExamplePawn::test()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::test"));
//    std::vector<AActor*> actors;
//    for (TActorIterator<AActor> actor_itr(GetWorld(), AActor::StaticClass()); actor_itr; ++actor_itr) {
//        actors.emplace_back(*actor_itr);
//    }
//
//    VWPhysicsManager::updatePhysicalMaterial(actors,stat?1065:1074);
//    std::vector<std::string> maps;
//    VWLevelManager::getAllMapsInPak(maps);
//    std::string local_pak_file_path = "/home/xichen/Downloads/Map_235551809_linux.pak";
//    VWLevelManager::mountPakFromPath(local_pak_file_path);
//    for (auto& map:maps){
//        std::cout<<"map - "<<map<<std::endl;
//    }

    VWLightManager::EnableDistanceFieldShadows(GetWorld(), stat);
    stat =!stat;
}