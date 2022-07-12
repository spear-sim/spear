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
	GetWorld()->GetFirstPlayerController()->InputComponent->BindAction("ActionThree", IE_Pressed, this,
		&AExamplePawn::switchScene);


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
    CustomRenderingManager::setLambertianRendering(actors,rendering_mode_%2==1);
    rendering_mode_++;
}

void AExamplePawn::switchDoor(){
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchDoor"));
    // initialization is required
    // DoorManager::initLevelDoorInfo(GetWorld());
    //change door status
    DoorManager::moveAllDoor(door_stat_);

    door_stat_ = !door_stat_;
}

void AExamplePawn::switchScene(){
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchScene"));

    std::string local_pak_file_path = "/home/xichen/Downloads/Map_235551809_linux.pak";
    bool val = LevelManager::mountPakFromPath(local_pak_file_path);
    UE_LOG(LogTemp, Log, TEXT("LevelManager::mountPakFromPath: %d"), val?1:0);

    std::vector<std::string> maps;
    LevelManager::getAllMapsInPak(maps);
    UE_LOG(LogTemp, Log, TEXT("AExamplePawn::switchScene #maps found: %d"), maps.size());

    for (auto& map:maps){
        UE_LOG(LogTemp, Log, TEXT("AExamplePawn::switchScene found map %s"),*UTF8_TO_TCHAR(maps[0].c_str()));
    }
    // move to next scene if available
    if (maps.size()>0){
        UGameplayStatics::OpenLevel(this, FName(UTF8_TO_TCHAR(maps[0].c_str())), false);
    }else{
        UE_LOG(LogTemp, Log, TEXT("AExamplePawn::switchScene zero map found"));
    }
}

void AExamplePawn::switchPhysicalMaterial()
{
    UE_LOG(LogTemp, Warning, TEXT("AExamplePawn::switchPhysicalMaterial"));
    std::vector<AActor*> actors;
    for (TActorIterator<AActor> actor_itr(GetWorld(), AActor::StaticClass()); actor_itr; ++actor_itr) {
        actors.emplace_back(*actor_itr);
    }
    PhysicsManager::updatePhysicalMaterial(actors, 1074);
}
