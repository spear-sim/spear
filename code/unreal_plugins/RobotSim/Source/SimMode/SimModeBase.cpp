#include "SimModeBase.h"

#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "NavMesh/NavMeshBoundsVolume.h"
#include "Engine/World.h"
#include "Engine/ObjectLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "RobotBlueprintLib.h"

ASimModeBase::ASimModeBase()
{
    collision_display_template = nullptr;

    PrimaryActorTick.bCanEverTick = true;

    static ConstructorHelpers::FClassFinder<AActor> sky_sphere_class(
        TEXT("Blueprint'/Engine/EngineSky/BP_Sky_Sphere'"));
    sky_sphere_class_ = sky_sphere_class.Succeeded() ? sky_sphere_class.Class : nullptr;
}

void ASimModeBase::BeginPlay()
{
    Super::BeginPlay();

    // get player start
    // this must be done from within actor otherwise we don't get player start
    APlayerController* player_controller =
        this->GetWorld()->GetFirstPlayerController();
    FTransform player_start_transform =
        player_controller->GetViewTarget()->GetActorTransform();
    global_ned_transform_.reset(new NedTransform(
        player_start_transform, URobotBlueprintLib::GetWorldToMetersScale(this)));

    setupInputBindings();

    URobotBlueprintLib::LogMessage(TEXT("Press F1 to see help"), TEXT(""), LogDebugLevel::Informational);

    setupVehiclesAndCamera();
}

const NedTransform& ASimModeBase::getGlobalNedTransform()
{
    return *global_ned_transform_;
}

void ASimModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    spawned_actors_.Empty();
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
}

void ASimModeBase::setupInputBindings()
{
    URobotBlueprintLib::EnableInput(this);
}

const RobotSim::RobotSimSettings& ASimModeBase::getSettings() const
{
    return RobotSimSettings::singleton();
}

void ASimModeBase::traceGround(FVector& spawnPosition, FRotator& spawnRotator,
                               const FVector& boxHalfSize)
{
    FVector startLoc = spawnPosition + FVector(0, 0, 100);
    FVector endLoc = spawnPosition + FVector(0, 0, -1000);

    FCollisionQueryParams collisionParams(FName(TEXT("trace2ground")), true, this);
    FHitResult hit(ForceInit);
    if (UKismetSystemLibrary::BoxTraceSingle(
            GetWorld(), startLoc, endLoc, boxHalfSize, spawnRotator, ETraceTypeQuery::TraceTypeQuery1, false, TArray<AActor*>(), EDrawDebugTrace::Type::ForDuration, hit, true)) {
        spawnPosition = hit.Location;
    }
}

void ASimModeBase::setupVehiclesAndCamera() {}

void ASimModeBase::getExistingVehiclePawns(TArray<RobotBase*>& pawns) const
{
    // derived class should override this method to retrieve types of pawns they
    // support
}

ARecastNavMesh* ASimModeBase::GetNavMesh()
{
    UNavigationSystemV1* NavSys =
        FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());

    auto navData = NavSys->GetMainNavData();
    return Cast<ARecastNavMesh>(navData);
}

void ASimModeBase::Test()
{
    TArray<FString> mapList;
    this->GetAllMaps(mapList);
    if (mapList.Num() == 0) {
        UE_LOG(LogTemp, Log, TEXT("no map found"));
        URobotBlueprintLib::LogMessage(FString("no map found "), "", LogDebugLevel::Informational, 30);
        return;
    }
    FString currentMap = this->GetWorld()->GetName();
    URobotBlueprintLib::LogMessage(
        FString("current map: "), currentMap, LogDebugLevel::Informational, 30);
    int currentIndex = -1;
    for (int i = 0; i < mapList.Num(); i++) {
        FString MapName = mapList[i];
        FString MapShortName = FPackageName::GetShortName(MapName);
        UE_LOG(LogTemp, Log, TEXT("Found map name: %s"), *(MapShortName));
        URobotBlueprintLib::LogMessage(
            FString("Found map name " + MapName), "", LogDebugLevel::Informational, 30);
        if (currentMap.Equals(MapShortName)) {
            currentIndex = i;
            break;
        }
    }
    FString NextMap = mapList[(currentIndex + 1) % mapList.Num()];
    this->LoadMap(NextMap);
}

bool ASimModeBase::NavSystemRebuild(float AgentRadius)
{
    UNavigationSystemV1* NavSys =
        FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    if (!NavSys) {
        return false;
    }
    auto navData = NavSys->GetMainNavData();
    ARecastNavMesh* navMesh = Cast<ARecastNavMesh>(navData);
    if (!navMesh) {
        return false;
    }

    ANavMeshBoundsVolume* NavmeshBounds = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(this->GetWorld()); it; ++it) {
        NavmeshBounds = *it;
    }
    if (NavmeshBounds == nullptr) {
        return false;
    }

    navMesh->AgentRadius = AgentRadius;
    navMesh->CellSize = 10;
    navMesh->AgentMaxSlope = 0.1f;
    navMesh->AgentMaxStepHeight = 0.1f;
    navMesh->MergeRegionSize = 0;
    // ignore region that are too small
    navMesh->MinRegionArea = 400;

    // dynamic update navMesh location and size
    NavmeshBounds->GetRootComponent()->SetMobility(EComponentMobility::Movable);

    FBox worldBox = GetWorldBoundingBox();
    NavmeshBounds->SetActorLocation(worldBox.GetCenter(), false);
    NavmeshBounds->SetActorRelativeScale3D(worldBox.GetSize() / 200.0f);
    NavmeshBounds->GetRootComponent()->UpdateBounds();
    // NavmeshBounds->SupportedAgents.bSupportsAgent0;
    NavSys->OnNavigationBoundsUpdated(NavmeshBounds);
    // redo modify frequency change
    NavmeshBounds->GetRootComponent()->SetMobility(EComponentMobility::Static);

    // rebuild NavMesh, required for update AgentRadius
    NavSys->Build();

    return true;
}

FBox ASimModeBase::GetWorldBoundingBox(bool bScaleCeiling)
{
    FBox box(ForceInit);
    for (TActorIterator<AActor> it(this->GetWorld()); it; ++it) {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture")) {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // remove ceiling be very careful about ceiling, some asset have very strange bbox causing Max.Z way above ceiling
    return !bScaleCeiling ? box
                          : box.ExpandBy(box.GetSize() * 0.1f)
                                .ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
}

void ASimModeBase::GetAllMaps(TArray<FString>& MapList) const
{
#if WITH_EDITOR
    auto ObjectLibrary =
        UObjectLibrary::CreateLibrary(UWorld::StaticClass(), false, true);
    ObjectLibrary->LoadAssetDataFromPath(TEXT("/Game"));
    TArray<FAssetData> AssetDatas;
    ObjectLibrary->GetAssetDataList(AssetDatas);
    UE_LOG(LogTemp, Log, TEXT("Found maps count: %d"), AssetDatas.Num());
    for (int32 i = 0; i < AssetDatas.Num(); i++) {
        FAssetData& AssetData = AssetDatas[i];
        MapList.AddUnique(AssetData.AssetName.ToString());
    }
#else
    // only scan .pak for standalone. Could cause error in Editor note that maps from might be included twice
    levelManager->GetAllMapsInPak(MapList);
#endif
    // sort the output to ensure maps always in the same order.
    MapList.Sort();
}

void ASimModeBase::LoadMap(FString mapName)
{
    UE_LOG(LogTemp, Log, TEXT("LoadMap: %s"), *(mapName));
    UGameplayStatics::OpenLevel(this, FName(mapName), false);
}
