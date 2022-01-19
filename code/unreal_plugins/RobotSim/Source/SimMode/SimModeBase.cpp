#include "SimModeBase.h"
#include "Misc/MessageDialog.h"
#include "Misc/EngineVersion.h"
#include "Runtime/Engine/Public/EngineUtils.h"
#include "Runtime/Launch/Resources/Version.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/OutputDeviceNull.h"
#include "Engine/World.h"

#include "NavMesh/NavMeshBoundsVolume.h"

#include <memory>
#include "RobotBlueprintLib.h"

#include "DrawDebugHelpers.h"

ASimModeBase::ASimModeBase()
{
    // static ConstructorHelpers::FClassFinder<APIPCamera>
    // external_camera_class(
    //    TEXT("Blueprint'/RobotSim/Blueprints/BP_PIPCamera'"));
    // external_camera_class_ = external_camera_class.Succeeded()
    //                             ? external_camera_class.Class
    //                             : nullptr;
    // not used
    //    static ConstructorHelpers::FObjectFinder<UParticleSystem>
    //    collision_display(
    //        TEXT("ParticleSystem'/RobotSim/StarterContent/Particles/"
    //             "P_Explosion.P_Explosion'"));
    //    if (!collision_display.Succeeded())
    //        collision_display_template = collision_display.Object;
    //    else
    collision_display_template = nullptr;

    // static ConstructorHelpers::FClassFinder<APIPCamera> pip_camera_class_val(
    //    TEXT("Blueprint'/RobotSim/Blueprints/BP_PIPCamera'"));
    // pip_camera_class =
    //    pip_camera_class_val.Succeeded() ? pip_camera_class_val.Class :
    //    nullptr;

    PrimaryActorTick.bCanEverTick = true;

    static ConstructorHelpers::FClassFinder<AActor> sky_sphere_class(
        TEXT("Blueprint'/Engine/EngineSky/BP_Sky_Sphere'"));
    sky_sphere_class_ =
        sky_sphere_class.Succeeded() ? sky_sphere_class.Class : nullptr;
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
    global_ned_transform_.reset(
        new NedTransform(player_start_transform,
                         URobotBlueprintLib::GetWorldToMetersScale(this)));

    record_tick_count = 0;
    setupInputBindings();

    setupTimeOfDay();

    URobotBlueprintLib::LogMessage(TEXT("Press F1 to see help"), TEXT(""),
                                   LogDebugLevel::Informational);

    setupVehiclesAndCamera();
}

const NedTransform& ASimModeBase::getGlobalNedTransform()
{
    return *global_ned_transform_;
}

void ASimModeBase::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // FRecordingThread::stopRecording();
    // world_sim_api_.reset();
    // api_provider_.reset();
    // api_server_.reset();
    // global_ned_transform_.reset();

    // CameraDirector = nullptr;
    sky_sphere_ = nullptr;
    sun_ = nullptr;

    spawned_actors_.Empty();
    // vehicle_sim_apis_.clear();

    // Super::EndPlay(EndPlayReason);
}

void ASimModeBase::setupTimeOfDay()
{
    sky_sphere_ = nullptr;

    const auto& tod_setting = getSettings().tod_setting;

    if (tod_setting.enabled)
    {
        TArray<AActor*> sky_spheres;
        UGameplayStatics::GetAllActorsOfClass(this->GetWorld(),
                                              sky_sphere_class_, sky_spheres);
        if (sky_spheres.Num() == 0)
            URobotBlueprintLib::LogMessage(
                TEXT("BP_Sky_Sphere was not found. "),
                TEXT("TimeOfDay settings would be ignored."),
                LogDebugLevel::Failure);
        else if (sky_spheres.Num() > 1)
            URobotBlueprintLib::LogMessage(
                TEXT("More than BP_Sky_Sphere were found. "),
                TEXT("TimeOfDay settings would be applied to first one."),
                LogDebugLevel::Failure);

        if (sky_spheres.Num() >= 1)
        {
            sky_sphere_ = sky_spheres[0];
            static const FName sun_prop_name(TEXT("Directional light actor"));
            auto* p = sky_sphere_class_->FindPropertyByName(sun_prop_name);
            UObjectProperty* sun_prop = Cast<UObjectProperty>(p);
            UObject* sun_obj =
                sun_prop->GetObjectPropertyValue_InContainer(sky_sphere_);
            sun_ = Cast<ADirectionalLight>(sun_obj);
            if (sun_)
            {
                sun_->GetRootComponent()->Mobility =
                    EComponentMobility::Movable;
            }

            // tod_sim_clock_start_ = ClockFactory::get()->nowNanos();
            tod_last_update_ = 0;
            if (tod_setting.start_datetime != "")
                tod_start_time_ =
                    Utils::to_time_t(tod_setting.start_datetime,
                                     tod_setting.is_start_datetime_dst);
            else
                tod_start_time_ = std::time(nullptr);
        }
    }
    // else ignore
}

void ASimModeBase::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
}

void ASimModeBase::setupInputBindings()
{
    URobotBlueprintLib::EnableInput(this);

    // URobotBlueprintLib::BindActionToKey("InputEventResetAll",
    // EKeys::BackSpace, this, &ASimModeBase::reset);
}

const RobotSim::RobotSimSettings& ASimModeBase::getSettings() const
{
    return RobotSimSettings::singleton();
}

void ASimModeBase::traceGround(FVector& spawnPosition,
                               FRotator& spawnRotator,
                               const FVector& boxHalfSize)
{
    FVector startLoc = spawnPosition + FVector(0, 0, 100);
    FVector endLoc = spawnPosition + FVector(0, 0, -1000);

    FCollisionQueryParams collisionParams(FName(TEXT("trace2ground")), true,
                                          this);
    FHitResult hit(ForceInit);
    if (UKismetSystemLibrary::BoxTraceSingle(
            GetWorld(), startLoc, endLoc, boxHalfSize, spawnRotator,
            ETraceTypeQuery::TraceTypeQuery1, false, TArray<AActor*>(),
            EDrawDebugTrace::Type::ForDuration, hit, true))
    {
        spawnPosition = hit.Location;
    }
}

void ASimModeBase::setupVehiclesAndCamera()
{
}

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

void ASimModeBase::NavSystemTest()
{
    ARecastNavMesh* navMesh = GetNavMesh();
    // change navMesh size
    if (navMesh->AgentRadius == 40)
    {
        NavSystemRebuild(20);
    }
    else
    {
        NavSystemRebuild(40);
    }

    FBox worldBox = GetWorldBoundingBox();
    TArray<FNavPoly> Polys;
    navMesh->GetPolysInBox(worldBox, Polys);
    for (auto& PolyId : Polys)
    {
        TArray<FVector> array;
        navMesh->GetPolyVerts(PolyId.Ref, array);
        for (auto& v : array)
        {
            UE_LOG(LogTemp, Display, TEXT("vertices: %lld ->%s"), PolyId.Ref,
                   *v.ToString());
        }
    }
}

bool ASimModeBase::NavSystemRebuild(float AgentRadius)
{
    UNavigationSystemV1* NavSys =
        FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    if (!NavSys)
    {
        return false;
    }
    auto navData = NavSys->GetMainNavData();
    ARecastNavMesh* navMesh = Cast<ARecastNavMesh>(navData);
    if (!navMesh)
    {
        return false;
    }

    ANavMeshBoundsVolume* NavmeshBounds = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(this->GetWorld()); it; ++it)
    {
        NavmeshBounds = *it;
    }
    if (NavmeshBounds == nullptr)
    {
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
    for (TActorIterator<AActor> it(this->GetWorld()); it; ++it)
    {
        if (it->ActorHasTag("architecture") || it->ActorHasTag("furniture"))
        {
            box += it->GetComponentsBoundingBox(false, true);
        }
    }
    // remove ceiling
    return !bScaleCeiling
               ? box
               : box.ExpandBy(box.GetSize() * 0.1f)
                     .ShiftBy(FVector(0, 0, -0.3f * box.GetSize().Z));
}
