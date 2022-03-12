#include "SimpleVehicleBrain.h"

#include <string>
#include <utility>
#include <vector>

#include <Kismet/KismetMathLibrary.h>
#include <UObject/ConstructorHelpers.h>

#include <SimpleVehicle/SimModeSimpleVehicle.h>
#include <SimpleVehicle/SimpleVehiclePawn.h>

#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h>
#include <EngineUtils.h>
#include <Engine/CollisionProfile.h>
#include <Engine/EngineTypes.h>

#include <Math/Rotator.h>

#include "Action.h"
#include "ActionSpec.h"
#include "Observation.h"
#include "ObservationSpec.h"
#include "ServerManager.h"
#include "Types.h"

USimpleVehicleBrain::USimpleVehicleBrain(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
}

void USimpleVehicleBrain::OnActorHit(AActor* SelfActor,
                                     AActor* OtherActor,
                                     FVector NormalImpulse,
                                     const FHitResult& Hit)
{
    check(OtherActor);

    if (OtherActor->ActorHasTag("goal"))
    {
        HitInfo = UHitInfo::Goal;
    }
    // TODO: Does instid1227 apply to all obstacles?
    // If not, include all obstacles or provide an user interface to specify
    // obstacles
    else if (!OtherActor->GetName().Contains(TEXT("instid1227"),
                                             ESearchCase::IgnoreCase))
    {
        HitInfo = UHitInfo::Edge;
    }
};

void USimpleVehicleBrain::Init()
{
    for (TActorIterator<ASimpleVehiclePawn> it(this->GetWorld()); it; ++it)
    {
        Owner = Cast<ASimpleVehiclePawn>(*it);
    }

    check(Owner);

    APlayerController* Controller = GetWorld()->GetFirstPlayerController();
    check(Controller != nullptr);

    // Look for the desired observation camera among all available camera
    // actors:
    APIPCamera* mainCamera;
    for (TActorIterator<APIPCamera> it(this->GetWorld()); it; ++it)
    {
        // This is quick and dirty fix to get access to the camera actors
        // attached to the vehicle. Note that these actors are defined within
        // the settings.json parameter file.
        // TODO: point to a specific PIPcamera actor, defined within a parameter
        // file.
        mainCamera = Cast<APIPCamera>(*it);
    }

    check(mainCamera);

    // Set the observation camera:
    Controller->SetViewTarget(mainCamera);

#if USE_IMAGE_OBSERVATIONS
    // Point to the vehicle camera capture component:
    captureComponent2D = mainCamera->GetSceneCaptureComponent();
#endif

    Owner->OnActorHit.AddDynamic(this, &USimpleVehicleBrain::OnActorHit);

    // Store actor refs required during simulation.
    for (TActorIterator<AActor> ActorItr(GetWorld(), AActor::StaticClass());
         ActorItr; ++ActorItr)
    {
        if ((*ActorItr)->ActorHasTag("goal"))
        {
            Goal = *ActorItr;
            break;
        }
    }

    check(Goal);

    // Initialize ObservationSpec and ActionSpec for this agent

    std::string SimpleVehicleActionDescription =
        "The OpenBot action space consists of a 2D vector containing the "
        "percentages of "
        "battery voltage to be applied to the left (resp. right) vehicle "
        "motors.\n"
        "These commands are in the [-1, 1] range.\n";
    unrealrl::ActionSpec SimpleVehicleActionSpec(
        false, unrealrl::DataType::Float32, {2}, std::make_pair(-1.0f, 1.0f),
        SimpleVehicleActionDescription); // HACK: should specify constants in a
                                         // config file

    std::string SimpleVehicleObservationDescription =
        "The agent has following observations:\n"
        "left wheel commands in the range [-1, 1]\n"
        "right wheel commands in the range [-1, 1]\n"
        "Euclidean distance between current x-y position and target x-y "
        "position.\n"
        "Sinus of the relative yaw between current pose and target pose.\n"
        "Cosinus of the relative yaw between current pose and target pose.\n";
    unrealrl::ObservationSpec SimpleVehicleObservationSpec(
        {5}, unrealrl::DataType::Float32, SimpleVehicleObservationDescription);

#if USE_IMAGE_OBSERVATIONS
    std::string CameraObservationDescription =
        "This observation is an egocentric RGB image.\n";
    unrealrl::ObservationSpec CameraObservationSpec(
        {{512, 512, 3}}, unrealrl::DataType::UInteger8,
        CameraObservationDescription); // HACK: should specify constants in a
                                       // config file

    SetObservationSpecs({SimpleVehicleObservationSpec, CameraObservationSpec});
#else
    SetObservationSpecs({SimpleVehicleObservationSpec});
#endif
    SetActionSpecs({SimpleVehicleActionSpec});
}

bool USimpleVehicleBrain::IsAgentReady()
{
    check(Owner);
    return (Owner->GetVelocity().Size() >= 0.0 && 
            Owner->GetVelocity().Size() < 0.1); // Wait for the agent to settle...
}

void USimpleVehicleBrain::OnEpisodeBegin()
{
    // reset by reload entire map
    UGameplayStatics::OpenLevel(this, FName(*GetWorld()->GetName()), false);
}

void USimpleVehicleBrain::SetAction(const std::vector<unrealrl::Action>& Action)
{
    check(Owner);

    check(Action.size() == 1);

    ActionVec = Action.at(0).GetActions();

    check(ActionVec.size() == 2);

    Owner->Move(ActionVec[0],
                ActionVec[1]); // ActionVec are in the [-1.0; 1.0] range
}

void USimpleVehicleBrain::GetObservation(
    std::vector<unrealrl::Observation>& Observations)
{
    check(Owner);
    check(Goal);

    // Get OpenBot position and orientation:
    const FVector CurrentLocation =
        Owner->GetActorLocation(); // Relative to global coordinate system
    const FRotator CurrentOrientation =
        Owner->GetActorRotation(); // Relative to global coordinate system

    // Get relative position to target in the global coordinate system:
    const FVector2D RelativePositionToTarget(
        (Goal->GetActorLocation() - CurrentLocation).X,
        (Goal->GetActorLocation() - CurrentLocation).Y);

    // Compute Euclidean distance to target:
    float dist = RelativePositionToTarget.Size();

    // Compute robot forward axis (global coordinate system)
    FVector forward = FVector(1.f, 0.f, 0.f); // Front axis is the X axis.
    FVector forwardRotated = CurrentOrientation.RotateVector(forward);

    // Distance vector to goal (global coordinate system)
    float dx = (Goal->GetActorLocation() - CurrentLocation).X;
    float dy = (Goal->GetActorLocation() - CurrentLocation).Y;

    // Compute yaw:
    float deltaYaw =
        std::atan2f(forwardRotated.Y, forwardRotated.X) - std::atan2f(dy, dx);

    // Fit to range (-pi, pi]:
    if (deltaYaw > PI)
    {
        deltaYaw -= 2 * PI;
    }
    else
    {
        if (deltaYaw <= -PI)
        {
            deltaYaw += 2 * PI;
        }
    }

    // Check the actual OpenBot code:
    // https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
    float sinYaw = std::sinf(deltaYaw);
    float cosYaw = std::cosf(deltaYaw);

    // Scaled distance to goal.
    SetCurrentReward(-dist / 100000); // TODO: set proper reward !!!!

    if (HitInfo == UHitInfo::Goal) // NOTE: so far we never get there... 
    {
        SetCurrentReward(100);
        EndEpisode();
    }
    else if (HitInfo == UHitInfo::Edge) // NOTE: so far we never get there... 
    {
        SetCurrentReward(-100);
        EndEpisode();
    }
    else
    {
        // TODO : set that correctly...
        // SetCurrentReward(...
    }

    // Get observation data

    Observations.resize(GetObservationSpecs().size());

    // vector observations

    if (ActionVec.size() == 2)
    {
        Owner->GetExecutedAction(
            ActionVec); // Fuses the actions received from the python client
                        // with those received from the keyboard interface (if
                        // this interface is activated in the settings.json
                        // file)

        Observations.at(0).Copy(std::vector<float>{ActionVec[0], ActionVec[1],
                                                   dist, sinYaw, cosYaw});
    }
    else
    {
        Observations.at(0).Copy(
            std::vector<float>{0.f, 0.f, dist, sinYaw, cosYaw});
    }

    // Reset HitInfo.
    HitInfo = UHitInfo::NoHit;

#if USE_IMAGE_OBSERVATIONS
    check(IsInGameThread());

    FTextureRenderTargetResource* TargetResource =
        captureComponent2D->TextureTarget->GameThread_GetRenderTargetResource();
    if (TargetResource == nullptr)
    {
        UE_LOG(
            LogTemp, Error,
            TEXT("Could not get RenderTarget Resource from GameThread!! :("));
        check(false);
    }

    TArray<FColor> RawPixels;
    RawPixels.Reset();

    struct FReadSurfaceContext
    {
        FRenderTarget* SrcRenderTarget;
        TArray<FColor>* OutData;
        FIntRect Rect;
        FReadSurfaceDataFlags Flags;
    };

    FReadSurfaceContext Context = {
        TargetResource, &RawPixels,
        FIntRect(0, 0, TargetResource->GetSizeXY().X,
                 TargetResource->GetSizeXY().Y),
        FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

    // Required for uint8 read mode
    Context.Flags.SetLinearToGamma(false);

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)
    (
        [Context](FRHICommandListImmediate& RHICmdList)
        {
            RHICmdList.ReadSurfaceData(
                Context.SrcRenderTarget->GetRenderTargetTexture(), Context.Rect,
                *Context.OutData, Context.Flags);
        });

    FRenderCommandFence ReadPixelFence;
    ReadPixelFence.BeginFence();
    ReadPixelFence.Wait(true); // true if you want to process gamethreadtasks
    // in parallel while waiting for RT to complete the enqueued task

    std::vector<uint8_t>* PixelData = &Observations.at(1).Data;

    PixelData->resize(Height * Width *
                      3); // HACK: should specify constants in a config file

    for (uint32 i = 0; i < static_cast<uint32>(RawPixels.Num()); ++i)
    {
        uint32 j = i;
        PixelData->at(3 * j) = RawPixels[i].R;
        PixelData->at(3 * j + 1) = RawPixels[i].G;
        PixelData->at(3 * j + 2) = RawPixels[i].B;
    }

    Observations.at(1).bIsSet = true;

#endif
}
