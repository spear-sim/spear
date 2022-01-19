// Modified from Carla (https://carla.org/) project, thanks to MIT license.

#pragma once

#include "UnrealRLManager.h"

#if UNREALRL_DEBUG
#include <ostream>
#include <iomanip>
#endif

#include "Kismet/GameplayStatics.h"
#include "Misc/App.h"
#include "Misc/CoreDelegates.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "UnrealEngine.h"
#include "Utils/ThreadingHelpers.h"

#include "RL/Brain.h"
#include "RPC/RpcServerWrapper.h"
#include "ServerManager.h"
#include "UnrealRL.h"
#include "Utils/IgnoreCompilerWarnings.h"

ENABLE_IGNORE_COMPILER_WARNINGS
#include "asio.hpp"
DISABLE_IGNORE_COMPILER_WARNINGS

#if UNREALRL_DEBUG
// For UE4 Profiler ~ Stat
DECLARE_CYCLE_STAT(TEXT("UnrealRLManager ~ SyncRun"),
                   STAT_SyncRun,
                   STATGROUP_UnrealRLManager);
DECLARE_CYCLE_STAT(TEXT("UnrealRLManager ~ Tick"),
                   STAT_Tick,
                   STATGROUP_UnrealRLManager);
DECLARE_CYCLE_STAT(TEXT("UnrealRLManager ~ ResetWorkGuard"),
                   STAT_ResetWorkGuard,
                   STATGROUP_UnrealRLManager);
DECLARE_CYCLE_STAT(TEXT("UnrealRLManager ~ GetObservation"),
                   STAT_GetObservation,
                   STATGROUP_UnrealRLManager);
DECLARE_CYCLE_STAT(TEXT("UnrealRLManager ~ GetObservations"),
                   STAT_GetObservations,
                   STATGROUP_UnrealRLManager);

#endif

namespace unrealrl
{

// static variable initializations for UnrealRLManager class
uint64_t UnrealRLManager::FrameCounter = 0;
bool UnrealRLManager::bSynchronousMode = false;

//~=============================================================================
// ServerNode class definition and Implementation

/**
 * This wraps rpc::Server class.
 */
class UnrealRLManager::ServerNode
{
public:
    // @todo : currently passing non empty ServerAddress is causing issues.
    // resolve if need be.
    ServerNode(UnrealRLManager* InRLManager,
               std::string ServerAddress,
               uint16_t Port)
        : Server(UnrealRLManager::bSynchronousMode, Port)
    {

        SetUnrealRLManager(InRLManager);

        BindFunctions();
    }

    FORCEINLINE UnrealRLManager* GetUnrealRLManager() const
    {
        return RLManagerBackPointer;
    }

    unrealrl::rpc::Server Server;

private:
    UnrealRLManager* RLManagerBackPointer;

    void BindFunctions();
    void SetUnrealRLManager(UnrealRLManager* InRLManager);
};

void UnrealRLManager::ServerNode::SetUnrealRLManager(
    UnrealRLManager* InRLManager)
{
    if (InRLManager)
    {
        RLManagerBackPointer = InRLManager;
    }
    else
    {
        UE_LOG(LogRL, Error,
               TEXT("UnrealRLManager.cpp : Could not get valid reference to "
                    "UnrealRLManager."));
        check(false);
    }
}

//~=============================================================================
// UnrealRLManager class implementation

UnrealRLManager::UnrealRLManager(std::string ServerAddress, uint16_t Port)
{
    // ServerNodePtr.Reset(new ServerNode(ServerAddress, Port));
    ServerNodePtr = MakeUnique<ServerNode>(this, ServerAddress, Port);

    bIsGameRunning = false;

    NotifyInitGame();
}

void UnrealRLManager::NotifyInitGame()
{
    if (!bIsGameRunning)
    {
        ResetFrameCounter();
        BindDelegates();
        bIsGameRunning = true;
    }
    else
    {
        UE_LOG(LogRL, Warning,
               TEXT("Server is already running. Will not create another one."));
    }
}

void UnrealRLManager::BindDelegates()
{
    OnBeginFrameDelegate = FCoreDelegates::OnBeginFrame.AddRaw(
        this, &UnrealRLManager::OnBeginFrame);

    /**Other useful delegates*/
#if UNREALRL_DEBUG
    OnWorldTickStartDelegate = FWorldDelegates::OnWorldTickStart.AddRaw(
        this, &UnrealRLManager::OnWorldTickStart);
    OnWorldPostActorTickDelegate = FWorldDelegates::OnWorldPostActorTick.AddRaw(
        this, &UnrealRLManager::OnWorldPostActorTick);
    OnEndFrameDelegate =
        FCoreDelegates::OnEndFrame.AddRaw(this, &UnrealRLManager::OnEndFrame);
    OnBeginFrameRTDelegate = FCoreDelegates::OnBeginFrameRT.AddRaw(
        this, &UnrealRLManager::OnBeginFrameRT);
    OnEndFrameRTDelegate = FCoreDelegates::OnEndFrameRT.AddRaw(
        this, &UnrealRLManager::OnEndFrameRT);
#endif
}

void UnrealRLManager::SetSynchronousMode(bool bIsSyncMode)
{
    // don't act upon unwanted inputs
    if (IsSynchronousMode() == bIsSyncMode)
    {
        return;
    }

    // store value before changing
    bool bWasSyncMode = IsSynchronousMode();

    // set new value
    bSynchronousMode = bIsSyncMode;
    ServerNodePtr->Server.SetSyncMode(bIsSyncMode);

    // need to reset work_guard so that control exits from io_context.run()
    // had it been in synchronous mode
    if (bWasSyncMode && !IsSynchronousMode())
    {
        ServerNodePtr->Server.ResetWorkGuard();
    }

    return;
}

void UnrealRLManager::AsyncRun(uint32 NumberOfWorkerThreads)
{
    check(ServerNodePtr != nullptr);
    ServerNodePtr->Server.AsyncRun(NumberOfWorkerThreads);
}

void UnrealRLManager::SyncRun()
{
    check(ServerNodePtr != nullptr);
    ServerNodePtr->Server.SyncRun();
}

void UnrealRLManager::Stop()
{
    check(ServerNodePtr != nullptr);
    ServerNodePtr->Server.Stop();
}

UnrealRLManager::~UnrealRLManager()
{
    if (bIsGameRunning)
    {
        FCoreDelegates::OnBeginFrame.Remove(OnBeginFrameDelegate);

        /**Other useful delegates*/
#if UNREALRL_DEBUG
        FCoreDelegates::OnBeginFrameRT.Remove(OnBeginFrameRTDelegate);
        FWorldDelegates::OnWorldTickStart.Remove(OnWorldTickStartDelegate);
        FWorldDelegates::OnWorldPostActorTick.Remove(
            OnWorldPostActorTickDelegate);
        FCoreDelegates::OnEndFrame.Remove(OnEndFrameDelegate);
        FCoreDelegates::OnEndFrameRT.Remove(OnEndFrameRTDelegate);
#endif
        bIsGameRunning = false;
    }
    Stop();
}

void UnrealRLManager::OnBeginFrame()
{

#if UNREALRL_DEBUG
    std::cout
        << "==================================================================="
           "==========================================================="
        << std::endl;
    std::cout << "OnBeginFrame callback ";
    t_begin_frame = std::chrono::high_resolution_clock::now();
    auto duration = t_begin_frame.time_since_epoch();
    std::cout
        << std::setprecision(15)
        << std::chrono::duration_cast<std::chrono::seconds>(duration).count()
        << ":"
        << std::chrono::duration_cast<std::chrono::milliseconds>(duration)
               .count()
        << std::endl;
#endif

    bOneTickCompleted.store(true, std::memory_order_seq_cst);

    {
#if UNREALRL_DEBUG
        SCOPE_CYCLE_COUNTER(STAT_SyncRun);
#endif
        // update frame counter
        UpdateFrameCounter();

        // If in Sync Mode,
        // This is a blocking run due to asio::work_guard_executor.
        // Need to reset work_guard pointer to unblock.
        // Client resets work_guard for performing one world tick.
        // Else if in Async mode,
        // This runs non-blocking way (i.e without work_guard bound)
        // Hence, here, we don't wait for client to send a tick, in fact, client
        // need not tick.
        SyncRun();
    }
}

#if UNREALRL_DEBUG
void UnrealRLManager::OnBeginFrameRT()
{
    std::cout << "OnBeginFrameRT callback ";
    t_begin_frame_rt = std::chrono::high_resolution_clock::now();
    auto duration = t_begin_frame_rt.time_since_epoch();
    std::cout
        << std::setprecision(15)
        << std::chrono::duration_cast<std::chrono::seconds>(duration).count()
        << ":"
        << std::chrono::duration_cast<std::chrono::milliseconds>(duration)
               .count()
        << std::endl;
}

void UnrealRLManager::OnWorldTickStart(UWorld* World,
                                       ELevelTick TickType,
                                       float DeltaSeconds)
{
    if (TickType == ELevelTick::LEVELTICK_All)
    {
        std::cout << "OnWorldTickStart " << std::endl;
    }

    // @todo : track deltaseconds elapsed since start of the game?
}

void UnrealRLManager::OnWorldPostActorTick(UWorld* World,
                                           ELevelTick TickType,
                                           float DeltaSeconds)
{
    if (TickType == ELevelTick::LEVELTICK_All)
    {
        std::cout << "OnWorldPostActorTick " << std::endl;
    }
}

void UnrealRLManager::OnEndFrame()
{
    std::cout << "OnEndFrame callback ";
    t_end_frame = std::chrono::high_resolution_clock::now();
    auto duration = t_end_frame.time_since_epoch();
    std::cout
        << std::setprecision(15)
        << std::chrono::duration_cast<std::chrono::seconds>(duration).count()
        << ":"
        << std::chrono::duration_cast<std::chrono::milliseconds>(duration)
               .count()
        << std::endl;
}

void UnrealRLManager::OnEndFrameRT()
{
    std::cout << "OnEndFrameRT callback ";
    t_end_frame_rt = std::chrono::high_resolution_clock::now();
    auto duration = t_end_frame_rt.time_since_epoch();
    std::cout
        << std::setprecision(15)
        << std::chrono::duration_cast<std::chrono::seconds>(duration).count()
        << ":"
        << std::chrono::duration_cast<std::chrono::milliseconds>(duration)
               .count()
        << std::endl;
    std::chrono::duration<double> tspan =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            t_end_frame_rt - t_begin_frame);
    std::cout << "time it took from beginframe to endframeRT "
              << std::setprecision(15) << tspan.count() << " seconds."
              << std::endl;
    std::cout << "OnEndFrameRT: GFrameCounter " << GFrameCounter << std::endl;
    std::cout << "OnEndFrameRT: GFrameNumber " << GFrameNumber << std::endl;
    std::cout << "OnEndFrameRT: GFrameNumberRenderThread "
              << GFrameNumberRenderThread << std::endl;
    std::cout
        << "==================================================================="
           "==========================================================="
        << std::endl;
}
#endif

/**
 * Utility class to provide nice way of binding functions to rpc server.
 */
class ServerBinder
{
public:
    constexpr ServerBinder(const char* name,
                           unrealrl::rpc::Server& srv,
                           bool sync)
        : _name(name), _server(srv), _sync(sync)
    {
    }

    template <typename FuncT> auto operator<<(FuncT func)
    {
        if (_sync)
        {
            _server.BindSync(_name, func);
        }
        else
        {
            _server.BindAsync(_name, func);
        }
        return func;
    }

private:
    const char* _name;

    unrealrl::rpc::Server& _server;

    bool _sync;
};

#define BIND_SYNC(name) auto name = ServerBinder(#name, Server, true)
#define BIND_ASYNC(name) auto name = ServerBinder(#name, Server, false)

/**
 * You can bind functions that need not be run on GameThread using BIND_ASYNC
 * For functions that require to be run on GameThread use BIND_SYNC.
 */
void UnrealRLManager::ServerNode::BindFunctions()
{
    BIND_ASYNC(Ping) << []() -> bool { return true; };

    BIND_ASYNC(Echo) << [](std::string const& s) -> std::string {
        return std::string("Echoing -> ") + s;
    };

    BIND_SYNC(Pause) << [&](bool bPause) -> bool {
        return UGameplayStatics::SetGamePaused(
            UServerManager::Get().GetWorld()->GetAuthGameMode(), bPause);
    };

    BIND_SYNC(IsPaused) << [&]() -> bool {
        return UServerManager::Get().GetWorld()->GetAuthGameMode()->IsPaused();
    };

    BIND_SYNC(Reset) << [&]() -> void {
        UServerManager::Get().GetWorld()->GetAuthGameMode()->ResetLevel();
    };

    BIND_SYNC(Close) << [&]() -> void {
        FGenericPlatformMisc::RequestExit(false);
        if (GetUnrealRLManager()->IsSynchronousMode())
        {
            Server.ResetWorkGuard();
        }
    };

    BIND_SYNC(IsEnvReady) <<
        [&]() -> bool { return UServerManager::Get().IsEnvironmentReady(); };

    BIND_ASYNC(Tick) << [&]() -> void {
#if UNREALRL_DEBUG
        SCOPE_CYCLE_COUNTER(STAT_Tick);
#endif

        // reset work guard only in synchronous mode
        if (GetUnrealRLManager()->IsSynchronousMode())
        {
#if UNREALRL_DEBUG
            SCOPE_CYCLE_COUNTER(STAT_ResetWorkGuard);
#endif
            // This releases io_context.run()'s control, moving simulation by 1
            // frame
            GetUnrealRLManager()->bOneTickCompleted.store(
                false, std::memory_order_seq_cst);
            Server.ResetWorkGuard();

            // wait until the tick has completed to return back to client only
            // in sync mode
            while (GetUnrealRLManager()->bOneTickCompleted.load(
                       std::memory_order_seq_cst) == false)
            {
                FPlatformProcess::Sleep(0.f);
            }
        }
        // when not in sync mode, unreal ticks at different rate than client

        return;
    };

    BIND_ASYNC(StartExperiment) << [&]() -> void {
        // temporarily set synchronous mode to false so that the env can tick
        // until it's ready
        GetUnrealRLManager()->SetSynchronousMode(false);

        // block return to client until env is ready
        while (UServerManager::Get().IsEnvironmentReady() == false)
        {
            TFuture<void> result =
                AsyncNamed<void>(ENamedThreads::GameThread, [&]() {
                    UServerManager::Get().ValidateEnvReadiness();
                });
            result.Get();
        }

        // set synchronous mode back to true to begin RL experiment
        GetUnrealRLManager()->SetSynchronousMode(true);

        return;
    };

    BIND_ASYNC(SetSynchronousMode) << [&](bool bIsSyncMode) -> void {
        GetUnrealRLManager()->SetSynchronousMode(bIsSyncMode);

        return;
    };

    BIND_ASYNC(IsSynchronousMode)
        << [&]() -> bool { return GetUnrealRLManager()->IsSynchronousMode(); };

    BIND_SYNC(SetFixedDeltaSeconds) << [&](double DeltaTime) -> void {
        FApp::SetBenchmarking(true);
        FApp::SetFixedDeltaTime(DeltaTime);
    };

    BIND_SYNC(RemoveFixedDeltaSeconds)
        << [&]() -> void { FApp::SetBenchmarking(false); };

    BIND_SYNC(GetFixedDeltaSeconds) << [&]() -> double {
        return FApp::IsBenchmarking() ? FApp::GetFixedDeltaTime() : -2.0;
    };

    // Still in experimental phase
    BIND_SYNC(AllowOneFrameRTLag) << [&](bool bAllow) -> void {
        // static FFrameEndSync FrameEndSync;
        // FrameEndSync.Sync(bAllow);
        if (bAllow)
        {
            GEngine->Exec(UServerManager::Get().GetWorld(),
                          TEXT("r.OneFrameThreadLag 1"));
        }
        else
        {
            GEngine->Exec(UServerManager::Get().GetWorld(),
                          TEXT("r.OneFrameThreadLag 0"));
        }
    };

    // BIND_SYNC(SetPhysicsSubStepping) << [&]() -> void {
    //     UPhysicsSettings* PhysSettings = UPhysicsSettings::Get();
    //     PhysSettings->bSubstepping = true;
    //     PhysSettings->MaxSubstepDeltaTime = 0.001;
    //     PhysSettings->MaxSubsteps = 10;
    // };

    BIND_ASYNC(GetFrameCount)
        << [&]() -> uint64_t { return UnrealRLManager::GetFrameCounter(); };

    BIND_SYNC(SetGameViewPortRenderingFlag) << [&](bool bFlag) -> void {
        UServerManager::Get()
            .GetWorld()
            ->GetGameViewport()
            ->bDisableWorldRendering = !bFlag;
    };

    // used for debug purposes
    BIND_SYNC(RunConsoleCommand) << [&](const std::string& CMD) -> bool {
        auto* playerController = UGameplayStatics::GetPlayerController(
            UServerManager::Get().GetWorld(), 0);

        if (playerController != nullptr)
        {
            FString Ret = playerController->ConsoleCommand(CMD.c_str(), true);
            // #if UNREALRL_DEBUG
            //             // std::cout << "Running command: " << CMD << " 's
            //             result is " << TCHAR_TO_UTF8(*Ret) << std::endl;
            // #endif
        }
        return playerController != nullptr;
    };

    BIND_SYNC(GetActorListOfTag)
        << [this](const std::string& Tag) -> std::vector<std::string> {
        std::vector<std::string> result;

        // if tag is empty, return all actors in world
        if (Tag.empty())
        {
            for (auto& Ele : UServerManager::Get().ActorNameToRefMap)
            {
                result.emplace_back(
                    TCHAR_TO_UTF8(*Ele.Key)); // TCHAR_TO_UTF8 to convert
                                              // FString -> std::string
            }
        }
        else
        {
            for (auto& Ele : UServerManager::Get().ActorNameToRefMap)
            {
                if (Ele.Value->ActorHasTag(Tag.c_str()))
                {
                    result.emplace_back(
                        TCHAR_TO_UTF8(*Ele.Key)); // TCHAR_TO_UTF8 to convert
                                                  // FString -> std::string
                }
            }
        }
        return result;
    };

    BIND_SYNC(GetAgentList) << [this]() -> std::vector<std::string> {
        std::vector<std::string> result;

        if (UServerManager::Get().AgentActors.Num() > 0)
        {
            for (auto& Ele : UServerManager::Get().AgentActors)
            {
                result.emplace_back(TCHAR_TO_UTF8(*(Ele->GetName())));
            }
        }
        else
        {
            UE_LOG(LogRL, Warning,
                   TEXT("There are no agents in the environment!!"));
        }

        return result;
    };

    BIND_SYNC(GetObservationSpecs)
        << [&](const std::string& ActorName) -> unrealrl::ObservationSpecs {
        if (UServerManager::Get().ActorNameToRefMap.Contains(ActorName.c_str()))
        {
            AActor* ParentActor =
                UServerManager::Get().ActorNameToRefMap[ActorName.c_str()];
            if (ParentActor)
            {
                UBrain* Agent = Cast<UBrain>(
                    ParentActor->GetComponentByClass(UBrain::StaticClass()));
                if (Agent)
                {
                    return Agent->GetObservationSpecs();
                }
                else
                {
                    UE_LOG(
                        LogRL, Warning,
                        TEXT("GetObservationSpecs: Input agent %s is not an RL "
                             "agent."),
                        *ActorName.c_str());
                }
            }
        }
        else
        {
            UE_LOG(LogRL, Warning,
                   TEXT("GetObservationSpecs: Does not contain required agent "
                        "in the envrionment. Skipping for agent %s"),
                   *ActorName.c_str());
        }

        return unrealrl::ObservationSpecs();
    };

    BIND_SYNC(GetActionSpecs)
        << [&](const std::string& ActorName) -> unrealrl::ActionSpecs {
        if (UServerManager::Get().ActorNameToRefMap.Contains(ActorName.c_str()))
        {
            AActor* ParentActor =
                UServerManager::Get().ActorNameToRefMap[ActorName.c_str()];
            if (ParentActor)
            {
                UBrain* Agent = Cast<UBrain>(
                    ParentActor->GetComponentByClass(UBrain::StaticClass()));
                if (Agent)
                {
                    return Agent->GetActionSpecs();
                }
                else
                {
                    UE_LOG(LogRL, Warning,
                           TEXT("GetActionSpecs: Input agent %s is not an RL "
                                "agent."),
                           *ActorName.c_str());
                }
            }
        }
        else
        {
            UE_LOG(
                LogRL, Warning,
                TEXT("GetActionSpecs: Does not contain required agent in the "
                     "envrionment. Skipping for agent %s"),
                *ActorName.c_str());
        }

        return unrealrl::ActionSpecs();
    };

    BIND_SYNC(GetObservation) << [&](const std::string& ActorName)
        -> std::vector<unrealrl::Observation> {
        std::vector<unrealrl::Observation> ReturnObs;

        if (UServerManager::Get().ActorNameToRefMap.Contains(ActorName.c_str()))
        {
            AActor* ParentActor =
                UServerManager::Get().ActorNameToRefMap[ActorName.c_str()];
            if (ParentActor)
            {
                UBrain* Agent = Cast<UBrain>(
                    ParentActor->GetComponentByClass(UBrain::StaticClass()));
                if (Agent)
                {
#if UNREALRL_DEBUG
                    SCOPE_CYCLE_COUNTER(STAT_GetObservation);
#endif
                    Agent->GetObservation(ReturnObs);
                }
                else
                {
                    UE_LOG(LogRL, Warning,
                           TEXT("GetObservation: Input agent %s is not an RL "
                                "agent."),
                           *ActorName.c_str());
                }
            }
        }
        else
        {
            UE_LOG(LogRL, Warning,
                   TEXT("GetObservation: Does not contain required agent in "
                        "the envrionment. Skipping for agent %s"),
                   *ActorName.c_str());
        }

        return ReturnObs;
    };

    BIND_SYNC(GetObservations)
        << [&](const std::vector<std::string>& VecActorNames)
        -> unrealrl::ObservationDict {
        unrealrl::ObservationDict ObservationDict;
        ObservationDict.GetAgentToObservationMap()->clear();

        for (const std::string& ActorName : VecActorNames)
        {
            if (UServerManager::Get().ActorNameToRefMap.Contains(
                    ActorName.c_str()))
            {
                AActor* ParentActor =
                    UServerManager::Get().ActorNameToRefMap[ActorName.c_str()];
                if (ParentActor)
                {
                    UBrain* Agent =
                        Cast<UBrain>(ParentActor->GetComponentByClass(
                            UBrain::StaticClass()));
                    if (Agent)
                    {
#if UNREALRL_DEBUG
                        SCOPE_CYCLE_COUNTER(STAT_GetObservations);
#endif
                        ObservationDict.GetAgentToObservationMap()->emplace(
                            ActorName, std::vector<unrealrl::Observation>());
                        Agent->GetObservation(
                            ObservationDict.GetAgentToObservationMap()->at(
                                ActorName));
                    }
                    else
                    {
                        UE_LOG(
                            LogRL, Warning,
                            TEXT("GetObservations: Input agent %s is not an RL "
                                 "agent. Skipping..."),
                            *ActorName.c_str());
                    }
                }
            }
            else
            {
                UE_LOG(LogRL, Warning,
                       TEXT("GetObservations : Does not contain required "
                            "agent. Skipping for agent with name %s"),
                       *ActorName.c_str());
            }
        }

        return ObservationDict;
    };

    BIND_SYNC(SetAction) <<
        [&](const std::string& ActorName,
            const std::vector<unrealrl::Action>& Action) -> void {
        if (UServerManager::Get().ActorNameToRefMap.Contains(ActorName.c_str()))
        {
            AActor* ParentActor =
                UServerManager::Get().ActorNameToRefMap[ActorName.c_str()];
            if (ParentActor)
            {
                UBrain* Agent = Cast<UBrain>(
                    ParentActor->GetComponentByClass(UBrain::StaticClass()));
                if (Agent)
                {
                    Agent->SetAction(Action);
                }
                else
                {
                    UE_LOG(LogRL, Warning,
                           TEXT("SetAction: Input agent %s is not an RL "
                                "agent."),
                           *ActorName.c_str());
                }
            }
        }
        else
        {
            UE_LOG(LogRL, Warning,
                   TEXT("SetAction: Does not contain required agent in "
                        "the envrionment. Skipping for agent %s"),
                   *ActorName.c_str());
        }

        return;
    };

    BIND_SYNC(SetActions) << [&](unrealrl::ActionDict Actions) -> void {
        // #if UNREALRL_DEBUG
        //     // std::cout << &Actions << std::endl;
        // #endif
        for (auto IteratorAgentToActionMap =
                 Actions.GetAgentToActionMap()->begin();
             IteratorAgentToActionMap != Actions.GetAgentToActionMap()->end();
             ++IteratorAgentToActionMap)
        {
            if (UServerManager::Get().ActorNameToRefMap.Contains(
                    IteratorAgentToActionMap->first.c_str()))
            {
                AActor* ParentActor =
                    UServerManager::Get().ActorNameToRefMap
                        [IteratorAgentToActionMap->first.c_str()];
                if (ParentActor)
                {
                    UBrain* Agent =
                        Cast<UBrain>(ParentActor->GetComponentByClass(
                            UBrain::StaticClass()));
                    if (Agent)
                    {
                        Agent->SetAction(IteratorAgentToActionMap->second);
                    }
                    else
                    {
                        UE_LOG(LogRL, Warning,
                               TEXT("SetActions: Input agent %s is not an RL "
                                    "agent. Skipping..."),
                               *(IteratorAgentToActionMap->first.c_str()));
                    }
                }
            }
            else
            {
                UE_LOG(LogRL, Warning,
                       TEXT("SetActions: Does not contain required agent. "
                            "Skipping for agent with name %s"),
                       *(IteratorAgentToActionMap->first.c_str()));
            }
        }
        return;
    };

    BIND_SYNC(BeginEpisode) << [&]() -> void {
        for (auto& Ele : UServerManager::Get().AgentActors)
        {
            UBrain* Agent =
                Cast<UBrain>(Ele->GetComponentByClass(UBrain::StaticClass()));
            if (Agent)
            {
                Agent->BeginEpisode();
            }
            else
            {
                UE_LOG(LogRL, Warning,
                       TEXT("BeginEpisode: Input agent %s is not an RL "
                            "agent. Skipping..."),
                       *Ele->GetName());
            }
        }

        return;
    };

    BIND_SYNC(GetAgentStepInfo)
        << [&](const std::string& ActorName) -> unrealrl::StepInfo {
        unrealrl::StepInfo Info;

        AActor* ParentActor =
            UServerManager::Get().ActorNameToRefMap[ActorName.c_str()];
        if (ParentActor)
        {
            UBrain* Agent = Cast<UBrain>(
                ParentActor->GetComponentByClass(UBrain::StaticClass()));
            if (Agent)
            {
                Agent->GetStepInfo(Info);
            }
            else
            {
                UE_LOG(LogRL, Warning,
                       TEXT("GetStepInfo: Input agent %s is not an RL "
                            "agent."),
                       *ActorName.c_str());
            }
        }
        else
        {
            UE_LOG(LogRL, Warning,
                   TEXT("GetStepInfo: Does not contain required agent. "
                        "Skipping for agent with name %s"),
                   *ActorName.c_str());
        }

        return Info;
    };
}

} // namespace unrealrl
