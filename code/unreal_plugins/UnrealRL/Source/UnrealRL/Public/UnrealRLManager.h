// Modified from Carla (https://carla.org/) project, thanks to MIT license.

#pragma once

#if UNREALRL_DEBUG
#include <chrono>
#endif

#include <string>
#include <atomic> //since TAtomic is deprecated

#include "Templates/UniquePtr.h"

#if UNREALRL_DEBUG
// For UE4 Profiler ~ Stat Group
DECLARE_STATS_GROUP(TEXT("UnrealRLManager"),
                    STATGROUP_UnrealRLManager,
                    STATCAT_Advanced);
#endif

namespace unrealrl
{
class UNREALRL_API UnrealRLManager
{
public:
    UnrealRLManager(std::string ServerAddress = "", uint16_t port = 8080u);
    ~UnrealRLManager();

    /** Dispatch workerthreads for rpc server asynchronously */
    void AsyncRun(uint32 NumberOfWorkerThreads);

    /** Use this function to run rpc functions that are bound via io_context to
     * run until all work is finished */
    void SyncRun();

    /** To stop server thread */
    void Stop();

    /** Bind all sync and async funtions here */
    void BindFunctions();

    /** Called at the beginning of the simulation/game */
    void NotifyInitGame();

    /**
     * Bind all useful delegates!
     */
    void BindDelegates();

    /** Query whether simulation is running in synchronous mode */
    static bool IsSynchronousMode()
    {
        return bSynchronousMode;
    }

    /** Use this to change synchronous mode of simulation
     * @param bIsSyncMode: false to set async mode, true for sync mode
     */
    void SetSynchronousMode(bool bIsSyncMode);

    /** Query frame count **/
    static uint64_t GetFrameCounter()
    {
        return FrameCounter;
    }

    /** Increment frame counter and return it's value */
    static uint64_t UpdateFrameCounter()
    {
        FrameCounter += 1;
        return FrameCounter;
    }

    /** Reset frame counter to desired value, default is 0*/
    static void ResetFrameCounter(uint64_t Value = 0)
    {
        FrameCounter = Value;
    }

    /**
     * Callback function for when OnBeginFrame event broadcasts
     */
    void OnBeginFrame();

    /**
     * Delegate to catch beginning of a frame.
     * Caution: This is likely the earliest begin frame event called during
     * Engine Tick().
     */
    FDelegateHandle OnBeginFrameDelegate;

#if UNREALRL_DEBUG
    /** Callback function when OnWorldTickStart event broadcasts */
    void OnWorldTickStart(UWorld*, ELevelTick TickType, float DeltaSeconds);

    /** Callback function for when OnWorldPostActorTick event broadcasts */
    void OnWorldPostActorTick(UWorld*, ELevelTick TickType, float DeltaSeconds);

    /**
     * Callback function for when OnEndFrame event broadcasts
     */
    void OnEndFrame();

    /**
     * Callback function for when OnBeginFrameRT event broadcasts
     */
    void OnBeginFrameRT();

    /**
     * Callback function for when OnEndFrameRT event broadcasts
     */
    void OnEndFrameRT();

    /**
     * Delegate to catch world tick start event.
     * This also gets notified before all actors tick.
     */
    FDelegateHandle OnWorldTickStartDelegate;

    /**
     * Delegate to catch world tick's post actor tick event.
     * @see LevelTick.cpp's UWorld::Tick() function for more details about how
     * Ticks are ordered.
     */
    FDelegateHandle OnWorldPostActorTickDelegate;

    /**
     * Delegate to catch end of a frame.
     * Caution: This is likely the latest end frame event called during Engine
     * Tick().
     */
    FDelegateHandle OnEndFrameDelegate;

    /**
     * Delegate to catch beginning of a Rendering frame.
     * This is called after BeginFrame event is broadcasted.
     */
    FDelegateHandle OnBeginFrameRTDelegate;

    /**
     * Delegate to catch end of a Rendering frame.
     * This is called after EndFrame event is broadcasted.
     */
    FDelegateHandle OnEndFrameRTDelegate;
#endif

    /** keep track of number of frames elapsed since the start of simulation */
    static uint64_t FrameCounter;

    /** Keep track of game's running state */
    bool bIsGameRunning;

    /** Keep track of whether tick completed or not */
    std::atomic<bool> bOneTickCompleted;

    /** Allow GameThread and RHIThread sync */
    bool bAllowOneFrameRTThreadLag = false;

#if UNREALRL_DEBUG
    std::chrono::high_resolution_clock::time_point t_begin_frame;
    std::chrono::high_resolution_clock::time_point t_end_frame;
    std::chrono::high_resolution_clock::time_point t_begin_frame_rt;
    std::chrono::high_resolution_clock::time_point t_end_frame_rt;
#endif

private:
    /** Keep track of the mode in which simulation is running */
    static bool bSynchronousMode;

    /**
     * Forward declare ServerNode class
     * This follows the pImpl idiom.
     */
    class ServerNode;
    TUniquePtr<ServerNode> ServerNodePtr;
};
} // namespace unrealrl
