//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/OutputLog.h"

#include <boost/predef.h> // BOOST_OS_LINUX

#include <iostream> // std::cout
#include <memory>   // std::make_unique, std::unique_ptr
#include <mutex>    // std::lock_guard, std::mutex
#include <string>
#include <vector>

#include <CoreGlobals.h>       // GLog
#include <Logging/LogVerbosity.h>
#include <Misc/CommandLine.h>  // FCommandLine
#include <Misc/OutputDevice.h> // FOutputDevice
#include <Misc/Parse.h>        // FParse
#include <UObject/NameTypes.h> // FName

#include "SpCore/Assert.h"
#include "SpCore/Config.h"
#include "SpCore/Log.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

class StdOutputDevice : public FOutputDevice
{
public:
    StdOutputDevice() = default;
    ~StdOutputDevice() = default;

    // GLog can invoke output devices from arbitrary threads and from a panic/crash context, so we need to
    // explicitly opt in to both, otherwise GLog will skip this device in those situations and we will
    // silently miss messages, e.g., messages emitted while handling a crash.
    bool CanBeUsedOnAnyThread() const override { return true; }
    bool CanBeUsedOnPanicThread() const override { return true; }

    void Serialize(const TCHAR* message, ELogVerbosity::Type verbosity, const FName& category) override
    {
        //
        // In SPEAR, we want to print SP_LOG messages to stdout by default without relying on Unreal logging.
        // But we also always want these messages to go through the UE_LOG macro so they are visible in the
        // editor. However, due to an inconsistency in how UE_LOG is implemented on Linux, SP_LOG messages
        // will be duplicated by default. Each message will get printed once internally by SP_LOG and then
        // again by an internal Linux-only printf() statement in Unreal. In order to avoid this duplication
        // on Linux, a user must pass -nostdout. We include this command-line argument by default on all
        // platforms in default_config.spear.yaml, but it only has an effect on Linux.
        //
        // Because we print all non-SPEAR-originating Unreal messages to stdout by default in this Serialize(...)
        // function, it means there is no reason for a user to pass in -stdout (or -fullstdoutlogoutput),
        // unless they want to see output extremely late in the application lifecycle. If a user wants to
        // suppress our default behavior, they can pass in -sp-no-stdout or set the SP_CORE.OUTPUT_LOG.STDOUT
        // config value to false (default is True).
        //

        // SP_LOG(...) always writes LogSpear messages directly to std::cout itself (see Log::logString(...)),
        // regardless of platform, so echoing them here again is always redundant.
        if (category == LogSpear.GetCategoryName()) {
            return;
        }

        // On Linux, return early if we know Unreal's native console device is going to print this message
        // for us, i.e., when -nostdout is absent. When -nostdout is present, Unreal's native console device
        // goes silent, so we don't automatically know that we want to return early here, i.e., we still have
        // a decision to make, so we fall through to the remaining checks in this function.
        if (BOOST_OS_LINUX && !FParse::Param(FCommandLine::Get(), Unreal::toTCharPtr("nostdout"))) {
            return;
        }

        // On all platforms, check the -sp-no-stdout (or its equivalent config value SP_CORE.OUTPUT_LOG.STDOUT).
        bool sp_stdout = true;
        if (sp_stdout) {
            sp_stdout = !FParse::Param(FCommandLine::Get(), Unreal::toTCharPtr("sp-no-stdout"));
        } if (sp_stdout && Config::isInitialized()) {
            sp_stdout = Config::get<bool>("SP_CORE.OUTPUT_LOG.STDOUT");
        }

        if (!sp_stdout) {
            return;
        }

        // GLog can invoke this Serialize(...) call from a dedicated background thread rather than the
        // thread that originally logged the message, so we share a mutex with every other place that
        // writes to the terminal (see Log::getStdoutMutex()'s comment) to guarantee this write can
        // never be torn by an interleaved write from another thread.
        std::lock_guard<std::recursive_mutex> lock(Log::getStdoutMutex());
        std::cout << Unreal::toStdString(category) << ": " << Unreal::toStdString(message) << std::endl;
    }
};

class BufferedOutputDevice : public FOutputDevice
{
public:
    BufferedOutputDevice() = delete;
    BufferedOutputDevice(int num_messages) : messages_(num_messages) {}
    ~BufferedOutputDevice() = default;

    // See the identical overrides in StdOutputDevice above for why we need these.
    bool CanBeUsedOnAnyThread() const override { return true; }
    bool CanBeUsedOnPanicThread() const override { return true; }

    void Serialize(const TCHAR* message, ELogVerbosity::Type verbosity, const FName& category) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (messages_.full()) {
            num_evicted_++;
        }

        std::string message_str = Unreal::toStdString(category) + ": " + Unreal::toStdString(message);
        messages_.push_back(message_str);
    }

    SpOutputLogMessages flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        SpOutputLogMessages output_log_messages;
        output_log_messages.messages_ = Std::toVector<std::string>(messages_);
        output_log_messages.num_evicted_ = num_evicted_;

        messages_.clear();
        num_evicted_ = 0;

        return output_log_messages;
    }

private:
    boost::circular_buffer<std::string> messages_;
    int num_evicted_ = 0;
    std::mutex mutex_;
};

std::unique_ptr<StdOutputDevice> g_std_output_device;
std::unique_ptr<BufferedOutputDevice> g_buffered_output_device;

void OutputLog::requestInitialize()
{
    SP_ASSERT(!g_std_output_device);
    SP_ASSERT(!g_buffered_output_device);

    SP_LOG("    Initializing StdOutputDevice...");
    g_std_output_device = std::make_unique<StdOutputDevice>();
    GLog->AddOutputDevice(g_std_output_device.get());
    GLog->SerializeBacklog(g_std_output_device.get()); // recovers early messages, editor only, see GIsEditor in LaunchEngineLoop.cpp

    bool enable_buffered_output_device = true;
    int buffered_output_device_num_messages = 8192;

    if (Config::isInitialized()) {
        enable_buffered_output_device = Config::get<bool>("SP_CORE.OUTPUT_LOG.ENABLE_BUFFERED_OUTPUT_DEVICE");
        buffered_output_device_num_messages = Config::get<int>("SP_CORE.OUTPUT_LOG.BUFFERED_OUTPUT_DEVICE_NUM_MESSAGES");
    }

    if (enable_buffered_output_device) {
        SP_ASSERT(buffered_output_device_num_messages > 0);
        SP_LOG("    Initializing BufferedOutputDevice...");
        g_buffered_output_device = std::make_unique<BufferedOutputDevice>(buffered_output_device_num_messages);
        GLog->AddOutputDevice(g_buffered_output_device.get());
        GLog->SerializeBacklog(g_buffered_output_device.get());
    }
}

void OutputLog::terminate()
{
    if (g_buffered_output_device) {
        SP_LOG("    Terminating BufferedOutputDevice...");
        GLog->RemoveOutputDevice(g_buffered_output_device.get());
        g_buffered_output_device = nullptr;
    }

    if (g_std_output_device) {
        SP_LOG("    Terminating StdOutputDevice...");
        GLog->RemoveOutputDevice(g_std_output_device.get());
        g_std_output_device = nullptr;
    }
}

SpOutputLogMessages OutputLog::flush()
{
    if (g_buffered_output_device != nullptr) {
        return g_buffered_output_device->flush();
    } else {
        return SpOutputLogMessages();
    }
}
