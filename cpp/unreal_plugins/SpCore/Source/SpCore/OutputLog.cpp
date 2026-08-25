//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpCore/OutputLog.h"

#include <iostream> // std::cout
#include <memory>   // std::make_unique, std::unique_ptr
#include <mutex>    // std::lock_guard, std::mutex
#include <string>
#include <vector>

#include <CoreGlobals.h>       // GLog
#include <Logging/LogVerbosity.h>
#include <Misc/OutputDevice.h> // FOutputDevice
#include <UObject/NameTypes.h> // FName

#include "SpCore/Assert.h"
#include "SpCore/Boost.h"
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
        if (category != LogSpear.GetCategoryName()) {
            // GLog can invoke this Serialize(...) call from a dedicated background thread rather than the
            // thread that originally logged the message, so we share a mutex with every other place that
            // writes to the terminal (see Log::getStdoutMutex()'s comment) to guarantee this write can
            // never be torn by an interleaved write from another thread.
            std::lock_guard<std::recursive_mutex> lock(Log::getStdoutMutex());
            std::cout << Unreal::toStdString(category) << ": " << Unreal::toStdString(message) << std::endl;
        }
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
