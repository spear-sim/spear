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

class SpOutputDevice : public FOutputDevice
{
public:
    SpOutputDevice() = delete;
    SpOutputDevice(int num_messages) : messages_(num_messages) {}
    ~SpOutputDevice() = default;

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

std::unique_ptr<SpOutputDevice> g_output_device;

void OutputLog::requestInitialize()
{
    SP_ASSERT(!g_output_device);

    bool enable = true;
    int num_messages = 8192;

    if (Config::isInitialized()) {
        enable = Config::get<bool>("SP_CORE.OUTPUT_DEVICE.ENABLE");
        num_messages = Config::get<int>("SP_CORE.OUTPUT_DEVICE.NUM_MESSAGES");
    }

    if (enable) {
        SP_ASSERT(num_messages > 0);
        SP_LOG("    Initializing output device system...");
        g_output_device = std::make_unique<SpOutputDevice>(num_messages);
        GLog->AddOutputDevice(g_output_device.get());
    }
}

void OutputLog::terminate()
{
    if (g_output_device) {
        SP_LOG("    Terminating output device system...");
        GLog->RemoveOutputDevice(g_output_device.get());
        g_output_device = nullptr;
    }
}

bool OutputLog::isInitialized()
{
    return g_output_device != nullptr;
}

SpOutputLogMessages OutputLog::flush()
{
    SP_ASSERT(isInitialized());
    return g_output_device->flush();
}
