//
// Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <memory>  // std::make_unique, std::unique_ptr
#include <mutex>   // std::lock_guard
#include <string>
#include <utility> // std::move
#include <vector>

#include "SpCore/Assert.h"
#include "SpCore/SharedMemory.h"
#include "SpCore/SpArray.h"
#include "SpCore/Std.h"
#include "SpCore/Unreal.h"

#include "SpServices/EntryPointBinder.h"
#include "SpServices/Service.h"

class SharedMemoryService : public Service
{
public:
    SharedMemoryService() = delete;
    SharedMemoryService(CUnrealEntryPointBinder auto* unreal_entry_point_binder) : Service("SharedMemoryService")
    {
        SP_ASSERT(unreal_entry_point_binder);

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("shared_memory_service", "create_shared_memory_region",
            [this](std::string& shared_memory_name, uint64_t& num_bytes, std::vector<std::string>& usage_flag_strings) -> SpArraySharedMemoryView {
                std::lock_guard<std::mutex> lock(mutex_);

                std::string name = shared_memory_name; // need to copy so we can pass to functions that expect a const ref
                SP_ASSERT(!Std::containsKey(shared_memory_regions_, name));
                SP_ASSERT(!Std::containsKey(shared_memory_views_, name));

                std::unique_ptr<SharedMemoryRegion> shared_memory_region = std::make_unique<SharedMemoryRegion>(num_bytes);
                SP_ASSERT(shared_memory_region);            
                SharedMemoryView view = shared_memory_region->getView(); // get view before moving 
                Std::insert(shared_memory_regions_, shared_memory_name, std::move(shared_memory_region));

                SpArraySharedMemoryUsageFlags usage_flags = Unreal::getCombinedEnumFlagValueFromStringsAs<SpArraySharedMemoryUsageFlags, ESpArraySharedMemoryUsageFlags>(usage_flag_strings);
                SpArraySharedMemoryView shared_memory_view = SpArraySharedMemoryView(view, usage_flags);
                Std::insert(shared_memory_views_, shared_memory_name, shared_memory_view);

                return shared_memory_view;
        });

        unreal_entry_point_binder->bindFuncToExecuteOnWorkerThread("shared_memory_service", "destroy_shared_memory_region",
            [this](std::string& shared_memory_name) -> void {
                std::lock_guard<std::mutex> lock(mutex_);

                std::string name = shared_memory_name; // need to copy so we can pass to functions that expect a const ref
                SP_ASSERT(Std::containsKey(shared_memory_regions_, name));
                SP_ASSERT(Std::containsKey(shared_memory_views_, name));
                Std::remove(shared_memory_regions_, name);
                Std::remove(shared_memory_views_, name);
        });
    }

    std::map<std::string, SpArraySharedMemoryView> getSharedMemoryViews()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return shared_memory_views_;
    }

private:
    std::mutex mutex_; // used to coordinate read and write access to shared_memory_regions_ and shared_memory_views_
    std::map<std::string, std::unique_ptr<SharedMemoryRegion>> shared_memory_regions_;
    std::map<std::string, SpArraySharedMemoryView> shared_memory_views_;
};
