//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

//
// This header participates in SP_PLATFORM_PURE builds.
//

#pragma once

#include <stddef.h> // size_t
#include <stdint.h> // int64_t, uint8_t, uint64_t

#include <map>
#include <span>
#include <string>
#include <vector>

#include "SpCore/SpTypes.h"

#include "SpServices/Platform.h"

#if SP_PLATFORM_SERVER
    #include <Delegates/IDelegateInstance.h> // FDelegateHandle
#elif SP_PLATFORM_CLIENT
    #include <nanobind/ndarray.h>
#else
    #error
#endif

//
// This header defines types that are exchanged between the server and the client via msgpack. Each type has a server
// definition and a mirrored client definition, and the two definitions must remain compatible with each other according
// to the msgpack adaptors in SpServices/MsgpackAdaptors.h. We place each pair of definitions next to each other so
// they're easier to keep in sync. If a type has the same definition on the server and the client, we define it in an
// SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT block to indicate that it is a mirrored type. Any type that is only needed
// by one side is defined in the corresponding SP_PLATFORM_SERVER or SP_PLATFORM_CLIENT block. See SpCore/SpTypes.h for
// lower-level mirrored types.
//

//
// SpFuncSignatureDesc is a helper struct for tracking the function signatures of RPC entry points.
//

#if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
    struct SpFuncSignatureDesc // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        std::string name_;
        std::vector<int> type_ids_;
        std::vector<std::string> type_names_;
    };
#endif

//
// SpFuture is a weakly typed wrapper around an std::future<T>, with sufficient metadata to ensure that it is
// being cast correctly when casting back to a particular type.
//

#if SP_PLATFORM_SERVER
    struct SpFuture // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        void* future_ptr_ = nullptr;
        std::string type_id_;
    };
#elif SP_PLATFORM_CLIENT
    struct SpFuture
    {
        uint64_t future_ptr_ = 0;
        std::string type_id_;
    };
#else
    #error
#endif

//
// SpFunctionDesc stores metadata about a UFunction.
//

#if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
    struct SpFunctionDesc // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        SpHandle function_;     // SpHandle needed so SpFunctionDesc can be serialized safely on the worker thread
        std::string function_name_;
        SpHandle static_class_; // SpHandle needed so SpFunctionDesc can be serialized safely on the worker thread
        std::string static_class_name_;
    };
#endif

//
// SpStaticStructDesc stores reflection metadata for a UScriptStruct.
//

#if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
    struct SpStaticStructDesc // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        SpHandle static_struct_; // SpHandle needed so SpStaticStructDesc can be serialized safely on the worker thread
        std::string name_;
    };
#endif

//
// SpStaticClassDesc stores reflection metadata for a UClass.
//

#if SP_PLATFORM_SERVER || SP_PLATFORM_CLIENT
    struct SpStaticClassDesc // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        SpHandle static_class_;                         // SpHandle needed so SpStaticClassDesc can be serialized safely on the worker thread
        std::string name_;
        std::vector<SpHandle> derived_to_base_classes_; // SpHandle needed so SpStaticClassDesc can be serialized safely on the worker thread; includes the class itself
        std::vector<std::string> derived_to_base_class_names_;
        std::map<std::string, SpFunctionDesc> function_descs_;
    };
#endif

//
// SpWorldDesc stores the state of a tracked UWorld.
//

#if SP_PLATFORM_SERVER
    struct SpWorldDesc // exposed to Python via SpServices/MsgpackAdaptors.h
    {
        SpHandle world_; // SpHandle needed so SpWorldDesc can be serialized safely on the worker thread
        int64_t world_id_ = -1;
        bool is_editor_world_ = false;
        bool is_game_world_ = false;
        bool is_playing_ = false;
        FDelegateHandle world_begin_play_handle_;
    };
#elif SP_PLATFORM_CLIENT
    struct SpWorldDesc
    {
        SpHandle world_;
        int64_t world_id_ = -1;
        bool is_editor_world_ = false;
        bool is_game_world_ = false;
        bool is_playing_ = false;
    };
#else
    #error
#endif

//
// The server definitions of SpPackedArray and SpFuncDataBundle are in SpCore/SpTypes.h. The client definitions
// depend on nanobind, so they are defined here instead.
//

#if SP_PLATFORM_CLIENT
    struct SpPackedArray
    {
        nanobind::ndarray<nanobind::numpy> data_;
        std::string data_source_ = "Invalid";
        std::vector<size_t> shape_;
        std::string shared_memory_name_;
    };

    struct SpFuncDataBundle
    {
        std::map<std::string, SpPackedArray> packed_arrays_;
        std::map<std::string, std::string> unreal_obj_strings_;
        std::string info_;
    };
#endif

//
// SpPackedArrayView and SpFuncDataBundleView are client-only types that provide zero-copy views into msgpack
// data received from the server. They can be converted to SpPackedArray and SpFuncDataBundle respectively.
//

#if SP_PLATFORM_CLIENT
    struct SpPackedArrayView
    {
        std::span<uint8_t> view_;
        std::string data_source_ = "Invalid";
        std::vector<size_t> shape_;
        std::string shared_memory_name_;
        std::string data_type_;
    };

    struct SpFuncDataBundleView
    {
        std::map<std::string, SpPackedArrayView> packed_array_views_;
        std::map<std::string, std::string> unreal_obj_strings_;
        std::string info_;
    };
#endif

//
// SpClientBase is the base class for the Client class in python_ext. It contains the client's configuration flags,
// so they can be accessed by code that can't include python_ext/cpp/client.h, e.g., MsgpackUtils::convert(...).
//

#if SP_PLATFORM_CLIENT
    struct SpClientBase
    {
        bool force_return_aligned_arrays_ = false;
        bool suppress_default_logging_    = false;
        bool verbose_rpc_calls_           = false;
        bool verbose_allocations_         = false;
        bool verbose_exceptions_          = false; // owner should set to true once a connection to the server is established
    };
#endif
