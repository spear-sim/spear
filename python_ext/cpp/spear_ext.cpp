//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t, uintptr_t

#ifdef _MSC_VER
    #include <malloc.h> // _aligned_alloc
#else
    #include <stdlib.h> // posix_memalign
#endif

#include <chrono> 
#include <concepts>    // std::same_as
#include <cstring>     // std::memcpy
#include <exception>   // std::terminate
#include <iostream>
#include <map>
#include <memory>      // std::make_shared, std::make_unique, std::shared_ptr, std::unique_ptr
#include <span>
#include <sstream>
#include <stdexcept>   // std::runtime_error
#include <string>
#include <thread>      // std::this_thread
#include <type_traits> // std::is_void_v
#include <utility>     // std::move
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <rpc/client.h>

//
// Minimal assert macro that is enabled in release builds
//

#define SP_ASSERT(expression)                                                                     \
    do {                                                                                          \
        if (!(expression)) {                                                                      \
            std::stringstream ss;                                                                 \
            ss << "Assert failed: " << #expression << " (" << __FILE__ << ":" << __LINE__ << ")"; \
            std::cout << ss.str() << std::endl;                                                   \
            throw std::runtime_error(ss.str());                                                   \
        }                                                                                         \
    }                                                                                             \
    while (false)

//
// Minimal current function macro
//

#ifdef _MSC_VER
    #define SP_CURRENT_FUNCTION __FUNCSIG__
#else
    #define SP_CURRENT_FUNCTION __PRETTY_FUNCTION__
#endif

//
// Minimal 16-bit floating point data type
//

struct sp_float16_t { uint16_t data; };
template <>
struct nanobind::detail::dtype_traits<sp_float16_t>
{
    //                                       type code,                                       size in bits, SIMD lanes (usually set to 1) 
    static constexpr dlpack::dtype value = { static_cast<uint8_t>(dlpack::dtype_code::Float), 16,           1};
    static constexpr auto name = const_name("sp_float16_t");
};

//
// Minimal aligned allocator
//

static constexpr int g_alignment_padding_bytes = 4096;

template <typename TValue>
struct SpPackedArrayAllocatorImpl
{
    using value_type = TValue;

    SpPackedArrayAllocatorImpl() noexcept {};

    template <typename TOtherValue>
    SpPackedArrayAllocatorImpl(const SpPackedArrayAllocatorImpl<TOtherValue>&) noexcept {}

    TValue* allocate(const std::size_t num_elements) const
    {
        SP_ASSERT(num_elements > 0);
        int num_bytes = num_elements*sizeof(TValue);
        int num_bytes_internal;
        if (num_bytes % g_alignment_padding_bytes == 0) {
            num_bytes_internal = num_bytes;
        } else {
            num_bytes_internal = num_bytes + g_alignment_padding_bytes - (num_bytes % g_alignment_padding_bytes);
        }
        SP_ASSERT(num_bytes_internal % g_alignment_padding_bytes == 0);

        #ifdef _MSC_VER
            return static_cast<TValue*>(_aligned_malloc(num_bytes_internal, g_alignment_padding_bytes));
        #else
            void* ptr = nullptr;
            int error = posix_memalign(&ptr, g_alignment_padding_bytes, num_bytes_internal);
            SP_ASSERT(!error);
            SP_ASSERT(ptr);
            return static_cast<TValue*>(ptr);
        #endif
    }

    void deallocate(TValue* const ptr, std::size_t) const
    {
        SP_ASSERT(ptr);
        #ifdef _MSC_VER
            _aligned_free(ptr);
        #else
            free(ptr);
        #endif
    }

    template <typename TOtherValue>
    bool operator==(const SpPackedArrayAllocatorImpl<TOtherValue>&) const noexcept { return true; }

    template <typename TOtherValue>
    bool operator!=(const SpPackedArrayAllocatorImpl<TOtherValue>&) const noexcept { return false; }
};

using SpPackedArrayAllocator = SpPackedArrayAllocatorImpl<uint8_t>;

//
// Statics
//

struct Statics
{
    inline static bool s_force_return_aligned_arrays_ = false;
    inline static bool s_verbose_rpc_calls_ = false;
    inline static bool s_verbose_exceptions_ = true;
    inline static bool s_verbose_allocations_ = false;
};

//
// Custom types
//

struct PropertyDesc
{
    uint64_t property_ = 0;
    uint64_t value_ptr_ = 0;
};

struct SharedMemoryView
{
    std::string id_;
    uint64_t num_bytes_ = 0;
    uint64_t offset_bytes_ = 0;
    std::vector<std::string> usage_flags_ = {"DoNotUse"};
};

struct PackedArray
{
    nanobind::ndarray<nanobind::numpy> data_;
    std::string data_source_ = "Invalid";
    std::vector<size_t> shape_;
    std::string shared_memory_name_;
};

struct Future
{
    uint64_t future_ptr_ = 0;
    std::string type_id_;
};

struct DataBundle
{
    std::map<std::string, PackedArray> packed_arrays_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};

//
// View types
//

struct PackedArrayView
{
    std::span<uint8_t> view_;
    std::string data_source_ = "Invalid";
    std::vector<size_t> shape_;
    std::string data_type_;
    std::string shared_memory_name_;
};

struct DataBundleView
{
    std::map<std::string, PackedArrayView> packed_array_views_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};

//
// Client
//

class Client
{
public:
    Client() = delete;
    Client(const std::string& address, const uint16_t& port)
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::Client(...)" << std::endl;
        client_ = std::make_unique<rpc::client>(address, port);
    };

    ~Client()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::~Client()" << std::endl;
        client_ = nullptr;
    };

    void initialize(const std::string& address, const uint16_t& port)
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::initialize(...)" << std::endl;
        SP_ASSERT(!client_);
        client_ = std::make_unique<rpc::client>(address, port);
    };

    void terminate()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::terminate()" << std::endl;
        SP_ASSERT(client_);
        client_ = nullptr;
    };

    int64_t getTimeout() const
    {
        SP_ASSERT(client_);
        nonstd::optional<int64_t> timeout = client_->get_timeout();
        if (timeout.has_value()) {
            return *timeout;
        } else {
            return -1;
        }
    };

    void setTimeout(int64_t value)
    {
        SP_ASSERT(client_);
        client_->set_timeout(value);
    };

    void clearTimeout()
    {
        SP_ASSERT(client_);
        client_->clear_timeout();
    };

    template <typename TReturn, typename... TArgs>
    TReturn call(const std::string& func_name, TArgs... args)
    {
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] call<...>(...): " << func_name << std::endl; }

        return executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> TReturn {
            clmdep_msgpack::object_handle object_handle = client_->call(func_name, args...);
            return getReturnValue<TReturn>(std::move(object_handle));
        });
    };

    template <typename... TArgs>
    Future callAsync(const std::string& func_name, TArgs... args)
    {
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] callAsync(...): " << func_name << std::endl; }

        return executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> Future {
            clmdep_msgpack::object_handle object_handle = client_->call(func_name, args...);
            return getReturnValue<Future>(std::move(object_handle));
        });
    };

    template <typename... TArgs>
    void sendAsync(const std::string& func_name, TArgs... args)
    {
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] sendAsync(...): " << func_name << std::endl; }

        executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> void {
            client_->call(func_name, args...);
        });
    };

    template <typename TReturn>
    TReturn getFutureResult(const Future& future)
    {
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] getFutureResult<...>(...): " << future.future_ptr_ << std::endl; }

        return executeFuncInTryCatch("getFutureResult<...>(...)", [this, &future]() -> TReturn {
            clmdep_msgpack::object_handle object_handle = client_->call("engine_service.get_future_result_as_" + getTypeName<TReturn>(), future);
            return getReturnValue<TReturn>(std::move(object_handle));
        });
    };

    template <typename... TArgs>
    uint64_t callAsyncFast(const std::string& func_name, TArgs... args)
    {
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] callAsyncFast(...): " << func_name << std::endl; }

        return executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> uint64_t {
            std::future<clmdep_msgpack::object_handle> std_future = client_->async_call(func_name, args...);
            return createFutureFast(std::move(std_future));
        });
    };

    template <typename... TArgs>
    void sendAsyncFast(const std::string& func_name, TArgs... args)
    {
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] sendAsyncFast(...): " << func_name << std::endl; }

        executeFuncInTryCatch(func_name, [this, &func_name, &args...]() -> void {
            client_->send(func_name, args...);
        });
    };

    template <typename TReturn>
    TReturn getFutureResultFast(uint64_t future_handle)
    {
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] getFutureResultFast<...>(...): " << future_handle << std::endl; }

        return destroyFutureFast<TReturn>(future_handle);
    };

    template <typename TReturn, typename... TArgs>
    static void registerEntryPointSignature(nanobind::class_<Client>& client_class)
    {
        std::string return_type_name                      = getTypeName<TReturn>();
        std::string call_as_return_type                   = "call_as_" + return_type_name;
        std::string get_future_result_as_return_type      = "get_future_result_as_" + return_type_name;
        std::string get_future_result_fast_as_return_type = "get_future_result_fast_as_" + return_type_name;

        client_class.def(call_as_return_type.c_str(), &Client::call<TReturn, TArgs...>);

        client_class.def("call_async",                             &Client::callAsync<TArgs...>);
        client_class.def("send_async",                             &Client::sendAsync<TArgs...>);
        client_class.def(get_future_result_as_return_type.c_str(), &Client::getFutureResult<TReturn>);

        client_class.def("call_async_fast",                             &Client::callAsyncFast<TArgs...>);
        client_class.def("send_async_fast",                             &Client::sendAsyncFast<TArgs...>);
        client_class.def(get_future_result_fast_as_return_type.c_str(), &Client::getFutureResultFast<TReturn>);
    };

private:
    template <typename TFunc>
    auto executeFuncInTryCatch(const std::string& func_name, const TFunc& func)
    {
        using TReturn = std::invoke_result_t<TFunc>;

        SP_ASSERT(client_);

        try {
            if constexpr (std::is_void_v<TReturn>) {
                if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Executing function: " << func_name << std::endl; }

                func();

                if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Finished executing function: " << func_name << std::endl; }

            } else {
                if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Executing function: " << func_name << std::endl; }

                TReturn return_value = func();

                if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Finished executing function: " << func_name << std::endl; }
                return return_value;
            }
        } catch (const std::exception& e) {
            if (Statics::s_verbose_exceptions_) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl; }
            std::rethrow_exception(std::current_exception());
        } catch (...) {
            if (Statics::s_verbose_exceptions_) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl; }
            std::rethrow_exception(std::current_exception());
        }

        return TReturn();
    };

    template <typename TReturn> static std::string getTypeName() { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Current function: " << SP_CURRENT_FUNCTION << std::endl; SP_ASSERT(false); return ""; }
    template <> std::string getTypeName<void>                                    () { return "void";                                }
    template <> std::string getTypeName<bool>                                    () { return "bool";                                }
    template <> std::string getTypeName<float>                                   () { return "float";                               }
    template <> std::string getTypeName<int64_t>                                 () { return "int64";                               }
    template <> std::string getTypeName<uint64_t>                                () { return "uint64";                              }
    template <> std::string getTypeName<std::string>                             () { return "string";                              }
    template <> std::string getTypeName<std::vector<double>>                     () { return "vector_of_double";                    }
    template <> std::string getTypeName<std::vector<uint64_t>>                   () { return "vector_of_uint64";                    }
    template <> std::string getTypeName<std::vector<std::string>>                () { return "vector_of_string";                    }
    template <> std::string getTypeName<std::map<std::string, uint64_t>>         () { return "map_of_string_to_uint64";             }
    template <> std::string getTypeName<std::map<std::string, std::string>>      () { return "map_of_string_to_string";             }
    template <> std::string getTypeName<std::map<std::string, SharedMemoryView>> () { return "map_of_string_to_shared_memory_view"; }
    template <> std::string getTypeName<std::map<std::string, PackedArray>>      () { return "map_of_string_to_packed_array";       }
    template <> std::string getTypeName<PropertyDesc>                            () { return "property_desc";                       }
    template <> std::string getTypeName<SharedMemoryView>                        () { return "shared_memory_view";                  }
    template <> std::string getTypeName<PackedArray>                             () { return "packed_array";                        }
    template <> std::string getTypeName<Future>                                  () { return "future";                              }
    template <> std::string getTypeName<DataBundle>                              () { return "data_bundle";                         }

    template <typename TReturn> static TReturn getReturnValue(clmdep_msgpack::object_handle&& object_handle) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Current function: " << SP_CURRENT_FUNCTION << std::endl; SP_ASSERT(false); return TReturn(); }
    template <> void                                    getReturnValue<void>                                    (clmdep_msgpack::object_handle&& object_handle) { return; }
    template <> bool                                    getReturnValue<bool>                                    (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<bool>                                                                       (std::move(object_handle)); }
    template <> float                                   getReturnValue<float>                                   (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<float>                                                                      (std::move(object_handle)); }
    template <> int64_t                                 getReturnValue<int64_t>                                 (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<int64_t>                                                                    (std::move(object_handle)); }
    template <> uint64_t                                getReturnValue<uint64_t>                                (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<uint64_t>                                                                   (std::move(object_handle)); }
    template <> std::string                             getReturnValue<std::string>                             (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::string>                                                                (std::move(object_handle)); }
    template <> std::vector<double>                     getReturnValue<std::vector<double>>                     (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::vector<double>>                                                        (std::move(object_handle)); }
    template <> std::vector<uint64_t>                   getReturnValue<std::vector<uint64_t>>                   (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::vector<uint64_t>>                                                      (std::move(object_handle)); }
    template <> std::vector<std::string>                getReturnValue<std::vector<std::string>>                (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::vector<std::string>>                                                   (std::move(object_handle)); }
    template <> std::map<std::string, uint64_t>         getReturnValue<std::map<std::string, uint64_t>>         (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, uint64_t>>                                            (std::move(object_handle)); }
    template <> std::map<std::string, std::string>      getReturnValue<std::map<std::string, std::string>>      (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, std::string>>                                         (std::move(object_handle)); }
    template <> std::map<std::string, SharedMemoryView> getReturnValue<std::map<std::string, SharedMemoryView>> (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, SharedMemoryView>>                                    (std::move(object_handle)); }
    template <> std::map<std::string, PackedArray>      getReturnValue<std::map<std::string, PackedArray>>      (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<std::map<std::string, PackedArray>, std::map<std::string, PackedArrayView>> (std::move(object_handle)); }
    template <> PropertyDesc                            getReturnValue<PropertyDesc>                            (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<PropertyDesc>                                                               (std::move(object_handle)); }
    template <> SharedMemoryView                        getReturnValue<SharedMemoryView>                        (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<SharedMemoryView>                                                           (std::move(object_handle)); }
    template <> PackedArray                             getReturnValue<PackedArray>                             (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<PackedArray, PackedArrayView>                                               (std::move(object_handle)); }
    template <> Future                                  getReturnValue<Future>                                  (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<Future>                                                                     (std::move(object_handle)); }
    template <> DataBundle                              getReturnValue<DataBundle>                              (clmdep_msgpack::object_handle&& object_handle) { return getReturnValueImpl<DataBundle, DataBundleView>                                                 (std::move(object_handle)); }

    template <typename TReturn>
    static TReturn getReturnValueImpl(clmdep_msgpack::object_handle&& object_handle)
    {
        return object_handle.template as<TReturn>(); // .template needed on macOS
    };

    template <typename TReturnDest, typename TReturnSrc>
    static TReturnDest getReturnValueImpl(clmdep_msgpack::object_handle&& object_handle)
    {
        std::shared_ptr<clmdep_msgpack::object_handle> object_handle_ptr = std::make_shared<clmdep_msgpack::object_handle>(std::move(object_handle));
        TReturnSrc return_value_src = object_handle_ptr->template as<TReturnSrc>(); // .template needed on macOS
        return convert<TReturnDest>(std::move(return_value_src), object_handle_ptr);
    };

    uint64_t createFutureFast(std::future<clmdep_msgpack::object_handle>&& std_future)
    {
        std::future<clmdep_msgpack::object_handle>* std_future_ptr = new std::future<clmdep_msgpack::object_handle>(std::move(std_future));
        SP_ASSERT(std_future_ptr);
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Allocated std::future<clmdep_msgpack::object_handle> object at memory location: " << std_future_ptr << std::endl; }
        uint64_t future_handle = reinterpret_cast<uint64_t>(std_future_ptr);
        return future_handle;
    }

    template <typename TReturn>
    TReturn destroyFutureFast(uint64_t future_handle)
    {
        SP_ASSERT(future_handle);
        std::future<clmdep_msgpack::object_handle>* std_future_ptr = reinterpret_cast<std::future<clmdep_msgpack::object_handle>*>(future_handle);
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Deleting std::future<clmdep_msgpack::object_handle> object at memory location: " << std_future_ptr << std::endl; }
        clmdep_msgpack::object_handle object_handle = std_future_ptr->get();
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Obtained clmdep_msgpack::object_handle from std::future<clmdep_msgpack::object_handle>..." << std::endl; }
        delete std_future_ptr;
        std_future_ptr = nullptr;

        Future future = object_handle.template as<Future>();
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Obtained Future from clmdep_msgpack::object_handle." << std::endl; }
        std::string return_type_name = getTypeName<TReturn>();
        std::string func_name = "engine_service.get_future_result_as_" + getTypeName<TReturn>();
        if (Statics::s_verbose_rpc_calls_) { std::cout << "[SPEAR | spear_ext.cpp] Preparing to call function to obtain future result: " << func_name << std::endl; }
        return call<TReturn>(func_name, future);
    }

    // needed to convert a custom view type, received as a return value from the server, into a custom type that can be returned to Python (specialized below)
    template <typename TDest, typename TSrc> static TDest convert(TSrc&& src, std::shared_ptr<clmdep_msgpack::object_handle> object_handle) { std::cout << "[SPEAR | spear_ext.cpp] ERROR: Current function: " << SP_CURRENT_FUNCTION << std::endl; SP_ASSERT(false); return TDest(); };

    std::unique_ptr<rpc::client> client_ = nullptr;
};

//
// Python module
//

NB_MODULE(spear_ext, module)
{
    //
    // Statics
    //

    auto statics_class = nanobind::class_<Statics>(module, "Statics");
    statics_class.def_rw_static("force_return_aligned_arrays", &Statics::s_force_return_aligned_arrays_);
    statics_class.def_rw_static("verbose_rpc_calls", &Statics::s_verbose_rpc_calls_);
    statics_class.def_rw_static("verbose_allocations", &Statics::s_verbose_allocations_);
    statics_class.def_rw_static("verbose_exceptions", &Statics::s_verbose_exceptions_);

    //
    // Client
    //

    auto client_class = nanobind::class_<Client>(module, "Client");
    client_class.def(nanobind::init<const std::string&, const uint16_t>());

    client_class.def("initialize", &Client::initialize);
    client_class.def("terminate", &Client::terminate);

    client_class.def("get_timeout", &Client::getTimeout);
    client_class.def("set_timeout", &Client::setTimeout);
    client_class.def("clear_timeout", &Client::clearTimeout);

    //
    // Specializations for Client::call(...), Client::callAsync(...), Client::sendAsync(...), and Client::getFutureResult(...)
    //

    // 0 args
    Client::registerEntryPointSignature<void>                                    (client_class);
    Client::registerEntryPointSignature<bool>                                    (client_class);
    Client::registerEntryPointSignature<int64_t>                                 (client_class);
    Client::registerEntryPointSignature<uint64_t>                                (client_class);
    Client::registerEntryPointSignature<std::string>                             (client_class);
    Client::registerEntryPointSignature<std::vector<double>>                     (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>>                   (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>>         (client_class);

    // 1 args 
    Client::registerEntryPointSignature<void,                                    bool>                (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t>            (client_class);
    Client::registerEntryPointSignature<void,                                    const std::string&>  (client_class);
    Client::registerEntryPointSignature<bool,                                    uint64_t>            (client_class);
    Client::registerEntryPointSignature<float,                                   uint64_t>            (client_class);
    Client::registerEntryPointSignature<int64_t,                                 uint64_t>            (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t>            (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&>  (client_class);
    Client::registerEntryPointSignature<std::string,                             uint64_t>            (client_class);
    Client::registerEntryPointSignature<std::string,                             const PropertyDesc&> (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   uint64_t>            (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&>  (client_class);
    Client::registerEntryPointSignature<std::vector<std::string>,                uint64_t>            (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t>            (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&>  (client_class);
    Client::registerEntryPointSignature<std::map<std::string, SharedMemoryView>, uint64_t>            (client_class);

    // 2 args
    Client::registerEntryPointSignature<void,                                    uint64_t,            bool>                            (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            uint64_t>                        (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            const std::string&>              (client_class);
    Client::registerEntryPointSignature<void,                                    PropertyDesc,        const std::string&>              (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            bool>                            (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            const std::string&>              (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t>                        (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  const std::string&>              (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  const std::vector<std::string>&> (client_class);
    Client::registerEntryPointSignature<std::string,                             uint64_t,            bool>                            (client_class);
    Client::registerEntryPointSignature<std::string,                             uint64_t,            uint64_t>                        (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   uint64_t,            bool>                            (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&,  const std::string&>              (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&,  const std::vector<std::string>&> (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,            bool>                            (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&,  const std::string&>              (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&,  const std::vector<std::string>&> (client_class);
    Client::registerEntryPointSignature<PropertyDesc,                            uint64_t,            const std::string&>              (client_class);

    // 3 args
    Client::registerEntryPointSignature<void,                                    uint64_t,            bool,                            const std::vector<std::string>&>           (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            int,                             const std::vector<std::string>&>           (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            float,                           const std::vector<std::string>&>           (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            uint64_t,                        const std::string&>                        (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            const std::string&,              float>                                     (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            const std::string&,              const std::string&>                        (client_class);
    Client::registerEntryPointSignature<void,                                    uint64_t,            const std::string&,              const std::vector<std::string>&>           (client_class);
    Client::registerEntryPointSignature<bool,                                    uint64_t,            bool,                            bool>                                      (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            uint64_t,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            uint64_t,                        const std::string&>                        (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            const std::string&,              const std::string&>                        (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        const std::string&>                        (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   uint64_t,            uint64_t,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&,  uint64_t,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&,  const std::vector<std::string>&, bool>                                      (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,            uint64_t,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&,  uint64_t,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&,  const std::vector<std::string>&, bool>                                      (client_class);
    Client::registerEntryPointSignature<SharedMemoryView,                        const std::string&,  int,                             const std::vector<std::string>&>           (client_class);
    Client::registerEntryPointSignature<PropertyDesc,                            uint64_t,            uint64_t,                        const std::string&>                        (client_class);
    Client::registerEntryPointSignature<DataBundle,                              uint64_t,            const std::string&,              const DataBundle&>                         (client_class);

    // 4 args
    Client::registerEntryPointSignature<void,                                    uint64_t,            const std::string&,              const std::string&,                        const std::string&>                        (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            uint64_t,                        uint64_t,                                  const std::string&>                        (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        uint64_t,                                  const std::string&>                        (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        const std::string&,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        const std::vector<std::string>&,           bool>                                      (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&,  uint64_t,                        const std::string&,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&,  uint64_t,                        const std::vector<std::string>&,           bool>                                      (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&,  uint64_t,                        const std::string&,                        bool>                                      (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&,  uint64_t,                        const std::vector<std::string>&,           bool>                                      (client_class);
    Client::registerEntryPointSignature<std::map<std::string, std::string>,      uint64_t,            uint64_t,                        const std::map<std::string, std::string>&, const std::string&>                        (client_class);

    // 5 args
    Client::registerEntryPointSignature<void,                                    uint64_t,            uint64_t,                        const std::string&,                        const std::vector<uint64_t>&,              const std::vector<uint64_t>&>              (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            const std::string&,              const std::string&,                        const std::string&,                        const std::vector<std::string>&>           (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  const std::string&,              const std::string&,                        const std::string&,                        const std::vector<std::string>&>           (client_class);
    Client::registerEntryPointSignature<std::vector<uint64_t>,                   const std::string&,  uint64_t,                        const std::vector<std::string>&,           bool,                                      bool>                                      (client_class);
    Client::registerEntryPointSignature<std::map<std::string, uint64_t>,         const std::string&,  uint64_t,                        const std::vector<std::string>&,           bool,                                      bool>                                      (client_class);
    Client::registerEntryPointSignature<PackedArray,                             uint64_t,            int64_t,                         uint64_t,                                  const std::map<std::string, PackedArray>&, const PackedArray&>                        (client_class);

    // 6 args
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            uint64_t,                        const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t>                        (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t>                        (client_class);

    // 7 args
    Client::registerEntryPointSignature<void,                                    uint64_t,            const std::string&,              const std::string&,                        const std::string&,                        const std::string&,                        const std::vector<uint64_t>&,    const std::vector<uint64_t>&>    (client_class);
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t,                        uint64_t>                        (client_class);
    Client::registerEntryPointSignature<std::map<std::string, PackedArray>,      uint64_t,            uint64_t,                        int64_t,                                   uint64_t,                                  const std::map<std::string, PackedArray>&, const std::vector<std::string>&, const std::vector<std::string>&> (client_class);

    // 8 args
    Client::registerEntryPointSignature<uint64_t,                                const std::string&,  uint64_t,                        const std::string&,                        const std::vector<std::string>&,           uint64_t,                                  bool,                            uint64_t,                        uint64_t>       (client_class);
    Client::registerEntryPointSignature<uint64_t,                                uint64_t,            uint64_t,                        const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t,                        bool,                            uint64_t>       (client_class);

    //
    // Custom types
    //

    auto property_desc_class = nanobind::class_<PropertyDesc>(module, "PropertyDesc");
    property_desc_class.def(nanobind::init<>());
    property_desc_class.def_rw("property",  &PropertyDesc::property_);
    property_desc_class.def_rw("value_ptr", &PropertyDesc::value_ptr_);

    auto shared_memory_view_class = nanobind::class_<SharedMemoryView>(module, "SharedMemoryView");
    shared_memory_view_class.def(nanobind::init<>());
    shared_memory_view_class.def_rw("id",           &SharedMemoryView::id_);
    shared_memory_view_class.def_rw("num_bytes",    &SharedMemoryView::num_bytes_);
    shared_memory_view_class.def_rw("offset_bytes", &SharedMemoryView::offset_bytes_);
    shared_memory_view_class.def_rw("usage_flags",  &SharedMemoryView::usage_flags_);

    auto packed_array_class = nanobind::class_<PackedArray>(module, "PackedArray");
    packed_array_class.def(nanobind::init<>());
    packed_array_class.def_rw("data",               &PackedArray::data_, nanobind::rv_policy::reference);
    packed_array_class.def_rw("data_source",        &PackedArray::data_source_);
    packed_array_class.def_rw("shape",              &PackedArray::shape_);
    packed_array_class.def_rw("shared_memory_name", &PackedArray::shared_memory_name_);

    auto future_class = nanobind::class_<Future>(module, "Future");
    future_class.def(nanobind::init<>());
    future_class.def_rw("future_ptr", &Future::future_ptr_);
    future_class.def_rw("type_id",    &Future::type_id_);

    auto data_bundle_class = nanobind::class_<DataBundle>(module, "DataBundle");
    data_bundle_class.def(nanobind::init<>());
    data_bundle_class.def_rw("packed_arrays",      &DataBundle::packed_arrays_, nanobind::rv_policy::reference);
    data_bundle_class.def_rw("unreal_obj_strings", &DataBundle::unreal_obj_strings_);
    data_bundle_class.def_rw("info",               &DataBundle::info_);
}

//
// Helper classes for implementing msgpack adaptors
//
 
class Std
{
public:
    Std() = delete;
    ~Std() = delete;

    template <typename TValue, typename... TVectorTraits>
    static void resizeUninitialized(std::vector<TValue, TVectorTraits...>& vector, int size)
    {
        struct TValueNoDefaultInit { TValue value; TValueNoDefaultInit() {} };

        using TVector = std::vector<TValue, TVectorTraits...>;
        using TAllocator = typename TVector::allocator_type;
        using TVectorNoDefaultInit = std::vector<TValueNoDefaultInit, typename std::allocator_traits<TAllocator>::template rebind_alloc<TValueNoDefaultInit>>;

        SP_ASSERT(sizeof(TValue)    == sizeof(TValueNoDefaultInit));
        SP_ASSERT(sizeof(TValue[2]) == sizeof(TValueNoDefaultInit[2]));
        SP_ASSERT(sizeof(TValue[4]) == sizeof(TValueNoDefaultInit[4]));

        TVectorNoDefaultInit* vector_no_default_init = reinterpret_cast<TVectorNoDefaultInit*>(&vector);
        vector_no_default_init->resize(size);

        SP_ASSERT(vector.size() == size);
    }
};

class MsgpackUtils
{
public:
    MsgpackUtils() = delete;
    ~MsgpackUtils() = delete;

    //
    // functions for receiving custom types as return values from the server
    //

    static std::map<std::string, clmdep_msgpack::object> toMapOfMsgpackObjects(clmdep_msgpack::object const& object)
    {
        SP_ASSERT(object.type == clmdep_msgpack::type::MAP);
        clmdep_msgpack::object_kv* object_kvs = static_cast<clmdep_msgpack::object_kv*>(object.via.map.ptr);
        std::map<std::string, clmdep_msgpack::object> objects;
        for (unsigned int i = 0; i < object.via.map.size; i++) { // unsigned int needed on Windows
            auto [itr, success] = objects.insert({MsgpackUtils::to<std::string>(object_kvs[i].key), std::move(object_kvs[i].val)});
            SP_ASSERT(success); // will only succeed if key wasn't already present
        }
        return objects;
    };

    template <typename T>
    static T to(clmdep_msgpack::object const& object)
    {
        T t;
        object.convert(t);
        return t;
    };

    template <>
    static std::span<uint8_t> to<std::span<uint8_t>>(clmdep_msgpack::object const& object)
    {
        SP_ASSERT(object.type == clmdep_msgpack::type::BIN);
        return std::span<uint8_t>(reinterpret_cast<uint8_t*>(const_cast<char*>(object.via.bin.ptr)), object.via.bin.size);
    }
};

class DataTypeUtils
{
public:
    DataTypeUtils() = delete;
    ~DataTypeUtils() = delete;

    static std::string getDataType(const nanobind::dlpack::dtype& data_type)
    {
        if (data_type == nanobind::dtype<uint8_t>())      return "u1";
        if (data_type == nanobind::dtype<int8_t>())       return "i1";
        if (data_type == nanobind::dtype<uint16_t>())     return "u2";
        if (data_type == nanobind::dtype<int16_t>())      return "i2";
        if (data_type == nanobind::dtype<uint32_t>())     return "u4";
        if (data_type == nanobind::dtype<int32_t>())      return "i4";
        if (data_type == nanobind::dtype<uint64_t>())     return "u8";
        if (data_type == nanobind::dtype<int64_t>())      return "i8";
        if (data_type == nanobind::dtype<sp_float16_t>()) return "f2";
        if (data_type == nanobind::dtype<float>())        return "f4";
        if (data_type == nanobind::dtype<double>())       return "f8";
        SP_ASSERT(false);
        return "Invalid";
    };

    static nanobind::dlpack::dtype getDataType(const std::string& data_type)
    {
        if (data_type == "u1") return nanobind::dtype<uint8_t>();
        if (data_type == "i1") return nanobind::dtype<int8_t>();
        if (data_type == "u2") return nanobind::dtype<uint16_t>();
        if (data_type == "i2") return nanobind::dtype<int16_t>();
        if (data_type == "u4") return nanobind::dtype<uint32_t>();
        if (data_type == "i4") return nanobind::dtype<int32_t>();
        if (data_type == "u8") return nanobind::dtype<uint64_t>();
        if (data_type == "i8") return nanobind::dtype<int64_t>();
        if (data_type == "f2") return nanobind::dtype<sp_float16_t>();
        if (data_type == "f4") return nanobind::dtype<float>();
        if (data_type == "f8") return nanobind::dtype<double>();
        SP_ASSERT(false);
        return nanobind::dtype<nanobind::ndarray<>::Scalar>();
    };
};

//
// PropertyDesc
//

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<PropertyDesc> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, PropertyDesc const& property_desc) const {
        packer.pack_map(2);
        packer.pack("property");  packer.pack(property_desc.property_);
        packer.pack("value_ptr"); packer.pack(property_desc.value_ptr_);
        return packer;
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<PropertyDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, PropertyDesc& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 2);
        property_desc.property_  = MsgpackUtils::to<uint64_t>(objects.at("property"));
        property_desc.value_ptr_ = MsgpackUtils::to<uint64_t>(objects.at("value_ptr"));
        return object;
    }
};

//
// SharedMemoryView
//

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<SharedMemoryView> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, SharedMemoryView const& shared_memory_view) const {
        packer.pack_map(4);
        packer.pack("id");           packer.pack(shared_memory_view.id_);
        packer.pack("num_bytes");    packer.pack(shared_memory_view.num_bytes_);
        packer.pack("offset_bytes"); packer.pack(shared_memory_view.offset_bytes_);
        packer.pack("usage_flags");  packer.pack(shared_memory_view.usage_flags_);
        return packer;
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<SharedMemoryView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SharedMemoryView& shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 4);
        shared_memory_view.id_           = MsgpackUtils::to<std::string>(objects.at("id"));
        shared_memory_view.num_bytes_    = MsgpackUtils::to<uint64_t>(objects.at("num_bytes"));
        shared_memory_view.offset_bytes_ = MsgpackUtils::to<uint16_t>(objects.at("offset_bytes"));
        shared_memory_view.usage_flags_  = MsgpackUtils::to<std::vector<std::string>>(objects.at("usage_flags"));
        return object;
    }
};

//
// PackedArray
//

template<> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<PackedArray> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, PackedArray const& packed_array) const {
        packer.pack_map(5);
        packer.pack("data");               packer.pack(clmdep_msgpack::type::raw_ref(static_cast<const char*>(packed_array.data_.data()), packed_array.data_.nbytes()));
        packer.pack("data_source");        packer.pack(packed_array.data_source_);
        packer.pack("shape");              packer.pack(packed_array.shape_);
        packer.pack("data_type");          packer.pack(DataTypeUtils::getDataType(packed_array.data_.dtype()));
        packer.pack("shared_memory_name"); packer.pack(packed_array.shared_memory_name_);
        return packer;
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<PackedArrayView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, PackedArrayView& packed_array_view) const {

        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 5);
        packed_array_view.view_               = MsgpackUtils::to<std::span<uint8_t>>(objects.at("data"));
        packed_array_view.data_source_        = MsgpackUtils::to<std::string>(objects.at("data_source"));
        packed_array_view.shape_              = MsgpackUtils::to<std::vector<size_t>>(objects.at("shape"));
        packed_array_view.data_type_          = MsgpackUtils::to<std::string>(objects.at("data_type"));
        packed_array_view.shared_memory_name_ = MsgpackUtils::to<std::string>(objects.at("shared_memory_name"));
        SP_ASSERT(packed_array_view.data_source_ == "Internal" || packed_array_view.data_source_ == "Shared");
        return object;
    }
};

template <> // needed to convert a custom view type, received as a return value from the server, into a custom type that be returned to Python
PackedArray Client::convert<PackedArray>(PackedArrayView&& packed_array_view, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
{
    PackedArray packed_array;

    if (Statics::s_verbose_allocations_) {
        std::cout << "[SPEAR | spear_ext.cpp] Constructing nanobind::ndarray<nanobind::numpy>..." << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.view_.data():        " << packed_array_view.view_.data() << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.view_.size():        " << packed_array_view.view_.size() << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.data_source_:        " << packed_array_view.data_source_ << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.shape_:              "; for (int i = 0; i < packed_array_view.shape_.size(); i++) { std::cout << packed_array_view.shape_.at(i) << " "; } std::cout << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.data_type_:          " << packed_array_view.data_type_ << std::endl;
        std::cout << "[SPEAR | spear_ext.cpp] packed_array_view.shared_memory_name_: " << packed_array_view.shared_memory_name_ << std::endl;
    }

    // construct ndarray
    nanobind::ndarray<nanobind::numpy> ndarray;
    if (packed_array_view.data_source_ == "Internal") {

        struct Capsule
        {
            std::vector<uint8_t, SpPackedArrayAllocator> data_;
            std::shared_ptr<clmdep_msgpack::object_handle> object_handle_ = nullptr;
        };

        // Allocate a Capsule on the heap. Deleted in the deletion callback below.
        Capsule* capsule_ptr = new Capsule();
        SP_ASSERT(capsule_ptr);
        if (Statics::s_verbose_allocations_) { std::cout << "[SPEAR | spear_ext.cpp] Allocated Capsule object at memory location: " << capsule_ptr << std::endl; }

        void* data_ptr = nullptr;

        // If we're forcing memory alignment, then copy the clmdep_msgpack::object_handle's internal data
        // buffer into the Capsule's internal data buffer, which is guaranteed to be aligned to very large
        // boundaries, and use the Capsule's internal data buffer as the persistent storage for our
        // nanobind array.

        if (Statics::s_force_return_aligned_arrays_) {
            Std::resizeUninitialized(capsule_ptr->data_, packed_array_view.view_.size());
            std::memcpy(capsule_ptr->data_.data(), packed_array_view.view_.data(), packed_array_view.view_.size());
            data_ptr = capsule_ptr->data_.data();

        // If we're not forcing memory alignment, then assign to the Capsule's std::shared_ptr to indicate
        // that we want to keep the clmdep_msgpack::object_handle alive, and use the clmdep_msgpack::object_handle's
        // internal data buffer as the persistent storage for our nanobind array.

        } else {
            capsule_ptr->object_handle_ = object_handle;
            data_ptr = packed_array_view.view_.data();
        }

        // Construct a nanobind capsule object that is responsible for deleting our heap-allocated Capsule.
        // In the in the deletion callback, we assign to our Capsule's std::shared_ptr object to indicate
        // that we don't need to keep the clmdep_msgpack::object_handle alive any more and clean up our
        // heap-allocated Capsule.

        nanobind::capsule capsule = nanobind::capsule(capsule_ptr, [](void* ptr) noexcept -> void {
            if (!ptr) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Unexpected nullptr in deletion callback for Capsule." << std::endl;
                std::terminate(); // can't assert because of noexcept
            }
            Capsule* capsule_ptr = static_cast<Capsule*>(ptr);
            if (Statics::s_verbose_allocations_) { std::cout << "[SPEAR | spear_ext.cpp] Deleting Capsule object at memory location: " << capsule_ptr << std::endl; }
            capsule_ptr->object_handle_ = nullptr;
            delete capsule_ptr;
            capsule_ptr = nullptr;
        });

        ndarray = nanobind::ndarray<nanobind::numpy>(
            data_ptr,                                                  // data_ptr
            packed_array_view.shape_.size(),                           // ndim
            packed_array_view.shape_.data(),                           // shape_ptr
            capsule,                                                   // owner
            nullptr,                                                   // strides
            DataTypeUtils::getDataType(packed_array_view.data_type_)); // dtype

    } else if (packed_array_view.data_source_ == "Shared") {

        ndarray = nanobind::ndarray<nanobind::numpy>(
            nullptr,                                                   // data_ptr
            0,                                                         // ndim
            nullptr,                                                   // shape_ptr
            {},                                                        // owner
            nullptr,                                                   // strides
            DataTypeUtils::getDataType(packed_array_view.data_type_)); // dtype

    } else {
        SP_ASSERT(false);
    }

    packed_array.data_               = std::move(ndarray);
    packed_array.data_source_        = std::move(packed_array_view.data_source_);
    packed_array.shape_              = std::move(packed_array_view.shape_);
    packed_array.shared_memory_name_ = std::move(packed_array_view.shared_memory_name_);

    return packed_array;
};

//
// std::map<std::string, PackedArray>
//

template <> // needed to convert a custom view type, received as a return value from the server, into a custom type that be returned to Python
std::map<std::string, PackedArray> Client::convert<std::map<std::string, PackedArray>>(std::map<std::string, PackedArrayView>&& packed_array_views, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
{
    std::map<std::string, PackedArray> packed_arrays;
    for (auto& [name, packed_array_view] : packed_array_views) {
        auto [itr, success] = packed_arrays.insert({std::move(name), convert<PackedArray>(std::move(packed_array_view), object_handle)});
        SP_ASSERT(success); // will only succeed if key wasn't already present
    }
    return packed_arrays;
};

//
// Future
//

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<Future> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, Future const& future) const {
        packer.pack_map(2);
        packer.pack("future_ptr"); packer.pack(future.future_ptr_);
        packer.pack("type_id");    packer.pack(future.type_id_);
        return packer;
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<Future> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, Future& future) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 2);
        future.future_ptr_ = MsgpackUtils::to<uint64_t>(objects.at("future_ptr"));
        future.type_id_    = MsgpackUtils::to<std::string>(objects.at("type_id"));
        return object;
    }
};

//
// DataBundle
//

template <> // needed to send a custom type as an arg to the server
struct clmdep_msgpack::adaptor::pack<DataBundle> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, DataBundle const& data_bundle) const {
        packer.pack_map(3);
        packer.pack("packed_arrays");      packer.pack(data_bundle.packed_arrays_);
        packer.pack("unreal_obj_strings"); packer.pack(data_bundle.unreal_obj_strings_);
        packer.pack("info");               packer.pack(data_bundle.info_);
        return packer;
    }
};

template <> // needed to receive a custom type as a return value from the server
struct clmdep_msgpack::adaptor::convert<DataBundleView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, DataBundleView& data_bundle_view) const {
        std::map<std::string, clmdep_msgpack::object> objects = MsgpackUtils::toMapOfMsgpackObjects(object);
        SP_ASSERT(objects.size() == 3);
        data_bundle_view.packed_array_views_ = MsgpackUtils::to<std::map<std::string, PackedArrayView>>(objects.at("packed_arrays"));
        data_bundle_view.unreal_obj_strings_ = MsgpackUtils::to<std::map<std::string, std::string>>(objects.at("unreal_obj_strings"));
        data_bundle_view.info_               = MsgpackUtils::to<std::string>(objects.at("info"));
        return object;
    }
};

template <> // needed to convert a custom view type, received as a return value from the server, into a custom type that be returned to Python
DataBundle Client::convert<DataBundle>(DataBundleView&& data_bundle_view, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
{
    DataBundle data_bundle;
    data_bundle.packed_arrays_      = convert<std::map<std::string, PackedArray>>(std::move(data_bundle_view.packed_array_views_), object_handle);
    data_bundle.unreal_obj_strings_ = std::move(data_bundle_view.unreal_obj_strings_);
    data_bundle.info_               = std::move(data_bundle_view.info_);
    return data_bundle;
};
