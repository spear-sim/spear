#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t, uintptr_t

#ifdef _MSC_VER
    #include <malloc.h> // _aligned_alloc
#else
    #include <stdlib.h> // posix_memalign
#endif

#include <concepts>  // std::same_as
#include <cstring>   // std::memcpy
#include <exception>
#include <iostream>
#include <map>
#include <memory>    // std::make_shared, std::make_unique, std::shared_ptr, std::unique_ptr
#include <span>
#include <sstream>
#include <stdexcept> // std::runtime_error
#include <string>
#include <utility>   // std::move
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
// Minimal sp_float16_t data type
//

struct sp_float16_t
{
    uint16_t data;
};

template <>
struct nanobind::detail::dtype_traits<sp_float16_t>
{
    static constexpr dlpack::dtype value {
        static_cast<uint8_t>(dlpack::dtype_code::Float), // type code
        16,                                              // size in bits
        1};                                              // lanes (simd), usually set to 1
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
    bool operator==(const SpPackedArrayAllocatorImpl<TOtherValue>&) const noexcept
    {
        return true;
    }

    template <typename TOtherValue>
    bool operator!=(const SpPackedArrayAllocatorImpl<TOtherValue>&) const noexcept
    {
        return false;
    }
};

using SpPackedArrayAllocator = SpPackedArrayAllocatorImpl<uint8_t>;

//
// Statics
//

struct Statics
{
    inline static bool s_force_return_aligned_arrays_ = false;
    inline static bool s_verbose_exceptions_ = true;
    inline static bool s_verbose_allocations_ = false;
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

    nonstd::optional<int64_t> getTimeout() const
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::getTimeout()" << std::endl;
        SP_ASSERT(client_);
        return client_->get_timeout();
    };

    void setTimeout(int64_t value)
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::setTimeout(...)" << std::endl;
        SP_ASSERT(client_);
        client_->set_timeout(value);
    };

    void clearTimeout()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::clearTimeout()" << std::endl;
        SP_ASSERT(client_);
        client_->clear_timeout();
    };

    template <typename TDest, typename TSrc>
    TDest convert(TSrc&& src, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
    {
        SP_ASSERT(false);
        return TDest();
    };

    template <typename TDest, typename TSrc> requires std::same_as<TDest, TSrc>
    TDest convert(TSrc&& src, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
    {
        return std::move(src);
    };

    template <typename... TArgs>
    void call(const std::string& func_name, TArgs... args)
    {
        SP_ASSERT(client_);

        try {
            client_->call(func_name, args...);
        } catch (const std::exception& e) {
            if (Statics::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl;
            }
            std::rethrow_exception(std::current_exception());
        } catch (...) {
            if (Statics::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl;
            }
            std::rethrow_exception(std::current_exception());
        }
    };

    template <typename TReturn, typename... TArgs>
    TReturn callAndGetReturnValue(const std::string& func_name, TArgs... args)
    {
        SP_ASSERT(client_);

        try {
            return client_->call(func_name, args...).template as<TReturn>(); // .template needed on macOS
        } catch (const std::exception& e) {
            if (Statics::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl;
            }
            std::rethrow_exception(std::current_exception());
            return TReturn();
        } catch (...) {
            if (Statics::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl;
            }
            std::rethrow_exception(std::current_exception());
            return TReturn();
        }
    };

    template <typename TReturnDest, typename TReturnSrc, typename... TArgs>
    TReturnDest callAndGetConvertedReturnValue(const std::string& func_name, TArgs... args)
    {
        SP_ASSERT(client_);

        try {

            std::shared_ptr<clmdep_msgpack::object_handle> object_handle = std::make_shared<clmdep_msgpack::object_handle>(client_->call(func_name, args...));
            TReturnSrc return_value_src = object_handle->template as<TReturnSrc>(); // .template needed on macOS
            TReturnDest return_value_dest = convert<TReturnDest>(std::move(return_value_src), object_handle);
            return return_value_dest;

        } catch (const std::exception& e) {
            if (Statics::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl;
            }
            std::rethrow_exception(std::current_exception());
            return TReturnDest();
        } catch (...) {
            if (Statics::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl;
            }
            std::rethrow_exception(std::current_exception());
            return TReturnDest();
        }
    };

private:
    std::unique_ptr<rpc::client> client_ = nullptr;
};

//
// Custom types
//

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

struct DataBundle
{
    std::map<std::string, PackedArray> packed_arrays_;
    std::map<std::string, std::string> unreal_obj_strings_;
    std::string info_;
};

struct PropertyDesc
{
    uint64_t property_ = 0;
    uint64_t value_ptr_ = 0;
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
// Python module
//

NB_MODULE(spear_ext, module)
{
    auto client_class = nanobind::class_<Client>(module, "Client");
    client_class.def(nanobind::init<const std::string&, const uint16_t>());

    client_class.def("initialize", &Client::initialize);
    client_class.def("terminate", &Client::terminate);

    client_class.def("get_timeout", &Client::getTimeout);
    client_class.def("set_timeout", &Client::setTimeout);
    client_class.def("clear_timeout", &Client::clearTimeout);

    //
    // Specializations for Client::call(...)
    //

    // 0 args
    client_class.def("call", &Client::call<>);

    // 1 arg
    client_class.def("call", &Client::call<uint64_t>);
    client_class.def("call", &Client::call<const std::string&>);

    // 2 args
    client_class.def("call", &Client::call<uint64_t,            bool>);
    client_class.def("call", &Client::call<uint64_t,            uint64_t>);
    client_class.def("call", &Client::call<uint64_t,            const std::string&>);
    client_class.def("call", &Client::call<const PropertyDesc&, const std::string&>);

    // 3 args
    client_class.def("call", &Client::call<uint64_t,            bool,                const std::vector<std::string>&>);
    client_class.def("call", &Client::call<uint64_t,            int,                 const std::vector<std::string>&>);
    client_class.def("call", &Client::call<uint64_t,            float,               const std::vector<std::string>&>);
    client_class.def("call", &Client::call<uint64_t,            uint64_t,            const std::string&>);
    client_class.def("call", &Client::call<uint64_t,            const std::string&,  float>);
    client_class.def("call", &Client::call<uint64_t,            const std::string&,  const std::string&>);
    client_class.def("call", &Client::call<uint64_t,            const std::string&,  const std::vector<std::string>&>);

    // 4 args
    client_class.def("call", &Client::call<uint64_t,            const std::string&,  const std::string&,               const std::string&>);

    // 5 args
    client_class.def("call", &Client::call<uint64_t,            uint64_t,            const std::string&,               const std::vector<uint64_t>&, const std::vector<uint64_t>&>);

    // 7 args
    client_class.def("call", &Client::call<uint64_t,            const std::string&,  const std::string&,               const std::string&,           const std::string&,            const std::vector<uint64_t>&, const std::vector<uint64_t>&>);

    //
    // Specializations for Client::callAndGetReturnValue(...)
    //

    // 0 args
    client_class.def("call_and_get_return_value_as_bool",                                &Client::callAndGetReturnValue<bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::callAndGetReturnValue<std::string>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>>);
    client_class.def("call_and_get_return_value_as_vector_of_double",                    &Client::callAndGetReturnValue<std::vector<double>>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>>);

    // 1 args
    client_class.def("call_and_get_return_value_as_bool",                                &Client::callAndGetReturnValue<bool,                                    uint64_t>);
    client_class.def("call_and_get_return_value_as_float",                               &Client::callAndGetReturnValue<float,                                   uint64_t>);
    client_class.def("call_and_get_return_value_as_int32",                               &Client::callAndGetReturnValue<int32_t,                                 uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::callAndGetReturnValue<std::string,                             uint64_t>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::callAndGetReturnValue<std::string,                             const PropertyDesc&>);
    client_class.def("call_and_get_return_value_as_vector_of_string",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   uint64_t>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&>);
    client_class.def("call_and_get_return_value_as_vector_of_string",                    &Client::callAndGetReturnValue<std::vector<std::string>,                uint64_t>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         uint64_t>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_shared_memory_view", &Client::callAndGetReturnValue<std::map<std::string, SharedMemoryView>, uint64_t>);

    // 2 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::callAndGetReturnValue<std::string,                             uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::callAndGetReturnValue<std::string,                             uint64_t,             uint64_t>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&,   const std::string&>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&,   const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&,   const std::string&>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&,   const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_property_desc",                       &Client::callAndGetReturnValue<PropertyDesc,                            uint64_t,             const std::string&>);

    // 3 args
    client_class.def("call_and_get_return_value_as_bool",                                &Client::callAndGetReturnValue<bool,                                    uint64_t,             bool,                             bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             uint64_t,                         const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             const std::string&,               const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         const std::string&>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   uint64_t,             uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&,   const std::vector<std::string>&,  bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         uint64_t,             uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&,   const std::vector<std::string>&,  bool>);
    client_class.def("call_and_get_return_value_as_shared_memory_view",                  &Client::callAndGetReturnValue<SharedMemoryView,                        const std::string&,   int,                              const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_property_desc",                       &Client::callAndGetReturnValue<PropertyDesc,                            uint64_t,             uint64_t,                         const std::string&>);

    // 4 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             uint64_t,                         uint64_t,                                  const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         uint64_t,                                  const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         const std::string&,                        bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         const std::string&,                        bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_string",             &Client::callAndGetReturnValue<std::map<std::string, std::string>,      uint64_t,             uint64_t,                         const std::map<std::string, std::string>&, const std::string&>);

    // 5 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             const std::string&,               const std::string&,                        const std::string&,                        const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   const std::string&,               const std::string&,                        const std::string&,                        const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::callAndGetReturnValue<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool,                                      bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::callAndGetReturnValue<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool,                                      bool>);

    // 6 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t>);

    // 7 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t,                         uint64_t>);

    // 8 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        const std::vector<std::string>&,           uint64_t,                                  bool,                             uint64_t,                         uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::callAndGetReturnValue<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t,                         bool,                             uint64_t>);

    //
    // Specializations for Client::callAndGetConvertedReturnValue(...)
    //

    // 3 args
    client_class.def("call_and_get_converted_return_value_as_data_bundle",                   &Client::callAndGetConvertedReturnValue<DataBundle,  DataBundleView,                                                uint64_t, const std::string&, const DataBundle&>);

    // 5 args
    client_class.def("call_and_get_converted_return_value_as_packed_array",                  &Client::callAndGetConvertedReturnValue<PackedArray, PackedArrayView,                                               uint64_t, int,                uint64_t,           const std::map<std::string, PackedArray>&, const PackedArray&>);

    // 7 args
    client_class.def("call_and_get_converted_return_value_as_map_of_string_to_packed_array", &Client::callAndGetConvertedReturnValue<std::map<std::string, PackedArray>, std::map<std::string, PackedArrayView>, uint64_t, uint64_t,           int,                uint64_t,                                  const std::map<std::string, PackedArray>&, const std::vector<std::string>&,  const std::vector<std::string>&>);

    //
    // Custom types
    //

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

    auto data_bundle_class = nanobind::class_<DataBundle>(module, "DataBundle");
    data_bundle_class.def(nanobind::init<>());
    data_bundle_class.def_rw("packed_arrays",      &DataBundle::packed_arrays_, nanobind::rv_policy::reference);
    data_bundle_class.def_rw("unreal_obj_strings", &DataBundle::unreal_obj_strings_);
    data_bundle_class.def_rw("info",               &DataBundle::info_);

    auto property_desc_class = nanobind::class_<PropertyDesc>(module, "PropertyDesc");
    property_desc_class.def(nanobind::init<>());
    property_desc_class.def_rw("property",  &PropertyDesc::property_);
    property_desc_class.def_rw("value_ptr", &PropertyDesc::value_ptr_);

    //
    // Statics
    //

    auto statics_class = nanobind::class_<Statics>(module, "Statics");
    statics_class.def_rw_static("force_return_aligned_arrays", &Statics::s_force_return_aligned_arrays_);
    statics_class.def_rw_static("verbose_exceptions", &Statics::s_verbose_exceptions_);
    statics_class.def_rw_static("verbose_allocations", &Statics::s_verbose_allocations_);
}

//
// Helper functions for implementing msgpack adaptors
//

class MsgpackUtils
{
public:
    MsgpackUtils() = delete;
    ~MsgpackUtils() = delete;

    //
    // functions for receiving custom types as return values from the server
    //

    template <typename T>
    static T to(clmdep_msgpack::object const& object)
    {
        T t;
        object.convert(t);
        return t;
    };

    static std::map<std::string, clmdep_msgpack::object> toMap(clmdep_msgpack::object const& object)
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

    static std::span<uint8_t> toSpan(clmdep_msgpack::object const& object)
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

    static uint8_t getSizeOf(const std::string& data_type)
    {
        if (data_type == "u1") return 1;
        if (data_type == "i1") return 1;
        if (data_type == "u2") return 2;
        if (data_type == "i2") return 2;
        if (data_type == "u4") return 4;
        if (data_type == "i4") return 4;
        if (data_type == "u8") return 8;
        if (data_type == "i8") return 8;
        if (data_type == "f2") return 2;
        if (data_type == "f4") return 4;
        if (data_type == "f8") return 8;
        SP_ASSERT(false);
        return 0;
    };
};

class Std
{
public:
    Std() = delete;
    ~Std() = delete;

    template <typename TValue, typename... TVectorTraits>
    static void resizeUninitialized(std::vector<TValue, TVectorTraits...>& vector, int size)
    {
        using TVector = std::vector<TValue, TVectorTraits...>;
        using TAllocator = typename TVector::allocator_type;

        struct TValueNoDefaultInit
        {
            TValue value;
            TValueNoDefaultInit() {}
        };

        using TVectorNoDefaultInit = std::vector<TValueNoDefaultInit, typename std::allocator_traits<TAllocator>::template rebind_alloc<TValueNoDefaultInit>>;

        SP_ASSERT(sizeof(TValue) == sizeof(TValueNoDefaultInit));
        SP_ASSERT(sizeof(TValue[2]) == sizeof(TValueNoDefaultInit[2]));
        SP_ASSERT(sizeof(TValue[4]) == sizeof(TValueNoDefaultInit[4]));

        TVectorNoDefaultInit* vector_no_default_init = reinterpret_cast<TVectorNoDefaultInit*>(&vector);
        vector_no_default_init->resize(size);

        SP_ASSERT(vector.size() == size);
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
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 4);
        shared_memory_view.id_           = MsgpackUtils::to<std::string>(map.at("id"));
        shared_memory_view.num_bytes_    = MsgpackUtils::to<uint64_t>(map.at("num_bytes"));
        shared_memory_view.offset_bytes_ = MsgpackUtils::to<uint16_t>(map.at("offset_bytes"));
        shared_memory_view.usage_flags_  = MsgpackUtils::to<std::vector<std::string>>(map.at("usage_flags"));
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

        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 5);
        packed_array_view.view_               = MsgpackUtils::toSpan(map.at("data"));
        packed_array_view.data_source_        = MsgpackUtils::to<std::string>(map.at("data_source"));
        packed_array_view.shape_              = MsgpackUtils::to<std::vector<size_t>>(map.at("shape"));
        packed_array_view.data_type_          = MsgpackUtils::to<std::string>(map.at("data_type"));
        packed_array_view.shared_memory_name_ = MsgpackUtils::to<std::string>(map.at("shared_memory_name"));
        SP_ASSERT(packed_array_view.data_source_ == "Internal" || packed_array_view.data_source_ == "Shared");
        return object;
    }
};

template <>
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
            std::shared_ptr<clmdep_msgpack::object_handle> object_handle = nullptr;
        };

        // Allocate a Capsule on the heap.
        Capsule* capsule_ptr = new Capsule();
        SP_ASSERT(capsule_ptr);

        if (Statics::s_verbose_allocations_) {
            std::cout << "[SPEAR | spear_ext.cpp] Allocating Capsule at memory location: " << capsule_ptr << std::endl;
        }

        // If we're forcing memory alignment, then copy the clmdep_msgpack::object_handle's internal data
        // buffer into the Capsule's internal data buffer, which is guaranteed to be aligned to very large
        // boundaries, and use the Capsule's internal data buffer as the persistent storage for our nanobind
        // array.

        // If we're not forcing memory alignment, the assign to the Capsule's std::shared_ptr to indicate
        // that we want to keep the clmdep_msgpack::object_handle alive, and use the clmdep_msgpack::object_handle's
        // internal data buffer as the persistent storage for our nanobind array.

        void* data_ptr = nullptr;

        if (Statics::s_force_return_aligned_arrays_) {
            Std::resizeUninitialized(capsule_ptr->data_, packed_array_view.view_.size());
            std::memcpy(capsule_ptr->data_.data(), packed_array_view.view_.data(), packed_array_view.view_.size());
            data_ptr = capsule_ptr->data_.data();
        } else {
            capsule_ptr->object_handle = object_handle;
            data_ptr = packed_array_view.view_.data();
        }

        // Construct a nanobind capsule that is responsible for deleting our heap-allocated Capsule.
        nanobind::capsule capsule = nanobind::capsule(capsule_ptr, [](void* ptr) noexcept -> void {
            Capsule* capsule_ptr = static_cast<Capsule*>(ptr);

            // Assign to the Capsule's std::shared_ptr object to indicate that we don't need to keep the
            // clmdep_msgpack::object_handle alive any more.
            capsule_ptr->object_handle = nullptr;

            // Clean up the heap-allocated Capsule.
            delete capsule_ptr;

            if (Statics::s_verbose_allocations_) {
                std::cout << "[SPEAR | spear_ext.cpp] Deleting Capsule at memory location: " << capsule_ptr << std::endl;
            }
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

template <>
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
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 3);
        data_bundle_view.packed_array_views_ = MsgpackUtils::to<std::map<std::string, PackedArrayView>>(map.at("packed_arrays"));
        data_bundle_view.unreal_obj_strings_ = MsgpackUtils::to<std::map<std::string, std::string>>(map.at("unreal_obj_strings"));
        data_bundle_view.info_               = MsgpackUtils::to<std::string>(map.at("info"));
        return object;
    }
};

template <>
DataBundle Client::convert<DataBundle>(DataBundleView&& data_bundle_view, std::shared_ptr<clmdep_msgpack::object_handle> object_handle)
{
    DataBundle data_bundle;
    data_bundle.packed_arrays_ = convert<std::map<std::string, PackedArray>>(std::move(data_bundle_view.packed_array_views_), object_handle);
    data_bundle.unreal_obj_strings_ = std::move(data_bundle_view.unreal_obj_strings_);
    data_bundle.info_ = std::move(data_bundle_view.info_);
    return data_bundle;
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
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 2);
        property_desc.property_  = MsgpackUtils::to<uint64_t>(map.at("property"));
        property_desc.value_ptr_ = MsgpackUtils::to<uint64_t>(map.at("value_ptr"));
        return object;
    }
};
