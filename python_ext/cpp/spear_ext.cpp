#include <stdint.h> // int8_t, int16_t, int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t

#include <exception>
#include <iostream>
#include <map>
#include <memory>    // std::make_unique, std::unique_ptr
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
// Globals
//

struct Globals
{
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

    nonstd::optional<int64_t> get_timeout() const
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::get_timeout()" << std::endl;
        SP_ASSERT(client_);
        return client_->get_timeout();
    };

    void set_timeout(int64_t value)
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::set_timeout(...)" << std::endl;
        SP_ASSERT(client_);
        client_->set_timeout(value);
    };

    void clear_timeout()
    {
        std::cout << "[SPEAR | spear_ext.cpp] Client::clear_timeout()" << std::endl;
        SP_ASSERT(client_);
        client_->clear_timeout();
    };

    template <typename... TArgs>
    void call(const std::string& func_name, TArgs... args)
    {
        SP_ASSERT(client_);

        try {
            client_->call(func_name, args...);
        } catch (const std::exception& e) {
            if (Globals::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl;
            }
            std::rethrow_exception(std::current_exception());
        } catch (...) {
            if (Globals::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl;
            }
            std::rethrow_exception(std::current_exception());
        }
    };

    template <typename TReturn, typename... TArgs>
    TReturn call_and_get_return_value(const std::string& func_name, TArgs... args)
    {
        SP_ASSERT(client_);

        try {
            return client_->call(func_name, args...).template as<TReturn>();
        } catch (const std::exception& e) {
            if (Globals::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught exception when calling \"" << func_name << "\": " << e.what() << std::endl;
            }
            std::rethrow_exception(std::current_exception());
            return TReturn();
        } catch (...) {
            if (Globals::s_verbose_exceptions_) {
                std::cout << "[SPEAR | spear_ext.cpp] ERROR: Caught unknown exception when calling \"" << func_name << "\"." << std::endl;
            }
            std::rethrow_exception(std::current_exception());
            return TReturn();
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
// Python module
//

NB_MODULE(spear_ext, module)
{
    auto client_class = nanobind::class_<Client>(module, "Client");
    client_class.def(nanobind::init<const std::string&, const uint16_t>());

    client_class.def("initialize", &Client::initialize);
    client_class.def("terminate", &Client::terminate);

    client_class.def("get_timeout", &Client::get_timeout);
    client_class.def("set_timeout", &Client::set_timeout);
    client_class.def("clear_timeout", &Client::clear_timeout);

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
    // Specializations for Client::call_and_get_return_value(...)
    //

    // 0 args
    client_class.def("call_and_get_return_value_as_bool",                                &Client::call_and_get_return_value<bool>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::call_and_get_return_value<std::string>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>>);
    client_class.def("call_and_get_return_value_as_vector_of_double",                    &Client::call_and_get_return_value<std::vector<double>>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>>);

    // 1 args
    client_class.def("call_and_get_return_value_as_bool",                                &Client::call_and_get_return_value<bool,                                    uint64_t>);
    client_class.def("call_and_get_return_value_as_float",                               &Client::call_and_get_return_value<float,                                   uint64_t>);
    client_class.def("call_and_get_return_value_as_int32",                               &Client::call_and_get_return_value<int32_t,                                 uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::call_and_get_return_value<std::string,                             uint64_t>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::call_and_get_return_value<std::string,                             const PropertyDesc&>);
    client_class.def("call_and_get_return_value_as_vector_of_string",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   uint64_t>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&>);
    client_class.def("call_and_get_return_value_as_vector_of_string",                    &Client::call_and_get_return_value<std::vector<std::string>,                uint64_t>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         uint64_t>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_shared_memory_view", &Client::call_and_get_return_value<std::map<std::string, SharedMemoryView>, uint64_t>);

    // 2 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::call_and_get_return_value<std::string,                             uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_string",                              &Client::call_and_get_return_value<std::string,                             uint64_t,             uint64_t>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&,   const std::string&>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&,   const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         uint64_t,             bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&,   const std::string&>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&,   const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_property_desc",                       &Client::call_and_get_return_value<PropertyDesc,                            uint64_t,             const std::string&>);

    // 3 args
    client_class.def("call_and_get_return_value_as_bool",                                &Client::call_and_get_return_value<bool,                                    uint64_t,             bool,                             bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             uint64_t,                         const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             const std::string&,               const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         const std::string&>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   uint64_t,             uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&,   const std::vector<std::string>&,  bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         uint64_t,             uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&,   const std::vector<std::string>&,  bool>);
    client_class.def("call_and_get_return_value_as_shared_memory_view",                  &Client::call_and_get_return_value<SharedMemoryView,                        const std::string&,   int,                              const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_data_bundle",                         &Client::call_and_get_return_value<DataBundle,                              uint64_t,             const std::string&,               const DataBundle&>);
    client_class.def("call_and_get_return_value_as_property_desc",                       &Client::call_and_get_return_value<PropertyDesc,                            uint64_t,             uint64_t,                         const std::string&>);

    // 4 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             uint64_t,                         uint64_t,                                  const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         uint64_t,                                  const std::string&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        bool>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         const std::string&,                        bool>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         const std::string&,                        bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_string",             &Client::call_and_get_return_value<std::map<std::string, std::string>,      uint64_t,             uint64_t,                         const std::map<std::string, std::string>&, const std::string&>);

    // 5 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             const std::string&,               const std::string&,                        const std::string&,                        const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   const std::string&,               const std::string&,                        const std::string&,                        const std::vector<std::string>&>);
    client_class.def("call_and_get_return_value_as_vector_of_uint64",                    &Client::call_and_get_return_value<std::vector<uint64_t>,                   const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool,                                      bool>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_uint64",             &Client::call_and_get_return_value<std::map<std::string, uint64_t>,         const std::string&,   uint64_t,                         const std::vector<std::string>&,           bool,                                      bool>);
    client_class.def("call_and_get_return_value_as_packed_array",                        &Client::call_and_get_return_value<PackedArray,                             uint64_t,             int,                              uint64_t,                                  const std::map<std::string, PackedArray>&, const PackedArray&>);

    // 6 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t>);

    // 7 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t,                         uint64_t>);
    client_class.def("call_and_get_return_value_as_map_of_string_to_packed_array",       &Client::call_and_get_return_value<std::map<std::string, PackedArray>,      uint64_t,             uint64_t,                         int,                                       uint64_t,                                  const std::map<std::string, PackedArray>&, const std::vector<std::string>&,  const std::vector<std::string>&>);

    // 8 args
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                const std::string&,   uint64_t,                         const std::string&,                        const std::vector<std::string>&,           uint64_t,                                  bool,                             uint64_t,                         uint64_t>);
    client_class.def("call_and_get_return_value_as_uint64",                              &Client::call_and_get_return_value<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                        const std::string&,                        const std::vector<std::string>&,           uint64_t,                         bool,                             uint64_t>);

    //
    // Custom types
    //

    auto shared_memory_view_class = nanobind::class_<SharedMemoryView>(module, "SharedMemoryView");
    shared_memory_view_class.def(nanobind::init<>());
    shared_memory_view_class.def_rw("id",          &SharedMemoryView::id_);
    shared_memory_view_class.def_rw("num_bytes",   &SharedMemoryView::num_bytes_);
    shared_memory_view_class.def_rw("usage_flags", &SharedMemoryView::usage_flags_);

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
    // Globals
    //

    auto globals_class = nanobind::class_<Globals>(module, "Globals");
    globals_class.def_rw_static("verbose_exceptions", &Globals::s_verbose_exceptions_);
    globals_class.def_rw_static("verbose_allocations", &Globals::s_verbose_allocations_);
}

//
// Helper functions for implementing msgpack adaptors
//

class MsgpackUtils
{
public:
    MsgpackUtils() = delete;
    ~MsgpackUtils() = delete;

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
// SharedMemoryView
//

template <> // needed to send a custom type as an arg
struct clmdep_msgpack::adaptor::pack<SharedMemoryView> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, SharedMemoryView const& shared_memory_view) const {
        packer.pack_map(3);
        packer.pack("id");          packer.pack(shared_memory_view.id_);
        packer.pack("num_bytes");   packer.pack(shared_memory_view.num_bytes_);
        packer.pack("usage_flags"); packer.pack(shared_memory_view.usage_flags_);
        return packer;
    }
};

template <> // needed to receive a custom type as a return value
struct clmdep_msgpack::adaptor::convert<SharedMemoryView> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, SharedMemoryView& shared_memory_view) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 3);
        shared_memory_view.id_          = MsgpackUtils::to<std::string>(map.at("id"));
        shared_memory_view.num_bytes_   = MsgpackUtils::to<uint64_t>(map.at("num_bytes"));
        shared_memory_view.usage_flags_ = MsgpackUtils::to<std::vector<std::string>>(map.at("usage_flags"));
        return object;
    }
};

//
// PackedArray
//

template<> // needed to send a custom type as an arg
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

template <> // needed to receive a custom type as a return value
struct clmdep_msgpack::adaptor::convert<PackedArray> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, PackedArray& packed_array) const {

        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 5);

        std::vector<uint8_t> data = MsgpackUtils::to<std::vector<uint8_t>>(map.at("data"));
        std::string data_source = MsgpackUtils::to<std::string>(map.at("data_source"));
        std::vector<size_t> shape = MsgpackUtils::to<std::vector<size_t>>(map.at("shape"));
        std::string data_type = MsgpackUtils::to<std::string>(map.at("data_type"));
        std::string shared_memory_name = MsgpackUtils::to<std::string>(map.at("shared_memory_name"));

        SP_ASSERT(data_source == "Internal" || data_source == "Shared");

        if (Globals::s_verbose_allocations_) {
            std::cout << "[SPEAR | spear_ext.cpp] Constructing nanobind::ndarray<nanobind::numpy>..." << std::endl;
            std::cout << "[SPEAR | spear_ext.cpp] data.size():        " << data.size() << std::endl;
            std::cout << "[SPEAR | spear_ext.cpp] data_source:        " << data_source << std::endl;
            std::cout << "[SPEAR | spear_ext.cpp] shape:              "; for (int i = 0; i < shape.size(); i++) { std::cout << shape.at(i) << " "; } std::cout << std::endl;
            std::cout << "[SPEAR | spear_ext.cpp] data_type:          " << data_type << std::endl;
            std::cout << "[SPEAR | spear_ext.cpp] shared_memory_name: " << shared_memory_name << std::endl;
        }

        // construct ndarray
        nanobind::ndarray<nanobind::numpy> ndarray;
        if (data_source == "Internal") {

            // construct an std::vector on the heap and move data into it
            std::vector<uint8_t>* data_ptr = new std::vector<uint8_t>(std::move(data));
            if (Globals::s_verbose_allocations_) {
                std::cout << "[SPEAR | spear_ext.cpp] Allocating std::vector<uint8_t> at memory location: " << data_ptr << std::endl;
            }

            // construct a capsule responsible for deleting the heap-allocated std::vector
            nanobind::capsule capsule = nanobind::capsule(data_ptr, [](void* ptr) noexcept -> void {
                delete static_cast<std::vector<uint8_t>*>(ptr);
                if (Globals::s_verbose_allocations_) {
                    std::cout << "[SPEAR | spear_ext.cpp] Deleting std::vector<uint8_t> at memory location: " << ptr << std::endl;
                }
            });

            //                                           data_ptr,         ndim,         shape_ptr,    owner,   strides_ptr, dtype
            ndarray = nanobind::ndarray<nanobind::numpy>(data_ptr->data(), shape.size(), shape.data(), capsule, nullptr,     DataTypeUtils::getDataType(data_type));

        } else if (data_source == "Shared") {
            ndarray = nanobind::ndarray<nanobind::numpy>(nullptr,          0,            nullptr,      {},      nullptr,     DataTypeUtils::getDataType(data_type));

        } else {
            SP_ASSERT(false);
        }

        packed_array.data_               = std::move(ndarray);
        packed_array.data_source_        = std::move(data_source);
        packed_array.shape_              = std::move(shape);
        packed_array.shared_memory_name_ = std::move(shared_memory_name);

        return object;
    }
};

//
// DataBundle
//

template <> // needed to send a custom type as an arg
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

template <> // needed to receive a custom type as a return value
struct clmdep_msgpack::adaptor::convert<DataBundle> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, DataBundle& data_bundle) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 3);
        data_bundle.packed_arrays_      = MsgpackUtils::to<std::map<std::string, PackedArray>>(map.at("packed_arrays"));
        data_bundle.unreal_obj_strings_ = MsgpackUtils::to<std::map<std::string, std::string>>(map.at("unreal_obj_strings"));
        data_bundle.info_               = MsgpackUtils::to<std::string>(map.at("info"));
        return object;
    }
};

//
// PropertyDesc
//

template <> // needed to send a custom type as an arg
struct clmdep_msgpack::adaptor::pack<PropertyDesc> {
    template <typename TStream>
    clmdep_msgpack::packer<TStream>& operator()(clmdep_msgpack::packer<TStream>& packer, PropertyDesc const& property_desc) const {
        packer.pack_map(2);
        packer.pack("property");  packer.pack(property_desc.property_);
        packer.pack("value_ptr"); packer.pack(property_desc.value_ptr_);
        return packer;
    }
};

template <> // needed to receive a custom type as a return value
struct clmdep_msgpack::adaptor::convert<PropertyDesc> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& object, PropertyDesc& property_desc) const {
        std::map<std::string, clmdep_msgpack::object> map = MsgpackUtils::toMap(object);
        SP_ASSERT(map.size() == 2);
        property_desc.property_  = MsgpackUtils::to<uint64_t>(map.at("property"));
        property_desc.value_ptr_ = MsgpackUtils::to<uint64_t>(map.at("value_ptr"));
        return object;
    }
};
