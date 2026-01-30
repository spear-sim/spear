//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include <stdint.h> // int64_t, uint64_t

#include <map>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "client.h"
#include "msgpack_adaptors.h"
#include "std.h"
#include "types.h"

NB_MODULE(spear_ext, module)
{
    //
    // Client
    //

    auto client_class = nanobind::class_<Client>(module, "Client");

    client_class.def(nanobind::init<const std::string&, const uint16_t>());

    client_class.def("initialize", &Client::initialize);
    client_class.def("terminate",  &Client::terminate);

    client_class.def("get_timeout",   &Client::getTimeout);
    client_class.def("set_timeout",   &Client::setTimeout);
    client_class.def("clear_timeout", &Client::clearTimeout);

    client_class.def_static("get_entry_point_signature_type_descs", &Client::getEntryPointSignatureTypeDescs);
    client_class.def_static("get_entry_point_signature_descs",      &Client::getEntryPointSignatureDescs);

    client_class.def_rw("force_return_aligned_arrays", &Client::force_return_aligned_arrays_);
    client_class.def_rw("verbose_rpc_calls",           &Client::verbose_rpc_calls_);
    client_class.def_rw("verbose_allocations",         &Client::verbose_allocations_);
    client_class.def_rw("verbose_exceptions",          &Client::verbose_exceptions_);

    // specializations for callSync, callAsync, callAsyncFast, sendAsync, sendAsyncFast, getFutureResult, getFutureResultFastFromWorkerThread, getFutureResultFastFromGameThread

    Client::initializeEntryPointSignatures();

    // Unfortunately, we need to register each individual entry point signature below, which is somewhat
    // cumbersome. It is easy for this list of signatures to get out of sync with the entry points that are
    // bound on the server, but as long as we register these two entry points below, then we will be able to
    // detect any other function signature mismatches that arise between the client and server.

    Client::registerWorkerThreadEntryPointSignature<std::vector<FuncSignatureTypeDesc>>                    (client_class);
    Client::registerWorkerThreadEntryPointSignature<std::map<std::string, std::vector<FuncSignatureDesc>>> (client_class);

    // worker thread entry points

    // 0 args
    Client::registerWorkerThreadEntryPointSignature<void>            (client_class);
    Client::registerWorkerThreadEntryPointSignature<bool>            (client_class);
    Client::registerWorkerThreadEntryPointSignature<int64_t>         (client_class);
    Client::registerWorkerThreadEntryPointSignature<uint64_t>        (client_class);
    Client::registerWorkerThreadEntryPointSignature<std::string>     (client_class);

    // 1 args
    Client::registerWorkerThreadEntryPointSignature<void,             bool>               (client_class);
    Client::registerWorkerThreadEntryPointSignature<void,             const std::string&> (client_class);
    Client::registerWorkerThreadEntryPointSignature<bool,             bool>               (client_class);

    // 3 args
    Client::registerWorkerThreadEntryPointSignature<SharedMemoryView, const std::string&,  uint64_t,       const std::vector<std::string>&>(client_class);

    // game thread entry points

    // 0 args
    Client::registerGameThreadEntryPointSignature<bool>                                   (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>>                  (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<StaticStructDesc>>          (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>>        (client_class);

    // 1 args 
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    const std::string&>  (client_class);
    Client::registerGameThreadEntryPointSignature<bool,                                    uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<float,                                   uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<int64_t,                                 uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                const std::string&>  (client_class);
    Client::registerGameThreadEntryPointSignature<std::string,                             uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<std::string>,                uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, SharedMemoryView>, uint64_t>            (client_class);
    Client::registerGameThreadEntryPointSignature<PropertyValue,                           const PropertyDesc&> (client_class);
    Client::registerGameThreadEntryPointSignature<StaticStructDesc,                        uint64_t>            (client_class);

    // 2 args
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             bool>                            (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             uint64_t>                        (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             const std::string&>              (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    PropertyDesc,         const std::string&>              (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             bool>                            (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             const std::string&>              (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             const std::vector<std::string>&> (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                const std::string&,   uint64_t>                        (client_class);
    Client::registerGameThreadEntryPointSignature<std::string,                             uint64_t,             bool>                            (client_class);
    Client::registerGameThreadEntryPointSignature<std::string,                             uint64_t,             uint64_t>                        (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t,             bool>                            (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t,             const std::string&>              (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t,             const std::vector<std::string>&> (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,             bool>                            (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,             const std::string&>              (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,             const std::vector<std::string>&> (client_class);
    Client::registerGameThreadEntryPointSignature<PropertyDesc,                            uint64_t,             const std::string&>              (client_class);
    Client::registerGameThreadEntryPointSignature<PropertyValue,                           uint64_t,             const std::string&>              (client_class);

    // 3 args
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             bool,                             const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             float,                            const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             int32_t,                          const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             uint64_t,                         const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             const std::string&,               float>                                     (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             const std::string&,               const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             const std::string&,               const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<bool,                                    uint64_t,             bool,                             bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             const std::string&,               const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t,             uint64_t,                         bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t,             const std::vector<std::string>&,  const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,             uint64_t,                         bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,             const std::vector<std::string>&,  const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<PropertyDesc,                            uint64_t,             uint64_t,                         const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<PropertyValue,                           uint64_t,             uint64_t,                         const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<DataBundle,                              uint64_t,             const std::string&,               const DataBundle&>                         (client_class);

    // 4 args
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             uint64_t,                         const std::string&,                         const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             const std::string&,               const std::string&,                         const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         uint64_t,                                   const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                         bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         const std::vector<std::string>&,            bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t,             uint64_t,                         const std::string&,                         bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<std::vector<uint64_t>,                   uint64_t,             uint64_t,                         const std::vector<std::string>&,            bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,             uint64_t,                         const std::string&,                         bool>                                      (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, uint64_t>,         uint64_t,             uint64_t,                         const std::vector<std::string>&,            bool>                                      (client_class);

    // 5 args
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             uint64_t,                         const std::string&,                         const std::vector<uint64_t>&,               const std::vector<uint64_t>&>              (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             const std::string&,               const std::string&,                         const std::string&,                         const std::vector<std::string>&>           (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, PropertyValue>,    uint64_t,             uint64_t,                         uint64_t,                                   const std::map<std::string, std::string>&,  const std::string&>                        (client_class);
    Client::registerGameThreadEntryPointSignature<PackedArray,                             uint64_t,             int64_t,                          uint64_t,                                   const std::map<std::string, PackedArray>&,  const PackedArray&>                        (client_class);

    // 6 args
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                         const std::string&,                         const std::vector<std::string>&,            uint64_t>                        (client_class);

    // 7 args
    Client::registerGameThreadEntryPointSignature<void,                                    uint64_t,             const std::string&,               const std::string&,                         const std::string&,                         const std::string&,                         const std::vector<uint64_t>&,     const std::vector<uint64_t>&>    (client_class);
    Client::registerGameThreadEntryPointSignature<std::map<std::string, PackedArray>,      uint64_t,             uint64_t,                         int64_t,                                    uint64_t,                                   const std::map<std::string, PackedArray>&,  const std::vector<std::string>&,  const std::vector<std::string>&> (client_class);

    // 8 args
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                         const std::string&,                         const std::vector<std::string>&,            uint64_t,                         bool,                             uint64_t>      (client_class);
    Client::registerGameThreadEntryPointSignature<uint64_t,                                uint64_t,             uint64_t,                         const std::string&,                         const std::vector<std::string>&,            uint64_t,                                   bool,                             uint64_t,                         uint64_t>      (client_class);

    //
    // Custom types
    //

    auto property_desc_class = nanobind::class_<PropertyDesc>(module, "PropertyDesc");
    property_desc_class.def(nanobind::init<>());
    property_desc_class.def_rw("property",  &PropertyDesc::property_);
    property_desc_class.def_rw("value_ptr", &PropertyDesc::value_ptr_);
    property_desc_class.def("__repr__", [](const PropertyDesc& self) {
        std::ostringstream oss;
        oss << "spear_ext.PropertyDesc(" << "property=" << self.property_ << ", value_ptr=" << self.value_ptr_ << ")";
        return oss.str();
    });

    auto property_value_class = nanobind::class_<PropertyValue>(module, "PropertyValue");
    property_value_class.def(nanobind::init<>());
    property_value_class.def_rw("value",   &PropertyValue::value_);
    property_value_class.def_rw("type_id", &PropertyValue::type_id_);
    property_value_class.def("__repr__", [](const PropertyValue& self) {
        std::ostringstream oss;
        oss << "spear_ext.PropertyValue(" << "value=" << self.value_ << ", type_id=" << self.type_id_ << ")";
        return oss.str();
    });

    auto shared_memory_view_class = nanobind::class_<SharedMemoryView>(module, "SharedMemoryView");
    shared_memory_view_class.def(nanobind::init<>());
    shared_memory_view_class.def_rw("id",           &SharedMemoryView::id_);
    shared_memory_view_class.def_rw("num_bytes",    &SharedMemoryView::num_bytes_);
    shared_memory_view_class.def_rw("offset_bytes", &SharedMemoryView::offset_bytes_);
    shared_memory_view_class.def_rw("name",         &SharedMemoryView::name_);
    shared_memory_view_class.def_rw("usage_flags",  &SharedMemoryView::usage_flags_);
    shared_memory_view_class.def("__repr__", [](const SharedMemoryView& self) {
        std::ostringstream oss;
        oss << "spear_ext.SharedMemoryView(id=" << self.id_ << ", num_bytes=" << self.num_bytes_ << ", offset_bytes=" << self.offset_bytes_ << ", usage_flags=[";
        for (int i = 0; i < self.usage_flags_.size(); i++) {
            if (i > 0) {
                oss << ", ";
            }
            oss << "\"" << Std::at(self.usage_flags_, i) << "\"";
        }
        oss << "]" << ")";
        return oss.str();
    });

    auto packed_array_class = nanobind::class_<PackedArray>(module, "PackedArray");
    packed_array_class.def(nanobind::init<>());
    packed_array_class.def_rw("data",               &PackedArray::data_, nanobind::rv_policy::reference);
    packed_array_class.def_rw("data_source",        &PackedArray::data_source_);
    packed_array_class.def_rw("shape",              &PackedArray::shape_);
    packed_array_class.def_rw("shared_memory_name", &PackedArray::shared_memory_name_);
    packed_array_class.def("__repr__", [](const PackedArray& self) {
        std::ostringstream oss;
        nanobind::object data = nanobind::cast(self.data_);
        std::string data_str;
        if (!self.data_.is_valid()) {
            data_str = "is_valid()==false";
        } else if (!self.data_.data()) {
            data_str = "data()==nullptr";
        } else {
            data_str = nanobind::repr(data).c_str();
        }
        oss << "spear_ext.PackedArray(data=" << data_str << ", data_source=\"" << self.data_source_ << "\", shape=[";
        for (int i = 0; i < self.shape_.size(); i++) {
            if (i > 0) {
                oss << ", ";
            }
            oss << Std::at(self.shape_, i);
        }
        oss << "]" << ", shared_memory_name=\"" << self.shared_memory_name_ << "\")";
        return oss.str();
    });

    auto data_bundle_class = nanobind::class_<DataBundle>(module, "DataBundle");
    data_bundle_class.def(nanobind::init<>());
    data_bundle_class.def_rw("packed_arrays",      &DataBundle::packed_arrays_, nanobind::rv_policy::reference);
    data_bundle_class.def_rw("unreal_obj_strings", &DataBundle::unreal_obj_strings_);
    data_bundle_class.def_rw("info",               &DataBundle::info_);
    data_bundle_class.def("__repr__", [](const DataBundle& self) {
        std::ostringstream oss;
        oss << "spear_ext.DataBundle(packed_arrays=" << self.packed_arrays_.size() << ", unreal_obj_strings=" << self.unreal_obj_strings_.size() << ", info=" << self.info_.size() << ")";
        return oss.str();
    });

    auto func_signature_type_desc_class = nanobind::class_<FuncSignatureTypeDesc>(module, "FuncSignatureTypeDesc");
    func_signature_type_desc_class.def(nanobind::init<>());
    func_signature_type_desc_class.def_rw("type_names",    &FuncSignatureTypeDesc::type_names_);
    func_signature_type_desc_class.def_rw("const_strings", &FuncSignatureTypeDesc::const_strings_);
    func_signature_type_desc_class.def_rw("ref_strings",   &FuncSignatureTypeDesc::ref_strings_);
    func_signature_type_desc_class.def("__repr__", [](const FuncSignatureTypeDesc& self) {
        std::ostringstream oss;
        oss << "spear_ext.FuncSignatureTypeDesc(type_names=" << self.type_names_.size() << ", const_strings=" << self.const_strings_.size() << ", ref_strings=" << self.ref_strings_.size() << ")";
        return oss.str();
    });

    auto func_signature_desc_class = nanobind::class_<FuncSignatureDesc>(module, "FuncSignatureDesc");
    func_signature_desc_class.def(nanobind::init<>());
    func_signature_desc_class.def_rw("name",              &FuncSignatureDesc::name_);
    func_signature_desc_class.def_rw("func_signature",    &FuncSignatureDesc::func_signature_);
    func_signature_desc_class.def_rw("func_signature_id", &FuncSignatureDesc::func_signature_id_);
    func_signature_desc_class.def("__repr__", [](const FuncSignatureDesc& self) {
        std::ostringstream oss;
        oss << "spear_ext.FuncSignatureDesc(name='" << self.name_ << "', func_signature_id=[";
        for (int i = 0; i < self.func_signature_id_.size(); i++) {
            if (i > 0) {
                oss << ", ";
            }
            oss << Std::at(self.func_signature_id_, i);
        }
        oss << "])";
        return oss.str();
    });

    auto future_class = nanobind::class_<Future>(module, "Future");
    future_class.def(nanobind::init<>());
    future_class.def_rw("future_ptr", &Future::future_ptr_);
    future_class.def_rw("type_id",    &Future::type_id_);
    future_class.def("__repr__", [](const Future& self) {
        std::ostringstream oss;
        oss << "spear_ext.Future(future_ptr=" << self.future_ptr_ << ", type_id=\"" << self.type_id_ << "\")";
        return oss.str();
    });

    auto static_struct_desc_class = nanobind::class_<StaticStructDesc>(module, "StaticStructDesc");
    static_struct_desc_class.def(nanobind::init<>());
    static_struct_desc_class.def_rw("static_struct", &StaticStructDesc::static_struct_);
    static_struct_desc_class.def_rw("name",          &StaticStructDesc::name_);
    static_struct_desc_class.def_rw("ufunctions",    &StaticStructDesc::ufunctions_);
    static_struct_desc_class.def("__repr__", [](const StaticStructDesc &self) {
        std::ostringstream oss;
        oss << "spear_ext.StaticStructDesc(name='" << self.name_ << "', static_struct=" << self.static_struct_ << ", ufunctions=" << self.ufunctions_.size() << ")";
        return oss.str();
    });
}
