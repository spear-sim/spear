//
// Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
// Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

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

    client_class.def("ping",                            &Client::ping);
    client_class.def("get_entry_point_signature_descs", &Client::getEntryPointSignatureDescs);

    client_class.def("call", &Client::call);

    client_class.def_rw("force_return_aligned_arrays", &Client::force_return_aligned_arrays_);
    client_class.def_rw("verbose_rpc_calls",           &Client::verbose_rpc_calls_);
    client_class.def_rw("verbose_allocations",         &Client::verbose_allocations_);
    client_class.def_rw("verbose_exceptions",          &Client::verbose_exceptions_);

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

    auto world_desc_class = nanobind::class_<WorldDesc>(module, "WorldDesc");
    world_desc_class.def(nanobind::init<>());
    world_desc_class.def_rw("world",           &WorldDesc::world_);
    world_desc_class.def_rw("world_id",        &WorldDesc::world_id_);
    world_desc_class.def_rw("is_editor_world", &WorldDesc::is_editor_world_);
    world_desc_class.def_rw("is_game_world",   &WorldDesc::is_game_world_);
    world_desc_class.def_rw("is_playing",      &WorldDesc::is_playing_);
    world_desc_class.def("__repr__", [](const WorldDesc& self) {
        std::ostringstream oss;
        oss << "spear_ext.WorldDesc(world=" << self.world_ << ", world_id=" << self.world_id_ << ", is_editor_world=" << self.is_editor_world_ << ", is_game_world=" << self.is_game_world_ << ", is_playing=" << self.is_playing_ << ")";
        return oss.str();
    });
}
