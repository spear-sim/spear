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
    client_class.def(nanobind::init<const std::string&, const uint16_t, bool>());

    client_class.def("initialize", &Client::initialize);
    client_class.def("terminate",  &Client::terminate);

    client_class.def("get_timeout",   &Client::getTimeout);
    client_class.def("set_timeout",   &Client::setTimeout);
    client_class.def("clear_timeout", &Client::clearTimeout);

    client_class.def("ping",                                 &Client::ping);
    client_class.def("get_entry_point_signature_descs",      &Client::getEntryPointSignatureDescs);
    client_class.def("get_entry_point_signature_type_names", &Client::getEntryPointSignatureTypeNames);

    client_class.def("call", &Client::call);

    client_class.def_rw("force_return_aligned_arrays", &Client::force_return_aligned_arrays_);
    client_class.def_rw("suppress_default_logging",    &Client::suppress_default_logging_);
    client_class.def_rw("verbose_rpc_calls",           &Client::verbose_rpc_calls_);
    client_class.def_rw("verbose_allocations",         &Client::verbose_allocations_);
    client_class.def_rw("verbose_exceptions",          &Client::verbose_exceptions_);

    //
    // Custom types
    //

    auto output_log_messages_class = nanobind::class_<OutputLogMessages>(module, "OutputLogMessages");
    output_log_messages_class.def(nanobind::init<>());
    output_log_messages_class.def_rw("messages",    &OutputLogMessages::messages_);
    output_log_messages_class.def_rw("num_evicted", &OutputLogMessages::num_evicted_);
    output_log_messages_class.def("__repr__", [](const OutputLogMessages& self) {
        std::ostringstream oss;
        oss << "spear_ext.OutputLogMessages(messages=" << self.messages_.size() << ", num_evicted=" << self.num_evicted_ << ")";
        return oss.str();
    });

    auto shared_memory_view_class = nanobind::class_<SharedMemoryView>(module, "SharedMemoryView");
    shared_memory_view_class.def(nanobind::init<>());
    shared_memory_view_class.def_rw("id",           &SharedMemoryView::id_);
    shared_memory_view_class.def_rw("num_bytes",    &SharedMemoryView::num_bytes_);
    shared_memory_view_class.def_rw("offset_bytes", &SharedMemoryView::offset_bytes_);
    shared_memory_view_class.def("__repr__", [](const SharedMemoryView& self) {
        std::ostringstream oss;
        oss << "spear_ext.SharedMemoryView(id=" << self.id_ << ", num_bytes=" << self.num_bytes_ << ", offset_bytes=" << self.offset_bytes_ << ")";
        return oss.str();
    });

    auto sp_property_desc_class = nanobind::class_<SpPropertyDesc>(module, "SpPropertyDesc");
    sp_property_desc_class.def(nanobind::init<>());
    sp_property_desc_class.def_rw("property",                  &SpPropertyDesc::property_);
    sp_property_desc_class.def_rw("value_ptr",                 &SpPropertyDesc::value_ptr_);
    sp_property_desc_class.def_rw("type_id",                   &SpPropertyDesc::type_id_);
    sp_property_desc_class.def_rw("notify_objects",            &SpPropertyDesc::notify_objects_);
    sp_property_desc_class.def_rw("notify_member_properties",  &SpPropertyDesc::notify_member_properties_);
    sp_property_desc_class.def_rw("notify_element_properties", &SpPropertyDesc::notify_element_properties_);
    sp_property_desc_class.def_rw("notify_array_indices",      &SpPropertyDesc::notify_array_indices_);
    sp_property_desc_class.def_rw("notify_map_keys",           &SpPropertyDesc::notify_map_keys_);
    sp_property_desc_class.def("__repr__", [](const SpPropertyDesc& self) {
        std::ostringstream oss;
        oss << "spear_ext.SpPropertyDesc(" << "property=" << self.property_ << ", value_ptr=" << self.value_ptr_ << ")";
        return oss.str();
    });

    auto sp_property_value_class = nanobind::class_<SpPropertyValue>(module, "SpPropertyValue");
    sp_property_value_class.def(nanobind::init<>());
    sp_property_value_class.def_rw("value",   &SpPropertyValue::value_);
    sp_property_value_class.def_rw("type_id", &SpPropertyValue::type_id_);
    sp_property_value_class.def("__repr__", [](const SpPropertyValue& self) {
        std::ostringstream oss;
        oss << "spear_ext.SpPropertyValue(" << "value=" << self.value_ << ", type_id=" << self.type_id_ << ")";
        return oss.str();
    });

    auto sp_array_shared_memory_view_class = nanobind::class_<SpArraySharedMemoryView>(module, "SpArraySharedMemoryView");
    sp_array_shared_memory_view_class.def(nanobind::init<>());
    sp_array_shared_memory_view_class.def_rw("id",           &SpArraySharedMemoryView::id_);
    sp_array_shared_memory_view_class.def_rw("num_bytes",    &SpArraySharedMemoryView::num_bytes_);
    sp_array_shared_memory_view_class.def_rw("offset_bytes", &SpArraySharedMemoryView::offset_bytes_);
    sp_array_shared_memory_view_class.def_rw("usage_flags",  &SpArraySharedMemoryView::usage_flags_);
    sp_array_shared_memory_view_class.def_rw("name",         &SpArraySharedMemoryView::name_);
    sp_array_shared_memory_view_class.def("__repr__", [](const SpArraySharedMemoryView& self) {
        std::ostringstream oss;
        oss << "spear_ext.SpArraySharedMemoryView(id=" << self.id_ << ", num_bytes=" << self.num_bytes_ << ", offset_bytes=" << self.offset_bytes_ << ", usage_flags=[";
        for (int i = 0; i < self.usage_flags_.size(); i++) {
            if (i > 0) {
                oss << ", ";
            }
            oss << "\"" << Std::at(self.usage_flags_, i) << "\"";
        }
        oss << "], name=\"" << self.name_ << "\")";
        return oss.str();
    });

    auto sp_packed_array_class = nanobind::class_<SpPackedArray>(module, "SpPackedArray");
    sp_packed_array_class.def(nanobind::init<>());
    sp_packed_array_class.def_rw("data",               &SpPackedArray::data_, nanobind::rv_policy::reference);
    sp_packed_array_class.def_rw("data_source",        &SpPackedArray::data_source_);
    sp_packed_array_class.def_rw("shape",              &SpPackedArray::shape_);
    sp_packed_array_class.def_rw("shared_memory_name", &SpPackedArray::shared_memory_name_);
    sp_packed_array_class.def("__repr__", [](const SpPackedArray& self) {
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
        oss << "spear_ext.SpPackedArray(data=" << data_str << ", data_source=\"" << self.data_source_ << "\", shape=[";
        for (int i = 0; i < self.shape_.size(); i++) {
            if (i > 0) {
                oss << ", ";
            }
            oss << Std::at(self.shape_, i);
        }
        oss << "]" << ", shared_memory_name=\"" << self.shared_memory_name_ << "\")";
        return oss.str();
    });

    auto sp_func_data_bundle_class = nanobind::class_<SpFuncDataBundle>(module, "SpFuncDataBundle");
    sp_func_data_bundle_class.def(nanobind::init<>());
    sp_func_data_bundle_class.def_rw("packed_arrays",      &SpFuncDataBundle::packed_arrays_, nanobind::rv_policy::reference);
    sp_func_data_bundle_class.def_rw("unreal_obj_strings", &SpFuncDataBundle::unreal_obj_strings_);
    sp_func_data_bundle_class.def_rw("info",               &SpFuncDataBundle::info_);
    sp_func_data_bundle_class.def("__repr__", [](const SpFuncDataBundle& self) {
        std::ostringstream oss;
        oss << "spear_ext.SpFuncDataBundle(packed_arrays=" << self.packed_arrays_.size() << ", unreal_obj_strings=" << self.unreal_obj_strings_.size() << ", info=" << self.info_.size() << ")";
        return oss.str();
    });

    auto sp_func_signature_desc_class = nanobind::class_<SpFuncSignatureDesc>(module, "SpFuncSignatureDesc");
    sp_func_signature_desc_class.def(nanobind::init<>());
    sp_func_signature_desc_class.def_rw("name",       &SpFuncSignatureDesc::name_);
    sp_func_signature_desc_class.def_rw("type_ids",   &SpFuncSignatureDesc::type_ids_);
    sp_func_signature_desc_class.def_rw("type_names", &SpFuncSignatureDesc::type_names_);
    sp_func_signature_desc_class.def("__repr__", [](const SpFuncSignatureDesc& self) {
        std::ostringstream oss;
        oss << "spear_ext.SpFuncSignatureDesc(name='" << self.name_ << "', type_ids=[";
        for (int i = 0; i < self.type_ids_.size(); i++) {
            if (i > 0) {
                oss << ", ";
            }
            oss << Std::at(self.type_ids_, i);
        }
        oss << "], type_names=[";
        for (int i = 0; i < self.type_names_.size(); i++) {
            if (i > 0) {
                oss << ", ";
            }
            oss << "'" << Std::at(self.type_names_, i) << "'";
        }
        oss << "])";
        return oss.str();
    });

    auto sp_future_class = nanobind::class_<SpFuture>(module, "SpFuture");
    sp_future_class.def(nanobind::init<>());
    sp_future_class.def_rw("future_ptr", &SpFuture::future_ptr_);
    sp_future_class.def_rw("type_id",    &SpFuture::type_id_);
    sp_future_class.def("__repr__", [](const SpFuture& self) {
        std::ostringstream oss;
        oss << "spear_ext.SpFuture(future_ptr=" << self.future_ptr_ << ", type_id=\"" << self.type_id_ << "\")";
        return oss.str();
    });

    auto sp_function_desc_class = nanobind::class_<SpFunctionDesc>(module, "SpFunctionDesc");
    sp_function_desc_class.def(nanobind::init<>());
    sp_function_desc_class.def_rw("function",          &SpFunctionDesc::function_);
    sp_function_desc_class.def_rw("function_name",     &SpFunctionDesc::function_name_);
    sp_function_desc_class.def_rw("static_class",      &SpFunctionDesc::static_class_);
    sp_function_desc_class.def_rw("static_class_name", &SpFunctionDesc::static_class_name_);
    sp_function_desc_class.def("__repr__", [](const SpFunctionDesc &self) {
        std::ostringstream oss;
        oss << "spear_ext.SpFunctionDesc(function_name='" << self.function_name_ << "', static_class_name='" << self.static_class_name_ << "')";
        return oss.str();
    });

    auto sp_static_struct_desc_class = nanobind::class_<SpStaticStructDesc>(module, "SpStaticStructDesc");
    sp_static_struct_desc_class.def(nanobind::init<>());
    sp_static_struct_desc_class.def_rw("static_struct", &SpStaticStructDesc::static_struct_);
    sp_static_struct_desc_class.def_rw("name",          &SpStaticStructDesc::name_);
    sp_static_struct_desc_class.def("__repr__", [](const SpStaticStructDesc &self) {
        std::ostringstream oss;
        oss << "spear_ext.SpStaticStructDesc(name='" << self.name_ << "', static_struct=" << self.static_struct_ << ")";
        return oss.str();
    });

    auto sp_static_class_desc_class = nanobind::class_<SpStaticClassDesc>(module, "SpStaticClassDesc");
    sp_static_class_desc_class.def(nanobind::init<>());
    sp_static_class_desc_class.def_rw("static_class",          &SpStaticClassDesc::static_class_);
    sp_static_class_desc_class.def_rw("name",                   &SpStaticClassDesc::name_);
    sp_static_class_desc_class.def_rw("derived_classes",        &SpStaticClassDesc::derived_classes_);
    sp_static_class_desc_class.def_rw("derived_class_names",    &SpStaticClassDesc::derived_class_names_);
    sp_static_class_desc_class.def_rw("function_descs",         &SpStaticClassDesc::function_descs_);
    sp_static_class_desc_class.def("__repr__", [](const SpStaticClassDesc &self) {
        std::ostringstream oss;
        oss << "spear_ext.SpStaticClassDesc(name='" << self.name_ << "', static_class=" << self.static_class_ << ", function_descs=" << self.function_descs_.size() << ")";
        return oss.str();
    });

    auto sp_world_desc_class = nanobind::class_<SpWorldDesc>(module, "SpWorldDesc");
    sp_world_desc_class.def(nanobind::init<>());
    sp_world_desc_class.def_rw("world",           &SpWorldDesc::world_);
    sp_world_desc_class.def_rw("world_id",        &SpWorldDesc::world_id_);
    sp_world_desc_class.def_rw("is_editor_world", &SpWorldDesc::is_editor_world_);
    sp_world_desc_class.def_rw("is_game_world",   &SpWorldDesc::is_game_world_);
    sp_world_desc_class.def_rw("is_playing",      &SpWorldDesc::is_playing_);
    sp_world_desc_class.def("__repr__", [](const SpWorldDesc& self) {
        std::ostringstream oss;
        oss << "spear_ext.SpWorldDesc(world=" << self.world_ << ", world_id=" << self.world_id_ << ", is_editor_world=" << self.is_editor_world_ << ", is_game_world=" << self.is_game_world_ << ", is_playing=" << self.is_playing_ << ")";
        return oss.str();
    });
}
