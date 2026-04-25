#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json
import numpy as np
import spear


_SP_OBJECT_IDS_PROXY_COMPONENT_MANAGER_NAME = "__SP_OBJECT_IDS_PROXY_COMPONENT_MANAGER__"

class SegmentationService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

        self.proxy_component_manager = None

    def initialize(self):
        proxy_component_managers = self.unreal_service.find_actors_by_class_as_dict(uclass="ASpObjectIdsProxyComponentManager")
        if len(proxy_component_managers) == 0:
            spear.log("No proxy component manager detected, spawning...")
            self.proxy_component_manager = self.unreal_service.spawn_actor(uclass="ASpObjectIdsProxyComponentManager", spawn_parameters={"ObjectFlags": ["RF_Transient"]})
            self.unreal_service.set_stable_name_for_actor(actor=self.proxy_component_manager, stable_name=_SP_OBJECT_IDS_PROXY_COMPONENT_MANAGER_NAME)
            self.proxy_component_manager.Initialize()
            spear.log("Finished spawning.")
        elif len(proxy_component_managers) == 1:
            if _SP_OBJECT_IDS_PROXY_COMPONENT_MANAGER_NAME in proxy_component_managers.keys():
                spear.log("Proxy component manager detected, reinitializing...")
                self.proxy_component_manager = proxy_component_managers[_SP_OBJECT_IDS_PROXY_COMPONENT_MANAGER_NAME]
                self.proxy_component_manager.Terminate()
                self.proxy_component_manager.Initialize()
                spear.log("Finished reinitializing.")
            else:
                spear.log("Unexpected proxy component manager detected, giving up...")
                assert False
        else:
            spear.log("Multiple proxy component managers detected, giving up...")
            assert False

    def terminate(self):
        if self.proxy_component_manager is not None:
            self.proxy_component_manager.Terminate()
            self.unreal_service.destroy_actor(actor=self.proxy_component_manager)
            self.proxy_component_manager = None

    def get_mesh_proxy_geometry_descs(self, include_debug_info=False, mesh_proxy_geometry_descs=None, as_raw=None, as_global=None, as_visible=None):
        assert self.proxy_component_manager is not None

        if as_raw is not None:
            assert as_global is None
            assert as_visible is None
        elif as_global is not None:
            assert as_raw is None
            assert as_visible is None
        elif as_visible is not None:
            assert as_raw is None
            assert as_global is None
        else:
            as_global = True

        # get descs from the manager or use caller-provided descs
        if mesh_proxy_geometry_descs is not None:
            descs = mesh_proxy_geometry_descs
        else:
            spear.log("Getting mesh proxy geomery descs...")
            descs = self.proxy_component_manager.GetMeshProxyGeometryDescs(bIncludeDebugInfo=include_debug_info)
            spear.log("Finished getting mesh proxy geomery descs.")

        # convert pointer strings to handles (no-op if already handles)
        for desc in descs:
            desc["component"] = spear.to_handle(obj=desc["component"])
            desc["material"] = spear.to_handle(obj=desc["material"])
            desc["actor"] = spear.to_handle(obj=desc["actor"])

        # parse property JSON strings into dicts and relabel
        if include_debug_info:
            for desc in descs:
                desc["componentProperties"] = json.loads(desc.pop("componentPropertiesString"))
                desc["materialProperties"] = json.loads(desc.pop("materialPropertiesString"))
                desc["actorProperties"] = json.loads(desc.pop("actorPropertiesString"))

        if as_raw:
            return descs

        # as_global or as_visible: sort descs by raw ID
        descs_sort_indices = np.argsort([ desc["rawId"] for desc in descs ])
        descs = [ descs[i] for i in descs_sort_indices ]

        return descs

    def get_segmentation_data(self, object_ids_bgra_uint8_image=None, object_ids_rgba_float16_image=None, include_debug_info=False, mesh_proxy_geometry_descs=None, as_raw=None, as_global=None, as_visible=None):
        assert self.proxy_component_manager is not None

        if object_ids_bgra_uint8_image is not None:
            assert object_ids_rgba_float16_image is None
        elif object_ids_rgba_float16_image is not None:
            assert object_ids_bgra_uint8_image is None
        else:
            assert False

        if as_raw is not None:
            assert as_global is None
            assert as_visible is None
        elif as_global is not None:
            assert as_raw is None
            assert as_visible is None
        else:
            as_visible = True

        # decode raw segmentation image to raw IDs
        if object_ids_bgra_uint8_image is not None:
            raw_id_image = spear.rendering.get_object_ids_bgra_uint8_as_uint32(object_ids_bgra_uint8=object_ids_bgra_uint8_image)
        elif object_ids_rgba_float16_image is not None:
            raw_id_image = spear.rendering.get_object_ids_rgba_float16_as_uint32(object_ids_rgba_float16=object_ids_rgba_float16_image)
        else:
            assert False

        # get global descs or use caller-provided descs
        if mesh_proxy_geometry_descs is not None:
            global_descs = mesh_proxy_geometry_descs
        else:
            global_descs = self.get_mesh_proxy_geometry_descs(include_debug_info=include_debug_info, as_raw=as_raw, as_global=as_global, as_visible=as_visible)

        if as_raw:
            return raw_id_image, global_descs

        # caller-provided descs must be sorted for as_global and as_visible
        global_raw_ids = np.array([ desc["rawId"] for desc in global_descs ])
        if mesh_proxy_geometry_descs is not None:
            assert np.all(global_raw_ids[:-1] <= global_raw_ids[1:])

        # get visible raw IDs in the image and their inverse mapping
        visible_raw_ids, visible_raw_ids_inverse = np.unique(raw_id_image, return_inverse=True)

        if not np.all(np.isin(visible_raw_ids, global_raw_ids)):
            import matplotlib.pyplot as plt
            visible_raw_ids_not_in_global_raw_ids = visible_raw_ids[np.logical_not(np.isin(visible_raw_ids, global_raw_ids))]
            spear.log("visible_raw_ids: ", visible_raw_ids)
            spear.log("global_raw_ids: ", global_raw_ids)
            spear.log("visible_raw_ids not in global_raw_ids: ", visible_raw_ids_not_in_global_raw_ids)
            fig, (ax1, ax2) = plt.subplots(2, 1)
            ax1.imshow(object_ids_bgra_uint8_image)
            ax2.imshow(np.isin(raw_id_image, visible_raw_ids_not_in_global_raw_ids))
            plt.show()

        # verify all visible raw IDs are known to the manager
        assert np.all(np.isin(visible_raw_ids, global_raw_ids))

        # map visible raw IDs to global array indices via searchsorted (both arrays are sorted)
        visible_global_ids = np.searchsorted(global_raw_ids, visible_raw_ids)

        # remap raw ID image to global ID image
        global_id_image = visible_global_ids[visible_raw_ids_inverse].reshape(raw_id_image.shape)

        if as_global:
            return global_id_image, global_descs

        # ensure the background entry (global index 0) is always included
        if 0 not in visible_global_ids:
            visible_global_ids = np.concatenate(([0], visible_global_ids))

        # filter descs to only include visible raw IDs
        visible_descs = [ global_descs[i] for i in visible_global_ids ]

        # remap global ID image to visible ID image
        global_visible_ids = np.zeros(len(global_descs), dtype=np.int32)
        global_visible_ids[visible_global_ids] = np.arange(len(visible_global_ids))
        visible_id_image = global_visible_ids[global_id_image]

        return visible_id_image, visible_descs

    #
    # Allow/ignore lists
    #

    def get_allowed_actors(self):
        assert self.proxy_component_manager is not None
        return self.proxy_component_manager.GetAllowedActors()

    def set_allowed_actors(self, allowed_actors):
        assert self.proxy_component_manager is not None
        self.proxy_component_manager.SetAllowedActors(AllowedActors=allowed_actors)

    def get_allowed_components(self):
        assert self.proxy_component_manager is not None
        return self.proxy_component_manager.GetAllowedComponents()

    def set_allowed_components(self, allowed_components):
        assert self.proxy_component_manager is not None
        self.proxy_component_manager.SetAllowedComponents(AllowedComponents=allowed_components)

    def get_ignored_actors(self):
        assert self.proxy_component_manager is not None
        return self.proxy_component_manager.GetIgnoredActors()

    def set_ignored_actors(self, ignored_actors):
        assert self.proxy_component_manager is not None
        self.proxy_component_manager.SetIgnoredActors(IgnoredActors=ignored_actors)

    def get_ignored_components(self):
        assert self.proxy_component_manager is not None
        return self.proxy_component_manager.GetIgnoredComponents()

    def set_ignored_components(self, ignored_components):
        assert self.proxy_component_manager is not None
        self.proxy_component_manager.SetIgnoredComponents(IgnoredComponents=ignored_components)
