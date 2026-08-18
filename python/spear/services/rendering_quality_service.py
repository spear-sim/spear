#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


class RenderingQualityService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, console_service, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

        self._console_service = console_service
        self._sp_scalability = None
        self._restore_stack = []

    def initialize(self):
        self._sp_scalability = self.get_unreal_object(uclass="USpScalability")

    #
    # set_mrq_rendering_quality(...) mirrors UMoviePipelineGameOverrideSetting::ApplyCVarSettings(bOverrideValues=true).
    # The default arguments match the switches and values in the UMoviePipelineGameOverrideSetting constructor. Each
    # call stores the previous state of everything it overrides, and restore_rendering_quality() undoes the most recent
    # call.
    #

    def set_default_mrq_rendering_quality(
        self,
        cinematic_quality_settings=True,
        texture_streaming="Disabled",
        use_lod_zero=True,
        disable_hlods=True,
        use_high_quality_shadows=True,
        shadow_distance_scale=10.0,
        shadow_radius_threshold=0.001,
        override_view_distance_scale=True,
        view_distance_scale=50.0,
        disable_gpu_timeout=True,
        flush_streaming_managers=True):

        spear.log_current_function()

        assert texture_streaming in ["None", "Disabled", "FullyLoad"]

        # Capture the true original state BEFORE applying any override, so restore returns to the pre-call
        # state. We capture the full Scalability state first, then each individual cvar. Capturing before we
        # change anything avoids the pollution that UMoviePipelineGameOverrideSetting works around, i.e., it
        # applies Scalability first and so caches individual cvar values that have already been bumped to
        # Cinematic.

        # bCinematicQualitySettings and bOverrideValues
        previous_quality_levels = None
        if cinematic_quality_settings:
            previous_quality_levels = self._sp_scalability.GetQualityLevels()

        cvar_descs = []

        # TextureStreaming
        if texture_streaming == "FullyLoad":
            cvar_descs.append({"name": "r.Streaming.FramesForFullUpdate", "value": 0, "type": "int", "optional": False})
            cvar_descs.append({"name": "r.Streaming.FullyLoadUsedTextures", "value": 1, "type": "int", "optional": False})
        elif texture_streaming == "Disabled":
            cvar_descs.append({"name": "r.TextureStreaming", "value": 0, "type": "int", "optional": False})
        elif texture_streaming == "None":
            pass
        else:
            assert False

        # bUseLODZero
        if use_lod_zero:
            cvar_descs.append({"name": "r.ForceLOD", "value": 0, "type": "int", "optional": False})
            cvar_descs.append({"name": "r.SkeletalMeshLODBias", "value": -10, "type": "int", "optional": False})
            cvar_descs.append({"name": "r.ParticleLODBias", "value": -10, "type": "int", "optional": False})
            cvar_descs.append({"name": "foliage.DitheredLOD", "value": 0, "type": "int", "optional": False})
            cvar_descs.append({"name": "foliage.ForceLOD", "value": 0, "type": "int", "optional": False})

        # bUseHighQualityShadows
        if use_high_quality_shadows:
            cvar_descs.append({"name": "r.Shadow.DistanceScale", "value": shadow_distance_scale, "type": "float", "optional": False})
            cvar_descs.append({"name": "r.ShadowQuality", "value": 5, "type": "int", "optional": False})
            cvar_descs.append({"name": "r.Shadow.RadiusThreshold", "value": shadow_radius_threshold, "type": "float", "optional": False})

        # bOverrideViewDistanceScale
        if override_view_distance_scale:
            cvar_descs.append({"name": "r.ViewDistanceScale", "value": view_distance_scale, "type": "float", "optional": False})

        # bDisableGPUTimeout (this cvar only exists if the D3D12RHI module is loaded)
        if disable_gpu_timeout:
            cvar_descs.append({"name": "r.D3D12.GPUTimeout", "value": 0, "type": "int", "optional": True})

        # bFlushStreamingManagers
        if flush_streaming_managers:
            cvar_descs.append({"name": "r.Streaming.SyncStatesWhenBlocking", "value": 1, "type": "int", "optional": False})

        # GeometryCache streamer cvars (only exist in editor builds)
        cvar_descs.append({"name": "GeometryCache.Streamer.BlockTillFinishStreaming", "value": 1, "type": "int", "optional": True})
        cvar_descs.append({"name": "GeometryCache.Streamer.ShowNotification", "value": 0, "type": "int", "optional": True})

        # Cvars that UMoviePipelineGameOverrideSetting always overrides, regardless of the switches above.
        cvar_descs.append({"name": "a.URO.Enable", "value": 0, "type": "int", "optional": False})
        cvar_descs.append({"name": "au.NeverMuteNonRealtimeAudioDevices", "value": 1, "type": "int", "optional": False})
        cvar_descs.append({"name": "r.SkyLight.RealTimeReflectionCapture.TimeSlice", "value": 0, "type": "int", "optional": False})
        cvar_descs.append({"name": "r.VolumetricRenderTarget", "value": 1, "type": "int", "optional": False})
        cvar_descs.append({"name": "r.VolumetricRenderTarget.Mode", "value": 3, "type": "int", "optional": False})
        cvar_descs.append({"name": "wp.Runtime.BlockOnSlowStreaming", "value": 0, "type": "int", "optional": False})
        cvar_descs.append({"name": "p.Chaos.ImmPhys.MinStepTime", "value": 0.0, "type": "float", "optional": False})
        cvar_descs.append({"name": "r.SkipRedundantTransformUpdate", "value": 0, "type": "int", "optional": False})
        cvar_descs.append({"name": "p.ChaosCloth.UseTimeStepSmoothing", "value": 0, "type": "int", "optional": False})
        cvar_descs.append({"name": "r.Water.SkipWaterInfoTextureRenderWhenWorldRenderingDisabled", "value": 0, "type": "int", "optional": True})
        cvar_descs.append({"name": "r.Nanite.VSMInvalidateOnLODDelta", "value": 1, "type": "int", "optional": False})

        # Drop any optional cvars that aren't currently registered (e.g., r.D3D12.GPUTimeout without D3D12RHI).
        cvar_descs = [ cvar_desc for cvar_desc in cvar_descs if not (cvar_desc["optional"] and not self._console_service.exists(name=cvar_desc["name"])) ]

        # Cache previous cvar values.
        previous_cvar_descs = [ {**cvar_desc, "value": self._get_cvar_value(cvar_desc=cvar_desc)} for cvar_desc in cvar_descs ]

        # bCinematicQualitySettings and bOverrideValues: compute the new Scalability quality levels (if any).
        cinematic_quality_levels = None
        if cinematic_quality_settings:
            cinematic_quality_levels = self._sp_scalability.GetQualityLevelsFromSingleQualityLevelRelativeToMax(SpQualityLevels=previous_quality_levels, Value=0)

        self.log_quality_levels(label="Previous quality levels:", quality_levels=previous_quality_levels)
        self.log_cvar_descs(label="Previous cvars:", cvar_descs=previous_cvar_descs)
        self.log_quality_levels(label="New quality levels:", quality_levels=cinematic_quality_levels)
        self.log_cvar_descs(label="New cvars:", cvar_descs=cvar_descs)

        if cinematic_quality_levels is not None:
            self._sp_scalability.SetQualityLevels(SpQualityLevels=cinematic_quality_levels)
        for cvar_desc in cvar_descs:
            self._console_service.set(name=cvar_desc["name"], value=cvar_desc["value"], set_with_current_priority=True)

        # bDisableHLODs
        if disable_hlods:
            spear.log("    Executing console command: r.HLOD 0")
            self._console_service.execute(command="r.HLOD 0")

        self._restore_stack.append({"cvar_descs": previous_cvar_descs, "quality_levels": previous_quality_levels})


    def restore_rendering_quality(self, enable_hlods=True):
        spear.log_current_function()

        assert len(self._restore_stack) > 0
        restore_data = self._restore_stack.pop()

        if enable_hlods:
            spear.log("    Executing console command: r.HLOD 1")
            self._console_service.execute(command="r.HLOD 1")

        self.log_cvar_descs(label="Restoring cvars:", cvar_descs=restore_data["cvar_descs"])
        self.log_quality_levels(label="Restoring quality levels:", quality_levels=restore_data["quality_levels"])

        for cvar_desc in restore_data["cvar_descs"]:
            self._console_service.set(name=cvar_desc["name"], value=cvar_desc["value"], set_with_current_priority=True)
        if restore_data["quality_levels"] is not None:
            self._sp_scalability.SetQualityLevels(SpQualityLevels=restore_data["quality_levels"])


    def log_quality_levels(self, label, quality_levels):
        if quality_levels is not None:
            spear.log(f"    {label}")
            for name, value in quality_levels.items():
                spear.log(f"        {name} = {value}")

    def log_cvar_descs(self, label, cvar_descs):
        spear.log(f"    {label}")
        for cvar_desc in cvar_descs:
            spear.log(f"        {cvar_desc['name']} = {cvar_desc['value']}")


    def _get_cvar_value(self, cvar_desc):
        assert cvar_desc["type"] in ["int", "float"]
        if cvar_desc["type"] == "int":
            return self._console_service.get_as_int(name=cvar_desc["name"])
        elif cvar_desc["type"] == "float":
            return self._console_service.get_as_float(name=cvar_desc["name"])
        else:
            assert False
