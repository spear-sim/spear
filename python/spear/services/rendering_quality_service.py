#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


class RenderingQualityService(spear.Service):
    def __init__(self, entry_point_caller, sp_func_service, unreal_service, engine_globals_service, console_service, cdo, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()
        assert engine_globals_service.is_top_level_service()

        super().__init__(
            entry_point_caller=entry_point_caller,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            cdo=cdo,
            config=config)

        self._engine_globals_service = engine_globals_service
        self._console_service = console_service
        self._restore_stack = []

    #
    # set_default_mrq_rendering_quality(...) mirrors UMoviePipelineGameOverrideSetting::ApplyCVarSettings(bOverrideValues=true).
    # The default arguments match the switches and values in the UMoviePipelineGameOverrideSetting constructor. Each
    # call stores the previous state of everything it overrides, and restore_rendering_quality() undoes the most recent
    # call.
    #

    def set_default_mrq_rendering_quality(
        self,
        cinematic_quality_settings=True,
        texture_streaming="Disabled",
        use_lod_zero=True,
        use_high_quality_shadows=True,
        shadow_distance_scale=10.0,
        shadow_radius_threshold=0.001,
        override_view_distance_scale=True,
        view_distance_scale=50.0,
        disable_gpu_timeout=True,
        flush_streaming_managers=True,
        enable_vt_invalidate_on_nanite_lod=True,
        disable_hlods=True):

        spear.log_current_function()

        assert texture_streaming in ["None", "Disabled", "FullyLoad"]

        # Capture the true original state before applying any override, so restore returns to the pre-call
        # state. We capture the full Scalability state first, then each individual cvar. Capturing before we
        # change anything avoids the pollution that UMoviePipelineGameOverrideSetting works around, i.e., it
        # applies Scalability first and so caches individual cvar values that have already been bumped to
        # Cinematic.

        # bCinematicQualitySettings and bOverrideValues
        previous_quality_levels = None
        if cinematic_quality_settings:
            previous_quality_levels = self.cdo.USpScalability.GetQualityLevels()

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

        # bUseLODZero (r.ForceLOD is compiled out of Shipping and Test builds, see Engine/Source/Runtime/RenderCore/Private/RenderCore.cpp)
        if use_lod_zero:
            cvar_descs.append({"name": "r.ForceLOD", "value": 0, "type": "int", "optional": True})
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

        # MoviePipeline.EnableVTInvalidateOnNaniteLOD
        if enable_vt_invalidate_on_nanite_lod:
            cvar_descs.append({"name": "r.Nanite.VSMInvalidateOnLODDelta", "value": 1, "type": "int", "optional": False})

        # Drop any optional cvars that aren't currently registered (e.g., r.D3D12.GPUTimeout without D3D12RHI).
        cvar_descs = [ cvar_desc for cvar_desc in cvar_descs if not (cvar_desc["optional"] and not self._console_service.exists(name=cvar_desc["name"])) ]

        # Cache previous cvar values.
        previous_cvar_descs = [ {**cvar_desc, "value": self._get_cvar_value(cvar_desc=cvar_desc)} for cvar_desc in cvar_descs ]

        # bCinematicQualitySettings and bOverrideValues. To mimic UMoviePipelineGameOverrideSetting::ApplyCVarSettings(...)
        # exactly, we could call QualityLevelsFromSingleQualityLevelRelativeToMax(Value=0). But this is functionally
        # equivalent to calling QualityLevelsFromSingleQualityLevel(QualityLevel="Cinematic") with the default number of
        # scalability levels, and we prefer QualityLevelsFromSingleQualityLevel(QualityLevel="Cinematic") because it is
        # more readable.
        cinematic_quality_levels = None
        if cinematic_quality_settings:
            cinematic_quality_levels = self.cdo.USpQualityLevelsUtils.QualityLevelsFromSingleQualityLevel(QualityLevel="Cinematic")

        # Scalability::SetQualityLevels(...) overwrites every cvar in the scalability INI sections of each group whose
        # quality level changes, both when we apply the new quality levels and when we restore the previous ones. So
        # restoring the previous quality levels doesn't necessarily restore the previous values of these cvars (e.g., if
        # they were set by the user after the previous quality levels were applied). To restore these cvars exactly, we
        # cache their previous values here.
        previous_scalability_cvar_descs = None
        if cinematic_quality_settings:
            scalability_cvar_names = sorted(
                set(self._get_scalability_cvar_names(quality_levels=previous_quality_levels)) |
                set(self._get_scalability_cvar_names(quality_levels=cinematic_quality_levels)))
            scalability_cvar_descs = [ {"name": name, "type": "str", "optional": False} for name in scalability_cvar_names ]
            previous_scalability_cvar_descs = [ {**cvar_desc, "value": self._get_cvar_value(cvar_desc=cvar_desc)} for cvar_desc in scalability_cvar_descs ]

        self.log_quality_levels(label="Previous quality levels:", quality_levels=previous_quality_levels)
        self.log_cvar_descs(label="Previous scalability cvars:", cvar_descs=previous_scalability_cvar_descs)
        self.log_cvar_descs(label="Previous cvars:", cvar_descs=previous_cvar_descs)
        self.log_quality_levels(label="New quality levels:", quality_levels=cinematic_quality_levels)
        self.log_cvar_descs(label="New cvars:", cvar_descs=cvar_descs)

        if cinematic_quality_levels is not None:
            self.cdo.USpScalability.SetQualityLevels(QualityLevels=cinematic_quality_levels)

        for cvar_desc in cvar_descs:
            self._console_service.set_cvar(name=cvar_desc["name"], value=cvar_desc["value"], set_with_current_priority=True)

        # bDisableHLODs. r.HLOD is a console command rather than a cvar, so there is no previous value to cache. Instead,
        # it calls SetActorHiddenInGame(...) on every ALODActor, see Engine/Source/Runtime/Engine/Private/LODActor.cpp.
        # r.HLOD is compiled out of Shipping builds, so we skip it in Shipping builds.
        if disable_hlods:
            if self._engine_globals_service.get_ue_build() == "shipping":
                spear.log("Skipping console command because it is unavailable in Shipping builds: r.HLOD 0")
            else:
                spear.log("Executing console command: r.HLOD 0")
                self._console_service.execute_command(command="r.HLOD 0")

        self._restore_stack.append({
            "quality_levels": previous_quality_levels,
            "scalability_cvar_descs": previous_scalability_cvar_descs,
            "cvar_descs": previous_cvar_descs,
            "disable_hlods": disable_hlods})


    #
    # restore_rendering_quality(...) restores state in the reverse order that set_default_mrq_rendering_quality(...)
    # overrides it, i.e., r.HLOD, then individual cvars, then quality levels. Restoring quality levels overwrites the
    # cvars in the corresponding scalability INI sections, so we restore these scalability cvars immediately after
    # restoring quality levels. If enable_hlods is None, we execute r.HLOD 1 only if the corresponding call to
    # set_default_mrq_rendering_quality(...) executed r.HLOD 0.
    #

    def restore_rendering_quality(self, enable_hlods=None):
        spear.log_current_function()

        assert len(self._restore_stack) > 0
        restore_data = self._restore_stack.pop()

        if enable_hlods is None:
            enable_hlods = restore_data["disable_hlods"]

        self.log_cvar_descs(label="Restoring cvars:", cvar_descs=restore_data["cvar_descs"])
        self.log_quality_levels(label="Restoring quality levels:", quality_levels=restore_data["quality_levels"])
        self.log_cvar_descs(label="Restoring scalability cvars:", cvar_descs=restore_data["scalability_cvar_descs"])

        if enable_hlods:
            if self._engine_globals_service.get_ue_build() == "shipping":
                spear.log("Skipping console command because it is unavailable in Shipping builds: r.HLOD 1")
            else:
                spear.log("Executing console command: r.HLOD 1")
                self._console_service.execute_command(command="r.HLOD 1")

        for cvar_desc in restore_data["cvar_descs"]:
            self._console_service.set_cvar(name=cvar_desc["name"], value=cvar_desc["value"], set_with_current_priority=True)

        if restore_data["quality_levels"] is not None:
            self.cdo.USpScalability.SetQualityLevels(QualityLevels=restore_data["quality_levels"])
            for cvar_desc in restore_data["scalability_cvar_descs"]:
                self._console_service.set_cvar(name=cvar_desc["name"], value=cvar_desc["value"], set_with_current_priority=True)


    def log_quality_levels(self, label, quality_levels):
        if quality_levels is not None:
            spear.log(f"    {label}")
            for name, value in quality_levels.items():
                spear.log(f"        {name} = {value}")

    def log_cvar_descs(self, label, cvar_descs):
        if cvar_descs is not None:
            spear.log(f"    {label}")
            for cvar_desc in cvar_descs:
                spear.log(f"        {cvar_desc['name']} = {cvar_desc['value']}")


    def _get_cvar_value(self, cvar_desc):
        assert cvar_desc["type"] in ["int", "float", "str"]
        if cvar_desc["type"] == "int":
            return self._console_service.get_cvar_as(name=cvar_desc["name"], as_type=int)
        elif cvar_desc["type"] == "float":
            return self._console_service.get_cvar_as(name=cvar_desc["name"], as_type=float)
        elif cvar_desc["type"] == "str":
            return self._console_service.get_cvar_as(name=cvar_desc["name"], as_type=str)
        else:
            assert False

    def _get_scalability_cvar_names(self, quality_levels):
        # See the int32 fields of Scalability::FQualityLevels in Engine/Source/Runtime/Engine/Public/Scalability.h.
        scalability_groups = [
            "ViewDistanceQuality",
            "AntiAliasingQuality",
            "ShadowQuality",
            "GlobalIlluminationQuality",
            "ReflectionQuality",
            "PostProcessQuality",
            "TextureQuality",
            "EffectsQuality",
            "FoliageQuality",
            "ShadingQuality",
            "LandscapeQuality"]
        cvar_names = []
        for group_name, quality_level in quality_levels.items():
            if group_name in scalability_groups:
                num_levels = self._console_service.get_cvar_as(name=f"sg.{group_name}.NumLevels", as_type=int)
                section_name = self.cdo.USpScalability.GetScalabilitySectionString(InGroupName=group_name, InQualityLevel=quality_level, InNumLevels=num_levels) # example return value: "ShadowQuality@3"
                return_values = self.cdo.USpConfigCacheIni.GetSection(Section=section_name, Filename="Scalability", as_dict=True)
                success = return_values["ReturnValue"]
                section_text = return_values["Result"]
                # Scalability INI sections might not exist (e.g., BaseScalability.ini doesn't define any LandscapeQuality
                # sections), in which case ApplyCVarSettingsFromIni(...) doesn't set any cvars. Scalability INI sections can
                # also refer to cvars that aren't registered (e.g., if they're defined in a module that isn't loaded), and
                # FindConsoleVariable(...) returns nullptr for these cvars. So we skip them, see ApplyCVarSettingsFromIni(...)
                # and ForEachCVarInSectionFromIni(...) in Engine/Source/Runtime/Core/Private/Misc/ConfigUtilities.cpp.
                if success:
                    for line in section_text:
                        cvar_name = line.split("=", 1)[0].strip()
                        if self._console_service.exists(name=cvar_name):
                            cvar_names.append(cvar_name)
        return cvar_names
