#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


#
# ClassDefaultObjectService maintains a cache of class default objects (CDOs), so callers can call functions on a
# CDO without storing it, e.g., self.cdo.USpScalability.GetQualityLevels() from inside a service, or
# game.cdo.UGameplayStatics.GetPlayerController(PlayerIndex=0) from user code. Each CDO is fetched from the Unreal
# instance the first time it is accessed. All CDO access must happen inside a begin_frame() or end_frame() block.
#

class ClassDefaultObjectService(spear.Service):
    def __init__(self, sp_func_service, unreal_service, config):
        assert sp_func_service.is_top_level_service()
        assert unreal_service.is_top_level_service()

        super().__init__(
            entry_point_caller=None,
            sp_func_service=sp_func_service,
            unreal_service=unreal_service,
            config=config)

        self._cdos = {}

    def invalidate_cache(self):
        self._cdos = {}

    def __getattr__(self, name):
        if name in self.unreal_service.get_static_class_descs_by_name():
            return self.get(uclass=name)
        else:
            raise AttributeError(name)

    def get(self, uclass):
        uclass = self.unreal_service.to_uclass(uclass=uclass)
        if uclass not in self._cdos:
            cdo = self.get_unreal_object(uclass=uclass)
            assert cdo is not None
            self._cdos[uclass] = cdo
        return self._cdos[uclass]
