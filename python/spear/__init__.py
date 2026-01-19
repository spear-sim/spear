#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

__version__ = "v0.6.0"

#
# Sometimes spear_ext is unavailable. For example, it can't be installed in the UE Python environment because
# UE doesn't ship with CPython headers. Additionally, the build script that installs spear_ext depends on the
# spear package for logging, so spear_ext will also be unavailable the first time this build script runs. So
# spear_ext needs to be conditionally imported, and we provide a spear.__can_import_spear_ext__ variable that
# can be used for this purpose.
#

try:
    import spear_ext
    __can_import_spear_ext__ = True
except ImportError:
    __can_import_spear_ext__ = False

#
# utils
#

from . import utils

from .utils.config_utils import get_config
from .utils.func_utils import to_array, to_arrays, to_data_bundle, to_data_bundle_dict, to_handle, to_handle_or_unreal_object, to_json_string, to_json_strings, to_packed_array, to_packed_arrays, to_ptr, to_shared, to_unreal_object
from .utils.func_utils import to_numpy_array_from_vector, to_numpy_matrix_from_rotator, to_rotator_from_numpy_array, to_vector_from_numpy_array
from .utils.func_utils import try_to_dict, try_to_dicts
from .utils.log_utils import log, log_current_function, log_get_prefix, log_no_prefix
from .utils.system_utils import configure_system

from .utils.func_utils import CallSyncEntryPointCaller, Future, PropertyValue, Service, ServiceWrapper, Shared

#
# services
#

from . import services

from .services.engine_service import EngineService
from .services.engine_globals_service import EngineGlobalsService, EngineGlobalsServiceWrapper
from .services.enhanced_input_service import EnhancedInputService
from .services.initialize_world_service import InitializeWorldService
from .services.input_service import InputService
from .services.navigation_service import NavigationService
from .services.shared_memory_service import SharedMemoryService
from .services.sp_func_service import SpFuncService
from .services.unreal_service import UnrealService

#
# unreal_object
#

from .unreal_object import UnrealObject

#
# instance
#

from .instance import Instance
