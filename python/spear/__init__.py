#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

__version__ = "v1.0.0"

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
# We define similar variables for other modules that are not always be available.
#

try:
    import msgpackrpc
    __can_import_msgpackrpc__ = True
except ImportError:
    __can_import_msgpackrpc__ = False

try:
    import unreal
    __can_import_unreal__ = True
except ImportError:
    __can_import_unreal__ = False

#
# utils
#

# import a curated set of functions directly into the spear namespace
from .utils.config_utils import get_config
from .utils.func_utils import to_array, to_arrays, to_data_bundle, to_data_bundle_dict, try_to_dict, try_to_dicts, to_json_string, to_json_strings, to_packed_array, to_packed_arrays, to_ptr, to_shared
from .utils.func_utils import to_handle_or_unreal_struct, to_handle_or_unreal_class, to_handle_or_unreal_object, to_handle, to_unreal_struct, to_unreal_class, to_unreal_object
from .utils.func_utils import to_numpy_array_from_quat, to_numpy_matrix_from_quat, to_quat_from_numpy_array, to_quat_from_numpy_matrix
from .utils.func_utils import to_numpy_array_from_rotator, to_numpy_matrix_from_rotator, to_rotator_from_numpy_array, to_rotator_from_numpy_matrix
from .utils.func_utils import to_numpy_array_from_vector, to_vector_from_numpy_array
from .utils.func_utils import from_script_result, to_script_expr, to_script_struct, to_script_struct_expr
from .utils.log_utils import log, log_current_function, log_get_prefix, log_no_prefix, register_log_func, unregister_log_func
from .utils.system_utils import configure_system

# import a curated set of classes directly into the spear namespace
from .utils.func_utils import CallSyncEntryPointCaller, EditorEntryPointCaller, Future, PropertyValue, Service, Shared

# import entire utils files into child namespaces
from .utils import pipeline_utils as pipeline
from .utils import tool_utils as tools

# conditional imports
if __can_import_spear_ext__:
    assert not __can_import_unreal__
    from spear_ext import DataBundle, PackedArray # directly into the spear namespace

if __can_import_unreal__:
    assert not __can_import_spear_ext__
    from .utils.editor_utils import DataBundle, PackedArray # directly into the spear namespace
    from .utils import editor_utils as editor # into a child namespace

#
# services
#

# import a curated set of classes directly into the spear namespace
from .services.debug_service import DebugService
from .services.engine_service import EngineService
from .services.engine_globals_service import EngineGlobalsService
from .services.enhanced_input_service import EnhancedInputService
from .services.initialize_world_service import InitializeWorldService
from .services.input_service import InputService
from .services.navigation_service import NavigationService
from .services.python_service import PythonService
from .services.shared_memory_service import SharedMemoryService
from .services.sp_func_service import SpFuncService
from .services.unreal_service import UnrealService
from .services.world_registry_service import WorldRegistryService

#
# unreal_object
#

# import a curated set of classes directly into the spear namespace
from .unreal_object import UnrealStruct, UnrealClass, UnrealObject

#
# instance
#

# import a curated set of classes directly into the spear namespace
from .instance import Instance
