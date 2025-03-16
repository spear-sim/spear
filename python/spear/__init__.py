#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# functions
from spear.config import get_config
from spear.log import log, log_current_function, log_no_prefix, log_get_prefix
from spear.path import path_exists, remove_path

from spear.func_utils import \
    to_array, to_arrays, to_handle, to_json_string, to_json_strings, to_matrix_from_rotator, to_ptr, to_packed_array, to_packed_arrays, to_shared, \
    to_vector_from_array, try_to_dict, try_to_dicts

from spear.system import configure_system

# classes
from spear.instance import Instance
from spear.func_utils import Ptr, Shared

# services

# EngineService
from spear.engine_service import EngineService

# services that depend on EngineService
from spear.enhanced_input_service import EnhancedInputService
from spear.input_service import InputService
from spear.legacy_service import LegacyService
from spear.shared_memory_service import SharedMemoryService
from spear.unreal_service import UnrealService

# services that depend on EngineService and SharedMemoryService
from spear.navigation_service import NavigationService
from spear.sp_func_service import SpFuncService

# TODO: remove legacy classes
from spear.env import Env

__version__ = "v0.6.0"
