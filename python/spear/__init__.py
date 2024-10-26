#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# functions
from spear.config import get_config
from spear.log import log, log_current_function, log_no_prefix, log_get_prefix
from spear.path import path_exists, remove_path
from spear.func_utils import to_handle, to_ptr, to_shared
from spear.system import configure_system

# classes
from spear.instance import Instance
from spear.func_utils import Ptr, Shared

# services
from spear.engine_service import EngineService
from spear.legacy_service import LegacyService
from spear.sp_func_service import SpFuncService
from spear.unreal_service import UnrealService

# TODO: remove legacy classes
from spear.env import Env

__version__ = "v0.5.0"
