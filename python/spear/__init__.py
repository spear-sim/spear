#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

__version__ = "v0.6.0"

# submodules
import spear.services

# functions
from spear.config import get_config
from spear.func_utils import to_handle, to_matrix_from_rotator, to_ptr, to_shared, to_vector_from_array
from spear.log import log, log_current_function, log_no_prefix, log_get_prefix
from spear.path import path_exists, remove_path
from spear.system import configure_system

# classes
from spear.instance import Instance
