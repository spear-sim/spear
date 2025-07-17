#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

__version__ = "v0.6.0"

# submodules
import spear.services
import spear.utils

# functions
from spear.utils.config_utils import get_config
from spear.utils.func_utils import to_handle, to_matrix_from_rotator, to_ptr, to_shared, to_vector_from_array
from spear.utils.log_utils import log, log_current_function, log_no_prefix, log_get_prefix
from spear.utils.system_utils import configure_system

# classes
from spear.instance import Instance
