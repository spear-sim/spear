#
# Copyright(c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

__version__ = "v0.6.0"

# Sometimes spear_ext is unavailable. For example, it can't be installed in the UE Python environment because
# UE doesn't ship with CPython headers. Additionally, the build script that installs spear_ext depends on the
# spear package for logging, so spear_ext will also be unavailable the first time this build script runs. So
# spear_ext needs to be conditionally imported, and we provide a spear.__can_import_spear_ext__ variable that
# can be used for this purpose.

try:
    import spear_ext
    __can_import_spear_ext__ = True
except:
    __can_import_spear_ext__ = False

# submodules
import spear.utils
import spear.services

# functions
from spear.utils.config_utils import get_config
from spear.utils.func_utils import to_handle, to_matrix_from_rotator, to_ptr, to_shared, to_vector_from_array
from spear.utils.log_utils import log, log_current_function, log_no_prefix, log_get_prefix
from spear.utils.system_utils import configure_system

# classes
from spear.instance import Instance
