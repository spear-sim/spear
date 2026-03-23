#
# Copyright (c) 2025 The SPEAR Development Team. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
# Copyright (c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import spear


@spear.editor.script
def script():
    spear.log("inner_script_raise: about to raise")
    yield
    raise ValueError("deliberate test error from inner_script_raise")


if __name__ == "__main__":
    script()
