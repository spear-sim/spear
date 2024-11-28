#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import shutil
import sys

def path_exists(path):
    if os.path.exists(path) or os.path.islink(path):
        return True

    head, tail = os.path.split(path)
    if os.path.exists(head) or os.path.islink(head):
        if tail == "":
            return os.path.exists(head) or os.path.islink(head)
        else:
            return tail in os.listdir(head)
    else:
        return False

def remove_path(path):
    if not path_exists(path=path):
        return

    if os.path.islink(path):
        os.unlink(path)
    elif os.path.isfile(path):
        os.remove(path)
    elif os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)
    else:
        # if we have a broken symlink, then try to use the command-line (we don't attempt to use
        # subprocess.run() because it returns an error when attempting to remove broken symlinks
        if sys.platform == "win32":
            rm_cmd = "del"
        elif sys.platform in ["darwin", "linux"]:
            rm_cmd = "rm"
        else:
            assert False
        cmd = f"{rm_cmd} {path}"
        cmd_result = os.system(cmd)
        assert cmd_result == 0
