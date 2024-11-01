#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import os
import spear
import sys

# perform configuration steps that should only be done once per system, as opposed to once per instance
def configure_system(config):

    # create a symlink to SPEAR.PAKS_DIR
    if config.SPEAR.LAUNCH_MODE == "standalone":

        assert os.path.exists(config.SPEAR.STANDALONE_EXECUTABLE)

        if sys.platform == "win32":
            executable_content_paks_dir = \
                os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(config.SPEAR.STANDALONE_EXECUTABLE)), "..", "..", "Content", "Paks"))
        elif sys.platform == "darwin":
            executable_content_paks_dir = \
                os.path.realpath(os.path.join(config.SPEAR.STANDALONE_EXECUTABLE, "Contents", "UE", "SpearSim", "Content", "Paks"))
        elif sys.platform == "linux":
            executable_content_paks_dir = \
                os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(config.SPEAR.STANDALONE_EXECUTABLE)), "SpearSim", "Content", "Paks"))
        else:
            assert False

        # we don't use os.path.realpath here because we don't want to resolve the symlink
        executable_content_paks_external_dir = os.path.join(executable_content_paks_dir, "__SP_EXTERNAL__")

        if config.SPEAR.PAKS_DIR != "":

            if config.SPEAR.PAKS_VERSION_TAG != "":
                external_paks_version_dir = os.path.realpath(os.path.join(config.SPEAR.PAKS_DIR, config.SPEAR.PAKS_VERSION_TAG))
            else:
                external_paks_version_dir = os.path.realpath(os.path.join(config.SPEAR.PAKS_DIR, __version__))

            assert os.path.exists(executable_content_paks_dir)
            assert os.path.exists(external_paks_version_dir)

            if spear.path_exists(executable_content_paks_external_dir):
                log(f"File or directory or symlink exists, removing: {executable_content_paks_external_dir}")
                spear.remove_path(executable_content_paks_external_dir)

            log(f"Creating symlink: {executable_content_paks_external_dir} -> {external_paks_version_dir}")
            os.symlink(external_paks_version_dir, executable_content_paks_external_dir)

        else:
            if spear.path_exists(executable_content_paks_external_dir):
                log(f"File or directory or symlink exists, removing: {executable_content_paks_external_dir}")
                spear.remove_path(executable_content_paks_external_dir)

    # set environment variables
    if config.SPEAR.LAUNCH_MODE in ["editor", "standalone"]:
        for environment_var_name, environment_var_value in config.SPEAR.ENVIRONMENT_VARS.items():
            log("Setting environment variable ", environment_var_name, ": ", environment_var_value)
            os.environ[environment_var_name] = environment_var_value
