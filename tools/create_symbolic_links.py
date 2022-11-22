import json
import os


TOOLS_DIR           = os.path.dirname(os.path.realpath(__file__))
UNREAL_PROJECTS_DIR = os.path.realpath(os.path.join(TOOLS_DIR, "..", "cpp", "unreal_projects"))
UNREAL_PLUGINS_DIR  = os.path.realpath(os.path.join(TOOLS_DIR, "..", "cpp", "unreal_plugins"))
THIRD_PARTY_DIR     = os.path.realpath(os.path.join(TOOLS_DIR, "..", "third_party"))


def create_symlink(src, dst):
    try:
        os.symlink(src, dst)
    except OSError as e:
        print(e)
        print("\n\n\nIf you are running this script on Windows, you need admin privileges.\n\n")
        assert False


if __name__ == "__main__":

    # get list of the projects and plugins we maintain
    our_projects = os.listdir(UNREAL_PROJECTS_DIR)
    our_plugins = os.listdir(UNREAL_PLUGINS_DIR)

    # for each plugin...
    for plugin in our_plugins:

        # ...if the plugin is a valid plugin (i.e., plugin dir has a uplugin file)
        uplugin = os.path.join(UNREAL_PLUGINS_DIR, plugin, plugin+'.uplugin')
        if os.path.exists(uplugin):

            print(f"Found plugin: {plugin}")

            # create a symlink to code/third_party
            symlink_third_party_dir = os.path.join(UNREAL_PLUGINS_DIR, plugin, "ThirdParty")
            if not os.path.exists(symlink_third_party_dir):
                print(f"    Creating symlink: {symlink_third_party_dir} -> {THIRD_PARTY_DIR}")
                create_symlink(THIRD_PARTY_DIR, symlink_third_party_dir)
            else:
                print(f"    {symlink_third_party_dir} already exists, so we do not create a symlink...")
    print()

    # for each project...
    for project in our_projects:

        # ...if the project is a valid project (i.e., project dir has a uproject file)
        uproject = os.path.join(UNREAL_PROJECTS_DIR, project, project+'.uproject')
        if os.path.exists(uproject):

            print(f"Found project: {project}")

            # create a symlink to code/third_party
            symlink_third_party_dir = os.path.join(UNREAL_PROJECTS_DIR, project, "ThirdParty")
            if not os.path.exists(symlink_third_party_dir):
                print(f"    Creating symlink: {symlink_third_party_dir} -> {THIRD_PARTY_DIR}")
                create_symlink(THIRD_PARTY_DIR, symlink_third_party_dir)
            else:
                print(f"    {symlink_third_party_dir} already exists, so we do not create a symlink...")

            # get list of plugins from the uproject file
            with open(os.path.join(UNREAL_PROJECTS_DIR, project, project+'.uproject')) as f:
                project_plugins = [ p["Name"] for p in json.load(f)["Plugins"] ]
            print(f"    Plugin dependencies: {project_plugins}")

            # create a Plugins dir in the project dir
            project_plugins_dir = os.path.join(UNREAL_PROJECTS_DIR, project, "Plugins")
            if not os.path.exists(project_plugins_dir):
                os.makedirs(project_plugins_dir)

            # create symlink for each plugin listed in the project, if the plugin is in our unreal_plugins dir
            for project_plugin in project_plugins:
                if project_plugin in our_plugins:
                    our_plugin_dir = os.path.abspath(os.path.join(UNREAL_PLUGINS_DIR, project_plugin))
                    symlink_plugin_dir = os.path.abspath(os.path.join(project_plugins_dir, project_plugin))
                    if not os.path.exists(symlink_plugin_dir):
                        print(f"        Creating symlink: {symlink_plugin_dir} -> {our_plugin_dir}")
                        create_symlink(our_plugin_dir, symlink_plugin_dir)
                    else:
                        print(f"        {symlink_plugin_dir} already exists, so we do not create a symlink...")
                else:
                    print(f"        {project} depends on {project_plugin}, but this plugin is not in {UNREAL_PLUGINS_DIR}, so we do not create a symlink...")
    print()

    print("Done.")
