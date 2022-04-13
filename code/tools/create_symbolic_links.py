import json
import os

SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
UNREAL_PROJECTS_PATH = os.path.join(SCRIPT_DIR_PATH, "..", "unreal_projects")
UNREAL_PLUGINS_PATH = os.path.join(SCRIPT_DIR_PATH, "..", "unreal_plugins")

def create_symbolic_links():

    # get list of the projects and plugins we maintain
    our_plugins = os.listdir(UNREAL_PLUGINS_PATH)
    our_projects = os.listdir(UNREAL_PROJECTS_PATH)

    # for each project dir
    for project in our_projects:

        # if the project is a valid project (i.e., project dir has a uproject file)
        uproject = os.path.join(UNREAL_PROJECTS_PATH, project, project+'.uproject')
        if os.path.exists(uproject):

            # get list of plugins from the uproject file
            with open(os.path.join(UNREAL_PROJECTS_PATH, project, project+'.uproject')) as f:
                project_plugins = [ p["Name"] for p in json.load(f)["Plugins"] ]
            print(f"{project} plugin dependencies: {project_plugins}")

            # create a Plugins dir in the project dir
            project_plugins_dir = os.path.join(UNREAL_PROJECTS_PATH, project, "Plugins")
            if not os.path.exists(project_plugins_dir):
                os.makedirs(project_plugins_dir)

            # create symlink for each plugin listed in the project...
            for project_plugin in project_plugins:

                # ...if the plugin is maintained by us
                if project_plugin in our_plugins:                
                    our_plugin_path = os.path.abspath(os.path.join(UNREAL_PLUGINS_PATH, project_plugin))
                    symlink_plugin_path = os.path.abspath(os.path.join(project_plugins_dir, project_plugin))
                    if not os.path.exists(symlink_plugin_path):
                        print(f"    Creating symlink: {symlink_plugin_path} -> {our_plugin_path}")
                        os.symlink(our_plugin_path, symlink_plugin_path)
                    else:
                        print(f"    {symlink_plugin_path} already exists, so we do not create a symlink...")
                else:
                    print(f"    {project} depends on {project_plugin}, but this plugin is not in {UNREAL_PLUGINS_PATH}, so we do not create a symlink...")

    print(f"Done.")

if __name__ == "__main__":    
    create_symbolic_links()    
