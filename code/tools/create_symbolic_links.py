import json
import os

SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
UNREAL_PROJECTS_PATH = os.path.join(SCRIPT_DIR_PATH, "..", "unreal_projects")
UNREAL_PLUGINS_PATH = os.path.join(SCRIPT_DIR_PATH, "..", "unreal_plugins")

def create_symbolic_links():

    for unreal_project in os.listdir(UNREAL_PROJECTS_PATH):
        project_plugins = []
        with open(os.path.join(UNREAL_PROJECTS_PATH, unreal_project, unreal_project+'.uproject')) as f:
            uproject_contents = json.load(f)
            uproject_plugins = uproject_contents["Plugins"]
            for item in uproject_plugins:
                project_plugins.append(item["Name"])

            print(f"{unreal_project} has dependency on {project_plugins} plugins. Creating symbolic links to the plugins that are part of our repo.")
        
        project_plugins_dir = os.path.join(UNREAL_PROJECTS_PATH, unreal_project, "Plugins")
        symlink_plugins = os.listdir(UNREAL_PLUGINS_PATH)

        for plugin_name in project_plugins:
            if plugin_name in symlink_plugins:
                if not os.path.exists(os.path.join(project_plugins_dir, plugin_name)):
                    os.symlink(os.path.join(UNREAL_PLUGINS_PATH, plugin_name), os.path.join(project_plugins_dir, plugin_name))
                    print(f"creating symbolic link to {plugin_name} for {unreal_project}.")
                else:
                    print(f"{unreal_project} already has {plugin_name} in it's plugins folder. So, skipping...")
            else:
                print(f"{unreal_project} does not have dependency on {plugin_name}. So, skipping...")

if __name__ == "__main__":    
    create_symbolic_links()    
