import json
import os

SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
UNREAL_PROJECTS_PATH = os.path.join(SCRIPT_DIR_PATH, "..", "unreal_projects")
UNREAL_PLUGINS_PATH = os.path.join(SCRIPT_DIR_PATH, "..", "unreal_plugins")

def create_symbolic_links():
    unreal_plugins = [os.path.join(UNREAL_PLUGINS_PATH, x) for x in os.listdir(UNREAL_PLUGINS_PATH)]

    for unreal_project in os.listdir(UNREAL_PROJECTS_PATH):
        required_plugins = []
        with open(os.path.join(UNREAL_PROJECTS_PATH, unreal_project, unreal_project+'.uproject')) as f:
            uproject_contents = json.load(f)
            uproject_plugins = uproject_contents["Plugins"]
            for item in uproject_plugins:
                required_plugins.append(item["Name"])

            print(f"{unreal_project} has dependency on {required_plugins} plugins. Creating symbolic links to the plugins that are part of our repo.")
        
        assert len(required_plugins) != 0

        project_plugins_dir = os.path.join(UNREAL_PROJECTS_PATH, unreal_project, "Plugins")
        for plugin in unreal_plugins:
            plugin_name = plugin.split('/')[-1]
            if plugin_name in required_plugins:
                if not os.path.exists(os.path.join(project_plugins_dir, plugin_name)):
                    os.symlink(plugin, os.path.join(project_plugins_dir, plugin_name))
                    print(f"creating symbolic link to {plugin_name} for {unreal_project}.")
                else:
                    print(f"{unreal_project} already has {plugin_name} in it's plugins folder. So, skipping...")
            else:
                print(f"{unreal_project} does not have dependency on {plugin_name}, so skipping...")

if __name__ == "__main__":    
    create_symbolic_links()    
