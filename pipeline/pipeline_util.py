import json

pipeline_config = json.load(open("config.json", mode='r'))

default_values = {
    'convex_decomposition_strategy': "coacd",
    "body_type": "static"
}


def get_config_value(actor_name, key):
    actor_name_short = actor_name.split("/")[-1]
    actor_type = actor_name.split("/")[1]
    actor_type = actor_type[actor_type.find("_")+1:]
    if actor_name_short in pipeline_config['actors']:
        actor_config = pipeline_config['actors'][actor_name_short]
        if key in actor_config:
            return actor_config[key]
    if actor_type in pipeline_config['semantics']:
        actor_config = pipeline_config['semantics'][actor_type]
        if key in actor_config:
            return actor_config[key]
    return default_values[key]
