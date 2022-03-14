import getopt
import glob
import json
import os
import sys
import getopt

def print_help():
    print(
        """
parse all available meta.json and output statistics result: sum and average based on style        
scene_manager_meta_parser.py -k <keys>
e.g: python scene_manager_meta_parser.py --keys "room, assets,Living room"
 -k: required. comma separated key list. Available key: 
 room, area, asset, asset_unique, Living room, Bedroom, Bathroom, Kitchen, Aisle, Balcony, Study, Customize, Dinning room	
"""
    )
    sys.exit(2)


def load_all_meta():
    """
    load all metadata from metadata/*.json
    :return: style_map {"style1":[],"style2":[]}
    """
    style_map = {}
    for file_path in glob.glob("metadata/metadata_{}.json".format("*")):
        with open(file_path, mode="r") as f:
            metadata = json.load(f)
            if metadata["style"] not in style_map:
                style_map[metadata["style"]] = []
            style_map[metadata["style"]].append(metadata)
    return style_map


def parse_key(style_map, key):
    """

    :param style_map: data from load_all_meta()
    :param key: parsed key
    :return:
    """
    style_result = {}
    for style in style_map:
        if not style in style_result:
            style_result[style] = {}
            style_result[style]["count"] = 0
            style_result[style]["val"] = 0
        for vw in style_map[style]:
            style_result[style]["count"] += 1
            if key in vw:
                style_result[style]["val"] += vw[key]

    print(f"----------{key}--------")
    for style in style_result:
        style_result[style]["avg"] = float(style_result[style]["val"]) / float(
            style_result[style]["count"]
        )
        print(
            "{}\t{}\t{}\t{}\t{}\t".format(
                style[0:7],
                key[0:6],
                style_result[style]["count"],
                style_result[style]["val"],
                style_result[style]["avg"],
            )
        )
    print(f"----------{key}--------")
    return style_result


if __name__ == "__main__":
    keys = []
    virtualworld_id = ""

    try:
        opts, args = getopt.getopt(
            sys.argv[1:],
            "h:k:",
            ["help=", "keys="],
        )
    except getopt.GetoptError:
        print_help()

    for opt, arg in opts:
        if opt == ("-h", "--help"):
            print_help()
        elif opt in ("-k", "--keys"):
            keys = arg.split(",")
    print("keys: ", keys)
    metadata = load_all_meta()
    for key in keys:
        parse_key(metadata, key.strip())
