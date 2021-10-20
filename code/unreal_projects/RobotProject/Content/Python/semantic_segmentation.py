import unreal
import csv
import os

data = {}


def load_csv_data():
    path = os.path.dirname(os.path.abspath(__file__)) + "/semantic_label_color.csv"
    print(path)
    sf = open(path, "r")
    csv_reader = csv.reader(sf)
    for row in csv_reader:
        data[row[1]] = row[0]
    sf.close()


def apply_seg_id_all_components():

    components = unreal.EditorLevelLibrary.get_all_level_actors_components()

    for comp in components:
        try:
            unreal.StaticMeshComponent.cast(comp)
        except TypeError:
            continue

        name = comp.get_full_name()

        cat_id = ""
        for sub_name in name.split("_"):
            if "INSTid" in sub_name:
                for i in sub_name:
                    if i.isdigit():
                        cat_id += i

        comp.set_render_custom_depth(True)

        if cat_id:
            stencil_value = data[cat_id]
            # print("cat_id: {}, stencil_value : {}".format(cat_id, stencil_value))
            if stencil_value is not None:
                comp.set_custom_depth_stencil_value(int(stencil_value))
                continue

        comp.set_custom_depth_stencil_value(0)


load_csv_data()
apply_seg_id_all_components()
