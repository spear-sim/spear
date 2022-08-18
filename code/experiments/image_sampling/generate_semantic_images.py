import os 
import shutil
import numpy as np
import argparse
import csv
import cv2

if __name__ == "__main__":
    
    # Parse input script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_dir", type=str, help="enter path to stored dataset.", required=True)
    parser.add_argument("--semantic_mappings_file", type=str, required=True)
    args = parser.parse_args()
    
    # output semantic_mappings_interiorsim.txt
    semantic_labels_file = open(args.semantic_mappings_file, "w")
    sem_csv_writer = csv.writer(semantic_labels_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    sem_csv_writer.writerow(["id"])

    # get stencil id to rgb mapping
    possible_stencil_values = np.arange(start=0, stop=114)

    all_possible_colors = [153,108,6,112,105,191,89,121,72,190,225,64,206,190,59,81,13,36,115,176,195,161,171,27,135,169,180,29,26,199,102,16,239,242,107,146,156,198,23,49,89,160,68,218,116,11,236,9,196,30,8,121,67,28,0,53,65,146,52,70,226,149,143,151,126,171,194,39,7,205,120,161,212,51,60,211,80,208,189,135,188,54,72,205,103,252,157,124,21,123,19,132,69,195,237,132,94,253,175,182,251,87,90,162,242,199,29,1,254,12,229,35,196,244,220,163,49,86,254,214,152,3,129,92,31,106,207,229,90,125,75,48,98,55,74,126,129,238,222,153,109,85,152,34,173,69,31,37,128,125,58,19,33,134,57,119,218,124,115,120,0,200,225,131,92,246,90,16,51,155,241,202,97,155,184,145,182,96,232,44,133,244,133,180,191,29,1,222,192,99,242,104,91,168,219,65,54,217,148,66,130,203,102,204,216,78,75,234,20,250,109,206,24,164,194,17,157,23,236,158,114,88,245,22,110,67,17,35,181,213,93,170,179,42,52,187,148,247,200,111,25,62,174,100,25,240,191,195,144,252,36,67,241,77,149,237,33,141,119,230,85,28,34,108,78,98,254,114,161,30,75,50,243,66,226,253,46,104,76,8,234,216,15,241,102,93,14,71,192,255,193,253,41,164,24,175,120,185,243,231,169,233,97,243,215,145,72,137,21,160,113,101,214,92,13,167,140,147,101,109,181,53,118,126,3,177,32,40,63,99,186,139,153,88,207,100,71,146,227]
    stencil_id_color_map = {}
    for stencil_value in possible_stencil_values:
        if stencil_value > 0:
            stencil_id_color_map[stencil_value] = (all_possible_colors[3*(stencil_value-1)+0], all_possible_colors[3*(stencil_value-1)+1], all_possible_colors[3*(stencil_value-1)+2])
        else:
            stencil_id_color_map[stencil_value] = (0, 0, 0)
        
        sem_csv_writer.writerow([stencil_value])
    semantic_labels_file.close() # close file

    print(stencil_id_color_map)

    scenes = [fp for fp in os.listdir(args.input_dir) if os.path.isdir(os.path.join(args.input_dir, fp))]

    for scene in scenes[:]:
        print(f"processing scene {scene}")
        semantic_images = os.listdir(os.path.join(args.input_dir, scene, "seg"))
        
        if not os.path.exists(os.path.join(args.input_dir, scene, f"sem_seg")):
            os.makedirs(os.path.join(args.input_dir, scene, f"sem_seg"))

        for image in semantic_images:
            mat = cv2.imread(os.path.join(args.input_dir, scene, f"seg/{image}"))
            # cv2.imshow("before", mat)
            mat = mat[:,:,[2,1,0]] # bgr to rgb
            label_seg = np.zeros((mat.shape[:2]), dtype=np.uint8)
            for stencil_id, color in stencil_id_color_map.items():
                label_seg[(mat==color).all(axis=2)] = stencil_id

            # cv2.imshow("after", label_seg)
            # cv2.waitKey(0)
            idx = image.split('/')[-1]
            ret = cv2.imwrite(os.path.join(args.input_dir, scene, f"sem_seg/{idx}"), label_seg)
            assert ret == True
        
        cv2.destroyAllWindows()


