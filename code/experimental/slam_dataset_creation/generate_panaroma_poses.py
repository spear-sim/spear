import argparse
import csv

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_file", type=str, required=True)
    parser.add_argument("--output_file", type=str, required=True)

    args = parser.parse_args()

    output_file = open(args.output_file, "w", newline='')
    csv_writer = csv.writer(output_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    

    with open(args.input_file, 'r') as f:
        csv_reader = csv.reader(f, delimiter=',')

        csv_writer.writerow(["pos_x_cms","pos_y_cms","pos_z_cms","rot_r_deg","rot_p_deg","rot_y_deg"])

        hfov =  60
        overlap = 0.25
        increments = hfov * (1-overlap)

        for row in list(csv_reader)[1:]:
            for new_yaw in range(int(row[5]), 360+int(row[5]), int(increments)):
                csv_writer.writerow([row[0], row[1], row[2], row[3], row[4], new_yaw])

    output_file.close()
