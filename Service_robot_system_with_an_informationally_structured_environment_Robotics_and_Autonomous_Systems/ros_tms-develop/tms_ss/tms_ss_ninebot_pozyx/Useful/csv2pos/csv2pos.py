import csv
import sys

args = sys.argv

if (len(args) != 2):
    print("Usage: # python %s filename.csv" % args[0])
    quit()

num_lines = len(open(args[1]).readlines())

with open('outPos_arduino.txt', 'w') as f_arduino:
    with open('outPos_yaml.txt', 'w') as f_yaml:
        with open(str(args[1]), 'r') as f_csv:
            reader = csv.reader(f_csv)
#            header = next(reader)

            f_arduino.write("anchor_t anchors[num_anchors] = {" + "\n")
            f_yaml.write("pozyx_anchors:" + "\n")
            n = 0
            for row in reader:
                n += 1

                print(row)

                f_arduino.write(" " * 33)
                f_arduino.write("{" + row[0] + "," + row[1] + "," + row[2] + "}")
                if n != num_lines:
                    f_arduino.write("," + "\n")
                else:
                    f_arduino.write("\n")

                f_yaml.write("# " + str(n) + "\n")
                f_yaml.write("  - id: " + row[0] + "\n")
                f_yaml.write("    coor_x: " + row[1] + "\n")
                f_yaml.write("    coor_y: " + row[2] + "\n")

            f_arduino.write(" " * 33)
            f_arduino.write("};")
