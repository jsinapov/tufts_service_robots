#!/usr/bin/env python3
import os
import re


def main():
    strip_space_re = re.compile(".* ")  # remove everything up to and including the last space character

    map_points_dir = '/Users/eharvey/Projects/ros_sandbox/map_points'

    for file_name in os.listdir(map_points_dir):
        if not file_name.endswith(".txt"):
            continue
        file_path = os.path.join(map_points_dir, file_name)

        file_part, extension = os.path.splitext(file_name)

        print("")
        print("{}:".format(file_part))

        get_x = False
        get_y = False
        with open(file_path, 'r') as f:
            for line in f.readlines():
                line = line.rstrip()
                if line == "    position:":
                    get_x = True
                    continue
                if get_x:
                    x = strip_space_re.sub("", line)
                    get_x = False
                    get_y = True
                    continue
                if get_y:
                    y = strip_space_re.sub("", line)
                    print("  x: {}".format(x))
                    print("  y: {}".format(y))
                    break


if __name__ == "__main__":
    main()
