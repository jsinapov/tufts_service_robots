#### amcl_pose_to_yaml.py

`amcl_pose_to_yaml.py` converts amcl_poses to yaml format.

Requires python3, but could be trivially converted to python2 if desired.

Directory containing .txt files is hard-coded in script, so script needs to be edited before use.

Assuming you get the robot pose using a command like this:

    rostopic echo /amcl_pose -n 1 > some_location.txt

Results print to stdout.