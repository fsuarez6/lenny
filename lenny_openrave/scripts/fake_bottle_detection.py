#! /usr/bin/env python
import rospy
import argparse
import numpy as np
import criutils as cu
import openravepy as orpy
# Messages
from lenny_msgs.msg import BottleDetection


if "__main__" == __name__:
    np.set_printoptions(precision=6, suppress=True)
    # Parse the args
    parser = argparse.ArgumentParser(
                formatter_class=argparse.RawDescriptionHelpFormatter,
                description="Fake bottle detection node",
                fromfile_prefix_chars="@")
    parser.add_argument("--debug", action="store_true",
              help="If set, will show additional debugging information")
    parser.add_argument("--ipython", action="store_true",
              help="If set, will embed an IPython console. Good for debugging")
    options = parser.parse_args(rospy.myargv()[1:])
