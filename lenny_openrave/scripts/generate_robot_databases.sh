#!/bin/sh
FILENAME=$(basename "$0")
# Usage
usage() {
cat <<EOF
usage: rosrun sda10f_openrave $FILENAME [-h] arm

Generate the OpenRAVE databases for the sda10f robot

positional arguments:
  arm   The name of the arm (left or right)

optional arguments:
  -h    Show this help message and exit
EOF
}
while getopts hsd opts; do
  case ${opts} in
    h)
      usage
      exit 1 ;;
  esac
done
ARM_NAME="$1"
# Check we have a valid arm name
if [ $ARM_NAME != "left" ] ; then
  if [ $ARM_NAME != "right" ] ; then
    usage
    exit 1
  fi
fi
# Generate the database (0.261 rad ~= 15 deg)
openrave.py --database inversekinematics --robot=robots/sda10f.robot.xml  \
--manipname=arm_${ARM_NAME}_tool0 --iktype=Transform6D --precision=8      \
--freeinc=0.261 --freejoint=arm_${ARM_NAME}_joint_1_s
