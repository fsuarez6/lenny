#!/bin/sh
# Working files and directory
XACRO=`rospack find motoman_sda10f_support`/urdf/sda10f.xacro
WORK_DIR="/tmp/sda10f_openrave"
URDF="sda10f.urdf"
COLLADA=`rospack find sda10f_openrave`/data/robots/sda10f.dae
mkdir -p $WORK_DIR
cd $WORK_DIR
# Generate the robot URDF
rosrun xacro xacro --inorder $XACRO > $URDF
# Generate the collada
rosrun collada_urdf urdf_to_collada $URDF sda10f.dae > /dev/null
# Round collada to 5 significant figures
rosrun moveit_kinematics round_collada_numbers.py sda10f.dae $COLLADA 5 > /dev/null
# Done
cd - > /dev/null
echo "Collada file successfully generated:"
echo "$COLLADA"
