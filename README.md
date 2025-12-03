# CyPR_Robot_Submarino
Diseño de método de control para un robot submarino cuyo modelo se importará desde otro proyecto.

# ARCHIVO MOTORES
TERMINAL 1:
cd ~/cpr/colcon_ws
colcon build 
source install/setup.bash
ros2 launch orca_bringup sim_launch.py

TERMINAL 2:
cd ~/cpr/colcon_ws
colcon build 
source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
/model/orca4/joint/thruster1_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double \
/model/orca4/joint/thruster2_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double \
/model/orca4/joint/thruster3_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double \
/model/orca4/joint/thruster4_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double \
/model/orca4/joint/thruster5_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double \
/model/orca4/joint/thruster6_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double

TERMINAL3:
cd ~/cpr/colcon_ws
colcon build 
source install/setup.bash
ros2 run controlador_submarino mapeo
