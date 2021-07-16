# spin_demo

source /opt/ros/noetic/setup.bash
git clone https://github.com/Miro001/spin_demo.git test_ws
cd test_ws/
catkin_make
source devel/setup.bash 
roslaunch positioner_control ros_control.launch

"Then press Next in the GUI to plan  and execute motion"
