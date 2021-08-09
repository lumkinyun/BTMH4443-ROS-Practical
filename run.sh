cd src/webotsim/src
chmod 755 *.*
cd ../../..
catkin_make
source devel/setup.bash
clear
roslaunch webotsim webotsim.launch