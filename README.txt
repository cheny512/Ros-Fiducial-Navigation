To run this program you need to cd into /my_ros_data/catkin_ws/src/cosi119_src/fiducial_nav
then you have to open the campus rover link, log in and then hit terminal and type these commands after you have ssh into a turtlebot:
bringup (onboard the robot)
roslaunch fiducial_nav fiducials_real.launch (on your vnc)
rosrun rviz rviz (on your vnc. Tune the RViz to visualize what you want.)
teleop is also nice

Put the robot where you want it to start
go into terminal
rosrun fiducial_nav nav_real.py