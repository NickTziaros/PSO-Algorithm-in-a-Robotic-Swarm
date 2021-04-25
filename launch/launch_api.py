#!/usr/bin/env python3
import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nikos/catkin_ws/src/Swarm/launch/gazebo.launch"])
launch.start()
rospy.loginfo("started")
rospy.sleep(5)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nikos/catkin_ws/src/Swarm/launch/PSO_SM_Node.launch"])
launch.start()

try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.shutdown()