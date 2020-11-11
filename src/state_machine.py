#!/usr/bin/env python
import	rospy
import 	sys
import 	math
from 	geometry_msgs.msg import Twist,Point
from 	nav_msgs.msg import Odometry
import	smach
import  smach_ros


while not rospy.is_shutdown():	
	args=rospy.myargv(argv=sys.argv)
	robotname= args[1]
		
	#Define Attraction State
	class Attract(smach.State):
		def __init__(self):
                    smach.State.__init__(self, outcomes=['finished','failed'])
		    self.subscriber = rospy.Subscriber("/{}/odom".format(robotname),Odometry,self.callback)

		def callback(self,msg):
                        
			self.robot_pose_x=msg.pose.pose.position.x
			self.robot_pose_y=msg.pose.pose.position.y
			rospy.loginfo("Odom {} {}".format(self.robot_pose_x,self.robot_pose_y))

		def execute(self, userdata):
		        rospy.loginfo('Changing to..')
                        return 'finished'

	class Repulse(smach.State):
		def __init__(self):
			smach.State.__init__(self, outcomes=['finished', 'failed'])

		def execute(self, userdata):
			rospy.loginfo('Change to Attract')
			return 'finished'



		


		
# def main():

 
		


	
	# Initialize node
	rospy.init_node('State_machine_node')
	# robot=rospy.Subscriber("/robot_1/odom", Odometry ,callback)
	# robot1=rospy.Subscriber("/{}/odom".format(robotname),Odometry, callback1)
	# Create the actual State Machine
	# outcome4 is the final
	sm =smach.StateMachine(outcomes=['I_have_finished'])
	#Passing arguments in States 
	# sm.user_data.Attraction_Force=......
	# sm.user_data.Repulsion_Force=.......






	# Adding states
	with sm:
		smach.StateMachine.add('Attract', Attract(),
                                transitions={'finished': 'Repulse', 'failed': 'Attract'})
		smach.StateMachine.add('Repulse', Repulse(),
                                transitions={'finished': 'Attract', 'failed': 'Repulse'})
		# smach.StateMachine.add('Attract',CBState(Attract_state_callback),{'finished':'Repulse', 'failed':'Attract'})
		# smach.StateMachine.add('Repulse',CBState(Repulse_state_callback),{'finished':'Attract', 'failed':'Repulse'})



	# Start State_Machine
	outcome =sm.execute()
rospy.spin()




# if __name__ == '__main__':
# 	main()





