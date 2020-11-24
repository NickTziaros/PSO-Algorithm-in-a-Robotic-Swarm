#!/usr/bin/env python
import	rospy
import 	sys
import 	math
from 	geometry_msgs.msg import Twist,Point
from 	nav_msgs.msg import Odometry
import	smach
import  smach_ros
import	time
from 	robot_Class import robot


	
args=rospy.myargv(argv=sys.argv)
robotname= args[1]

# States of state machine
#----------------------------------------------------------------------------------------------------------

	#Define Attraction State
class Attract(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished','failed'])
		# na kanw to r meros ths classhs
		global r
		global l
		r=robot(robotname)
		



	def execute(self, userdata):
		foo=r.go2goal(10,10)
		
		if(foo==True):
			return 'finished'

class Repulse(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished', 'failed'])

 

	def execute(self, userdata):
		r.stop()
		return 'finished'
		







# Initializations of SM && publishers
# -------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':		


# Initialize node
	rospy.init_node('State_machine_node')
	sm =smach.StateMachine(outcomes=['I_have_finished'])
	pub = rospy.Publisher("/{}/cmd_vel".format(robotname),Twist, queue_size=10)


	
# Create and start the introspection server for visualising state machine

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()




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
	sis.stop()
	rospy.spin()









