#! /usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
import thread
import threading


from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import *

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse
from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import ConveyorBeltControlRequest
from pkg_vb_sim.srv import ConveyorBeltControlResponse

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages

from collections import namedtuple
import requests
import json
import yaml
import os
import math
import time
import sys
import copy

import tf2_ros
import tf2_msgs.msg

from std_srvs.srv import Empty

class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name):

		rospy.init_node('node_moveit_eg6', anonymous=True)

		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
				
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''


		# Attribute to store computed trajectory by the planner	
		self._computed_plan = ''

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_moveit_examples')
		self._file_path = self._pkg_path + '/config/saved_trajectories/'
		rospy.loginfo( "Package Path: {}".format(self._file_path) )
		self.handle_sub = rospy.Subscriber("/eyrc/vb/hmritmteam/orders", msgMqttSub, self.order_callback)
		self.weed = rospy.Subscriber("messagetopic", msgMqttSub, self.sub_order_callback)
		self.invent=[]
		self.inventb=[]
		self.ord=[]
		self.tq=["n","n","n","n","n","n","n","n","n","n","n","n","n"]
		self.tqi=0
		self.invindex=0
		self.oi=0
		self.x=[]
		self.xi=0
		self.URL="https://script.google.com/macros/s/AKfycbwqWUnbFf2h9LW0zFYUplS5Xp-wNNzwX0DZ63hFRAzBMxVlqprAHMQbAw/exec"
		self._handle_ros_pub_disp = rospy.Publisher("orderdisp", msgMqttSub, queue_size=10)
		
		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def task_queue(self):
	    
		'''Compares sku of package order received and accordingly gives function calls for ur5_1 trajectory.'''	
		while(True):
			if(self.tq[self.tqi]!='n'):				
				if(self.tq[self.tqi]=="R320221"):
					self.pkg32()
				elif(self.tq[self.tqi]=="R120221"):
					self.pkg12()	
				elif(self.tq[self.tqi]=="R210221"):
					self.pkg21()
				elif(self.tq[self.tqi]=="R000221"):
					self.pkg00()		
                # elif(self.tq[self.tqi]=="Y300221"):
				# 	self.pkg30()
				elif(self.tq[self.tqi]=="Y220221"):
					self.pkg22()
				elif(self.tq[self.tqi]=="Y110221"):
					self.pkg11()
				elif(self.tq[self.tqi]=="Y010221"):
					self.pkg01()		
				elif(self.tq[self.tqi]=="G200221"):
					self.pkg20()
				elif(self.tq[self.tqi]=="G020221"):
					self.pkg02()
				# elif(self.tq[self.tqi]=="G310221"):
				# 	self.pkg31()				
				elif(self.tq[self.tqi]=="G102121"):
					self.pkg10()
				self.tqi+=1				

	def sub_order_callback(self,message):
		     
			payload = str(message.message.decode("utf-8"))
			x=json.loads(payload, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
			self.invent.append(x)
			self.inventb.append(True)		
			self.invindex+=1

	def func1(self,str):
		'''Manages inventory by checking order type.'''
		f=0

		for i in xrange(len(self.invent)):                
				if(str=="Medicine" and self.invent[i].sku[0]=="R" and self.inventb[i]==True and f==0):
					f=1
					self.tq[self.oi]=self.invent[i].sku
					self.oi+=1
					self.inventb[i]=False
                
               
			    
				if(str=="Food" and self.invent[i].sku[0]=="Y" and self.inventb[i]==True and f==0):
					f=1
					self.tq[self.oi]=self.invent[i].sku					                    
					self.oi+=1
					self.inventb[i]=False   

                
				if(str=="Clothes" and self.invent[i].sku[0]=="G" and self.inventb[i]==True and f==0):
					f=1
					self.tq[self.oi]=self.invent[i].sku                   
					self.oi+=1                    
					self.inventb[i]=False   


	def order_callback(self,message):
			payload = str(message.message.decode("utf-8"))
			x=json.loads(payload, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
			print(x)
			self.x.append(x)
			
			
			self.func1(x.item)
    
	def orderdispatch(self):
		msg_mqtt=msgMqttSub()
        #msg_mqtt_sub.timestamp = rospy.Time.now()
		msg_mqtt.message=""
		self._handle_ros_pub_disp.publish(msg_mqtt)
      
        # self._handle_ros_pub_disp.publish(msg_mqtt)
       


	def pkg00(self):
	    
		    
			'''To make ur5_1 pick package00 from the shelf and place onto the conveyor belt.'''	
			lst_joint_angles_8 = [math.radians(-55.1467596155),  
							math.radians(-69.7506888718),
							math.radians(8.45554063666),
							math.radians(-120.986916688),
							math.radians(-124.839030768),
							math.radians(-5.8126031965)]
			
			lst_joint_angles_9 = [math.radians(-27.977280775),  
							math.radians(-77.0973988033),
							math.radians(20.6653731956),
							math.radians(-158.937755318),
							math.radians(-145.382225088),
							math.radians(-35.0602048793)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]

			self.hard_set_joint_angles(lst_joint_angles_8, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_9, 30)	

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_grsipper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)
			self.orderdispatch()

	def pkg01(self):

		    '''To make ur5_1 pick package01 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_10 = [math.radians(-120.253415639),  
							math.radians(-64.2153594322),
							math.radians(-26.3126328697),
							math.radians(-88.5394501223),
							math.radians(-59.6806360195),
							math.radians(1.13223804717)]

			lst_joint_angles_11 = [math.radians(-164.067520509),  
							math.radians(-75.3114816842),
							math.radians(-14.2547717145),
							math.radians(-158.937755318),
							math.radians(-31.2253207976),
							math.radians(58.1600386589)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]

		
			self.hard_set_joint_angles(lst_joint_angles_10, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_11, 30)	

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)
			self.orderdispatch()

	def pkg02(self):

		    '''To make ur5_1 pick package02 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_5 = [math.radians(26.757761732),  
							math.radians(-124.228581571),
							math.radians(29.3403177677),
							math.radians(-51.9677995576),
							math.radians(145.329242584),
							math.radians(-151.763551383)]
			
			lst_joint_angles_4 = [math.radians(56.8871231095),  
							math.radians(-112.240860439),
							math.radians(0.836220892346),
							math.radians(-68.6032820749),
							math.radians(119.800578379),
							math.radians(-179.996250294)]

			lst_joint_angles_5 = [math.radians(26.757761732),  
							math.radians(-124.228581571),
							math.radians(29.3403177677),
							math.radians(-51.9677995576),
							math.radians(145.329242584),
							math.radians(-151.763551383)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]

			self.hard_set_joint_angles(lst_joint_angles_5, 30)

			self.hard_set_joint_angles(lst_joint_angles_4, 30)

			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_5, 30)

			self.hard_set_joint_angles(lst_joint_angles_3, 30)	
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)
			self.orderdispatch()
			
	def pkg10(self):

		    '''To make ur5_1 pick package10 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_20 = [math.radians(-53.943859581),  
							math.radians(-96.5926117887),
							math.radians(81.8356349382),
							math.radians(-165.957874664),
							math.radians(-126.110576182),
							math.radians(-0.462055779516)]

			lst_joint_angles_21 = [math.radians(-34.0093666439),  
							math.radians(-101.753313882),
							math.radians(86.316464409),
							math.radians(-179.698167823),
							math.radians(-145.110988501),
							math.radians(-12.5453090498)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]


			
			self.hard_set_joint_angles(lst_joint_angles_20, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_21, 30)	

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)
			self.orderdispatch()

	def pkg11(self):

		    '''To make ur5_1 pick package11 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_22 = [math.radians(-117.897027453),  
						  math.radians(-115.002813851),
						  math.radians(93.8546666505),
						  math.radians(-158.930859028),
						  math.radians(-62.1220704064),
						  math.radians(-0.0356195200287)]	

			lst_joint_angles_23 = [math.radians(-164.943555526),  
								math.radians(-129.423886664),
								math.radians(97.4093917081),
								math.radians(-175.049529224),
								math.radians(-16.822421373),
								math.radians(26.0014363467)]	    

			lst_joint_angles_24 = [math.radians(-50.7562889798),  
								math.radians(-129.421224677),
								math.radians(97.404945348),
								math.radians(-175.049155747),
								math.radians(-16.8219406873),
								math.radians(25.9990192031)] 

			lst_joint_angles_25 = [math.radians(-50.7529318333),  
								math.radians(-137.613953247),
								math.radians(85.8024721667),
								math.radians(-53.9565723222),
								math.radians(-44.8373977431),
								math.radians(-68.3675144914)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]
							
			self.hard_set_joint_angles(lst_joint_angles_22, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_23, 30)

			self.hard_set_joint_angles(lst_joint_angles_24, 30)
			self.hard_set_joint_angles(lst_joint_angles_25, 30)

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)	
			self.orderdispatch()

	def pkg12(self):	

		    '''To make ur5_1 pick package12 from the shelf and place onto the conveyor belt.'''		
			lst_joint_angles_6 = [math.radians(56.6955096333),  
							math.radians(-80.5134456163),
							math.radians(-83.9174444815),
							math.radians(-16.0141371973),
							math.radians(125.795228225),
							math.radians(-0.27462074122)]

			lst_joint_angles_7 = [math.radians(31.1522078374),  
							math.radians(-80.1129383464),
							math.radians(-82.7477034037),
							math.radians(17.291278397),
							math.radians(143.787494939),
							math.radians(8.9262169163)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]

			self.hard_set_joint_angles(lst_joint_angles_6, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_7, 30)	

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)
			self.orderdispatch()

	def pkg20(self):

		    '''To make ur5_1 pick package20 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_14 = [math.radians(-51.7146657414),  
							math.radians(14.3870400093),
							math.radians(-114.090138923),
							math.radians(-79.1075896008),
							math.radians(-128.30115429),
							math.radians(0.691595882752)]

			lst_joint_angles_15 = [math.radians(-32.9829609578),  
							math.radians(14.7568986488),
							math.radians(-116.158003112),
							math.radians(-94.6112849319),
							math.radians(-145.987697768),
							math.radians(-13.4300481919)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]
			

			
			self.hard_set_joint_angles(lst_joint_angles_14, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_15, 30)	

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)
			self.orderdispatch()

	def pkg21(self):

		    '''To make ur5_1 pick package21 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_12 = [math.radians(69.522461241),  
							math.radians(-30.2196596752),
							math.radians(-132.963254693),
							math.radians(-15.6747012151),
							math.radians(123.346910333),
							math.radians(  0.379479436957)]

			lst_joint_angles_13 = [math.radians(115.122028064),  
							math.radians(-59.632657237),
							math.radians( -126.468402429),
							math.radians(  3.03846806182),
							math.radians( 64.91334404),
							math.radians( 1.28335625367)]

			lst_joint_angles_12 = [math.radians(69.522461241),  
							math.radians(-30.2196596752),
							math.radians(-132.963254693),
							math.radians(-15.6747012151),
							math.radians(123.346910333),
							math.radians(  0.379479436957)]

			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]
			
			
			self.hard_set_joint_angles(lst_joint_angles_12, 30)		

			self.hard_set_joint_angles(lst_joint_angles_13, 30)	
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)	

			self.hard_set_joint_angles(lst_joint_angles_12, 30)

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)	
			self.orderdispatch()

	def pkg22(self):

		    '''To make ur5_1 pick package22 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_0 = [math.radians(57.5142270928),  
							math.radians(-80.082725558),
							math.radians(-117.579028271),
							math.radians(17.0526474229),
							math.radians(122.452824089),
							math.radians(-0.335674068816)]

			lst_joint_angles_1 = [math.radians(42.7403006091),
							math.radians(-70.9251417738),
							math.radians(-121.830654927),
							math.radians(11.9998355154),
							math.radians(137.225189265),
							math.radians(-0.567866312443)]
			
			lst_joint_angles_2 = [math.radians( 42.7324594351),
							math.radians( -82.9080276835), 
							math.radians(-98.9916829612),
							math.radians(91.3378281426),
							math.radians( 90.6300408392),
							math.radians( 47.2327450529)]

			
			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]


			self.hard_set_joint_angles(lst_joint_angles_0, 5)

			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)	
			

			self.hard_set_joint_angles(lst_joint_angles_1, 30)

			self.hard_set_joint_angles(lst_joint_angles_2, 30)

			self.hard_set_joint_angles(lst_joint_angles_3, 30)

			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)
			self.orderdispatch()

	# def pkg30(self):

	      #  '''To make ur5_1 pick package30 from the shelf and place onto the conveyor belt.'''
		# 	self.hard_set_joint_angles(lst_joint_angles_8, 30)
		#         rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
		#     attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		#     activate_vacuum_gripper = attach(True)

		#     self.hard_set_joint_angles(lst_joint_angles_9, 30)	

		#     self.hard_set_joint_angles(lst_joint_angles_3, 30)
		#     rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
		#     attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		#     activate_vacuum_gripper = attach(False)
		# self.orderdispatch()
			
	# def pkg31(self):

	       # '''To make ur5_1 pick package31 from the shelf and place onto the conveyor belt.'''
		# 	self.hard_set_joint_angles(lst_joint_angles_8, 30)
		#         rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
		#     attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		#     activate_vacuum_gripper = attach(True)

		#     self.hard_set_joint_angles(lst_joint_angles_9, 30)	

		#     self.hard_set_joint_angles(lst_joint_angles_3, 30)
		#     rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
		#     attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
		#     activate_vacuum_gripper = attach(False)	
		# self.orderdispatch()

	def pkg32(self):


		    '''To make ur5_1 pick package32 from the shelf and place onto the conveyor belt.'''
			lst_joint_angles_17 = [math.radians(44.7402657226),  
							math.radians(-100.561707761),
							math.radians(-138.004488343),
							math.radians( 58.6234376065),
							math.radians( 135.232096136),
							math.radians(-0.018925817811)]
			
			lst_joint_angles_16 = [math.radians(55.1126704714),  
							math.radians(-105.251134278),
							math.radians(-131.398517407),
							math.radians(56.0355067226),
							math.radians(124.866419063),
							math.radians(-0.410720929415)]


			lst_joint_angles_18 = [math.radians(32.874404551),  
							math.radians(-88.6493929772),
							math.radians(-143.849147511),
							math.radians(33.4468064078),
							math.radians(145.626738105),
							math.radians(-15.9833964881)]

			lst_joint_angles_19 = [math.radians(32.8700970014),  
							math.radians(-64.4957715319),
							math.radians(-142.973856981),
							math.radians(-62.2368335076),
							math.radians(90.3312742488),
							math.radians(-57.1312457306)]
			
			lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]
			
			
			self.hard_set_joint_angles(lst_joint_angles_17, 30)

			self.hard_set_joint_angles(lst_joint_angles_16, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(True)

			self.hard_set_joint_angles(lst_joint_angles_17, 30)
			self.hard_set_joint_angles(lst_joint_angles_18, 30)
			self.hard_set_joint_angles(lst_joint_angles_19, 30)

			self.hard_set_joint_angles(lst_joint_angles_3, 30)
			rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
			attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			activate_vacuum_gripper = attach(False)	
			self.orderdispatch()


	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()
	

	def set_joint_angles(self, arg_list_joint_angles):

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._computed_plan = self._group.plan()
		flag_plan = self._group.go(wait=True)

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		if (flag_plan == True):
			pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		else:
			pass
			# rospy.logerr(
			#     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()

	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def callthis():
	'''To set conveyor belt's moving speed to 100.'''
    attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    set_power = attach(100)   

def callthis1():
	'''To set conveyor belt's moving speed to 0.'''
    attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    set_power = attach(0)  

def main():   
	ur5 = Ur5Moveit('ur5_1')

        lst_joint_angles_3 = [math.radians(0.710975551187),
							math.radians(-121.925396199),
							math.radians(-79.1837345918),
							math.radians(-68.8448776714),
							math.radians(89.0125916857),
							math.radians(-89.2687055596)]

	thread.start_new_thread( ur5.task_queue,()  )	
	rospy.sleep(15)
	ur5.hard_set_joint_angles(lst_joint_angles_3, 30)

	attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        set_power = attach(100)
 
	rospy.spin()

	del ur5



if __name__ == '__main__':
	main()