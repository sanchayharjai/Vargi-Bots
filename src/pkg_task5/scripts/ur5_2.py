#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

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

import ur5_1
from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages

from datetime import date
import yaml
import requests
import os
import math
import time
import calendar
import sys
import copy
import json
import tf2_ros
import tf2_msgs.msg
import cv2

import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

from std_srvs.srv import Empty
import thread

class Camera1:
  

  def __init__(self):
	  	self.arr=["aa","aa","aa","aa","aa","aa","aa","aa","aa","aa","aa","aa"]
		self.red=0
		self.green=0
		self.yellow=0
		self.count=0
                self.bridge = CvBridge()


                self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)


	        

  def arraret(self):
	  return self.arr
  
  def unsharp_mask(self,image, kernel_size=(5, 5), sigma=4.0, amount=2.0, threshold=0):
    """Return a sharpened version of the image, using an unsharp mask."""
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)
    sharpened = float(amount + 1) * image - float(amount) * blurred
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)
    return sharpened

  def get_qr_data(self, arg_image):
	
        qr_result = decode(arg_image)
	redc=0
	greenc=0
	yellowc=0
    
	for i in range(len(qr_result)):
                if(qr_result[i].data=="green"):
				    greenc+=1

		elif(qr_result[i].data=="red"):
					redc+=1

		elif(qr_result[i].data=="yellow"):
				yellowc+=1

	return qr_result
			



	
			    
	    
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    image = self.unsharp_mask(cv_image)



    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(image, (720/2, 1280/2)) 

    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
    self.arr=self.get_qr_data(image)
    cv2.waitKey(3)


class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name):

		rospy.init_node('node_moveit_eg6', anonymous=True)
		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		
		self.qr=[]
		self.flag=0
		self.packagearr=[""]
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
		self._handle_ros_pub1 = rospy.Publisher("order", msgMqttSub, queue_size=10)
		self._handle_ros_pub = rospy.Publisher("dashboard", msgMqttSub, queue_size=10)
		


		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        def package(self,arr,package): 
			'''Compares the package positions stored and fetched from logical camera and returns package colour.''' 
			
			if(package=="packagen00"):
                
				for i in xrange(len(arr)):

					if(arr[i].rect.left==127 and arr[i].rect.top==315):

						return arr[i].data
	        
			if(package=="packagen01"):
	            
				for i in xrange(len(arr)):
		            
					if(arr[i].rect.left==316 and arr[i].rect.top==316):
			        
					    return arr[i].data
	        
			if(package=="packagen02"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==502 and arr[i].rect.top==315):
			
			            return arr[i].data
	        
			if(package=="packagen10"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==128 and arr[i].rect.top==643):
			
			            return arr[i].data
	        
			if(package=="packagen11"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==316 and arr[i].rect.top==497):
			
			            return arr[i].data
	        
			if(package=="packagen12"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==502 and arr[i].rect.top==496):
			
			            return arr[i].data
	        
			if(package=="packagen20"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==128 and arr[i].rect.top==643):
			
			            return arr[i].data
	        
			if(package=="packagen21"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==315 and arr[i].rect.top==643):
			
			            return arr[i].data
	        
			if(package=="packagen22"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==503 and arr[i].rect.top==644):
			    
				        return arr[i].data
	        
			if(package=="packagen30"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==131 and arr[i].rect.top==797):
			
			            return arr[i].data
	        
			if(package=="packagen31"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==317 and arr[i].rect.top==796):
			
			            return arr[i].data
	        
			if(package=="packagen32"):
	        
			    for i in xrange(len(arr)):
		    
			        if(arr[i].rect.left==501 and arr[i].rect.top==796):
			
			            return arr[i].data
	

	def func_callback_topic_my_topic(self, message):
		'''Gives call for functions to make ur5_2 execute trajectories as per the colour of boxes.'''
		for i in xrange(len(message.models)):
		        if(message.models[i].type=="packagen00" or message.models[i].type=="packagen01" or message.models[i].type=="packagen02" or message.models[i].type=="packagen10" or message.models[i].type=="packagen11" or message.models[i].type=="packagen12" or message.models[i].type=="packagen20" or  message.models[i].type=="packagen21" or message.models[i].type=="packagen22" or message.models[i].type=="packagen30" or message.models[i].type=="packagen31" or message.models[i].type=="packagen32" ):
                            if(message.models[i].pose.position.y < 0.347462417442):
								
								flagp=False
								for j in xrange(len(self.packagearr)):
								    if(self.packagearr[j]==message.models[i].type):
								#print(self.qr)
								        flagp=True
								self.packagearr.append(message.models[i].type)
								if(flagp==False):
									msg_mqtt_sub = msgMqttSub()
                                        
									msg_mqtt_sub.timestamp = rospy.Time.now()
									msg_mqtt_sub.topic = "message.topic"
									msg_mqtt_sub.message = message.models[i].type
									self._handle_ros_pub1.publish(msg_mqtt_sub)
                                
								    #print(self.package(self.qr,message.models[i].type))
									packagecol=self.package(self.qr,message.models[i].type)
									callthis()
									if(packagecol=="red"):
										#rospy.sleep(2)
										self.red()
									if(packagecol=="green"):
										#rospy.sleep(2)
										self.green()
									if(packagecol=="yellow"):
										#rospy.sleep(2)
										self.yellow()
									
								

							    
        def red(self):

			    '''Makes ur5_2 pick the red box from conveyor belt and place in the red bin.'''
			    
			    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
                            attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                            activate_vacuum_gripper = attach(True)
                            thread.start_new_thread( callthis1,()  )							
                            			
    		            # attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                        #     set_power = attach(100)
			    lst_joint_angles_0 = [math.radians(-83.4007470448),
						  math.radians(-119.110256159),
						  math.radians(-70.8112705851),
						  math.radians(-68.8448776714),
						  math.radians(89.9460564459),
						  math.radians(-173.385773933)]
		            self.hard_set_joint_angles(lst_joint_angles_0,10)
					

	                    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
                            attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                            activate_vacuum_gripper = attach(False)
						
                            msg_mqtt_sub = msgMqttSub()
                            msg_mqtt_sub.timestamp = rospy.Time.now()
                            msg_mqtt_sub.topic = "message.topic"
                            msg_mqtt_sub.message = ""
                            self._handle_ros_pub.publish(msg_mqtt_sub)	
    		            # attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                        #     set_power = attach(100)

                            lst_joint_angles_1 = [math.radians(-14.6938575365),
						  math.radians(-135.475502602),
						  math.radians(-65.8797693022),
						  math.radians(-68.3575568511),
						  math.radians(89.0564003638),
						  math.radians(-104.671176544)]
                            self.hard_set_joint_angles(lst_joint_angles_1,10)

        def yellow(self):
	                
				'''Makes ur5_2 pick the yellow box from conveyor belt and place in the yellow bin.'''	
				rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
                                attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                                activate_vacuum_gripper = attach(True)
                                thread.start_new_thread( callthis1,()  )															
			        # attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                    #             set_power = attach(100)
			        lst_joint_angles_0 = [math.radians(-164.799369276),
						  math.radians(-118.510135031),
						  math.radians(-82.3782326951),
						  math.radians(-68.92040206),
						  math.radians(91.0091406899),
						  math.radians(105.205408192)]
		                self.hard_set_joint_angles(lst_joint_angles_0,10)

			        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
                                attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                                activate_vacuum_gripper = attach(False)
                                msg_mqtt_sub = msgMqttSub()
                                msg_mqtt_sub.timestamp = rospy.Time.now()
                                msg_mqtt_sub.topic = "message.topic"
                                msg_mqtt_sub.message = ""
                                self._handle_ros_pub.publish(msg_mqtt_sub)	
			        # attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                    #             set_power = attach(100)

                                lst_joint_angles_1 = [math.radians(-14.6938575365),
						  math.radians(-135.475502602),
						  math.radians(-65.8797693022),
						  math.radians(-68.3575568511),
						  math.radians(89.0564003638),
						  math.radians(-104.671176544)]
                                self.hard_set_joint_angles(lst_joint_angles_1,10)

        def green(self):

      
			    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
                            attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                            activate_vacuum_gripper = attach(True)
                            thread.start_new_thread( callthis1,()  )														
     			    # attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                    #         set_power = attach(100)                           
			    lst_joint_angles_0 = [math.radians(93.9285696457),
			 			  math.radians(-108.784230152),
						  math.radians(-99.2034780036),
						  math.radians(-63.0163831599),
						  math.radians(90.0077846121),
						  math.radians(3.95309461215)]
		            self.hard_set_joint_angles(lst_joint_angles_0,10)
                       
			    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
                            attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                            activate_vacuum_gripper = attach(False)	
                            msg_mqtt_sub = msgMqttSub()
                            msg_mqtt_sub.timestamp = rospy.Time.now()
                            msg_mqtt_sub.topic = "message.topic"
                            msg_mqtt_sub.message = ""
                            self._handle_ros_pub.publish(msg_mqtt_sub)
    			    # attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                    #         set_power = attach(100)

                            lst_joint_angles_1 = [math.radians(-14.6938575365),
						  math.radians(-135.475502602),
						  math.radians(-65.8797693022),
						  math.radians(-68.3575568511),
						  math.radians(89.0564003638),
						  math.radians(-104.671176544)]
                            self.hard_set_joint_angles(lst_joint_angles_1,10)
	

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


def activating():
			    rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
                            attach = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                            activate_vacuum_gripper = attach(True)	

def deactivating():
   		            attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
                            set_power = attach(100)	

def callthis():
        attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        set_power = attach(0)   

def callthis1():
	rospy.sleep(1)
        attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        set_power = attach(100)  



def inventory(arr):
	todays_date = date.today() 
	r=-1
	c=-1
	if(arr.rect.left>=125 and arr.rect.left<=132):
		c=0
	elif(arr.rect.left>=313 and arr.rect.left<=320):
		c=1
	elif(arr.rect.left<=505 and arr.rect.left>=500):
		c=2

	if(arr.rect.top>=313 and arr.rect.top<=318):
		r=0
	elif(arr.rect.top>=494 and arr.rect.top<=499):
		r=1
	elif(arr.rect.top>=640 and arr.rect.top<=645):	
		r=2
	elif(arr.rect.top>=794 and arr.rect.top<=799):
		r=3	
	return arr.data[0]+str(r)+str(c)
	
def order_callback(message):
	print("fffffffffffffffffffffff")
	payload = str(message.message.decode("utf-8"))
def sheetpush(arr):
	todays_date = date.today()
	
	for i in xrange(len(arr)):
		rospy.sleep(1)
		z=inventory(arr[i])
		sku=z[0].upper()+str(z[1])+str(z[2])+'0'+str(todays_date.month)+str(todays_date.year)[2:]
		stn="R"+z[1]+" "+"C"+z[2]
		item=""
		cost=0
		priority=""
		if(z[0]=="r"):
			item="Medicine"
			priority="HP"
			cost=450
		elif(z[0]=="y"):
			item="Food"
			priority="MP"
			cost=250
		elif(z[0]=="g"):
			item="Clothes"
			priority="LP"
			cost=150		

	        parameters = {"sku":sku, "item":item, "priority":priority, "storagenumber":stn, "cost":cost, "quantity":"1"}  

		result = json.dumps(parameters) 
		publish(result)

def publish(param):
	_handle_ros_pub = rospy.Publisher("messagetopic", msgMqttSub, queue_size=10)
	msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = "messagetopic"
        msg_mqtt_sub.message = param

        _handle_ros_pub.publish(msg_mqtt_sub)
	print("no")
	
def main():


	rospy.sleep(15)
        ur5 = Ur5Moveit('ur5_2')
	ic = Camera1()
	rospy.sleep(2)

	ur5.qr=ic.arr
	print(ur5.qr)
	publish("")
	sheetpush(ic.arr)
	handle_sub = rospy.Subscriber("/eyrc/vb/hmritmteam/orders", msgMqttSub, order_callback)      
	sub=rospy.Subscriber("/eyrc/vb/logical_camera_2", LogicalCameraImage, ur5.func_callback_topic_my_topic)
	lst_joint_angles_3 = [math.radians(-14.6938575365),
						  math.radians(-135.475502602),
						  math.radians(-65.8797693022),
						  math.radians(-68.3575568511),
						  math.radians(89.0564003638),
						  math.radians(-104.671176544)]
   
	attach = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        set_power = attach(100) 
     
        ur5.hard_set_joint_angles(lst_joint_angles_3, 30)
	rospy.spin()
	rospy.sleep(2)
        cv2.destroyAllWindows()
	del ur5
    

if __name__ == '__main__':
	main()

