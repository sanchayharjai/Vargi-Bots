#!/usr/bin/env python
# ROS Node - Action Server - IoT ROS Bridge
import rospy
import actionlib
import threading
import requests

from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback    # Message Class that is used for Feedback Messages    

from pkg_ros_iot_bridge.msg import msgMqttSub           # Message Class for MQTT Subscription Messages
from pkg_vb_sim.msg import LogicalCameraImage
from pyiot import iot                                 # Custom Python Module to perfrom MQTT Tasks
import json
from collections import namedtuple
import datetime
from datetime import date , timedelta
import time

class RosIotBridgeActionServer:

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_iot_ros',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_pyiot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_pyiot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_pyiot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_pyiot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_pyiot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_pyiot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_pyiot['mqtt']['sub_cb_ros_topic']
        print(param_config_pyiot)
        self._server_url = "broker.mqttdashboard.com"
        self._server_port = 1883
        self._qos = 0
        self._topic = "/eyrc/vb/hmritmteam/orders"
        # self._interval_list = [20, 25, 30, 60, 65, 70, 100, 105, 110]
        self.odi=0
        self.ordersdispatch=[]
        self.x=[]
        self.xi=0
        self.si=0
    #----------------------------
        self.ordershipped=[]
        self.di=0
        self.ddis=[]
        self.ki=0
        self.dship=[]
        self.hi=0
        self.dord=[]
        self.gi=0
        self.disp=0
        
        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.

        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)


        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_pyiot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._server_url, 
                                                        self._server_port, 
                                                        self._topic, 
                                                        self._qos   )
        if(ret == 0):
            rospy.loginfo("MQTT Subscribe Thread Started(action server)")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread(action server)")


        # Start the Action Server
        self._as.start()
        
        rospy.loginfo("Started ROS-IoT Bridge Action Server.")
    def func_callback_topic_my_topic(self, message):
        for i in xrange(len(message.models)):
		        if(message.models[i].type=="packagen00" or message.models[i].type=="packagen01" or message.models[i].type=="packagen02" or message.models[i].type=="packagen10" or message.models[i].type=="packagen11" or message.models[i].type=="packagen12" or message.models[i].type=="packagen20" or  message.models[i].type=="packagen21" or message.models[i].type=="packagen22" or message.models[i].type=="packagen30" or message.models[i].type=="packagen31" or message.models[i].type=="packagen32" ):
                                if message.models[i].type not in self.ordersdispatch:
                                    self.ordersdispatch.append(message.models[i].type)

                                    self.ddis.append(datetime.datetime.now())
                                    self.xi+=1

    def func_callback_topic_my_topic1(self, message):
        '''shipped'''
        payload = str(message.message.decode("utf-8"))
        self.dship.append(datetime.datetime.now())
        print(payload)
        if payload not in self.ordershipped:
            self.ordershipped.append(payload)
            print(self.x[self.si])
            if(self.itemv(self.x[self.si].item)[0:2]=="HP"):
                t=1
            if(self.itemv(self.x[self.si].item)[0:2]=="MP"):
                t=3
            if(self.itemv(self.x[self.si].item)[0:2]=="LP"):
                t=5
            
            parameters = {"id":"order shipped", "team id":"VB#2213", "unique id":"hmritmteam", "order id":self.x[self.si].order_id, "city":self.x[self.si].city, "item":self.x[self.si].item, "priority":self.itemv(self.x[self.si].item)[0:2], "shipped quantity":1, "cost":self.itemv(self.x[self.si].item)[2:], "shipped status":"YES", "shipped date and time":datetime.datetime.now(), "estimated time of delivery":datetime.datetime.now()+timedelta(t)}  
            URL ="https://script.google.com/macros/s/AKfycbwqWUnbFf2h9LW0zFYUplS5Xp-wNNzwX0DZ63hFRAzBMxVlqprAHMQbAw/exec"
            response = requests.get(URL, params=parameters)                                    
            self.si+=1

    def order_callback1(self,message):
        '''dashboard'''
        payload = str(message.message.decode("utf-8"))
        parameters = {"id":"dashboard", "team id":"VB#2213", "unique id":"hmritmteam", "order id":self.x[self.di].order_id, "item":self.x[self.di].item, "priority":self.itemv(self.x[self.di].item)[0:2], "quantity":1, "city":self.x[self.di].city, "longitude":self.x[self.di].lon, "latitude":self.x[self.di].lat, "order dispatched":"YES", "order shipped":"YES", "order time":self.dord[self.hi],"dispatch time":self.ddis[self.hi], "shipping time":self.dship[self.hi], "time taken":self.dship[self.hi]-self.ddis[self.hi]}  
        URL ="https://script.google.com/macros/s/AKfycbwqWUnbFf2h9LW0zFYUplS5Xp-wNNzwX0DZ63hFRAzBMxVlqprAHMQbAw/exec"
        response = requests.get(URL, params=parameters)
        self.di+=1
        self.hi+=1
    
    def orderdisp(self,message):
        '''dispatch'''
    	parameters = {"id":"order dispatched", "team id":"VB#2213", "unique id":"hmritmteam", "order id":self.x[self.disp].order_id, "city":self.x[self.disp].city, "item":self.x[self.disp].item, "priority":self.itemv(self.x[self.disp].item)[0:2], "dispatch qty":1, "cost":self.itemv(self.x[self.disp].item)[2:], "dispatch status":"YES", "dispatch date and time":datetime.datetime.now()}  
        URL ="https://script.google.com/macros/s/AKfycbwqWUnbFf2h9LW0zFYUplS5Xp-wNNzwX0DZ63hFRAzBMxVlqprAHMQbAw/exec"  
        response = requests.get(URL, params=parameters)
        self.disp+=1            

        
   
    
    
    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        '''incoming orders'''
        payload = str(message.payload.decode("utf-8"))
        x=json.loads(payload, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
                
        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload
        self._handle_ros_pub.publish(msg_mqtt_sub)
        self.x.append(x)

        parameters = {"id":"incoming orders", "team id":"VB#2213", "unique id":"hmritmteam", "order id":x.order_id, "order date and time":x.order_time, "item":x.item, "priority":self.itemv(x.item)[0:2], "order qty":x.qty, "city":x.city, "longitude":x.lon, "latitude":x.lat, "cost":self.itemv(x.item)[2:]}  
        URL ="https://script.google.com/macros/s/AKfycbwqWUnbFf2h9LW0zFYUplS5Xp-wNNzwX0DZ63hFRAzBMxVlqprAHMQbAw/exec"
        response = requests.get(URL, params=parameters)
        print(response.content)
        self.dord.append(x.order_time)
    
    def itemv(self,str ):
        if(str=="Clothes"):
            return "LP150"
        elif(str=="Food"):
            return "MP250"
        elif(str=="Medicine"):
            return "HP450"           

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client(action server)")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if(goal.protocol == "mqtt"):
            
            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()
                
                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(  name="worker",
                                            target=self.process_goal,
                                            args=(goal_handle,) )
                thread.start()

            else:
                goal_handle.set_rejected()
                return
        
        else:
            goal_handle.set_rejected()
            return


    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        
        # Goal Processing
        if(goal.protocol == "mqtt"):
            rospy.logwarn("MQTT")

            if(goal.mode == "pub"):
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish( self._config_mqtt_server_url, 
                                        self._config_mqtt_server_port,
                                        goal.topic, 
                                        goal.message, 
                                        self._config_mqtt_qos   )

                if(ret == 0):
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif(goal.mode == "sub"):
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(  self.mqtt_sub_callback, 
                                                        self._config_mqtt_server_url, 
                                                        self._config_mqtt_server_port, 
                                                        goal.topic, 
                                                        self._config_mqtt_qos   )
                if(ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client from action server")
        if (result.flag_success == True):
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    
    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()

def sub_order_callback(message):
    '''inventory'''
    payload = str(message.message.decode("utf-8"))
    x=json.loads(payload, object_hook=lambda d: namedtuple('X', d.keys())(*d.values()))
    t=str(x.sku)
    print(x)
    parameters = {"id":"Inventory", "team id":"VB#2213", "unique id":"hmritmteam", "sku":t, "item":x.item, "priority":x.priority, "storage number":x.storagenumber, "cost":x.cost, "quantity":x.quantity}  
    URL ="https://script.google.com/macros/s/AKfycbwqWUnbFf2h9LW0zFYUplS5Xp-wNNzwX0DZ63hFRAzBMxVlqprAHMQbAw/exec"
    response = requests.get(URL, params=parameters)
    print(response)


def order_callback(message):
	payload = str(message.message.decode("utf-8"))


    
# Main
def main():
    rospy.init_node('node_action_server_ros_iot_bridge', anonymous=True)
    handle_sub = rospy.Subscriber("/eyrc/vb/hmritmteam/orders", msgMqttSub, order_callback)
    

    abc = rospy.Subscriber("messagetopic", msgMqttSub, sub_order_callback)

    action_server = RosIotBridgeActionServer()
    xyz = rospy.Subscriber("order", msgMqttSub, action_server.func_callback_topic_my_topic1)
    xyz1= rospy.Subscriber("orderdisp", msgMqttSub, action_server.orderdisp)
    sub=rospy.Subscriber("/eyrc/vb/logical_camera_1", LogicalCameraImage, action_server.func_callback_topic_my_topic)
    handle_sub = rospy.Subscriber("dashboard", msgMqttSub, action_server.order_callback1)
    print(action_server._config_mqtt_sub_cb_ros_topic)
    print("main")

    rospy.spin()


if __name__ == '__main__':
    main()
