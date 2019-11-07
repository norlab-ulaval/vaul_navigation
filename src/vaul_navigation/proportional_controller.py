#!/usr/bin/env python
from threading import Thread
import math

import rospy
from tf2_ros import TransformListener
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from sensor_msgs.msg import Joy
from vaul_navigation_msgs.msg import Goal, GoalArray

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)   
        return thread
    return wrapper

class ProportionalController(object):
    def __init__(self, frame_id, linear_gain, angular_gain, max_linear_speed, max_angular_speed, linear_tolerance, angular_tolerance):
        # Frame id of the robot
        self.frame_id = frame_id

        # Controller parameters
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.linear_tolerance = linear_tolerance
        self.angular_tolerance = angular_tolerance
        self.frequency = None
        self.control_loop_thread = None

        # Subscriber and publishers informations
        self.pose_subscriber = None
        self.goal_subscriber = None
        self.joy_subscriber = None
        self.cmd_vel_publisher = None

        # Current goal
        self.goal = None

        # Pose of the robot in the goals frame
        self.pose = None

        # Current velocity published
        self.cmd_vel = Twist()

        # Control enabling variables
        self.has_sent_stop_cmd = False
        self.max_time_without_pose = None
        self.max_time_without_deadman_switch = None
        self.deadman_switch_pressed = False
        self.deadman_switch_button = None
        self.deadman_switch_stamp = rospy.Time()

    def start(self, frequency, deadman_switch_button, max_time_without_deadman_switch, max_time_without_pose):
        # Set necessary parameters for control loop
        self.frequency = frequency
        self.deadman_switch_button = deadman_switch_button
        self.max_time_without_deadman_switch = max_time_without_deadman_switch
        self.max_time_without_pose = max_time_without_pose

        # Start control loop thread
        self.control_loop_thread = self.control_loop()
        self.control_loop_thread.start()

    def set_pose(self, pose):
        self.pose = pose

    def set_goal(self, goal):
        self.goal = goal

    def set_goal_from_array(self, goal_array):
        # The current goal is the first in the array
        if len(goal_array.goals) > 0:
            self.set_goal(goal_array.goals[0])
        else:
            self.set_goal(None)

    def set_deadman_switch_pressed(self, joy):
        # If button is set, we can tell if deadman switch is pressed or not
        if(self.deadman_switch_button != None):
            self.deadman_switch_pressed = (joy.buttons[self.deadman_switch_button] == 1)
            self.deadman_switch_stamp = rospy.Time.now()

    def get_cmd_vel(self):
        return self.cmd_vel

    def get_control_loop_thread(self):
        return self.control_loop_thread
   
    @threaded
    def control_loop(self):
        rospy.loginfo("Controller started")
        frequency_controller = rospy.Rate(self.frequency)

        send_stop_cmd = True
        
        try:
            while(not rospy.is_shutdown()):
                
                # If we have all necessary lastest information and movement is enabled, calculate cmd_vel
                if(self.has_goal() and self.has_pose() and self.movement_enabled()):
                    # Calculate linear speed and clamp between distance tolerance and max linear speed
                    distance_to_goal = self.get_distance_to_goal()
                    linear_vel = distance_to_goal * self.linear_gain
                    
                    if distance_to_goal <= self.linear_tolerance:
                        linear_vel = 0
                    elif linear_vel >= self.max_linear_speed:
                        linear_vel = self.max_linear_speed
                    
                    # Calculate angular speed and clamp between angular tolerance and max  angular speed
                    angle_to_goal = self.get_angle_to_goal()
                    angular_vel = angle_to_goal * self.angular_gain

                    if abs(angle_to_goal) <= self.angular_tolerance:
                        angular_vel = 0
                    elif(angular_vel >= self.max_angular_speed):
                        angular_vel = self.max_angular_speed
                    elif(angular_vel <= -self.max_angular_speed):
                        angular_vel = -self.max_angular_speed
                            
                    self.cmd_vel.linear.x = linear_vel
                    self.cmd_vel.angular.z = angular_vel

                    send_stop_cmd = False

                # Else, send stop command
                else:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = 0

                    send_stop_cmd = True

                # Send cmd_vel or stop cmd
                if not send_stop_cmd:
                    self.send_cmd_vel()
                else:
                    self.send_stop_cmd()

                frequency_controller.sleep()
        
        except rospy.ROSInterruptException:
                rospy.loginfo("ROS interrupt exception catched in control_loop of proportional controller")
        except Exception, e:
            rospy.logerr("Exception raised in control_loop of proportional controller: %s", e)            

    def get_distance_to_goal(self):
        distance_to_goal = 0
        if self.has_goal() and self.has_pose():
            distance_to_goal = math.sqrt((self.goal.pose.pose.position.x - self.pose.pose.position.x)**2 + \
                (self.goal.pose.pose.position.y - self.pose.pose.position.y)**2)
        else:
            rospy.logwarn("Could not calculate distance to goal since there is no goal and/or pose")
        
        # Return positive value, because distance can't be negative
        return abs(distance_to_goal)
        
    def get_angle_to_goal(self):
        angle_to_goal = 0
        if self.has_goal() and self.has_pose():

            orientation = euler_from_quaternion((self.pose.pose.orientation.x, self.pose.pose.orientation.y, \
            self.pose.pose.orientation.z, self.pose.pose.orientation.w))[2]
            steering_angle = math.atan2(self.goal.pose.pose.position.y - self.pose.pose.position.y, \
                self.goal.pose.pose.position.x - self.pose.pose.position.x)

            # Angle error is difference between wanted steering angle and actual
            angle_to_goal = steering_angle - orientation
            
            # Always take the minimum distance (between -180 and 180 degrees)
            if angle_to_goal > math.pi:
                angle_to_goal -= 2 * math.pi
            elif angle_to_goal < -math.pi:
                angle_to_goal += 2 * math.pi
            
        else:
            rospy.logwarn("Could not calculate angle to goal since there is no goal and/or pose")
        
        return angle_to_goal

    def has_goal(self):
        return not self.goal == None

    def has_pose(self):
        return not self.pose == None

    def movement_enabled(self):
        movement_enabled_value = False
        # If we have all the necessary info we can provide movement_eanbled_value
        if(self.deadman_switch_button != None) and \
            (self.max_time_without_deadman_switch != None) and (self.max_time_without_pose != None):
            
            # Deadman switch must be pressed
            movement_enabled_value = self.deadman_switch_pressed
            
        return movement_enabled_value

    def send_cmd_vel(self):
        # If we have a publisher publish the actual cmd_vel
        if self.cmd_vel_publisher != None:
            self.cmd_vel_publisher.publish(self.cmd_vel)
            self.has_sent_stop_cmd = False

    def send_stop_cmd(self):
        # sned stop cmd if we have a publisher and have not yet sent stop cmd
        if self.cmd_vel_publisher != None:
            if not self.has_sent_stop_cmd:    
                self.cmd_vel_publisher.publish(Twist())
                self.has_sent_stop_cmd = True
            
    def enable_pose_subscribing(self, topic):
        # Instantiate subscriber
        self.pose_subscriber = rospy.Subscriber(topic, PoseStamped, self.set_pose)

    def enable_goal_array_subscribing(self, topic):
        # Instantiate subscriber
        self.goal_subscriber = rospy.Subscriber(topic, GoalArray, self.set_goal_from_array)

    def enable_goal_subscribing(self, topic):
        # Instantiate subscriber
        self.goal_subscriber = rospy.Subscriber(topic, GoalArray, self.set_goal)

    def enable_joy_subscribing(self, topic):
        # Instantiate subscriber        
        self.joy_subscriber = rospy.Subscriber(topic, Joy, self.set_deadman_switch_pressed)

    def enable_cmd_vel_publishing(self, topic):
        # Instantiate publisher
        self.cmd_vel_publisher = rospy.Publisher(topic, Twist, queue_size=10)
