#!/usr/bin/env python

import threading
from threading import RLock, Thread

from  vaul_navigation_msgs.srv import AddGoal, ClearGoals, ModifyGoal, RemoveGoal
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from vaul_navigation_msgs.msg import Goal, GoalArray

def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)   
        return thread
    return wrapper

class GlobalPlanner(object):
    def __init__(self, id_base, robot_frame_id, robot_pose_update_frequency, completion_check_frequency, completion_distance):

        self.goal_array = GoalArray()
        self.goals_mutex = RLock()

        self.id_seq = 0
        self.id_base = id_base

        self.robot_frame_id = robot_frame_id
        self.robot_pose = None
        self.robot_pose_mutex = RLock()

        self.goals_publish_thread = None
        self.goals_pub = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_update_thread = self.update_robot_pose(robot_pose_update_frequency)
        self.pose_update_thread.start()

        self.completion_distance = completion_distance
        self.completion_check_thread = self.check_for_completion(completion_check_frequency)
        self.completion_check_thread.start()

    
    def publish_goals(self):
        if self.goals_pub != None:
            with self.goals_mutex:
                self.goals_pub.publish(self.goal_array)


    def enable_publish_goals_periodically(self, frequency, topic):
        if frequency > 0:
            self.goals_pub = rospy.Publisher(topic, GoalArray, queue_size=1)
            self.goals_publish_thread = self.publish_goals_periodically(frequency)
            self.goals_publish_thread.start()

    
    @threaded
    def publish_goals_periodically(self, frequency):        
        frequency_controller = rospy.Rate(frequency)
        try:
            while not rospy.is_shutdown():
                self.publish_goals()
                frequency_controller.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupt exception catched in goals publishing thread of global planner")
        except Exception, e:
            rospy.logerr("Exception raised in publish_goals_periodically of global planner: %s", e)  


    @threaded
    def update_robot_pose(self, frequency):
        frequency_controller = rospy.Rate(frequency)
        try:
            while not rospy.is_shutdown():
                with self.goals_mutex:
                    with self.robot_pose_mutex:
                        # If there are no goals we can't find a robot pose relative to the current goal
                        if self.has_goal():
                            current_goal_frame_id = self.goal_array.goals[0].pose.header.frame_id
                            transform_success = False 
                            try:
                                transform = self.tf_buffer.lookup_transform(current_goal_frame_id, self.robot_frame_id,  rospy.Time())
                                transform_success = True
                            except Exception:
                                rospy.logwarn("No tf published from goal's frame (%s) to robot frame id", current_goal_frame_id)

                            if transform_success:
                                self.robot_pose = PoseStamped()
                                self.robot_pose.header = transform.header
                                self.robot_pose.pose.position.x = transform.transform.translation.x
                                self.robot_pose.pose.position.y = transform.transform.translation.y
                                self.robot_pose.pose.orientation = transform.transform.rotation
                frequency_controller.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupt exception catched in update robot pose thread of global_planner")
        except Exception, e:
            rospy.logerr("Exception raised in update_robot_pose of global_planner: %s", e)


    @threaded
    def check_for_completion(self, frequency):
        frequency_controller = rospy.Rate(frequency)
        try:
            while not rospy.is_shutdown():
                current_robot_pose = None
                current_goal = None
                with self.goals_mutex:
                    with self.robot_pose_mutex:
                        current_robot_pose = self.robot_pose
                        if self.has_goal():
                            current_goal = self.goal_array.goals[0]
                    
                        if (current_robot_pose != None) and (current_goal != None):
                            x_distance_to_goal = current_goal.pose.pose.position.x - current_robot_pose.pose.position.x
                            y_distance_to_goal = current_goal.pose.pose.position.y - current_robot_pose.pose.position.y
                            distance_to_goal = np.sqrt((x_distance_to_goal**2) + (y_distance_to_goal**2))

                            if distance_to_goal <= self.completion_distance:
                                self.remove_goal(current_goal.id)
                frequency_controller.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupt exception catched in completion checking thread in global_planner")
        except Exception, e:
            rospy.logerr("Exception raised in check_for_completion of global_planner: %s", e)
                


    def has_goal(self):
        with self.goals_mutex:
            has_goal_value = len(self.goal_array.goals) > 0

        return has_goal_value 


    def add_goal(self, pose):
        with self.goals_mutex:
            new_goal = Goal()
            new_goal.pose = pose
            new_goal.id = self.id_base + str(self.id_seq)
            self.id_seq += 1

            self.goal_array.goals.append(new_goal)

            self.publish_goals()

            return new_goal.id, True


    def remove_goal(self, goal_id ):
        with self.goals_mutex:
            for goal in self.goal_array.goals:
                if goal.id == goal_id:
                    self.goal_array.goals.remove(goal)
                    break
                
            self.publish_goals()

            return True


    def clear_goals(self):
        with self.goals_mutex:
            self.goal_array.goals[:] = [] 

            self.publish_goals()

            return True

        
    def modify_goal(self, goal_id, new_pose):
        with self.goals_mutex:
            for goal in self.goal_array.goals:
                if goal.id == goal_id:
                    goal.pose = new_pose
            
            self.publish_goals()

            return True


    def add_goal_srv(self, req):        
        with self.goals_mutex:
            return self.add_goal(req.new_goal)

        
    def remove_goal_srv(self, req):
        with self.goals_mutex:
            return self.remove_goal(req.goal_id)


    def clear_goals_srv(self):
        with self.goals_mutex:
            return self.clear_goals()


    def modify_goal_srv(self, req):
        with self.goals_mutex:
            return self.modify_goal(req.goal_id, req.new_pose)

    def enable_add_goal_srv(self, service):        
        self.add_goal_provider = rospy.Service(service, AddGoal, self.add_goal_srv)
        

    def enable_remove_goal_srv(self, service):
        self.remove_goal_provider = rospy.Service(service, RemoveGoal, self.remove_goal_srv)


    def enable_clear_goals_srv(self, service):
        self.clear_goals_provider = rospy.Service(service, ClearGoals, self.clear_goals_srv)


    def enable_modify_goal_srv(self, service):
        self.modify_goal_provider = rospy.Service(service, ModifyGoal, self.modify_goal_srv)