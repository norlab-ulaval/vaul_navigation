#!/usr/bin/env python

import rospy
from threading import RLock
import copy
import numpy as np
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
import visualization_msgs.msg as viz_msg
from vaul_navigation_msgs.msg import Goal, GoalArray
from vaul_navigation_msgs.srv import AddGoal, RemoveGoal, ModifyGoal

from geometry_msgs.msg import PoseStamped, PoseArray, Pose

class GoalMarkerServer(object):
    def __init__(self, add_goal_service, remove_goal_service, modify_goal_service):
        # Service that will be called
        self.add_goal_service = add_goal_service
        self.remove_goal_service = remove_goal_service
        self.modify_goal_service = modify_goal_service
        
        # Start marker server
        self.server =  InteractiveMarkerServer("goal_markers")        

        # Init the marker array and mutex
        self.displayed_goals = list()
        self.mutex = RLock()
        
        # Init the menu handler and entries
        self.init_menu()


    def init_menu(self):
        self.menu_handler = MenuHandler()

        # Remove Goal entry
        self.menu_remove_goal_title = "Remove Goal"
        self.menu_remove_goal_handle = self.menu_handler.insert(self.menu_remove_goal_title, callback=self.process_feedback)


    def process_feedback(self, feedback):
        # The goal was moved
        if feedback.event_type == viz_msg.InteractiveMarkerFeedback.MOUSE_UP:
            goal_id = feedback.marker_name
            new_pose = PoseStamped()
            new_pose.pose = feedback.pose
            new_pose.header = feedback.header

            try:
                modify_goal_handle = rospy.ServiceProxy(self.modify_goal_service, ModifyGoal)
                success = modify_goal_handle(new_pose, goal_id).success
                if not success:
                    rospy.logwarn("Failure in modifying goal %s in goal marker server" % goal_id)
            except rospy.ServiceException, error:
                rospy.logerr("Service call failed for modify_goal in goal marker server: %s" % error)

        # A menu entry was clicked
        if feedback.event_type == viz_msg.InteractiveMarkerFeedback.MENU_SELECT:
            # The goal needs to be removed
            if feedback.menu_entry_id == self.menu_remove_goal_handle:
                goal_id = feedback.marker_name
                try:
                    remove_goal_handle = rospy.ServiceProxy(self.remove_goal_service, RemoveGoal)
                    success = remove_goal_handle(goal_id).success
                    if not success:
                        rospy.logwarn("Failure in removing goal %s in goal marker server" % goal_id)
                except rospy.ServiceException, error:
                    rospy.logerr("Service call failed for remove_goal in goal marker server: %s" % error)


    def add_marker(self, goal):
        interactive_marker = viz_msg.InteractiveMarker()
        interactive_marker.header.frame_id = goal.pose.header.frame_id
        interactive_marker.pose = goal.pose.pose
        interactive_marker.name = goal.id
        interactive_marker.scale = 1
        interactive_marker.description = "Visual representation of goal"

        # Add a menu control to the interactive marker
        menu_control = viz_msg.InteractiveMarkerControl()
        menu_control.interaction_mode = viz_msg.InteractiveMarkerControl.MENU
        menu_control.name = "menu_" + goal.id

        interactive_marker.controls.append(menu_control)

        # Arrow marker control displaying the goal pose and enabling modification of goals
        arrow_marker = viz_msg.Marker()
        arrow_marker.type = arrow_marker.ARROW
        arrow_marker.scale.x = 0.5; arrow_marker.scale.y = 0.08; arrow_marker.scale.z = 0.08 
        arrow_marker.color.a = 1.0
        arrow_marker.color.b = 0; arrow_marker.color.g = 0; arrow_marker.color.r = 1.0

        arrow_marker_control = viz_msg.InteractiveMarkerControl()
        arrow_marker_control.always_visible = True
        arrow_marker_control.markers.append(arrow_marker)
        arrow_marker_control.name = "arrow_" + goal.id
        arrow_marker_control.interaction_mode = viz_msg.InteractiveMarkerControl.MOVE_ROTATE
        arrow_marker_control.orientation.w = 1 / np.sqrt(2.0)
        arrow_marker_control.orientation.x = 0
        arrow_marker_control.orientation.y = 1 / np.sqrt(2.0)
        arrow_marker_control.orientation.z = 0

        interactive_marker.controls.append(arrow_marker_control)

        # Text marker control displaying the goal id
        text_marker = viz_msg.Marker()
        text_marker.type = text_marker.TEXT_VIEW_FACING
        text_marker.scale.z = 0.15
        text_marker.color.a = 1.0
        text_marker.color.b = 1.0; text_marker.color.g = 1.0; text_marker.color.r = 1.0
        text_marker.text = goal.id
        text_marker.pose.position.x = 0.25
        text_marker.pose.position.z = 0.4

        text_marker_control = viz_msg.InteractiveMarkerControl()
        text_marker_control.always_visible = True
        text_marker_control.markers.append(text_marker)
        text_marker_control.name = "text_" + goal.id
        text_marker_control.interaction_mode = viz_msg.InteractiveMarkerControl.NONE

        interactive_marker.controls.append(text_marker_control)

        self.server.insert(interactive_marker, self.process_feedback)
        self.menu_handler.apply(self.server, interactive_marker.name)
        self.server.applyChanges()


    def highlight_marker(self, id):
        # Make a copy of the marker with yellow color, and replace the old one
        og_interacitve_marker = self.server.get(id)
        new_interactive_marker = copy.deepcopy(og_interacitve_marker)
        arrow_marker_control = None
        for control in new_interactive_marker.controls:
            if control.name == ("arrow_" + id):
                arrow_marker_control = control
        if arrow_marker_control != None:
            arrow_marker_control.markers[0].color.a = 1.0
            arrow_marker_control.markers[0].color.b = 0.0
            arrow_marker_control.markers[0].color.g = 1.0
            arrow_marker_control.markers[0].color.r = 1.0

        # This is done because goals are not updated automatically when attributes change
        self.server.erase(og_interacitve_marker.name)
        self.server.applyChanges()
        self.server.insert(new_interactive_marker, self.process_feedback)
        self.server.applyChanges()


    def remove_marker(self, goal):
        self.server.erase(goal.id)
        self.server.applyChanges()


    def update_goals(self, updated_goals):
        with self.mutex:
            # Will be used to highlight the first goal later
            first_goal_id = None
            if len(updated_goals.goals) > 0:
                first_goal_id = updated_goals.goals[0].id
                
            # If not present in updated goals, goal has been removed, else nothing changed
            for goal in self.displayed_goals[:]:
                if goal in updated_goals.goals:
                    updated_goals.goals.remove(goal)
                else:
                    self.displayed_goals.remove(goal)
                    self.remove_marker(goal)
            # The updated goals left are those not already displayed
            for goal in updated_goals.goals[:]:
                self.displayed_goals.append(goal)
                self.add_marker(goal)

            # If there is atleast one goal, highlight the first one as the current goal
            if first_goal_id != None:
                self.highlight_marker(first_goal_id)


    def add_new_goal(self, goal):
        # Try calling service to add goal
        try:
            add_goal_handle = rospy.ServiceProxy(self.add_goal_service, AddGoal)
            success = add_goal_handle(goal).success
            if not success:
                rospy.logwarn("Failure in adding new goal at %f in goal marker server" % goal.header.stamp)
        except rospy.ServiceException, error:
            rospy.logerr("Service call failed for add_goal in goal marker server: %s" % error)


    def enable_new_goal_subscribing(self, topic):
        # Instantiate subscriber
        self.new_goal_subscriber = rospy.Subscriber(topic, PoseStamped, callback=self.add_new_goal)


    def enable_goals_subscribing(self, topic):
        # Instantiate subscriber
        self.goals_subscriber = rospy.Subscriber(topic, GoalArray, callback=self.update_goals)
