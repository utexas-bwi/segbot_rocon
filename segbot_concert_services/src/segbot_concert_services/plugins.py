#!/usr/bin/env python

import actionlib
import rospy
import tf
import thread
import time

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from segbot_concert_services.msg import AvailableRobotArray

from python_qt_binding.QtCore import SIGNAL
from python_qt_binding.QtGui import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
from qt_gui.plugin import Plugin

class MultiRobotPatrollerPlugin(Plugin):

    def __init__(self, context):
        super(MultiRobotPatrollerPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MultiRobotPatrollerPlugin')

        # Create QWidget
        self.widget = QWidget()
        self.master_layout = QVBoxLayout(self.widget)

        self.buttons = []
        self.button_layout = QHBoxLayout()
        self.master_layout.addLayout(self.button_layout)
        for button_text in ["Play", "Pause"]:
            button = QPushButton(button_text, self.widget)
            button.clicked[bool].connect(self.handle_button)
            button.setCheckable(True)
            self.button_layout.addWidget(button)
            if button_text == "Pause":
                button.setChecked(True)
            self.buttons.append(button)

        self.text_labels = {}

        self.widget.setObjectName('MultiRobotPatrollerPluginUI')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

        self.points = rospy.get_param("points")
        self.flip_direction = rospy.get_param("flip_direction")

        self.available_robots = []
        self.global_start_counter = 0
        self.global_forward = True

        self.available_robot_subscriber = rospy.Subscriber("/available_robots", AvailableRobotArray,
                                                           self.available_robot_callback)

        self.robot_goals = {}
        self.paused = True

        self.connect(self.widget, SIGNAL("update"), self.update)

    def navigate_robot(self, robot_name, start_counter, forward):

        client = actionlib.SimpleActionClient('/' + robot_name + '/move_base_interruptable', MoveBaseAction)
        resolved_frame = '/' + robot_name + '/level_mux/map'

        # Waits until the action server has started up and started
        # listening for goals.
        while not rospy.is_shutdown():
            if client.wait_for_server(rospy.Duration(1)):
                break

        counter = start_counter
        failures = 0
        while not rospy.is_shutdown():
            # Creates a goal to send to the action server.
            if not self.paused:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = resolved_frame
                goal.target_pose.header.stamp = rospy.get_rostime()
                goal.target_pose.pose.position.x = self.points[counter][0]
                goal.target_pose.pose.position.y = self.points[counter][1]
                goal.target_pose.pose.position.z = 0.0
                q = tf.transformations.quaternion_from_euler(0, 0, self.points[counter][2])
                goal.target_pose.pose.orientation = Quaternion(*q)

                # Sends the goal to the action server.
                client.send_goal(goal)

                self.robot_goals[robot_name] = counter
                self.widget.emit(SIGNAL("update"))

                # Waits for the server to finish performing the action.
                while not rospy.is_shutdown():
                    if client.wait_for_result(rospy.Duration(1)):
                        break
                    elif self.paused:
                        client.cancel_goal()
                        break

                if client.get_state() == GoalStatus.SUCCEEDED or failures >= 3:
                    failures = 0
                    if forward:
                        counter = (counter + 1) % len(self.points)
                    else:
                        counter = counter - 1
                        if counter < 0:
                            counter = len(self.points) - 1
                else:
                    # Try the goal again after given seconds
                    time.sleep(3.0)
                    failures += 1
            else:
                time.sleep(1.0)

    def available_robot_callback(self, msg):
        for robot_resource in msg.robot_name:
            results = robot_resource.split('/')
            robot_name = results[2]
            if robot_name not in self.available_robots:
                self.available_robots.append(robot_name)
                thread.start_new_thread(self.navigate_robot, (robot_name, self.global_start_counter,
                                                              self.global_forward))
                if self.flip_direction:
                    self.global_forward = not self.global_forward
                self.global_start_counter = (self.global_start_counter + 1) % len(self.points)

    def update(self):

        for robot in self.robot_goals:
            point = self.points[self.robot_goals[robot]]
            text = robot + " -> (" + str(point[0]) + "," + str(point[1]) + ")"
            if robot not in self.text_labels:
                self.text_labels[robot] = QLabel(robot, self.widget)
                self.master_layout.addWidget(self.text_labels[robot])
            self.text_labels[robot].setText(text)

    def handle_button(self):
        source = self.sender()

        if ((source.text() == "Pause" and self.paused) or
            (source.text() == "Play" and not self.paused)):
            source.setChecked(True)
            return

        for button in self.buttons:
            if button != source:
                button.setChecked(False)

        self.paused = (source.text() == "Pause")

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


