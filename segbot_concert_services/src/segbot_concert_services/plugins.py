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
        self.button_layout = QHBoxLayout(self.widget)
        self.master_layout.addWidget(self.button_layout)

        self.widget.setObjectName('MultiRobotPatrollerPluginUI')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

        default_points = [(0, 0, 1.578), (0, 5, 0), (5, 5, -1.578), (5, 0, 3.1414)]
        self.points = rospy.get_param("points", default_points)
        self.flip_direction = rospy.get_param("flip_direction", False)

        self.available_robots = []
        self.global_start_counter = 0
        self.global_forward = True

        self.available_robot_subscriber = rospy.Subscriber("available_robots", AvailableRobotArray,
                                                           self.available_robot_callback)

        self.robot_goals = {}
        self.paused = True

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

                self.widget.emit(SIGNAL("update_button_status"))

                # Waits for the server to finish performing the action.
                while not rospy.is_shutdown():
                    if client.wait_for_result(rospy.Duration(1)):
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

    def update_buttons(self):
        self.clean()
        for index, level in enumerate(self.levels):
            self.text_label.setText("Choose Level: ")
            button = QPushButton(level.level_id, self.widget)
            button.clicked[bool].connect(self.handle_button)
            button.setCheckable(True)
            self.master_layout.addWidget(button)
            self.buttons.append(button)

        # Subscribe to the current level we are on.
        if self.status_subscriber is None:
            self.status_subscriber = rospy.Subscriber("level_mux/current_level", LevelMetaData, self.process_level_status)

    def update_button_status(self):
        for index, level in enumerate(self.levels):
            if self.current_level == level.level_id:
                self.buttons[index].setChecked(True)
            else:
                self.buttons[index].setChecked(False)

    def process_level_status(self, msg):
        level_found = False
        for level in self.levels:
            if msg.level_id == level.level_id:
                self.current_level = level.level_id
                level_found = True
                break
        if not level_found:
            self.current_level = None
        self.widget.emit(SIGNAL("update_button_status"))

    def handle_button(self):
        source = self.sender()

        if source.text() == self.current_level:
            source.setChecked(True)
            return

        # Construct a identity pose. The level selector cannot be used to choose the initialpose, as it does not have
        # the interface for specifying the position. The position should be specified via rviz.
        origin_pose = PoseWithCovarianceStamped()
        origin_pose.header.frame_id = frameIdFromLevelId(source.text())
        origin_pose.pose.pose.orientation.w = 1    # Makes the origin quaternion valid.
        origin_pose.pose.covariance[0] = 1.0
        origin_pose.pose.covariance[7] = 1.0
        origin_pose.pose.covariance[14] = 1.0
        origin_pose.pose.covariance[21] = 1.0
        origin_pose.pose.covariance[28] = 1.0
        origin_pose.pose.covariance[35] = 1.0

        # Don't actually publish the initial pose via the level selector. It doesn't know any better.
        self.level_selector_proxy(source.text(), False, origin_pose)

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


