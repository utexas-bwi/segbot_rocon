#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/concert_services/license/LICENSE
#
##############################################################################
# About
##############################################################################

# Simple script to manage spawning and killing of turtles across multimaster
# boundaries. Typically turtlesim clients would connect to the kill and
# spawn services directly to instantiate themselves, but since we can't
# flip service proxies, this is not possible. So this node is the inbetween
# go-to node and uses a rocon service pair instead.
#
# It supplements this relay role with a bit of herd management - sets up
# random start locations and feeds back aliased names when running with
# a concert.

##############################################################################
# Imports
##############################################################################

import copy
import os
import signal
import tempfile
import subprocess

import gateway_msgs.msg as gateway_msgs
import gateway_msgs.srv as gateway_srvs
import rocon_launch
import rospy
import rocon_console.console as console
import rocon_gateway_utils
import rocon_python_utils.ros

##############################################################################
# Utilities
##############################################################################

class ProcessInfo(object):
    def __init__(self, process, temp_file):
        self.process = process
        self.temp_file = temp_file

##############################################################################
# Turtle Herder
##############################################################################

class GazeboRobotManager:
    '''
      Shepherds the turtles!

      @todo get alised names from the concert client list if the topic is available

      @todo watchdog for killing turtles that are no longer connected.
    '''

    def __init__(self, robot_manager):
        self.robots = []
        self.robot_manager = robot_manager
        self._process_info = []
        self.is_disabled = False
        # gateway
        gateway_namespace = rocon_gateway_utils.resolve_local_gateway()
        rospy.wait_for_service(gateway_namespace + '/flip')
        self._gateway_flip_service = rospy.ServiceProxy(gateway_namespace + '/flip', gateway_srvs.Remote)

    def _spawn_simulated_robots(self, robots):
        """
        Very important to have checked that the turtle names are unique
        before calling this method.

        :param turtles str[]: names of the turtles to spawn.
        """
        for robot in robots:
            try:
                self.robot_manager.spawn_robot(robot["name"], robot["location"])
                self.robots.append(robot["name"])
            # TODO add failure exception
            except rospy.ROSInterruptException:
                rospy.loginfo("GazeboRobotManager : shutdown while spawning robot")
                continue

    def _launch_robot_clients(self, robot_names):
        # spawn the turtle concert clients
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        rocon_launch_text = self.robot_manager.prepare_rocon_launch_text(robot_names)
        rospy.loginfo("GazeboRobotManager: constructing robot client rocon launcher")
        print("\n" + console.green + rocon_launch_text + console.reset)
        temp.write(rocon_launch_text)
        temp.close()  # unlink it later
        rocon_launch_env = os.environ.copy()
        try:
            # ROS_NAMESPACE gets set since we are inside a node here
            # got to get rid of this otherwise it pushes things down
            del rocon_launch_env['ROS_NAMESPACE']
        except KeyError:
            pass
        rospy.loginfo("GazeboRobotManager : starting process %s" % ['rocon_launch', temp.name, '--screen'])
        if rocon_python_utils.ros.get_rosdistro() == 'hydro':
            process = subprocess.Popen(['rocon_launch', '--gnome', temp.name, '--screen'], env=rocon_launch_env)
        else:
            process = subprocess.Popen(['rocon_launch', temp.name, '--screen'], env=rocon_launch_env)
        self._process_info.append(ProcessInfo(process, temp))

    def spawn_robots(self, robots):
        unique_robots, unique_robot_names = self._establish_unique_names(robots)

        self._spawn_simulated_robots(unique_robots)
        self._launch_robot_clients(unique_robot_names)
        self._send_flip_rules(unique_robot_names, cancel=False)

    def _establish_unique_names(self, robots):
        """
        Make sure the turtle names don't clash with currently spawned turtles.
        If they do, postfix them with an incrementing counter.

        :param turtles str[]: list of new turtle names to uniquify.
        :return str[]: uniquified names for the turtles.
        """
        unique_robots = []
        unique_robot_names = []
        for robot in robots:
            robot_name = robot["name"]
            name_extension = ''
            count = 0
            while robot_name + name_extension in unique_robot_names:
                name_extension = str(count)
                count = count + 1
            unique_robot_names.append(robot_name + name_extension)
            robot_copy = copy.deepcopy(robot)
            robot_copy["name"] = robot_name + name_extension
            unique_robots.append(robot_copy)
        return unique_robots, unique_robot_names

    def _send_flip_rules(self, robot_names, cancel):
        for robot_name in robot_names:
            rules = self.robot_manager.get_flip_rule_list(robot_name)
            # send the request
            request = gateway_srvs.RemoteRequest()
            request.cancel = cancel
            remote_rule = gateway_msgs.RemoteRule()
            remote_rule.gateway = robot_name
            for rule in rules:
                remote_rule.rule = rule
                request.remotes.append(copy.deepcopy(remote_rule))
            try:
                self._gateway_flip_service(request)
            except rospy.ServiceException:  # communication failed
                rospy.logerr("GazeboRobotManager : failed to send flip rules")
                return
            except rospy.ROSInterruptException:
                rospy.loginfo("GazeboRobotManager : shutdown while contacting the gateway flip service")
                return

    def _ros_service_manager_disable_callback(self, msg):
        self.is_disabled = True

    def shutdown(self):
        """
          - Send unflip requests
          - Cleanup turtles on the turtlesim canvas.
          - Shutdown spawned terminals

        :todo: this should go in a service manager callable ros callback where we can
        call disable on this service and bring it down without having to SIGINT it.
        """
        for name in self.robots:
            try:
                self.robot_manager.delete_robot(name)
                #TODO quitely fail exception here
            except rospy.ROSInterruptException:
                break  # quietly fail

        for process_info in self._process_info:
            print("Pid: %s" % process_info.process.pid)
            roslaunch_pids = rocon_launch.get_roslaunch_pids(process_info.process.pid)
            print("Roslaunch Pids: %s" % roslaunch_pids)
        # The os.kills aren't required if a concert is doing a full shutdown since
        # it disperses signals everywhere anyway. This is important if we implement the
        # disable from the service manager though.
        for pid in roslaunch_pids:
            try:
                os.kill(pid, signal.SIGINT)  # sighup or sigint? I can't remember - this is same as rocon_launch code
            except OSError:
                continue
        for pid in roslaunch_pids:
            print("Waiting on roslaunch pid %s" % pid)
            result = rocon_python_utils.system.wait_pid(pid)
            print("Pid %s exited with result %s" % (pid, result))
#         time.sleep(1)  # Do we need this?
        for process_info in self._process_info:
            print("Now killing konsoles %s" % process_info.process.pid)
            try:
                os.killpg(process_info.process.pid, signal.SIGTERM)
                #process_info.process.terminate()
            except OSError:
                print("process already died naturally")
                pass
        for process_info in self._process_info:
            print("Unlinking %s" % process_info.temp_file.name)
            try:
                os.unlink(process_info.temp_file.name)
            except OSError:
                pass

