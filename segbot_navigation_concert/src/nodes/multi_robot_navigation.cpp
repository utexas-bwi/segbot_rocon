// subscribe to multimap
// inflate all maps
// subscribe to available_robots
// once available_robots have been achieved, subscribe to the path of each robot
// whenever a path message is received, see if there is a collision. This can be done by inflating the path and checking
// for overlap.
// if there is a collision send one of the robots to a hardcoded bypass location. 2 seconds later, restart the other
// robot. 
// keep track of the distance of the other robot from the bypass location. As soon as it starts increasing, resume first
// robot.

// Assumptions: don't check floors. assume 2 robots only.

#include <actionlib/client/simple_action_client.h>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <segbot_navigation_concert/AvailableRobotArray.h>

struct RobotStatus {
  NORMAL,
  HEADING_TO_BYPASS_POINT,
  WAITING_AT_BYPASS_POINT,
  WAITING_FOR_OTHER_ROBOT,
};

class MultiRobotNavigator {

  public:

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> RobotController;
    typedef boost::shared_ptr<RobotController> RobotControllerPtr;

    MultiRobotNavigator();
    void spin();

  protected:

    void multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr& multimap);
    ros::Subscriber multimap_subscriber_;
    bool multimap_available_;

    void availableRobotArrayHandler(const segbot_navigationConcert::AvailableRobotArray::ConstPtr& robots);
    rso::Subscriber available_robots_subscriber_;
    std::set<std::string> available_robots_;

    // TODO Assumes a single floor for now.
    nav_msgs::OccupancyGrid original_map_;
    nav_msgs::OccupancyGrid inflated_map_;

    std::map<std::string, RobotStatus> current_robot_status_;
    void robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, const std::string& robot_name);
    std::map<std::string, ros::Subscriber> robot_locations_sub_;
    std::map<std::string, geometry_msgs::PoseWithCovarianceStamped> robot_locations_;
    void robotPathHandler(const nav_msgs::Path::ConstPtr& path, const std::string& robot_name);
    std::map<std::string, ros::Subscriber> robot_path_sub_;
    std::map<std::string, nav_msgs::Path> global_plan_;
    std::map<std::string, nav_msgs::OccupancyGrid> expanded_plan_;
    std::map<std::string, RobotControllerPtr> robot_controller_;
    std::map<std::string, ros::ServiceClient> robot_resumer_;
    std::map<std::string, ros::ServiceClient> robot_pauser_;

    float inflation_radius_;

    std::set<std::pair<int, int> > intercepted_robots_;
    std::set<std::pair<int, int> > min_bypass_distance_;

};

MultiRobotNavigator::MultiRobotNavigator() : multimap_available_(false) {

  ros::param::param("~inflation_radius", inflation_radius_, 0.3);

  // Make sure you publish the default map at least once so that navigation can start up! Ensure this pub is latched.
  ros::NodeHandle nh;
  available_robots_subscriber_ = nh.subscribe("available_robots", 1, &MultiRobotNavigator::availableRobotArrayHandler,
                                              this);

  multimap_subscriber_ = nh.subscribe("map_metadata", 1, &MultiRobotNavigator::multimapHandler, this);

}

void MultiRobotNavigator::multimapHandler(const multi_level_map_msgs::MultiLevelMapData::ConstPtr& multimap) {

  // TODO assuming there is a single floor, read it in for now.
  const std::string& map_file = multimap->levels[0].map_file;
  bwi_mapper::MapLoader mapper(map_file);
  mapper.getMap(original_map_);
  bwi_mapper::inflateMap(inflation_radius_, original_map_, inflated_map_);

  multimap_available_ = true;

}

void MultiRobotNavigator::availableRobotArrayHandler(const segbot_navigationConcert::AvailableRobotArray::ConstPtr& robots) {
  BOOST_FOREACH(const std::string resource_str, robots->robot_name) {
    std::vector<std::string> results;
    boost::split(resource_str, results, boost::is_any_of("/"));
    std::string robot_name = results[2];
    available_robots_.add(robot_name);
  }
}

bool MultiRobotNavigator::pauseRobot(const std::string& robot_name, 
                                     RobotStatus post_success_status = WAITING_FOR_OTHER_ROBOT) {

  // TODO also change the robot status;
}

bool MultiRobotNavigator::resumeRobot(const std::string& robot_name) {

  // TODO also change robot status;
}

bool MultiRobotNavigator::sendNavigationGoal(const std:string& robot_name, 
                                             const geometry_msgs::PoseStamped& pose,
                                             RobotStatus post_send_status = NORMAL) {
  // TODO
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;
  robot_controller_[robot_name]->sendGoal(goal);
  bool navigation_request_complete = false;
  while (!navigation_request_complete) {
    if (execute_action_server_->isPreemptRequested() || !ros::ok()) {
      ROS_INFO("MultiRobotNavigator: Got pre-empted. Cancelling low level navigation task...");
      robot_controller_->cancelGoal();
      break;
    }
    navigation_request_complete = robot_controller_->waitForResult(ros::Duration(0.5));
  }

  if (navigation_request_complete) {
    actionlib::SimpleClientGoalState state = robot_controller_->getState();
    return state == actionlib::SimpleClientGoalState::SUCCEEDED;
  }

  // If we're here, then we preempted the request ourselves. Let's mark our current request as not successful.
  return false;
}

void MultiRobotNavigator::spin() {
  ros::Rate r(10);
  while (ros::ok()) {

    // Create controllers for each robot.
    BOOST_FOREACH(const std::string robot, available_robots_) {
      if (current_robot_status_.find(robot) == current_robot_status_.end()) {
        // Setup all the subcribers
        ros::NodeHandle nh("/");
        robot_locations_sub_[robot] = nh.subscribe(robot + "/amcl_pose", 1,
                                                   boost::bind(&MultiRobotNavigator::robotLocationHandler, this, _1,
                                                               robot)); 
        robot_path_sub_[robot] = nh.subscribe(robot + "/move_base/EBandPlannerROS/global_plan", 1,
                                              boost::bind(&MultiRobotNavigator::robotPathHandler, this, _1, robot)); 
        // The robot controllers
        robot_controller_[robot] = RobotControllerPtr(new RobotController("/" + robot + "/move_base_interruptable", true)); 
        robot_resumer_[robot] = nh.serviceClient<std_srvs::Empty>(robot + "move_base_interruptable/resume");
        robot_pauser_[robot] = nh.serviceClient<std_srvs::Empty>(robot + "move_base_interruptable/pause");
      }
    }

    if (multimap_available_) {

      std::vector<std::string> available_robots_vector(available_robots_.begin(), available_robots_.end());

      // For each available robot, check if they are about to collide.
      for (int i = 0; i < available_robots_vector.size(); ++i) {
        const std::string& ri = available_robots_vector[i];
        // Check for any new collisions.
        if (robot_status_[i] == NORMAL) {
          for (int j = i + 1; j < available_robots_vector.size(); ++j) {
            const std::string& rj = available_robots_vector[j];
            if (robot_status_[j] == NORMAL) { 
              // Compare the expanded plans for any collision.
              if (plansOverlap(expanded_plan_[ri], expanded_plan_[rj])) {

                // Let's pause both the robots. 
                pauseRobot(ri);
                pauseRobot(rj);

                // See which robot can be moved the most on the perpendicular.
                geometry_msgs::PoseStamped bypassi = calculateBypassPoint(ri, rj);
                geometry_msgs::PoseStamped bypassj = calculateBypassPoint(rj, ri);

                float bypassdistancei = getDistance(robot_locations_[ri], bypassi);
                float bypassdistancej = getDistance(robot_locations_[rj], bypassj);

                if (bypassdistancei > bypassdistancej) {
                  // Divert robot i to the bypass point.
                  sendNavigationGoal(ri, bypassi, HEADING_TO_BYPASS_POINT);
                } else {
                  sendNavigationGoal(rj, bypassj, HEADING_TO_BYPASS_POINT);
                }

                intercepted_robots_[i] = j;
                intercepted_robots_[j] = i;

                break;
              }
            }
          }
        }
      }

      // See if any robot heading to a bypass point has reached said bypass points.
      for (int i = 0; i < available_robots_vector.size(); ++i) {
        const std::string& ri = available_robots_vector[i];
        if (robot_status_[ri] == HEADING_TO_BYPASS_POINT) {
          if (hasNavigationActionCompleted(ri)) {
            int j = intercepted_robots_[i];
            const std::string& rj = available_robots_vector[j];
            robot_status_[ri] = WAITING_AT_BYPASS_POINT;
            min_bypass_distance_[i] = getDistance(robot_locations_[ri],
                                                  robot_locations_[rj]);
            resumeRobot(rj);
          }
        }
      }

      // If a robot is waiting at the bypass point, see if the other robot has passed by.
      // If the other robot has indeed passed by, then resume the robot waiting at the bypass point.
      for (int i = 0; i < available_robots_vector.size(); ++i) {
        const std::string& ri = available_robots_vector[i];
        if (robot_status_[ri] == WAITING_AT_BYPASS_POINT) {
          int j = intercepted_robots_[i];
          const std::string& rj = available_robots_vector[j];
          float current_distance = getDistance(robot_locations_[ri], robot_locations_[rj]);
          if (current_distance > min_bypass_distance_[i] + 0.50f) {
            intercepted_robots_.erase(i);
            intercepted_robots_.erase(j);
            resumeRobot(ri);
          }
        }
      }

    }

    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "segbot_logical_translator");
  ros::NodeHandle nh;

  ROS_INFO("MultiRobotNavigator: Starting up node...");
  MultiRobotNavigator handler;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  ROS_INFO("MultiRobotNavigator: Stopping node.");

  return 0;
}
