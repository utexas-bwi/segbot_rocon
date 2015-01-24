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
#include <boost/foreach.hpp>
#include <bwi_mapper/map_inflator.h>
#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/map_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <multi_level_map_msgs/MultiLevelMapData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <segbot_navigation_concert/AvailableRobotArray.h>
#include <std_srvs/Empty.h>

enum RobotStatus {
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

    void availableRobotArrayHandler(const segbot_navigation_concert::AvailableRobotArray::ConstPtr& robots);
    ros::Subscriber available_robots_subscriber_;
    std::set<std::string> available_robots_;

    // TODO Assumes a single floor for now.
    nav_msgs::OccupancyGrid original_map_;
    nav_msgs::OccupancyGrid inflated_map_;

    std::map<std::string, RobotStatus> robot_status_;
    void robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, std::string robot_name);
    std::map<std::string, ros::Subscriber> robot_locations_sub_;
    std::map<std::string, geometry_msgs::PoseStamped> robot_locations_;
    void robotPathHandler(const nav_msgs::Path::ConstPtr& path, std::string robot_name);
    std::map<std::string, ros::Subscriber> robot_path_sub_;
    std::map<std::string, nav_msgs::Path> global_plan_;
    std::map<std::string, nav_msgs::OccupancyGrid> expanded_plan_;
    std::map<std::string, RobotControllerPtr> robot_controller_;
    std::map<std::string, ros::ServiceClient> robot_resumer_;
    std::map<std::string, ros::ServiceClient> robot_pauser_;

    float inflation_radius_;

    std::map<int, int> intercepted_robots_;
    std::map<int, float> min_bypass_distance_;

    /* Some helper functions. */
    void pauseRobot(const std::string& robot_name, 
                    RobotStatus post_success_status = WAITING_FOR_OTHER_ROBOT);

    void resumeRobot(const std::string& robot_name);

    void sendNavigationGoal(const std::string& robot_name, 
                            const geometry_msgs::PoseStamped& pose,
                            RobotStatus post_send_status = NORMAL);
    bool hasNavigationActionCompleted(const std::string& robot_name);
    void expandPlan(const nav_msgs::Path& plan, nav_msgs::OccupancyGrid& expanded_plan);
    bool plansOverlap(const nav_msgs::OccupancyGrid& p1, const nav_msgs::OccupancyGrid& p2);
    float getDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
    geometry_msgs::PoseStamped calculateBypassPoint(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

};

MultiRobotNavigator::MultiRobotNavigator() : multimap_available_(false) {

  ros::param::param("~inflation_radius", inflation_radius_, 0.3f);

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

void MultiRobotNavigator::robotLocationHandler(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, std::string robot_name) {
  geometry_msgs::PoseStamped pose_only;
  pose_only.header = pose->header;
  pose_only.pose = pose->pose.pose;
  robot_locations_[robot_name] = pose_only;
}

void MultiRobotNavigator::expandPlan(const nav_msgs::Path& plan, nav_msgs::OccupancyGrid& expanded_plan) {

  // Create an opencv image and draw circles with the inflation radius using the plan.
  cv::Mat temp(original_map_.info.width, original_map_.info.height, CV_8UC1, cv::Scalar(100));

  BOOST_FOREACH(const geometry_msgs::PoseStamped& pose, plan.poses) {
    bwi_mapper::Point2f real_world_point(pose.pose.position.x, pose.pose.position.y);
    bwi_mapper::Point2f grid_pos = bwi_mapper::toGrid(real_world_point, original_map_.info);
    int grid_radius = inflation_radius_ / original_map_.info.resolution;
    cv::circle(temp, grid_pos, grid_radius, cv::Scalar(0));
  }

  // Copy over data from the image.
  // Setup a clear expanded plan.
  expanded_plan.info = original_map_.info;
  expanded_plan.data.resize(original_map_.data.size());

  int counter = 0;
  for(int row = 0; row < original_map_.info.height; row++) {
    const unsigned char* row_ptr = temp.ptr<unsigned char>(row);
    for(int col = 0; col < original_map_.info.width; col++) {
      expanded_plan.data[counter] = row_ptr[col];
      ++counter;
    }
  }
}

bool MultiRobotNavigator::plansOverlap(const nav_msgs::OccupancyGrid& p1, const nav_msgs::OccupancyGrid& p2) {
  // TODO could do a counter here instead of returning on a single pixel overlap.
  for (int val = 0; val < p1.data.size(); ++val) {
    if (p1.data[val] == 0 && p2.data[val] == 0) {
      return true;
    }
  }
  return false;
}

void MultiRobotNavigator::robotPathHandler(const nav_msgs::Path::ConstPtr& path, std::string robot_name) {
  global_plan_[robot_name] = *path;
  expandPlan(global_plan_[robot_name], expanded_plan_[robot_name]);
}

void MultiRobotNavigator::availableRobotArrayHandler(const segbot_navigation_concert::AvailableRobotArray::ConstPtr& robots) {
  BOOST_FOREACH(const std::string resource_str, robots->robot_name) {
    std::vector<std::string> results;
    boost::split(results, resource_str, boost::is_any_of("/"));
    std::string robot_name = results[2];
    available_robots_.insert(robot_name);
  }
}

void MultiRobotNavigator::pauseRobot(const std::string& robot_name, 
                                     RobotStatus post_success_status) {
  std_srvs::Empty empty_srv;
  if (robot_pauser_[robot_name].call(empty_srv)) {
    robot_status_[robot_name] = post_success_status;
  } else {
    ROS_ERROR_STREAM("Unable to pause " + robot_name + ". Check client logs for error message.");
  }
}

void MultiRobotNavigator::resumeRobot(const std::string& robot_name) {
  std_srvs::Empty empty_srv;
  if (robot_resumer_[robot_name].call(empty_srv)) {
    robot_status_[robot_name] = NORMAL;
  } else {
    ROS_ERROR_STREAM("Unable to resume " + robot_name + ". Check client logs for error message.");
  }
}

void MultiRobotNavigator::sendNavigationGoal(const std::string& robot_name, 
                                             const geometry_msgs::PoseStamped& pose,
                                             RobotStatus post_send_status) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;
  robot_controller_[robot_name]->sendGoal(goal);
  robot_status_[robot_name] = post_send_status;
}

bool MultiRobotNavigator::hasNavigationActionCompleted(const std::string& robot_name) {
  actionlib::SimpleClientGoalState state = robot_controller_[robot_name]->getState();
  return state != actionlib::SimpleClientGoalState::PENDING && state != actionlib::SimpleClientGoalState::ACTIVE;
}

float MultiRobotNavigator::getDistance(const geometry_msgs::PoseStamped& p1, 
                                      const geometry_msgs::PoseStamped& p2) {
  float xdiff = p1.pose.position.x - p2.pose.position.x;
  float ydiff = p1.pose.position.y - p2.pose.position.y;
  return xdiff * xdiff + ydiff * ydiff;
}

geometry_msgs::PoseStamped MultiRobotNavigator::calculateBypassPoint(const geometry_msgs::PoseStamped& p1, 
                                                                     const geometry_msgs::PoseStamped& p2) {

  float xdiff = p2.pose.position.x - p1.pose.position.x;
  float ydiff = p2.pose.position.y - p1.pose.position.y;
  float angle = atan2f(ydiff, xdiff);
  float angle_right = angle - M_PI/2;

  float padding = 0.1f;
  float counter = padding;

  geometry_msgs::PoseStamped ret = p1;
  while (counter < padding + 2 * inflation_radius_) {
    float xloc = p1.pose.position.x + counter * cosf(angle_right); 
    float yloc = p1.pose.position.y + counter * sinf(angle_right); 
    bwi_mapper::Point2f real_world_point(xloc, yloc);
    bwi_mapper::Point2f grid_pos = bwi_mapper::toGrid(real_world_point, original_map_.info);
    long map_idx = MAP_IDX(original_map_.info.width, grid_pos.x, grid_pos.y);
    if (inflated_map_.data[map_idx] == 0) {
      break;
    }
    ret.pose.position.x = xloc;
    ret.pose.position.y = yloc;
  }
  
  return ret;
}

void MultiRobotNavigator::spin() {
  ros::Rate r(10);
  while (ros::ok()) {

    // Create controllers for each robot.
    BOOST_FOREACH(const std::string robot, available_robots_) {
      if (robot_status_.find(robot) == robot_status_.end()) {
        ROS_INFO_STREAM("Adding robot " << robot << " to monitored robots.");
        // Setup all the subcribers
        ros::NodeHandle nh("/");
        robot_locations_sub_[robot] = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robot + "/amcl_pose", 1,
                                                   boost::bind(&MultiRobotNavigator::robotLocationHandler, this, _1,
                                                               robot)); 
        robot_path_sub_[robot] = nh.subscribe<nav_msgs::Path>(robot + "/move_base/EBandPlannerROS/global_plan", 1,
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
        if (robot_status_[ri] == NORMAL) {
          for (int j = i + 1; j < available_robots_vector.size(); ++j) {
            const std::string& rj = available_robots_vector[j];
            if (robot_status_[rj] == NORMAL) { 
              // Compare the expanded plans for any collision.
              if (plansOverlap(expanded_plan_[ri], expanded_plan_[rj])) {

                ROS_INFO_STREAM("Robots " << ri << " and " << rj << "are about to collide!");

                // Let's pause both the robots. 
                pauseRobot(ri);
                pauseRobot(rj);

                // See which robot can be moved the most on the perpendicular.
                geometry_msgs::PoseStamped bypassi = calculateBypassPoint(robot_locations_[ri], robot_locations_[rj]);
                geometry_msgs::PoseStamped bypassj = calculateBypassPoint(robot_locations_[rj], robot_locations_[ri]);

                float bypassdistancei = getDistance(robot_locations_[ri], bypassi);
                float bypassdistancej = getDistance(robot_locations_[rj], bypassj);

                if (bypassdistancei > bypassdistancej) {
                  // Divert robot i to the bypass point.
                  ROS_INFO_STREAM("Sending " << ri << " to bypass point, and pausing robot " << rj << ".");
                  sendNavigationGoal(ri, bypassi, HEADING_TO_BYPASS_POINT);
                } else {
                  ROS_INFO_STREAM("Sending " << rj << " to bypass point, and pausing robot " << ri << ".");
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
          // TODO if navigation action fails, should probably attempt to restart it with a counter
          if (hasNavigationActionCompleted(ri)) {
            int j = intercepted_robots_[i];
            const std::string& rj = available_robots_vector[j];
            robot_status_[ri] = WAITING_AT_BYPASS_POINT;
            min_bypass_distance_[i] = getDistance(robot_locations_[ri],
                                                  robot_locations_[rj]);
            ROS_INFO_STREAM("Robot " << ri << " has reached bypass point, resuming robot " << rj << " to normal goal.");
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
            ROS_INFO_STREAM("Robot " << rj << " has crossed bypass point, resuming robot " << ri << " to normal goal.");
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
  
  ros::init(argc, argv, "multi_robot_navigator");
  ros::NodeHandle nh;

  MultiRobotNavigator handler;
  handler.spin();

  return 0;
}
