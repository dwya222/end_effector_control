#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>
#include <ros/serialization.h>

class ContinuousCollisionChecker
{
protected:
  robot_model_loader::RobotModelLoaderPtr rm_loader_ptr;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr;
  ros::NodeHandle nh_;
  moveit::core::RobotState& current_state;
  ros::Subscriber current_path_sub, executing_to_state_sub, updated_planning_scene_sub;
  ros::Publisher edge_clear_pub;
  // may want to make this a deque so it's easy to pop off the front position when we advance states
  std::deque<std::vector<double>> current_path_;
  bool edge_clear_;

public:
  ContinuousCollisionChecker(ros::NodeHandle nh) :
    rm_loader_ptr(std::make_shared<robot_model_loader::RobotModelLoader>("robot_description")),
    psm_ptr(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(rm_loader_ptr)),
    current_state(psm_ptr->getPlanningScene()->getCurrentStateNonConst())
    {
      psm_ptr->startSceneMonitor();
      psm_ptr->startWorldGeometryMonitor();
      psm_ptr->startStateMonitor();
      if (psm_ptr->requestPlanningSceneState("/get_planning_scene"))
        ROS_INFO("Successfully retrieved planning scene state");
      else
        ROS_WARN("Unseccessful request to get planning scene state");
      nh_ = nh;
      edge_clear_ = false;
      initSubsPubs();
    }

  void initSubsPubs()
  {
    // Planner publishes when a new solution path is found
    current_path_sub = nh_.subscribe("/current_path", 1, &ContinuousCollisionChecker::currentPathCb, this);
    // Controller publishes progress
    executing_to_state_sub = nh_.subscribe("/executing_to_state", 1, &ContinuousCollisionChecker::executingToStateCb,
                                           this);
    // Move group publishes updates when the environment changes
    updated_planning_scene_sub = nh_.subscribe("/move_group/monitored_planning_scene", 1,
                                               &ContinuousCollisionChecker::planningSceneCb, this);
    // Latched topic that will notify when the next edge in the current path is clear
    edge_clear_pub = nh_.advertise<std_msgs::Bool>("/edge_clear", 1);//, true);// latched
  }

  void currentPathCb(const trajectory_msgs::JointTrajectory& current_path)
  {
    ROS_INFO("Collision detector recieved new path");
    current_path_.clear();

    for (int i=0; i < current_path.points.size(); i++)
      current_path_.emplace_back(current_path.points[i].positions);

    edge_clear_ = false;

    checkCollision();
  }

  void executingToStateCb(const trajectory_msgs::JointTrajectoryPoint& next_state)
  {
    ROS_INFO("Collision detector notified the controller is executing to next state");
    // Check to make sure the next state is as expected
    if (next_state.positions != current_path_[1])
    {
      ROS_ERROR("Collision detector: provided unexpected next state");
      return;
    }
    ROS_INFO("Collision detector: provided expected next state");
    current_path_.pop_front();
    edge_clear_ = false;
    checkCollision();
  }

  void planningSceneCb(const moveit_msgs::PlanningScene& scene)
  {
    checkCollision();
  }

  void checkCollision()
  {
    if (current_path_.size() <= 1)
    {
      ROS_WARN("Not checking collision: path to goal not yet set, or on way to goal");
      return;
    }
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    auto planning_scene = psm_ptr->getPlanningScene();
    planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                               true /* exclusive */);
    // Set robot states
    robot_state::RobotState& state = planning_scene->getCurrentStateNonConst();
    robot_state::RobotState& state2 = planning_scene->getCurrentStateNonConst();
    for (int i=0; i<=current_path_[0].size(); i++)
    {
      state.setJointPositions("panda_joint" + std::to_string(i+1), &current_path_[0][i]);
      state2.setJointPositions("panda_joint" + std::to_string(i+1), &current_path_[1][i]);
    }
    state.update();
    state2.update();
    planning_scene->getCollisionEnv()->checkRobotCollision(req, res, state2, state);
    ROS_INFO_STREAM("Collision Detector: " << (res.collision ? "In collision." : "Not in collision."));
    ROS_INFO_STREAM("Collision Detector state1: [" << current_path_[0][0] << ", " << current_path_[0][1] << ", " <<
                    current_path_[0][2] << ", " << current_path_[0][3] << ", " << current_path_[0][4] <<
                    current_path_[0][5] << ", " << current_path_[0][6] << "]");
    ROS_INFO_STREAM("Collision Detector state2: [" << current_path_[1][0] << ", " << current_path_[1][1] << ", " <<
                    current_path_[1][2] << ", " << current_path_[1][3] << ", " << current_path_[1][4] <<
                    current_path_[1][5] << ", " << current_path_[1][6] << "]");
    // Publish collision result
    std_msgs::Bool edge_clear_msg;
    edge_clear_msg.data = !res.collision;
    if (edge_clear_msg.data != edge_clear_)
    {
      edge_clear_ = edge_clear_msg.data;
      edge_clear_pub.publish(edge_clear_msg);
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_check_test");
  ros::NodeHandle nh("/collision_check_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ContinuousCollisionChecker ccc(nh);
  ros::waitForShutdown();
}
