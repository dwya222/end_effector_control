#include <iostream>
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
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <robo_demo_msgs/JointTrajectoryPointStamped.h>
#include <robo_demo_msgs/JointTrajectoryPointClearStamped.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>
#include <ros/serialization.h>

class ContinuousCollisionChecker
{
protected:
  robot_model_loader::RobotModelLoaderPtr rm_loader_ptr_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr_;
  ros::NodeHandle nh_;
  moveit::core::RobotState state1_, state2_;
  std::vector<double> positions1_, positions2_;

public:
  ContinuousCollisionChecker(ros::NodeHandle nh) :
    rm_loader_ptr_(std::make_shared<robot_model_loader::RobotModelLoader>("robot_description")),
    psm_ptr_(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(rm_loader_ptr_)),
    state1_(robot_state::RobotState(rm_loader_ptr_->getModel())),
    state2_(robot_state::RobotState(rm_loader_ptr_->getModel()))
  {
    psm_ptr_->startSceneMonitor();
    psm_ptr_->startWorldGeometryMonitor();
    psm_ptr_->startStateMonitor();
    if (psm_ptr_->requestPlanningSceneState("/get_planning_scene"))
      ROS_INFO("Successfully retrieved planning scene state");
    else
      ROS_WARN("Unseccessful request to get planning scene state");
    nh_ = nh;
    positions1_ = { 0.6974779786902082, -1.1247930854123802, -0.9410489571388111,
                   -1.9846089917922924, -0.01871075594262786, 3.102168489791953,
                   -0.037874744378963154};
    positions2_ = { 0.9023001111533858, -1.0006080743470505, -1.7603174792323928, -2.709852560671671,
                   -0.1420301684673516, 3.493584548220631, -0.3473675081746465 };
    for (int i=0; i<=positions1_.size(); i++)
    {
      state1_.setJointPositions("panda_joint" + std::to_string(i+1), &positions1_[i]);
      state2_.setJointPositions("panda_joint" + std::to_string(i+1), &positions2_[i]);
    }
    state1_.update();
    state2_.update();
    for (int i=0; i<7; i++)
      ROS_INFO_STREAM("state 1, joint " << i << ": " << *state1_.getJointPositions("panda_joint" + std::to_string(i+1)));
    for (int j=0; j<7; j++)
      ROS_INFO_STREAM("state 2, joint " << j << ": " << *state2_.getJointPositions("panda_joint" + std::to_string(j+1)));
    state1_.update();
    state2_.update();
    for (int i=0; i<7; i++)
      ROS_INFO_STREAM("state 1, joint " << i << ": " << *state1_.getJointPositions("panda_joint" + std::to_string(i+1)));
    for (int j=0; j<7; j++)
      ROS_INFO_STREAM("state 2, joint " << j << ": " << *state2_.getJointPositions("panda_joint" + std::to_string(j+1)));
  }

  void checkCollision()
  {
    ROS_WARN_THROTTLE(5.0, "Checking collision");
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    auto planning_scene = psm_ptr_->getPlanningScene();
    planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                               true /* exclusive */);
    planning_scene->getCollisionEnv()->checkRobotCollision(req, res, state2_, state1_);
    for (int i=0; i<7; i++)
      ROS_INFO_STREAM("state 1, joint " << i << ": " << *state1_.getJointPositions("panda_joint" + std::to_string(i+1)));
    for (int j=0; j<7; j++)
      ROS_INFO_STREAM("state 2, joint " << j << ": " << *state2_.getJointPositions("panda_joint" + std::to_string(j+1)));

    ROS_INFO_STREAM("Collision Detector state1: [" << positions1_[0] << ", " << positions1_[1] << ", " << positions1_[2]
                    << ", " << positions1_[3] << ", " << positions1_[4] << ", " << positions1_[5] << ", " <<
                    positions1_[6] << "]");
    ROS_INFO_STREAM("Collision Detector state2: [" << positions2_[0] << ", " << positions2_[1] << ", " << positions2_[2]
                    << ", " << positions2_[3] << ", " << positions2_[4] << ", " << positions2_[5] << ", " <<
                    positions2_[6] << "]");
    // Output collision result
    ROS_WARN_STREAM("Clear Result: " << !res.collision);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_check_test");
  ros::NodeHandle nh("/collision_check_test");

  ContinuousCollisionChecker ccc(nh);

  std::string input;
  while (ros::ok())
  {
    input = "";
    std::cout << "Enter anything to check collision\n";
    std::cin >> input;
    ccc.checkCollision();
  }
}
