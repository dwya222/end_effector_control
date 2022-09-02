#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
//#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>


class RobotArm
{
private:
  robot_model_loader::RobotModelLoaderPtr rm_loader_ptr;
  const robot_model::RobotModelPtr& kinematic_model_ptr;
  //planning_scene::PlanningScenePtr planning_scene_ptr;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr;
  kinematic_constraints::KinematicConstraintSetPtr kinematic_constraint_set_ptr;
  const std::string group;
  ros::NodeHandle nh_;
  ros::Publisher joint_position_pub;
  moveit::core::RobotState& current_state;

public:
  RobotArm(ros::NodeHandle nh) :
    rm_loader_ptr(std::make_shared<robot_model_loader::RobotModelLoader>("robot_description")),
    kinematic_model_ptr(rm_loader_ptr->getModel()),
    //planning_scene_ptr(std::make_shared<planning_scene::PlanningScene>(kinematic_model_ptr)),
    psm_ptr(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(rm_loader_ptr)),
    group("panda_arm"), current_state(psm_ptr->getPlanningScene()->getCurrentStateNonConst())
  {
    psm_ptr->startSceneMonitor();
    psm_ptr->startWorldGeometryMonitor();
    psm_ptr->startStateMonitor();
    bool success = psm_ptr->requestPlanningSceneState("/get_planning_scene");
    kinematic_constraint_set_ptr = std::make_shared<kinematic_constraints::KinematicConstraintSet>(kinematic_model_ptr);
    nh_ = nh;
    joint_position_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 100);
  }

  /* ~RobotArm() */
  /* { */
  /*   free(memory_); */
  /* } */

  bool isSelfCollision()
  {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    // planning_scene_ptr->checkSelfCollision(collision_request, collision_result);
    psm_ptr->getPlanningScene()->checkSelfCollision(collision_request, collision_result);
    return collision_result.collision;
  }

  bool isCollision()
  {
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    // planning_scene_ptr->checkCollision(collision_request, collision_result, current_state);
    psm_ptr->getPlanningScene()->checkCollision(collision_request, collision_result, current_state);
    return collision_result.collision;
  }

  bool isEnvCollision()
  {
    // Note this will return false if the robot is in collision with both itself and the environment
    return (isCollision() && !isSelfCollision());
  }

  bool satisfiesBounds()
  {
    return current_state.satisfiesBounds(kinematic_model_ptr->getJointModelGroup(group));
  }

  bool isStateValid()
  {
    /* return planning_scene_ptr->isStateValid(current_state, *kinematic_constraint_set_ptr, group); */
    return psm_ptr->getPlanningScene()->isStateValid(current_state, *kinematic_constraint_set_ptr, group);
  }

  void setStartState()
  {
    std::vector<double> start_positions = { 0, -0.785, 0, -2.356, 0, 1.571, 0.785 };
    setCurrentState(start_positions);
  }

  void setCurrentState(std::vector<double> joint_positions)
  {
    current_state.setJointGroupPositions(kinematic_model_ptr->getJointModelGroup(group), joint_positions);
    updateJointStateTopic();
  }

  void setCurrentStateRandom()
  {
    current_state.setToRandomPositions();
    updateJointStateTopic();
  }

  void updateJointStateTopic()
  {
    moveit_msgs::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(current_state, robot_state_msg);

    joint_position_pub.publish(robot_state_msg.joint_state);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_check_test");
  ros::NodeHandle nh("/collision_check_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RobotArm robot_arm(nh);

  char key = ' ';
  while (ros::ok())
  {
    std::cout << "Press q to quit or any other key to run next random state\n";
    std::cin >> key;
    if (key == 'q')
      break;
    else if (key == 's')
      robot_arm.setStartState();
    else
      robot_arm.setCurrentStateRandom();
    ROS_INFO_STREAM("self collision: " << robot_arm.isSelfCollision());
    ROS_INFO_STREAM("environment collision: " << robot_arm.isEnvCollision());
    ROS_INFO_STREAM("collision: " << robot_arm.isCollision());
    ROS_INFO_STREAM("within bounds: " << robot_arm.satisfiesBounds());
    ROS_INFO_STREAM("valid: " << robot_arm.isStateValid());
  }


}
