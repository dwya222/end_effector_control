#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>

#include <memory>
#include <string>


int main(int argc, char** argv) {
  ros::init(argc, argv, "create_goal_test");
  ros::NodeHandle nh("/create_goal_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::string planning_group = "panda_arm";
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group);
  const moveit::core::RobotModelConstPtr robot_model = move_group_interface->getRobotModel();

  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager(
      new constraint_samplers::ConstraintSamplerManager());
  ompl_interface::PlanningContextManager context_manager(robot_model, constraint_sampler_manager);


  // can get planning context spec info from the ompl_interface
  ompl_interface::OMPLInterface ompl_interface(robot_model, nh);
  const planning_interface::PlannerConfigurationMap pconfig_map = ompl_interface.getPlannerConfigurations();
  // We want the second configuration in the pconfig_map ("panda_arm")
  auto it = pconfig_map.begin();
  it++;
  const planning_interface::PlannerConfigurationSettings config = it->second;

  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model, config.group);
  ompl_interface::ModelBasedPlanningContextSpecification context_spec;
  const ompl_interface::ModelBasedStateSpaceFactoryPtr factory(new ompl_interface::JointModelStateSpaceFactory());
  context_spec.config_ = config.config;
  /* context_spec.planner_selector = getPlannerSelector(); // Need planner selector */
  context_spec.constraint_sampler_manager_ = constraint_sampler_manager;
  context_spec.state_space_ = factory->getNewStateSpace(space_spec); // Need factory

  ompl_interface::ModelBasedPlanningContextPtr context = std::make_shared<ompl_interface::ModelBasedPlanningContext>(config.name, context_spec);

  //
  // A lot to shore up here: create realistic pose goal, figure out how to include it in the pose_targets map
  // correspondeng to the end effector link. Figure out what the end effector link is named here, figure out good values
  // for the tolerances...test...
  /* geometry_msgs::PoseStamped pose_goal; */
  /* std::map<std::string, std::vector<geometry_msgs::PoseStamped>> pose_targets; */
  /* pose_targets["hand"] = pose_goal; */
  /* moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints( */
  /*     pose_target.first, pose_target.second, goal_position_tolerance, goal_orientation_tolerance); */
  /* std::vector<moveit_msgs::Constraints> goal_constraints; */
  /* goal_constraints.push_back(goal_constraint); */
  /* moveit_msgs::MoveItErrorCodes error; */
  /* context->setGoalConstraints(goal_constraints, path_constraints, &error) */
  /* ompl::base::GoalPtr goal = context->constructGoal(); */

  return 0;
}
