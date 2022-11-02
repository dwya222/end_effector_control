#include <ros/ros.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> TrajectoryClient;

class RobotArm
{
private:
  // Should definitely use make_unique here
  TrajectoryClient* trajectory_client_;

public:
  RobotArm()
  {
    trajectory_client_ = new TrajectoryClient(
        "/execute_trajectory", true);

    while (!trajectory_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the position_joint_trajectory_controller action server");
    }
  }

  ~RobotArm()
  {
    // If using make_unique I don't have to delete this anymore
    delete trajectory_client_;
  }

  void appendJointGoal(moveit_msgs::ExecuteTrajectoryGoal &goal, int index, std::vector<double> positions, double time_from_start)
  {
    if (index == 0)
    {
      goal.trajectory.joint_trajectory.joint_names.push_back("panda_joint1");
      goal.trajectory.joint_trajectory.joint_names.push_back("panda_joint2");
      goal.trajectory.joint_trajectory.joint_names.push_back("panda_joint3");
      goal.trajectory.joint_trajectory.joint_names.push_back("panda_joint4");
      goal.trajectory.joint_trajectory.joint_names.push_back("panda_joint5");
      goal.trajectory.joint_trajectory.joint_names.push_back("panda_joint6");
      goal.trajectory.joint_trajectory.joint_names.push_back("panda_joint7");
    }
    goal.trajectory.joint_trajectory.points.resize(index + 1);
    // positions
    goal.trajectory.joint_trajectory.points[index].positions.resize(7);
    goal.trajectory.joint_trajectory.points[index].positions[0] = positions[0];
    goal.trajectory.joint_trajectory.points[index].positions[1] = positions[1];
    goal.trajectory.joint_trajectory.points[index].positions[2] = positions[2];
    goal.trajectory.joint_trajectory.points[index].positions[3] = positions[3];
    goal.trajectory.joint_trajectory.points[index].positions[4] = positions[4];
    goal.trajectory.joint_trajectory.points[index].positions[5] = positions[5];
    goal.trajectory.joint_trajectory.points[index].positions[6] = positions[6];
    // velocities
    goal.trajectory.joint_trajectory.points[index].velocities.resize(7);
    for (size_t j = 0; j < 7; j++)
    {
      goal.trajectory.joint_trajectory.points[index].velocities[j] = 0.0;
    }
    goal.trajectory.joint_trajectory.points[index].time_from_start = ros::Duration(time_from_start);

  }

  void startTrajectory(moveit_msgs::ExecuteTrajectoryGoal goal)
  {
    goal.trajectory.joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    trajectory_client_->sendGoal(goal);
  }

  moveit_msgs::ExecuteTrajectoryGoal getStartTrajectory()
  {
    moveit_msgs::ExecuteTrajectoryGoal goal;

    std::vector<double> positions { 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785 };
    double time_from_start = 0.0;
    appendJointGoal(goal, 0, positions, time_from_start);

    return goal;
  }


  moveit_msgs::ExecuteTrajectoryGoal getTrajectory()
  {
    moveit_msgs::ExecuteTrajectoryGoal goal = getStartTrajectory();

    std::vector<double> positions { 0.252, -0.454, -0.936, -2.078, 1.529, 2.555, 1.966 };
    double time_from_start = 2.0;
    appendJointGoal(goal, 1, positions, time_from_start);

    positions = { 0.252, -0.454, -0.936, -2.078, 1.529, 2.555, 1.966 };
    time_from_start = 3.0;
    appendJointGoal(goal, 2, positions, time_from_start);

    positions = { -0.546, 0.019, -0.707, -2.150, 1.762, 1.825, 1.739 };
    time_from_start = 4.0;
    appendJointGoal(goal, 3, positions, time_from_start);

    positions = { 0.001, -0.786, 0.000, -2.356, 0.001, 1.572, 0.786 };
    time_from_start = 6.0;
    appendJointGoal(goal, 4, positions, time_from_start);

    return goal;
  }

  actionlib::SimpleClientGoalState getState()
  {
    return trajectory_client_->getState();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_driver");
  RobotArm arm;

  // Run through full trajectory uninterrupted
  arm.startTrajectory(arm.getTrajectory());
  while (!arm.getState().isDone() && ros::ok())
  {
    usleep(10000);
  }

  // Interrupt the trajectory
  /* arm.startTrajectory(arm.getTrajectory()); */

  /* ros::Duration(1.0).sleep(); */
  /* //ros::Duration(0.5).sleep(); */
  /* arm.startTrajectory(arm.getStartTrajectory()); */

  /* while (!arm.getState().isDone() && ros::ok()) */
  /* { */
  /*   usleep(10000); */
  /* } */
  return 0;
}

