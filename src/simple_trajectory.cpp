#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class RobotArm
{
private:
  // Should definitely use make_unique here
  TrajectoryClient* trajectory_client_;

public:
  RobotArm()
  {
    trajectory_client_ = new TrajectoryClient(
        "/position_joint_trajectory_controller/follow_joint_trajectory", true);

    while (!trajectory_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the position_joint_trajectory_controller action server");
    }
  }

  ~RobotArm()
  {
    // If using make_unique I don't have to delete this anymore
    delete trajectory_client_;
  }

  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    trajectory_client_->sendGoal(goal);
  }

  control_msgs::FollowJointTrajectoryGoal getStartTrajectory()
  {
    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("panda_joint1");
    goal.trajectory.joint_names.push_back("panda_joint2");
    goal.trajectory.joint_names.push_back("panda_joint3");
    goal.trajectory.joint_names.push_back("panda_joint4");
    goal.trajectory.joint_names.push_back("panda_joint5");
    goal.trajectory.joint_names.push_back("panda_joint6");
    goal.trajectory.joint_names.push_back("panda_joint7");

    goal.trajectory.points.resize(1);
    // Start joint state
    // positions
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].positions[0] = 0.001;
    goal.trajectory.points[0].positions[1] = -0.786;
    goal.trajectory.points[0].positions[2] = 0.000;
    goal.trajectory.points[0].positions[3] = -2.356;
    goal.trajectory.points[0].positions[4] = 0.001;
    goal.trajectory.points[0].positions[5] = 1.572;
    goal.trajectory.points[0].positions[6] = 0.786;
    // velocities
    goal.trajectory.points[0].velocities.resize(7);
    for (size_t j = 0; j < 7; j++)
    {
      goal.trajectory.points[0].velocities[j] = 0.0;
    }

    // time from start by which this state is to be reached
    goal.trajectory.points[0].time_from_start = ros::Duration(3.0);


    return goal;
  }


  control_msgs::FollowJointTrajectoryGoal getTrajectory()
  {
    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("panda_joint1");
    goal.trajectory.joint_names.push_back("panda_joint2");
    goal.trajectory.joint_names.push_back("panda_joint3");
    goal.trajectory.joint_names.push_back("panda_joint4");
    goal.trajectory.joint_names.push_back("panda_joint5");
    goal.trajectory.joint_names.push_back("panda_joint6");
    goal.trajectory.joint_names.push_back("panda_joint7");

    goal.trajectory.points.resize(3);

    // 1st joint state
    int ind = 0;
    // positions
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.252;
    goal.trajectory.points[ind].positions[1] = -0.454;
    goal.trajectory.points[ind].positions[2] = -0.936;
    goal.trajectory.points[ind].positions[3] = -2.078;
    goal.trajectory.points[ind].positions[4] = 1.529;
    goal.trajectory.points[ind].positions[5] = 2.555;
    goal.trajectory.points[ind].positions[6] = 1.966;
    // velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; j++)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }

    // TODO: change time from start by which this state is to be reached
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    // 2nd joint state
    ind = 1;
    // positions
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = -0.546;
    goal.trajectory.points[ind].positions[1] = 0.019;
    goal.trajectory.points[ind].positions[2] = -0.707;
    goal.trajectory.points[ind].positions[3] = -2.150;
    goal.trajectory.points[ind].positions[4] = 1.762;
    goal.trajectory.points[ind].positions[5] = 1.825;
    goal.trajectory.points[ind].positions[6] = 1.739;
    // velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; j++)
    {
      goal.trajectory.points[ind].velocities[j] = 0.5;
    }

    // TODO: change time from start by which this state is to be reached
    goal.trajectory.points[ind].time_from_start = ros::Duration(4.0);

    // 3rd joint state
    ind = 2;
    // positions
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.001;
    goal.trajectory.points[ind].positions[1] = -0.786;
    goal.trajectory.points[ind].positions[2] = 0.000;
    goal.trajectory.points[ind].positions[3] = -2.356;
    goal.trajectory.points[ind].positions[4] = 0.001;
    goal.trajectory.points[ind].positions[5] = 1.572;
    goal.trajectory.points[ind].positions[6] = 0.786;
    // velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; j++)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }

    // time from start by which this state is to be reached
    goal.trajectory.points[ind].time_from_start = ros::Duration(6.0);


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
  arm.startTrajectory(arm.getTrajectory());

  ros::Duration(1.0).sleep();
  //ros::Duration(0.5).sleep();
  arm.startTrajectory(arm.getStartTrajectory());

  while (!arm.getState().isDone() && ros::ok())
  {
    usleep(10000);
  }
  return 0;
}

