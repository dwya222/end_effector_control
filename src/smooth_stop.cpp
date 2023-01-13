#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <memory>

#define CONTROLLER_TOPIC "/position_joint_trajectory_controller/follow_joint_trajectory"
#define DESIRED_JOINT_STATE_TOPIC "/joint_states_desired"
#define JOINT_STATE_TOPIC "/joint_states"
#define VELOCITY_MULTIPLIER 0.2
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class RobotArm
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber desired_joint_state_sub;
  TrajectoryClient* trajectory_client_;

public:
  RobotArm(ros::NodeHandle &nh)
  {
    nh_ = nh;
    trajectory_client_ = new TrajectoryClient(CONTROLLER_TOPIC, true);
    while (!trajectory_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO_STREAM("Waiting for the " << CONTROLLER_TOPIC << " action server");
    }
  }

  ~RobotArm()
  {
    delete trajectory_client_;
  }

  void stopExecution()
  {
    ROS_WARN("Attempting to stop execution");

    control_msgs::FollowJointTrajectoryGoal goal_msg;
    setStopGoal(goal_msg);
    trajectory_client_->sendGoal(goal_msg);
  }

  void setStopGoal(control_msgs::FollowJointTrajectoryGoal &goal)
  {
    sensor_msgs::JointStateConstPtr desired_joint_state =
      ros::topic::waitForMessage<sensor_msgs::JointState>(DESIRED_JOINT_STATE_TOPIC, nh_);
    std::vector<double> positions = desired_joint_state->position;
    std::vector<double> velocities = desired_joint_state->velocity;

    // Resize msg vectors
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);

    // Time from start by which this state is to be reached
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);

    // Fill msg vectors
    for (int i=0; i<7; i++)
    {
      // Add joint names (if this is the first trajectory)
      goal.trajectory.joint_names.push_back("panda_joint" + std::to_string(i+1));
      // Add positions
      goal.trajectory.points[0].positions[i] = positions[i] + (velocities[i] * VELOCITY_MULTIPLIER);
      // Add velocities (ALL 0)
      goal.trajectory.points[0].velocities[i] = 0.0;
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_stop_test");
  ros::NodeHandle nh("/simple_stop_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  RobotArm arm(nh);

  arm.stopExecution();
}
