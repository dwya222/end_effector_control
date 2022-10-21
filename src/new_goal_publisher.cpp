#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "new_goal_publisher");
  ros::NodeHandle nh;
  ros::Publisher new_goal_pub = nh.advertise<std_msgs::Float64MultiArray>("new_planner_goal", 100);

  std_msgs::Float64MultiArray new_goal_msg;
  std_msgs::Float64MultiArray new_goal_msg2;
  new_goal_msg.data.resize(7);
  new_goal_msg2.data.resize(7);
  std::vector<double> new_goal_vec {-0.7849918264867694, 0.714459294555375, 2.081170739860527,
                                    -2.13964710731112, -2.1925090777104494, 1.6207882359697081,
                                    -0.6480528889305832};
  std::vector<double> new_goal_vec2 {0.605832, -0.790186, -1.064343, -2.110079, 1.155680,
                                     2.888041, 2.090992};
  for (int i=0; i < new_goal_vec.size(); i++)
  {
    new_goal_msg.data[i] = new_goal_vec[i];
    new_goal_msg2.data[i] = new_goal_vec2[i];
  }

  int pub_count = 0;
  while (ros::ok())
  {
    ROS_INFO("Press Enter to publish another message");
    std::cin.get();
    if (pub_count%2 == 0)
    {
      new_goal_pub.publish(new_goal_msg);
      ROS_INFO("Published msg1");
    }
    else
    {
      new_goal_pub.publish(new_goal_msg2);
      ROS_INFO("Published msg2");
    }
    pub_count++;
  }
}
