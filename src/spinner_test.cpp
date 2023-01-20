#include <ros/ros.h>
#include <std_msgs/String.h>

class SpinnerTest
{
public:
  SpinnerTest(ros::NodeHandle &nh)
  {
    nh_ = nh;
    initSubs();
  }
protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Subscriber sub4;
  ros::Subscriber sub5;
  const std_msgs::String *string_msg_ptr_ {nullptr};

  void initSubs()
  {
    sub1 = nh_.subscribe("/set_string_topic", 1, &SpinnerTest::setStringCb, this);
    sub2 = nh_.subscribe("/handle_string_topic", 1, &SpinnerTest::handleStringCb, this);
  }

  void setStringCb(const std_msgs::String &msg)
  {
    ROS_INFO_STREAM("Received string msg: " << msg.data);
    ROS_INFO("Setting ptr");
    string_msg_ptr_ = &msg;
  }

  void handleStringCb(const std_msgs::String &msg)
  {
    ROS_INFO("Handling cb if string_msg_ptr_ not null");
    if (string_msg_ptr_)
      ROS_INFO_STREAM("string_msg_ptr_ not null. value: " << *string_msg_ptr_);
    else
      ROS_INFO("string_msg_ptr_ null");
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "spinner_test");
  ros::NodeHandle nh;

  SpinnerTest st(nh);


  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}
