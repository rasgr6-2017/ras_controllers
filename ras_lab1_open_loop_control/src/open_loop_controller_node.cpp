#include <ros/ros.h>
#include <ras_lab1_msgs/PWM.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "open_loop_controller_node");

  ros::NodeHandle nh;

  ros::Publisher publisher = nh.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1);

  ros::Rate loop_rate(10.0);

  while(nh.ok())
  {
      ras_lab1_msgs::PWM pwm_message;
      pwm_message.PWM1 = 255;
      pwm_message.PWM2 = 255;
      publisher.publish(pwm_message);

      ros::spinOnce();
      loop_rate.sleep();
  }
}
