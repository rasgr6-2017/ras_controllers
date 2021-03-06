#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

double linear_speed = 0.0;
double angular_speed = 0.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_cartesian_controller");
    ros::NodeHandle nh("~");
    std::string twist_topic;
    nh.param<std::string>("twist_topic", twist_topic, "/motor_controller/twist");
    
    nh.getParam("linear_speed", linear_speed);
    nh.getParam("angular_speed", angular_speed);
    ROS_INFO("%f", linear_speed);
    
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(twist_topic.c_str(), 1);
    
    geometry_msgs::Twist twist;
    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;

    ros::Rate loop_rate(10);
    while(nh.ok())
    {
        twist_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
