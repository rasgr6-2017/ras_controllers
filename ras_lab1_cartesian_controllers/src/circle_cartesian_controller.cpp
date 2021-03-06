#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "circle_cartesian_controller");
    ros::NodeHandle nh("~");
    std::string twist_topic;
    nh.param<std::string>("twist_topic", twist_topic, "/motor_controller/twist");
    
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(twist_topic.c_str(), 1);
    
    double R = 0.5;
    double T = 10.0;

    geometry_msgs::Twist twist;
    twist.linear.x = 2.0*3.1415*R/T;
    twist.angular.z = 2.0*3.1415/T;

    ros::Rate loop_rate(10);
    while(nh.ok())
    {
        twist_pub.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
