#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ras_lab1_msgs/ADConverter.h>

class WallFollowingController
{
private:
    const double kp = 0.05;
    const double v = 0.2;
public:
    ros::NodeHandle nh;
    ros::Subscriber adc_sub;
    ros::Publisher twist_pub;

    ras_lab1_msgs::ADConverter adc;

    WallFollowingController()
    {
        adc_sub = nh.subscribe("/kobuki/adc", 1, &WallFollowingController::AdcCallback, this);

        twist_pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    }

    void AdcCallback(const ras_lab1_msgs::ADConverter::ConstPtr &msg)
    {
        adc = *msg;
    }

    void UpdatePWM()
    {
        geometry_msgs::Twist twist;
        twist.linear.x = v;
        twist.angular.z = kp*(adc.ch1 - adc.ch2);

        twist_pub.publish(twist);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_following_controller");
    WallFollowingController wfc;
    
    ros::Rate loop_rate(10);
    while(wfc.nh.ok())
    {
        wfc.UpdatePWM();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
