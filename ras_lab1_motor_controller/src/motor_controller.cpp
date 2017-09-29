#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <motor_controller.h>
#include <geometry_msgs/Twist.h>

class MotorController
{
private:
    const double b = 0.242;
    const double r = 0.036;
    double e1_sum = 0;
    double e2_sum = 0;
    double kp = 12.0;
    double ki = 5.0;
public:
    ros::NodeHandle nh;
    ros::Subscriber encoder_left_sub, encoder_right_sub, twist_sub;
    ros::Publisher vel_left_pub, vel_right_pub;

    geometry_msgs::Twist twist;
    phidgets::motor_encoder encoder_left, encoder_right;

    MotorController()
    {
        encoder_left_sub = nh.subscribe("/motorcontrol/encoder_left", 1, &MotorController::EncoderLeftCallback, this);
        encoder_right_sub = nh.subscribe("/motorcontrol/encoder_right", 1, &MotorController::EncoderRightCallback, this);

        twist_sub = nh.subscribe("/motor_controller/twist", 1, &MotorController::TwistCallback, this);

        vel_right_pub = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel_right", 1);
        vel_left_pub = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel_left", 1);
    }

    void EncoderLeftCallback(const phidgets::motor_encoder::ConstPtr &msg)
    {
        encoder_left = *msg;
    }

    void EncoderRightCallback(const phidgets::motor_encoder::ConstPtr &msg)
    {
        encoder_right = *msg;
    }

    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        twist = *msg;
    }

    void UpdateMotorControl()
    {
        double v = twist.linear.x;
        double w = twist.angular.z;

        double w1_ref = 0.5 * (2 * v + w * b) / r;
        double w2_ref = 0.5 * (2 * v - w * b) / r;

        double w1 = ((double)(encoder_right.count_change)) * 3.1415 / 180.0 * 10.0;
        double w2 = ((double)(encoder_left.count_change)) * 3.1415 / 180.0 * 10.0;

        double e1 = w1_ref - w1;
        double e2 = w2_ref - w2;

        e1_sum += e1 * 0.1;
        e2_sum += e2 * 0.1;

        double u1 = kp * e1 + ki * e1_sum;
        double u2 = kp * e2 + ki * e2_sum;

	std_msgs::Float32 f1, f2;
	f1.data = (float)u1;
	f2.data = (float)u2;
        vel_right_pub.publish(f1);
        vel_left_pub.publish(f2);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    MotorController mc;

    ros::Rate loop_rate(10);
    while(mc.nh.ok())
    {
        mc.UpdateMotorControl();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
