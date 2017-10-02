#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <phidgets/motor_encoder.h>
#include <motor_controller.h>
#include <geometry_msgs/Twist.h>

double encoder_res;
double kp_left;
double ki_left;
double kp_right;
double ki_right;
int frequency;

class MotorController
{
private:
    const double b = 0.242;
    const double r = 0.036;
    double e1_sum = 0;
    double e2_sum = 0;
public:
    ros::NodeHandle nh;
    ros::Subscriber encoder_left_sub, encoder_right_sub, twist_sub;
    ros::Publisher vel_left_pub, vel_right_pub;

    geometry_msgs::Twist twist;
    phidgets::motor_encoder encoder_left, encoder_right;

    MotorController()
    {
    	nh.param("kp_left", kp_left, 0.0);
    	nh.param("ki_left", ki_left, 0.0);
    	nh.param("kp_right", kp_right, 0.0);
    	nh.param("ki_right", ki_right, 0.0);
    	nh.param("encoder_res", encoder_res, 0.0);
    	nh.param("frequency", frequency, 10);
    
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

        double w1 = -((double)(encoder_right.count_change)) * 2 * 3.1415 / encoder_res * frequency;
        double w2 = ((double)(encoder_left.count_change)) * 2 * 3.1415 / encoder_res * frequency;

        double e1 = w1_ref - w1;
        double e2 = w2_ref - w2;
        ROS_INFO("w1 ref: %f, w1: %f", w1_ref, w1);
        ROS_INFO("w2 ref: %f, w2: %f", w2_ref, w2);

        e1_sum += e1 * 1.0/(double)frequency;
        e2_sum += e2 * 1.0/(double)frequency;

        double u1 = kp_right * e1 + ki_right * e1_sum;
        double u2 = kp_left * e2 + ki_left * e2_sum;

		std_msgs::Float32 f1, f2;
		f1.data = -(float)u1;
		f2.data = (float)u2;
        vel_right_pub.publish(f1);
        vel_left_pub.publish(f2);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    MotorController mc;
	
    ros::Rate loop_rate(frequency);
    while(mc.nh.ok())
    {
        mc.UpdateMotorControl();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
