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
double dead_region = 8.0;
int frequency;
double prev_time = -1;
double prev_left = 0;
double prev_right = 0;
int accumulated_left;
int accumulated_right;

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
    
        encoder_left_sub = nh.subscribe("/motorcontrol/encoder_left", 15, &MotorController::EncoderLeftCallback, this);
        encoder_right_sub = nh.subscribe("/motorcontrol/encoder_right", 15, &MotorController::EncoderRightCallback, this);

        twist_sub = nh.subscribe("/motor_controller/twist", 1, &MotorController::TwistCallback, this);

        vel_right_pub = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel_right", 1);
        vel_left_pub = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel_left", 1);
    }
    
    void EncoderLeftCallback(const phidgets::motor_encoder::ConstPtr &msg)
    {
        encoder_left = *msg;
        accumulated_left += encoder_left.count_change;
    }

    void EncoderRightCallback(const phidgets::motor_encoder::ConstPtr &msg)
    {
        encoder_right = *msg;
        accumulated_right += encoder_right.count_change;
    }

    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        twist = *msg;
    }

    void UpdateMotorControl()
    {
    	double delta_time;
    	
		ROS_INFO("accumulated %d", accumulated_left);
    	
    	if (prev_time < 0)
    	{
    		prev_time = ros::Time::now().toSec();
    		delta_time = 1.0/30.0;
    	}
    	else{
    		delta_time = ros::Time::now().toSec() - prev_time;
    		prev_time = ros::Time::now().toSec();
    	}
    	
        double v = twist.linear.x;
        double w = twist.angular.z;

        double w1_ref = 0.5 * (2 * v + w * b) / r;
        double w2_ref = 0.5 * (2 * v - w * b) / r;

        //double w1 = -((double)(encoder_right.count_change)) * 2 * 3.1415 / encoder_res * frequency;
        //double w2 = ((double)(encoder_left.count_change)) * 2 * 3.1415 / encoder_res * frequency;
        
        double w1 = -((double)(accumulated_right)) * 2 * 3.14159 / (encoder_res * delta_time);
        double w2 = ((double)(accumulated_left)) * 2 * 3.14159 / (encoder_res * delta_time);

		//ROS_INFO("delta time: %f, er: %d, el: %d", delta_time,encoder_right.count_change, encoder_left.count_change);

        double e1 = w1_ref - w1;
        double e2 = w2_ref - w2;
        ROS_INFO("w1 ref: %f, w1: %f", w1_ref, w1);
        ROS_INFO("w2 ref: %f, w2: %f", w2_ref, w2);
        //ROS_INFO("w2 ref: %f, w2: %f", w2_ref, w2);

        e1_sum += e1 * delta_time;
        e2_sum += e2 * delta_time;

		double acc1, acc2;
		acc1 = kp_right * e1 + ki_right * e1_sum;
		acc2 = kp_left * e2 + ki_left * e2_sum;

       	prev_right += acc1;
        prev_left += acc2;
        
        if ( w1_ref < 0.01 && w1_ref > -0.01 && prev_right < dead_region && prev_right > -dead_region)
        { prev_right = 0.0;}
        if ( w2_ref < 0.01 && w2_ref > -0.01 && prev_left < dead_region && prev_left > -dead_region)
        { prev_left = 0.0;}

		std_msgs::Float32 f1, f2;
		f1.data = -(float)prev_right;
		f2.data = (float)prev_left;
        vel_right_pub.publish(f1);
        vel_left_pub.publish(f2);
        ROS_INFO("f1: %f, f2: %f", f1.data, f2.data);
        
        accumulated_left = 0;
    	accumulated_right = 0;
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
