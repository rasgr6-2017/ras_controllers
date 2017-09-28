#include <ros/ros.h>
#include <ras_lab1_msgs/PWM.h>
#include <ras_lab1_msgs/Encoders.h>
#include <motor_controller.h>
#include <geometry_msgs/Twist.h>

class MotorController
{
private:
    const double b = 0.23;
    const double r = 0.0352;
    double e1_sum = 0;
    double e2_sum = 0;
    double kp = 12.0;
    double ki = 5.0;
public:
    ros::NodeHandle nh;
    ros::Subscriber encoders_sub, twist_sub;
    ros::Publisher pwm_pub;

    geometry_msgs::Twist twist;
    ras_lab1_msgs::Encoders encoders;

    MotorController()
    {
        encoders_sub = nh.subscribe("/kobuki/encoders", 1, &MotorController::EncodersCallback, this);

        twist_sub = nh.subscribe("/motor_controller/twist", 1, &MotorController::TwistCallback, this);

        pwm_pub = nh.advertise<ras_lab1_msgs::PWM>("/kobuki/pwm", 1);
    }

    void EncodersCallback(const ras_lab1_msgs::Encoders::ConstPtr &msg)
    {
        encoders = *msg;
    }

    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        twist = *msg;
    }

    void UpdatePWM()
    {
        double v = twist.linear.x;
        double w = twist.angular.z;

        double w1_ref = 0.5 * (2 * v + w * b) / r;
        double w2_ref = 0.5 * (2 * v - w * b) / r;

        double w1 = ((double)(encoders.delta_encoder1)) * 3.1415 / 180.0 * 10.0;
        double w2 = ((double)(encoders.delta_encoder2)) * 3.1415 / 180.0 * 10.0;

        double e1 = w1_ref - w1;
        double e2 = w2_ref - w2;

        e1_sum += e1 * 0.1;
        e2_sum += e2 * 0.1;

        double u1 = kp * e1 + (ki + 1.0) * e1_sum;//left wheel must get higher voltage to match right one's response speed
        double u2 = kp * e2 + ki * e2_sum;

        ras_lab1_msgs::PWM pwm;
        pwm.PWM1 = (int)(u1);
        pwm.PWM2 = (int)(u2);

        pwm_pub.publish(pwm);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    MotorController mc;

    ros::Rate loop_rate(10);
    while(mc.nh.ok())
    {
        mc.UpdatePWM();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
