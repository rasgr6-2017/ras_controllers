#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <ras_path_segment_controller/GoToPoseAction.h>

ras_path_segment_controller::GoToPoseFeedback feedback;
geometry_msgs::PoseStamped goal_pose, start_pose, current_pose;
ros::Subscriber pose_sub;
ros::Publisher twist_pub;

double theta_goal, theta_start, phi, theta_before_end_turn;
double v_max, w_max, kp, x_vmax = 0.05, x_wmax = 0.05;
double e;
double r;

bool turn_start_complete = true, translation_complete = true, turn_end_complete = true;

typedef actionlib::SimpleActionServer<ras_path_segment_controller::GoToPoseAction> Server;

void GetDirectionAndDistance(geometry_msgs::Pose start, geometry_msgs::Pose goal){
    double x1 = start.position.x;
    double y1 = start.position.y;
    double x2 = goal.position.x;
    double y2 = goal.position.y;
    phi = atan2(y2-y1, x2-x1);
    r = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

void GoalCallback(Server *s){
    goal_pose = s->acceptNewGoal()->goal_pose;
    theta_goal = 2 * asin(goal_pose.pose.orientation.z);
    start_pose = current_pose;
    theta_start = 2 * asin(start_pose.pose.orientation.z);
    GetDirectionAndDistance(start_pose.pose, goal_pose.pose);

    turn_start_complete = translation_complete = turn_end_complete = false;
}

void PreemptCallback(Server *s){
    s->setPreempted();
}

void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    current_pose = *msg;
}

bool PoseEquals(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2){
    double y1 = 2.0 * asin(p1.pose.orientation.z);
    double y2 = 2.0 * asin(p2.pose.orientation.z);
    if(fabs(p1.pose.position.x - p2.pose.position.x) < e &&
            fabs(p1.pose.position.y - p2.pose.position.y) < e &&
            fabs(y1 - y2) < e
            )
        return true;
    return false;
}

bool Equal(double a, double b){
    return fabs(a - b)<e;
}

double GetVel(double x_actual, double x_final, double x_start, double ramp_length, double vel_max){
    double vel;
    double C = vel_max / ramp_length;
    vel = fabs(x_actual-x_start) < 0.5 * fabs(x_final-x_start) ? vel_max : C * fabs(x_final - x_actual);
    vel = vel > vel_max ? vel_max : vel;
    vel = x_final - x_actual > 0 ? vel : -vel;
    return vel;
}

void UpdateRobotTwist(){
    double v = 0.0, w = 0.0;
    double x = current_pose.pose.position.x;
    double y = current_pose.pose.position.y;
    double theta = 2 * asin(current_pose.pose.orientation.z);

    double d = sqrt((x - start_pose.pose.position.x)*(x - start_pose.pose.position.x)
                    + (y - start_pose.pose.position.y)*(y - start_pose.pose.position.y));

    if(!turn_end_complete){
        if(!translation_complete){
            if(!turn_start_complete){
                if(!Equal(theta, phi)){
                    w = GetVel(theta, phi, theta_start, x_wmax, w_max);
                }
                else{
                    turn_start_complete = true;
                }
            }
            else{
                if(!Equal(d, r)){
                    v = GetVel(d, r, 0.0, x_vmax, 0.5*v_max);
                }
                else{
                    theta_before_end_turn = theta;
                    translation_complete = true;
                }
            }
        }
        else{
            if(turn_start_complete){
                if(!Equal(theta, theta_goal)){
                    w = GetVel(theta, theta_goal, theta_before_end_turn, x_wmax, w_max);
                }
                else{
                    turn_end_complete = true;
                }
            }
        }
    }

    geometry_msgs::Twist twist;
    twist.linear.x = v;
    twist.angular.z = w;
    twist_pub.publish(twist);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "path_segment_controller");
    ros::NodeHandle nh;
    int frequency;
    nh.param("server_frequency", frequency, 10);
    nh.param("v_max", v_max, 0.3);
    nh.param("w_max", w_max, 0.3);
    nh.param("kp_vw", kp, 30.0);
    nh.param("e", e, 0.01);

    pose_sub = nh.subscribe("localization/odometry_pose", 1, &PoseCallback);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);

    Server server(nh, "GoToPose", false);
    server.registerGoalCallback(boost::bind(&GoalCallback, &server));
    server.registerPreemptCallback(boost::bind(&PreemptCallback, &server));
    server.start();

    ros::Rate loop_rate(frequency);
    while(nh.ok())
    {
        if(server.isActive()){
            feedback.feedback_pose = current_pose;
            server.publishFeedback(feedback);
            if(PoseEquals(current_pose, goal_pose))
                server.setSucceeded();
        }
        UpdateRobotTwist();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
