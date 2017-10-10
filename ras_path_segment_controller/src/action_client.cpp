#include <ros/ros.h>
#include <ras_path_segment_controller/GoToPoseAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
 
typedef actionlib::SimpleActionClient<ras_path_segment_controller::GoToPoseAction> Client;

double x, y, theta;

int main(int argc, char** argv)
{
   ros::init(argc, argv, "action_client");
   ros::NodeHandle nh;
   nh.param("x", x, 1.0);
   nh.param("y", y, 0.0);
   nh.param("theta", theta, 0.0);
   Client client("GoToPose", true); // true -> don't need ros::spin()
   client.waitForServer();
   ras_path_segment_controller::GoToPoseGoal goal;
   geometry_msgs::PoseStamped ps;
   ps.pose.position.x = x;
   ps.pose.position.y = y;
   ps.pose.orientation.z = sin(0.5 * theta);
   ps.pose.orientation.w = cos(0.5 * theta);
   goal.goal_pose = ps;
   // Fill in goal here
   client.sendGoal(goal);
   client.waitForResult(ros::Duration(30.0));
   if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      printf("Yay! The dishes are now clean");
   printf("Current State: %s\n", client.getState().toString().c_str());
   return 0;
}
