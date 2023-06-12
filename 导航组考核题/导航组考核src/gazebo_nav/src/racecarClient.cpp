#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_actionlib_raecar");

  Client client("move_base", true); 
  client.waitForServer(); 
  ROS_INFO("Server connected. \r\n");

  float goal[4][2] = {{-1.8,7.6},{2.0,7.6},{0,6},{0,0}};
  int numGoal = sizeof(goal) / sizeof(float*);
  int count = 0;
  ros::Rate rate(1);
  while(ros::ok())
  {
      if(client.getState() == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
      {
        ROS_INFO("%s, racecar have reach point %d\n", client.getState().toString().c_str(),count);
        if(count == 4)
        {
            ROS_INFO("Over, We have circled the field!");
            break;
        }
      }

      if((client.getState() == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED || count == 0) && count < numGoal)
      {
          move_base_msgs::MoveBaseGoal action_goal;
          action_goal.target_pose.header.stamp = ros::Time::now();
          action_goal.target_pose.header.frame_id = "map";

          action_goal.target_pose.pose.position.x = goal[count][0];
          action_goal.target_pose.pose.position.y =goal[count][1];
          action_goal.target_pose.pose.position.z =0;

          action_goal.target_pose.pose.orientation.x =0;
          action_goal.target_pose.pose.orientation.y =0;
          action_goal.target_pose.pose.orientation.z =0;
          action_goal.target_pose.pose.orientation.w =1;
          client.sendGoal(action_goal); // Sends a goal to the ActionServer
          count+=1;
          ROS_INFO("Round %d Goal is (%f , %f)",count,action_goal.target_pose.pose.position.x,action_goal.target_pose.pose.position.y);
      }
      rate.sleep();
  }
  return 0;
}