#include <actionlib/client/simple_action_client.h>
#include "turtleline/turtleMoveAction.h"
#include <turtlesim/Pose.h> 
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h> 

bool isWorkOut = true;  //是否完成点位的标志

typedef actionlib::SimpleActionClient<turtleline::turtleMoveAction> Client;
struct Myturtle
{
    float x;
    float y;
    float theta;
}turtle_present_pose;
 
// 当action完成后会调用该回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const turtleline::turtleMoveResultConstPtr& result)
{
    if(state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("Yay! The turtleMove is finished!");
        isWorkOut = true;
    }
}
 
// 当action激活后会调用该回调函数一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}
 
// 收到feedback后调用该回调函数
void feedbackCb(const turtleline::turtleMoveFeedbackConstPtr& feedback)
{
    ROS_INFO(" present_pose : %f  %f  %f", feedback->present_turtle_x, feedback->present_turtle_y,feedback->present_turtle_theta);
}
 
int main(int argc, char** argv)
{
    /*******please select one of the following three pose*******/
    //R
    float pose[4][2] = {{5.544445, 8.544445},{6.544445,7.544445},{5.544445, 6.544445},{6.544445, 5.544445}};
    //RM
    // float pose[8][2] = {{5.544445, 8.544445},{6.544445,7.544445},{5.544445, 6.544445},{6.544445, 5.544445},{7.04445,8.54445},{7.54445,5.54445},{7.84445,8.54445},{8.54445,5.54445}};
    //quandrangle
    // float pose[4][2] = {{8.544445, 5.544445},{8.544445,8.544445},{5.544445, 8.544445},{5.544445, 5.544445}};

    int numPose = sizeof(pose) / sizeof(float*);
    
    ros::init(argc, argv, "turtleMoveClient");
 
    Client client("turtleMove", true);
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    
    int count = 0;
    while(ros::ok())
    {
        if(isWorkOut && count < numPose)
        {
            // 创建action的goal
            turtleline::turtleMoveGoal goal;
            goal.turtle_target_x = pose[count][0];
            goal.turtle_target_y = pose[count][1];
            goal.turtle_target_theta = 0;
        
            // 发送action的goal给服务器端，并且设置回调函数
            client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);
            isWorkOut = false;
            count+=1;
        }
        ros::spinOnce();
    }
    
    return 0;
}
