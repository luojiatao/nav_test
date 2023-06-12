#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "turtleline/turtleMoveAction.h"
#include <turtlesim/Pose.h> 
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

 
struct _pid{
    float SetSpeed;            //定义设定值
    float ActualSpeed;        //定义实际值
    float err;                //定义偏差值
    float err_next;            //定义上一个偏差值
    float err_last;            //定义最上前的偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float p_out,i_out,d_out,total_out;
}pid;

void PID_init(){
    pid.SetSpeed=0.0;
    pid.ActualSpeed=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.err_next=0.0;
    pid.Kp=1;
    pid.Ki=0;
    pid.Kd=0;
}

float PID_realize(float target,float current){
    // pid.SetSpeed=speed;
    // pid.err=pid.SetSpeed-pid.ActualSpeed;
    // float incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
    // pid.ActualSpeed+=incrementSpeed;
    // pid.err_last=pid.err_next;
    // pid.err_next=pid.err;
    // return pid.ActualSpeed;

    pid.SetSpeed=target;
    pid.ActualSpeed = current;
    pid.err = pid.SetSpeed - pid.ActualSpeed;
    pid.p_out = pid.Kp * pid.err;
    if(pid.err < 10)
    {
        pid.i_out += pid.Ki * pid.err;
    }else
    {
        pid.i_out = 0;
    }
    pid.d_out = pid.Kd * (pid.err - pid.err_last);
    pid.err_last = pid.err;
    pid.total_out = pid.p_out + pid.i_out + pid.d_out;
    return pid.total_out;

}

typedef actionlib::SimpleActionServer<turtleline::turtleMoveAction> Server;

struct Myturtle
{
    float x;
    float y;
    float theta;
}turtle_present_pose,turtle_target_pose;
 
ros::Publisher turtle_vel;
 
void posecallback(const turtlesim::PoseConstPtr& msg) 
{ 
  turtle_present_pose.x=msg->x; 
  turtle_present_pose.y=msg->y;
  turtle_present_pose.theta=msg->theta;
 }
 
// 收到action的goal后调用该回调函数
void execute(const turtleline::turtleMoveGoalConstPtr& goal, Server* as)
{
    turtleline::turtleMoveFeedback feedback;
 
    ROS_INFO("TurtleMove is working.");
    turtle_target_pose.x=goal->turtle_target_x;
    turtle_target_pose.y=goal->turtle_target_y; 
    turtle_target_pose.theta=goal->turtle_target_theta;
    
    geometry_msgs::Twist vel_msgs;
    double break_flag;
    bool isTurnComple = false;
    while(ros::ok())
    {  
        ros::Rate r(10);
        if(!isTurnComple)
        {
            vel_msgs.angular.z = 8 * (atan2(turtle_target_pose.y-turtle_present_pose.y,
                turtle_target_pose.x-turtle_present_pose.x)-turtle_present_pose.theta);
        }

        if(fabs(vel_msgs.angular.z - 0) < 0.0000001)
        {
            isTurnComple = true;
        }
        
        //上面角速度接近0说明已转到正确方向,下面再走直线
        if(isTurnComple)
        {
            // vel_msgs.linear.x = PID_realize(sqrt(pow(turtle_target_pose.x-turtle_present_pose.x, 2) +
            //     pow(turtle_target_pose.y-turtle_present_pose.y, 2)),vel_msgs.linear.x);
            vel_msgs.linear.x = 6 * sqrt(pow(turtle_target_pose.x-turtle_present_pose.x, 2) +
                pow(turtle_target_pose.y-turtle_present_pose.y, 2));
        }
        
        turtle_vel.publish(vel_msgs);
 
        //反馈当前位置给Client
        feedback.present_turtle_x=turtle_present_pose.x;
        feedback.present_turtle_y=turtle_present_pose.y;
        feedback.present_turtle_theta=turtle_present_pose.theta;
        as->publishFeedback(feedback);

        ROS_INFO("vel_msgs.angular.z: %f\t vel_msgs.linear.x is %f",vel_msgs.angular.z,vel_msgs.linear.x);
        ROS_INFO("turtle_target_position:(%f,%f,%f)",turtle_target_pose.x,turtle_target_pose.y,turtle_target_pose.theta);
        ROS_INFO("turtle_present_position:(%f,%f,%f)",turtle_present_pose.x,turtle_present_pose.y,turtle_present_pose.theta);
        //判断是否走到点位，定位精度为0.001
        break_flag=sqrt(pow(turtle_target_pose.x-turtle_present_pose.x, 2) +
                                        pow(turtle_target_pose.y-turtle_present_pose.y, 2));     
        ROS_INFO("break_flag is %f",break_flag);
        if(break_flag < 0.001) 
            break;
        r.sleep();
    }
        // 当action完成后，向客户端返回结果
        ROS_INFO("TurtleMove is finished.");
        as->setSucceeded();
}
 
int main(int argc, char** argv)
{
    PID_init();

    ros::init(argc, argv, "turtleMoveServer");
    ros::NodeHandle turtle_node;
    ros::Subscriber sub = turtle_node.subscribe("turtle1/pose",100,&posecallback); //订阅小乌龟的位置信息
    turtle_vel = turtle_node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);//发布控制小乌龟运动的速度
    // 定义一个服务器
    Server server(turtle_node, "turtleMove", boost::bind(&execute, _1, &server), false);
    // 服务器开始运行
    server.start();
    ROS_INFO("server has started.");
    ros::spin();
 
    return 0;
}



