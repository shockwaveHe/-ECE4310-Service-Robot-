#include <ros/ros.h>
#include "sstream"
#include "std_msgs/String.h"
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "move_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度100
    ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while(count < 10)
    {
    	//定义消息类型
    	geometry_msgs::TwistPtr move(new geometry_msgs::Twist());
    	move->linear.x = 1 ;
    	ROS_INFO("count=%d",count);
    	//发布运动消息
    	move_pub.publish(move);
    	count++;
    	//设置睡眠时间
    	loop_rate.sleep();
    }
    geometry_msgs::TwistPtr move(new geometry_msgs::Twist());
    move->linear.x = 0 ;
    move_pub.publish(move);


    return 0;
}
