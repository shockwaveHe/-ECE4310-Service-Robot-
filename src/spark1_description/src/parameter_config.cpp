#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
int main(int argc, char **argv)
{
	int go, go_circle;
	int count;

    // ROS节点初始化
	ros::init(argc, argv, "parameter_config");

    // 创建节点句柄
    	ros::NodeHandle n;
    	
    // 创建一个Publisher，发布名为/chatter的topic，消息类型为geometry_msgs::Twist，队列长度100
    	ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置循环的频率
   	ros::Rate loop_rate(1);
	
    // 读取运动选择参数
	ros::param::get("/go",go);
	ros::param::get("/go_circle", go_circle);
	
	ROS_INFO("Get motion[%d]",go);
	
     // 设置运动参数值	
	ros::param::set("/go",1);
	ros::param::get("/go",go);
	ROS_INFO("Get motion[%d]",go);
	count = 0;
	if (go == 1 )
	{
	for(count = 0;count <= 10; count++)
	  {
	//定义消息类型
    	geometry_msgs::TwistPtr move(new geometry_msgs::Twist());
    	move->linear.x = 1 ;
    	//发布运动消息
    	move_pub.publish(move);
    	loop_rate.sleep();
    	count++;
	  }
	geometry_msgs::TwistPtr move(new geometry_msgs::Twist());
    	move->linear.x = 0 ;
    	move_pub.publish(move);
	return 0;
	}
	if (go_circle == 1)
	{
	
	}
}
