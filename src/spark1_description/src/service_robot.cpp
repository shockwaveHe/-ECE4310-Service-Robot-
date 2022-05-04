#include <ros/ros.h>
#include <ros/rate.h>
#include <sstream>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <random>
#include <string>

enum Task_Type
{
    Terminate, // find all the tags
    Search,    // Search for the next tag when not locate the tag
    Deliever   // deliever to the tag when locate the tag
};
class robot
{
public:
    ros::NodeHandle n;
    ros::Subscriber arMarkerSub, robotPoseSub, PlanningStatusSub, laserSub;
    ros::Publisher nav_pub, cancel_pub, cmd_pub;
    tf::TransformListener tf_listener;
    robot();
    ~robot();

    void ARdetector(ar_track_alvar_msgs::AlvarMarkers marker_pose);
    // record the robot pose
    void robotPose(nav_msgs::Odometry odom);

    void pathStatusAdjustment(actionlib_msgs::GoalStatusArray status);

    void laserScanCb(const sensor_msgs::LaserScanConstPtr &scan);

    bool position_check(ar_track_alvar_msgs::AlvarMarker &tag, geometry_msgs::Pose &next_pose); // check whether the detected marker is the next tag
    bool position_check(float x, float y);
    geometry_msgs::Pose convert_tag_pose_to_world_frame(ar_track_alvar_msgs::AlvarMarker &tag); // world frame is /odom

    std::map<int, std::pair<int, int>> nextTagRange;
    std::vector<std::pair<int, geometry_msgs::Pose>> explored_tag;
    std::vector<std::pair<int, geometry_msgs::Pose>> detected_tag;
    // record explored pose for current to avoid frequently navigate to explored positions
    // assign with a probabilty
    std::deque<std::pair<float, float>> explored_pose;
    int explored_pose_max_num;
    float reject_probability;
    float explored_threshold;

    std::string last_goal_id;

    int previous_marker_id;
    int next_tag_to_deliever;
    bool previous_detected;
    bool reach_last_target;

    float obstacle_distance;

    bool init;
    nav_msgs::Odometry robot_pose, last_robot_pose;
    geometry_msgs::Pose next_tag_pose; // next_tag_pose in the world frame

    Task_Type current_task;

    float threshold;
    float min_x, min_y, max_x, max_y;

    // Timer for resending navigation goal
    ros::Time timer1, timer2, timer3;
    float time_threshold;
};

robot::robot()
{

    arMarkerSub = n.subscribe("/ar_pose_marker", 0, &robot::ARdetector, this);
    robotPoseSub = n.subscribe("/odom", 0, &robot::robotPose, this);
    PlanningStatusSub = n.subscribe("/move_base/status", 0, &robot::pathStatusAdjustment, this);
    laserSub = n.subscribe("/scan", 0, &robot::laserScanCb, this);
    nav_pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 0);
    cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
    cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
    current_task = Search;
    reach_last_target = false;
    last_goal_id = "NONE";
    explored_pose_max_num = 8;
    reject_probability = 0.7;
    explored_threshold = 4;

    time_threshold = 10;
    std::pair<int, int> tag0;
    tag0.first = 0;
    tag0.second = 45;
    nextTagRange.emplace(std::make_pair(0, tag0));

    std::pair<int, int> tag1;
    tag1.first = 45;
    tag1.second = 90;
    nextTagRange.emplace(std::make_pair(1, tag1));

    std::pair<int, int> tag2;
    tag2.first = 90;
    tag2.second = 135;
    nextTagRange.emplace(std::make_pair(2, tag2));

    std::pair<int, int> tag3;
    tag3.first = 135;
    tag3.second = 180;
    nextTagRange.emplace(std::make_pair(3, tag3));

    std::pair<int, int> tag4;
    tag4.first = -180;
    tag4.second = -135;
    nextTagRange.emplace(std::make_pair(4, tag4));

    std::pair<int, int> tag5;
    tag5.first = -135;
    tag5.second = -90;
    nextTagRange.emplace(std::make_pair(5, tag5));

    std::pair<int, int> tag6;
    tag6.first = -90;
    tag6.second = -45;
    nextTagRange.emplace(std::make_pair(6, tag6));

    std::pair<int, int> tag7;
    tag7.first = -45;
    tag7.second = 0;
    nextTagRange.emplace(std::make_pair(7, tag7));

    std::pair<int, int> tag8;
    tag8.first = 0;
    tag8.second = 0;
    nextTagRange.emplace(std::make_pair(8, tag8));

    previous_marker_id = -1;
    next_tag_to_deliever = -1;

    // hand measured travel range
    max_y = 3.28 - 1;
    min_y = -5.75 + 1;
    min_x = -7.6 + 1;
    max_x = 7.146 - 1;

    threshold = 14;                        // meter^2
    srand(static_cast<unsigned>(time(0))); // random seed for searching
    init = false;
    timer3 = ros::Time::now();
}

robot::~robot()
{
}

void robot::robotPose(nav_msgs::Odometry odom)
{
    robot_pose = odom;
    if (!init)
    {
        last_robot_pose = robot_pose;
        init = true;
    }
    if (ros::Time::now().toSec() - timer3.toSec() > 4 * time_threshold)
    {
        float diff_x = robot_pose.pose.pose.position.x - last_robot_pose.pose.pose.position.x;
        float diff_y = robot_pose.pose.pose.position.y - last_robot_pose.pose.pose.position.y;
        if (diff_x * diff_x + diff_y * diff_y < 0.04)
        {
            ROS_WARN("Detect robot stop for a long time. Randomly navigate to close position to recover robot.");
            actionlib_msgs::GoalID stop_robot;
            cancel_pub.publish(stop_robot);
            move_base_msgs::MoveBaseActionGoal nav_goal;
            nav_goal.goal.target_pose.header.stamp = ros::Time::now();
            nav_goal.goal.target_pose.header.frame_id = "odom";
            float stepsize = 3;
            float theta;
            while (true)
            {
                theta = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (M_PI)));
                nav_goal.goal.target_pose.pose.position.x = odom.pose.pose.position.x + stepsize * cos(theta);
                nav_goal.goal.target_pose.pose.position.y = odom.pose.pose.position.x + stepsize * sin(theta);
                nav_goal.goal.target_pose.pose.position.z = 0;
                if (nav_goal.goal.target_pose.pose.position.x > min_x && nav_goal.goal.target_pose.pose.position.x < max_x)
                {
                    if (nav_goal.goal.target_pose.pose.position.y > min_y && nav_goal.goal.target_pose.pose.position.y < max_y)
                    {
                        break;
                    }
                }
            }
            // random orientation
            tf::Quaternion q;
            q.setRPY(0, 0, theta);
            q.normalize();
            geometry_msgs::Quaternion q_;
            tf::quaternionTFToMsg(q, q_);
            nav_goal.goal.target_pose.pose.orientation = q_;
            // ROS_INFO("nav_goal: x= %f, y=%f", x, y);
            reach_last_target = false;
            nav_pub.publish(nav_goal);
            timer3 = ros::Time::now();
        }
    }
}

void robot::ARdetector(ar_track_alvar_msgs::AlvarMarkers marker_pose)
{
    // ROS_WARN("Robot current postion: (%f, %f)", robot_pose.pose.pose.position.x,robot_pose.pose.pose.position.y);
    ROS_INFO("ROBOT STATE: %d", current_task);
    ROS_INFO("Delievered tags:");
    for (auto i : explored_tag)
    {
        ROS_INFO("%d", i.first);
    }
    if (current_task == Terminate)
    {
        ROS_INFO("detect all markers");
        return;
    }
    // search for tag
    if (current_task == Search && explored_tag.size() != 0)
    {
        if (marker_pose.markers.size() != 0)
        {

            for (auto i : marker_pose.markers)
            {
                if (position_check(i, next_tag_pose))
                {
                    // Find the tag, Stop the robot
                    reach_last_target = true;
                    actionlib_msgs::GoalID stop_robot;
                    cancel_pub.publish(stop_robot);
                    next_tag_to_deliever = i.id;
                    current_task = Deliever;
                    ros::Rate rate(1.0);
                    rate.sleep();
                    return;
                }
            }
        }
        ROS_INFO("Timer1 Diff: %f", ros::Time::now().toSec() - timer1.toSec());
        if (reach_last_target || (ros::Time::now().toSec() - timer1.toSec()) > time_threshold)
        {
            //  estimate the possible position of next tag according to current odom, previous tag position
            auto last_tag_info = explored_tag.back();
            move_base_msgs::MoveBaseActionGoal nav_goal;
            nav_goal.goal.target_pose.header.stamp = ros::Time::now();
            nav_goal.goal.target_pose.header.frame_id = "odom";
            float x, y;
            while (true)
            {
                bool add_to_list = false;
                if (last_tag_info.first == 0 || last_tag_info.first == 1)
                {
                    // 0 1
                    // randomly sample the x y position in possible range
                    x = last_tag_info.second.position.x + 1.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_x - last_tag_info.second.position.x - 1.5)));
                    y = last_tag_info.second.position.y + 1.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_y - last_tag_info.second.position.y - 1.5)));
                }

                else if (last_tag_info.first == 2 || last_tag_info.first == 3)
                {
                    // 2 3
                    // randomly sample the x y position in possible range
                    x = min_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (last_tag_info.second.position.x - 1.5 - min_x)));
                    y = last_tag_info.second.position.y + 1.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_y - last_tag_info.second.position.y - 1.5)));
                }

                else if (last_tag_info.first == 4 || last_tag_info.first == 5)
                {
                    // 4 5
                    // randomly sample the x y position in possible range
                    x = min_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (last_tag_info.second.position.x - 1.5 - min_x)));
                    y = min_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (last_tag_info.second.position.y - 1.5 - min_y)));
                }

                else //(last_tag_info.first == 6 || last_tag_info.first == 7)
                {
                    // 6 7
                    // randomly sample the x y position in possible range
                    x = last_tag_info.second.position.x + 1.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_x - last_tag_info.second.position.x - 1.5)));
                    y = min_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (last_tag_info.second.position.y - 1.5 - min_y)));
                }
                if (!position_check(x, y))
                {
                    continue;
                }
                float diff_x_ = x - robot_pose.pose.pose.position.x;
                float diff_y_ = y - robot_pose.pose.pose.position.y;
                // to close to current robot, re-sample
                if (diff_x_ * diff_x_ + diff_y_ * diff_y_ < explored_threshold)
                    continue;
                if (explored_pose.size() == 0)
                {
                    explored_pose.push_back(std::make_pair(x, y));
                    break;
                }
                for (auto i : explored_pose)
                {
                    float diff_x = x - i.first;
                    float diff_y = y - i.second;
                    float distance = diff_x * diff_x + diff_y * diff_y;
                    if (distance < explored_threshold)
                    {
                        float P = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX));
                        if (P > reject_probability)
                        {
                            explored_pose.push_back(std::make_pair(x, y));
                            add_to_list = true;
                            while (explored_pose.size() > explored_pose_max_num)
                            {
                                explored_pose.pop_front();
                            }
                            break;
                        }
                        else
                        { // reject
                            break;
                        }
                    }
                }
                if (add_to_list)
                    break;
            }
            // not achive imediately
            {
                double goal_diff_x = x - robot_pose.pose.pose.position.x;
                double goal_diff_y = y - robot_pose.pose.pose.position.y;
                double goal_distance = goal_diff_x * goal_diff_x + goal_diff_y * goal_diff_y;
                if (goal_distance > 3 * threshold)
                {
                    ROS_WARN("Too far, uses close point as stage wise goal!");
                    float stepSize = 3;
                    float angle;
                    if (fabs(goal_diff_x) < 0.005)
                    {
                        if (goal_diff_y > 0)
                        {
                            angle = M_PI / 2;
                        }
                        else
                        {
                            angle = -M_PI / 2;
                        }
                    }
                    else
                    {
                        double tan = goal_diff_y / goal_diff_x;
                        // ROS_INFO("tangent %f:", tan);
                        angle = atan(tan);
                        if (goal_diff_y > 0 && goal_diff_x < 0)
                        {
                            angle = angle + M_PI;
                        }
                        if (goal_diff_y < 0 && goal_diff_x < 0)
                        {
                            angle = angle - M_PI;
                        }
                    }
                    x = robot_pose.pose.pose.position.x + stepSize * cos(angle);
                    y = robot_pose.pose.pose.position.y + stepSize * sin(angle);
                }
            }
            nav_goal.goal.target_pose.pose.position.x = x;
            nav_goal.goal.target_pose.pose.position.y = y;
            nav_goal.goal.target_pose.pose.position.z = 0;
            // random orientation
            float theta = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (M_PI)));
            tf::Quaternion q;
            q.setRPY(0, 0, theta);
            q.normalize();
            geometry_msgs::Quaternion q_;
            tf::quaternionTFToMsg(q, q_);
            nav_goal.goal.target_pose.pose.orientation = q_;
            ROS_WARN("nav_goal: x= %f, y=%f", x, y);
            reach_last_target = false;
            actionlib_msgs::GoalID stop_robot;
            cancel_pub.publish(stop_robot);
            ros::Rate rate(1.0);
            rate.sleep();
            nav_pub.publish(nav_goal);
            timer1 = ros::Time::now();
            return;
        }
        return;
    }

    if (current_task == Search && explored_tag.size() == 0)
    {
        if (marker_pose.markers.size() == 0)
        {
            ROS_WARN("Cannot find initial tag. An error of experiment settings!");
        }
        else
        {
            ROS_INFO("Detect the first tag");
            ar_track_alvar_msgs::AlvarMarker first_tag = marker_pose.markers[0];
            ROS_INFO("first mark");
            int marker_id = first_tag.id;
            ROS_INFO("marker id %d", marker_id);
            geometry_msgs::Pose first_tag_pose = convert_tag_pose_to_world_frame(first_tag);
            ROS_INFO("add to explored tag list");
            explored_tag.push_back(std::make_pair(marker_id, first_tag_pose));
            ROS_INFO("Deliever to the first tag");
            reach_last_target = true;
        }
        return;
    }
    if (current_task == Deliever)
    {
        // ROS_INFO("marker size: %d", marker_pose.markers.size());
        if (marker_pose.markers.size() == 0)
        {

            if (reach_last_target)
            {
                ROS_INFO("Lost tag when delievering. Re-estimating tag location.");
                current_task = Search;
                return;
            }
            ROS_INFO("Timer2 Diff: %f", ros::Time::now().toSec() - timer2.toSec());
            if (ros::Time::now().toSec() - timer2.toSec() > 1.5 * time_threshold)
            {
                ROS_INFO("Current FoV locates no tags. Use previous navigation information.");
                actionlib_msgs::GoalID stop_robot;
                cancel_pub.publish(stop_robot);
                move_base_msgs::MoveBaseActionGoal nav_goal;
                nav_goal.goal.target_pose.header.stamp = ros::Time::now();
                nav_goal.goal.target_pose.header.frame_id = "odom";
                geometry_msgs::Pose tag_pose_in_odom = next_tag_pose;
                // random offset to avoid collision
                double x_offset = -2 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2));
                double y_offset = -2 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2));
                if (tag_pose_in_odom.position.x + x_offset > min_x)
                {
                    if (tag_pose_in_odom.position.x + x_offset < max_x)
                    {
                        tag_pose_in_odom.position.x += x_offset;
                    }
                    else
                    {
                        tag_pose_in_odom.position.x = max_x;
                    }
                }
                else
                {
                    tag_pose_in_odom.position.x = min_x;
                }
                if (tag_pose_in_odom.position.y + y_offset > min_y)
                {
                    if (tag_pose_in_odom.position.y + y_offset < max_y)
                    {
                        tag_pose_in_odom.position.y += y_offset;
                    }
                    else
                    {
                        tag_pose_in_odom.position.y = max_y;
                    }
                }
                else
                {
                    tag_pose_in_odom.position.y = min_y;
                }
                {
                    // not achive imediately
                    double goal_diff_x = tag_pose_in_odom.position.x - robot_pose.pose.pose.position.x;
                    double goal_diff_y = tag_pose_in_odom.position.y - robot_pose.pose.pose.position.y;
                    double goal_distance = goal_diff_x * goal_diff_x + goal_diff_y * goal_diff_y;
                    if (goal_distance > 2 * threshold)
                    {
                        ROS_WARN("Too far, uses close point as stage wise goal!");
                        float stepSize = 5;
                        float angle;
                        if (fabs(goal_diff_x) < 0.005)
                        {
                            if (goal_diff_y > 0)
                            {
                                angle = M_PI / 2;
                            }
                            else
                            {
                                angle = -M_PI / 2;
                            }
                        }
                        else
                        {
                            double tan = goal_diff_y / goal_diff_x;
                            // ROS_INFO("tangent %f:", tan);
                            angle = atan(tan);
                            if (goal_diff_y > 0 && goal_diff_x < 0)
                            {
                                angle = angle + M_PI;
                            }
                            if (goal_diff_y < 0 && goal_diff_x < 0)
                            {
                                angle = angle - M_PI;
                            }
                        }
                        tag_pose_in_odom.position.x = robot_pose.pose.pose.position.x + stepSize * cos(angle);
                        tag_pose_in_odom.position.y = robot_pose.pose.pose.position.y + stepSize * sin(angle);
                    }
                }
                nav_goal.goal.target_pose.pose = tag_pose_in_odom;
                nav_pub.publish(nav_goal);
                ROS_WARN("nav_goal: x= %f, y=%f", tag_pose_in_odom.position.x, tag_pose_in_odom.position.y);
                reach_last_target = false;
                timer2 = ros::Time::now();
            }
            // publish navigation goal
            return;
        }
        ROS_INFO("Detect tags when deliver. Check");
        for (auto i : marker_pose.markers)
        {
            ROS_INFO("Tag ID %d", i.id);
            if (i.id == next_tag_to_deliever)
            {
                geometry_msgs::Pose tag_pose_in_odom = convert_tag_pose_to_world_frame(i);
                double diff_x = tag_pose_in_odom.position.x - robot_pose.pose.pose.position.x;
                double diff_y = tag_pose_in_odom.position.y - robot_pose.pose.pose.position.y;
                double distance = diff_x * diff_x + diff_y * diff_y;
                next_tag_pose = tag_pose_in_odom;
                ROS_INFO("Distance %lf:", distance);
                if (distance < threshold)
                {
                    ROS_INFO("Deliever to tag %d", i.id);
                    explored_tag.push_back(std::make_pair(i.id, tag_pose_in_odom));
                    actionlib_msgs::GoalID stop_robot;
                    cancel_pub.publish(stop_robot);
                    // clear explored pose
                    while (explored_pose.size() > 0)
                    {
                        explored_pose.pop_front();
                    }
                    if (i.id != 8)
                    {
                        geometry_msgs::Twist rotation;
                        rotation.angular.z = 1.57;
                        current_task = Search;
                    }
                    else
                    {
                        current_task = Terminate;
                    }
                    reach_last_target = true;
                    return;
                }
                else
                {
                    ROS_INFO("Found next tag %d, deliever to it.", i.id);
                    ROS_INFO("Timer2 Diff: %f", ros::Time::now().toSec() - timer2.toSec());
                    if (reach_last_target || (ros::Time::now().toSec() - timer2.toSec() > time_threshold))
                    {
                        actionlib_msgs::GoalID stop_robot;
                        cancel_pub.publish(stop_robot);
                        ros::Rate sleep_rate(1.0);
                        sleep_rate.sleep();
                        move_base_msgs::MoveBaseActionGoal nav_goal;
                        nav_goal.goal.target_pose.header.stamp = ros::Time::now();
                        nav_goal.goal.target_pose.header.frame_id = "odom";
                        // random offset to avoid collision
                        double x_offset = -2 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2));
                        double y_offset = -2 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2));
                        if (tag_pose_in_odom.position.x + x_offset > min_x)
                        {
                            if (tag_pose_in_odom.position.x + x_offset < max_x)
                            {
                                tag_pose_in_odom.position.x += x_offset;
                            }
                            else
                            {
                                tag_pose_in_odom.position.x = max_x;
                            }
                        }
                        else
                        {
                            tag_pose_in_odom.position.x = min_x;
                        }
                        if (tag_pose_in_odom.position.y + y_offset > min_y)
                        {
                            if (tag_pose_in_odom.position.y + y_offset < max_y)
                            {
                                tag_pose_in_odom.position.y += y_offset;
                            }
                            else
                            {
                                tag_pose_in_odom.position.y = max_y;
                            }
                        }
                        else
                        {
                            tag_pose_in_odom.position.y = min_y;
                        }
                        {
                            // not achive imediately
                            double goal_diff_x = tag_pose_in_odom.position.x - robot_pose.pose.pose.position.x;
                            double goal_diff_y = tag_pose_in_odom.position.y - robot_pose.pose.pose.position.y;
                            double goal_distance = goal_diff_x * goal_diff_x + goal_diff_y * goal_diff_y;
                            if (goal_distance > 2 * threshold)
                            {
                                ROS_WARN("Too far, uses close point as stage wise goal!");
                                float stepSize = 5;
                                float angle;
                                if (fabs(goal_diff_x) < 0.005)
                                {
                                    if (goal_diff_y > 0)
                                    {
                                        angle = M_PI / 2;
                                    }
                                    else
                                    {
                                        angle = -M_PI / 2;
                                    }
                                }
                                else
                                {
                                    double tan = goal_diff_y / goal_diff_x;
                                    // ROS_INFO("tangent %f:", tan);
                                    angle = atan(tan);
                                    if (goal_diff_y > 0 && goal_diff_x < 0)
                                    {
                                        angle = angle + M_PI;
                                    }
                                    if (goal_diff_y < 0 && goal_diff_x < 0)
                                    {
                                        angle = angle - M_PI;
                                    }
                                }
                                tag_pose_in_odom.position.x = robot_pose.pose.pose.position.x + stepSize * cos(angle);
                                tag_pose_in_odom.position.y = robot_pose.pose.pose.position.y + stepSize * sin(angle);
                            }
                        }
                        nav_goal.goal.target_pose.pose = tag_pose_in_odom;

                        nav_pub.publish(nav_goal);
                        ROS_WARN("nav_goal: x= %f, y=%f", tag_pose_in_odom.position.x, tag_pose_in_odom.position.y);
                        reach_last_target = false;
                        timer2 = ros::Time::now();
                    }
                }
                return;
            }
        }
        if (ros::Time::now().toSec() - timer2.toSec() > 1.5 * time_threshold)
        {
            ROS_INFO("Current FoV locates no tags. Use previous navigation information.");
            actionlib_msgs::GoalID stop_robot;
            cancel_pub.publish(stop_robot);
            move_base_msgs::MoveBaseActionGoal nav_goal;
            nav_goal.goal.target_pose.header.stamp = ros::Time::now();
            nav_goal.goal.target_pose.header.frame_id = "odom";
            geometry_msgs::Pose tag_pose_in_odom = next_tag_pose;
            // random offset to avoid collision
            double x_offset = -2 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2));
            double y_offset = -2 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 2));
            if (tag_pose_in_odom.position.x + x_offset > min_x)
            {
                if (tag_pose_in_odom.position.x + x_offset < max_x)
                {
                    tag_pose_in_odom.position.x += x_offset;
                }
                else
                {
                    tag_pose_in_odom.position.x = max_x;
                }
            }
            else
            {
                tag_pose_in_odom.position.x = min_x;
            }
            if (tag_pose_in_odom.position.y + y_offset > min_y)
            {
                if (tag_pose_in_odom.position.y + y_offset < max_y)
                {
                    tag_pose_in_odom.position.y += y_offset;
                }
                else
                {
                    tag_pose_in_odom.position.y = max_y;
                }
            }
            else
            {
                tag_pose_in_odom.position.y = min_y;
            }
            {
                // not achive imediately
                double goal_diff_x = tag_pose_in_odom.position.x - robot_pose.pose.pose.position.x;
                double goal_diff_y = tag_pose_in_odom.position.y - robot_pose.pose.pose.position.y;
                double goal_distance = goal_diff_x * goal_diff_x + goal_diff_y * goal_diff_y;
                if (goal_distance > 2 * threshold)
                {
                    ROS_WARN("Too far, uses close point as stage wise goal!");
                    float stepSize = 5;
                    float angle;
                    if (fabs(goal_diff_x) < 0.005)
                    {
                        if (goal_diff_y > 0)
                        {
                            angle = M_PI / 2;
                        }
                        else
                        {
                            angle = -M_PI / 2;
                        }
                    }
                    else
                    {
                        double tan = goal_diff_y / goal_diff_x;
                        // ROS_INFO("tangent %f:", tan);
                        angle = atan(tan);
                        if (goal_diff_y > 0 && goal_diff_x < 0)
                        {
                            angle = angle + M_PI;
                        }
                        if (goal_diff_y < 0 && goal_diff_x < 0)
                        {
                            angle = angle - M_PI;
                        }
                    }
                    tag_pose_in_odom.position.x = robot_pose.pose.pose.position.x + stepSize * cos(angle);
                    tag_pose_in_odom.position.y = robot_pose.pose.pose.position.y + stepSize * sin(angle);
                }
            }
            nav_goal.goal.target_pose.pose = tag_pose_in_odom;
            nav_pub.publish(nav_goal);
            ROS_WARN("nav_goal: x= %f, y=%f", tag_pose_in_odom.position.x, tag_pose_in_odom.position.y);
            reach_last_target = false;
            timer2 = ros::Time::now();
        }
    }
    // ROS_INFO("Program enter no block.");
}

void robot::pathStatusAdjustment(actionlib_msgs::GoalStatusArray status)
{
    if (status.status_list.size() == 0)
        return;
    int st = status.status_list[0].status;
    if (st == 4)
    {
        if (status.status_list[0].goal_id.id != last_goal_id)
        {
            ROS_INFO("Current path target cannot be achieved");
            actionlib_msgs::GoalID stop_robot;
            cancel_pub.publish(stop_robot);
            last_goal_id = status.status_list[0].goal_id.id;
            if (current_task == Search)
            {
                ROS_INFO("Reestimate possible pose");
                reach_last_target = true;
            }

            else if (current_task == Deliever)
            {
                ROS_INFO("Hard to deliever to the tag due to local obstacles");
                ROS_INFO("Will try weaker threshold");
                double diff_x = next_tag_pose.position.x - robot_pose.pose.pose.position.x;
                double diff_y = next_tag_pose.position.y - robot_pose.pose.pose.position.y;
                double distance = diff_x * diff_x + diff_y * diff_y;
                ROS_INFO("Distance %lf:", distance);
                if (distance < 1.5 * threshold)
                {
                    ROS_INFO("Deliever to tag %d", next_tag_to_deliever);
                    explored_tag.push_back(std::make_pair(next_tag_to_deliever, next_tag_pose));
                    // clear explored pose
                    while (explored_pose.size() > 0)
                    {
                        explored_pose.pop_front();
                    }
                    if (next_tag_to_deliever != 8)
                    {
                        current_task = Search;
                    }
                    else
                    {
                        current_task = Terminate;
                    }
                    reach_last_target = true;
                    return;
                }
                else
                {
                    ROS_INFO("Cannot reach tag %d, restart searching", next_tag_to_deliever);
                    current_task = Search;
                    reach_last_target = true;
                }
            }
        }
        return;
    }
    if (st == 3)
    {
        if (status.status_list[0].goal_id.id != last_goal_id)
        {
            ROS_INFO("reach last target according to path planning");
            reach_last_target = true;
            last_goal_id = status.status_list[0].goal_id.id;
            ros::Rate loop_rate(1.0);
            loop_rate.sleep();
        }
        return;
    }
    // if (st == 2 && reach_last_target == false)
    // {
    //     if (status.status_list[0].goal_id.id != last_goal_id)
    //     {
    //         ROS_WARN("Invalid path reported by path planner");
    //         reach_last_target = true;
    //         last_goal_id = status.status_list[0].goal_id.id;
    //         ros::Rate loop_rate(1.0);
    //         loop_rate.sleep();
    //     }
    // }
}

void robot::laserScanCb(const sensor_msgs::LaserScanConstPtr &scan)
{
    return;
    float min_dist = 10;
    for (int i = 0; i < scan->ranges.size(); i += 10)
    {
        if (isnan(scan->ranges[i]))
        {
            continue;
        }
        if (scan->ranges[i] < min_dist)
        {
            min_dist = scan->ranges[i];
        }
    }
    if (min_dist < 0.05)
    {
        ROS_INFO("Possible collision min_dist = %f, stop current navigation", min_dist);
        reach_last_target = true;
        actionlib_msgs::GoalID stop_robot;
        cancel_pub.publish(stop_robot);
    }
}
bool robot::position_check(ar_track_alvar_msgs::AlvarMarker &tag, geometry_msgs::Pose &next_pose)
{
    for (auto i : explored_tag)
    {
        if (tag.id == i.first)
        {
            ROS_INFO("Found explored tag");
            return false;
        }
    }
    auto previous_tag = explored_tag.back();

    geometry_msgs::Pose next_tag_pose_ = convert_tag_pose_to_world_frame(tag);
    double diff_x = next_tag_pose_.position.x - previous_tag.second.position.x;
    double diff_y = next_tag_pose_.position.y - previous_tag.second.position.y;
    float angle;
    if (fabs(diff_x) < 0.005)
    {
        if (diff_y > 0)
        {
            angle = M_PI / 2;
        }
        else
        {
            angle = -M_PI / 2;
        }
    }
    else
    {
        double tan = diff_y / diff_x;
        // ROS_INFO("tangent %f:", tan);
        angle = atan(tan);
        if (diff_y > 0 && diff_x < 0)
        {
            angle = angle + M_PI;
        }
        if (diff_y < 0 && diff_x < 0)
        {
            angle = angle - M_PI;
        }
    }
    // To degree
    angle /= (M_PI / 180);
    int previous_id = previous_tag.first;
    auto range = nextTagRange.find(previous_id)->second;
    ROS_INFO("Previous id = %d, Range: (%d ,%d)", previous_id, range.first, range.second);
    ROS_INFO("Current id: %d,computed pos: %f (%f/%f)", tag.id, angle, diff_y, diff_x);
    if (angle > range.first && angle < range.second)
    {
        next_pose = next_tag_pose_;
        return true;
    }
    return false;
}

bool robot::position_check(float x, float y)
{
    auto previous_tag = explored_tag.back();

    double diff_x = x - previous_tag.second.position.x;
    double diff_y = y - previous_tag.second.position.y;
    float angle;
    if (fabs(diff_x) < 0.005)
    {
        if (diff_y > 0)
        {
            angle = M_PI / 2;
        }
        else
        {
            angle = -M_PI / 2;
        }
    }
    else
    {
        double tan = diff_y / diff_x;
        // ROS_INFO("tangent %f:", tan);
        angle = atan(tan);
        if (diff_y > 0 && diff_x < 0)
        {
            angle = angle + M_PI;
        }
        if (diff_y < 0 && diff_x < 0)
        {
            angle = angle - M_PI;
        }
    }
    // To degree
    angle /= (M_PI / 180);
    int previous_id = previous_tag.first;
    auto range = nextTagRange.find(previous_id)->second;
    // ROS_INFO("id = %d, Range: (%d ,%d)", previous_id, range.first, range.second);
    // ROS_INFO("computed pos: %f (%f/%f)", angle, diff_y, diff_x);
    if (angle > range.first - 10 && angle < range.second + 10) // sightly larger
    {
        return true;
    }
    return false;
}

geometry_msgs::Pose robot::convert_tag_pose_to_world_frame(ar_track_alvar_msgs::AlvarMarker &tag)
{
    // /virtual_camera_link to /odom
    tf::StampedTransform transform;
    try
    {
        // tf_listener.lookupTransform(robot_pose.header.frame_id,tag.header.frame_id,
        //                              ros::Time(0), transform);
        tf_listener.lookupTransform("odom", "virtual_camera_link",
                                    ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("ERROR: %s", ex.what());
    }
    tf::Transform tag_pose_in_cam = tf::Transform(
        tf::Quaternion(tag.pose.pose.orientation.x,
                       tag.pose.pose.orientation.y,
                       tag.pose.pose.orientation.z,
                       tag.pose.pose.orientation.w),
        tf::Vector3(tag.pose.pose.position.x,
                    tag.pose.pose.position.y,
                    tag.pose.pose.position.z));
    tf::Transform tag_pose_in_odom = transform * tag_pose_in_cam;
    geometry_msgs::Pose tag_pose;
    tf::poseTFToMsg(tag_pose_in_odom, tag_pose);

    tag_pose.position.x = (tag_pose.position.x > min_x) ? tag_pose.position.x : (min_x);
    tag_pose.position.x = (tag_pose.position.x < max_x) ? tag_pose.position.x : (max_x);
    tag_pose.position.y = (tag_pose.position.y > min_y) ? tag_pose.position.y : (min_y);
    tag_pose.position.y = (tag_pose.position.y < max_y) ? tag_pose.position.y : (max_y);
    ROS_INFO("marker pose x: %f, y: %f", tag_pose.position.x, tag_pose.position.y);
    return tag_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_robot");
    robot service_robot;
    ros::spin();
    return 1;
}