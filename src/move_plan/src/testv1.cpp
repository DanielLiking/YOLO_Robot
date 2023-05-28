#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
// 定义点的结构体

struct Point {
    double x;
    double y;
};

std::vector<Point> samplePointsOnCircle(double center_x, double center_y, double radius, int num_points) 
{
    std::vector<Point> points;
    double angle_step = 360.0 / num_points;

    for (int i = 0; i < num_points; i++) 
    {
        double angle = angle_step * i;
        double radians = angle * M_PI / 180.0;

        Point point;
        point.x = center_x + radius * cos(radians);
        point.y = center_y + radius * sin(radians);

        points.push_back(point);
    }

    return points;
}
void pausePublish(ros::Publisher pause_pub, bool para)
{
    std_msgs::Bool bool_msg;
    bool_msg.data = para;
    pause_pub.publish(bool_msg);
}
void pointPublish(ros::Publisher sample_points_pub,Point point)
{
    double roll = 0.0;    // 绕X轴旋转的角度
    double pitch = 0.0;   // 绕Y轴旋转的角度
    double yaw =280.0;  // 绕Z轴旋转的角度

    tf::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion msg;
    tf::quaternionTFToMsg(quaternion, msg);
    
    geometry_msgs::PoseStamped posestamped;

    posestamped.header.frame_id = "map";
    posestamped.pose.position.x = point.x;
    posestamped.pose.position.y = point.y;
    posestamped.pose.position.z = 0.0;
    posestamped.pose.orientation.x = msg.x;
    posestamped.pose.orientation.y = msg.y;
    posestamped.pose.orientation.z = msg.z;
    posestamped.pose.orientation.w = msg.w;
    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.goal.target_pose.header = posestamped.header;
    actionGoal.goal.target_pose.pose = posestamped.pose;
    sample_points_pub.publish(actionGoal);
}
void Capopen(ros::Publisher image_pub)
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) 
    {
        ROS_ERROR("Can't open the camera");
    }
    cv::Mat frame;
    if (cap.read(frame))
    {
        ROS_INFO("succeed!");
    }
    sensor_msgs::Image image_msg;
    cap >> frame;
    if (frame.empty()) 
    {
        ROS_ERROR("Failed to capture frame from camera");
    }
    image_msg = sensor_msgs::Image();
    image_msg.header.stamp = ros::Time::now();
    image_msg.height = frame.rows;
    image_msg.width = frame.cols;
    image_msg.encoding = "rgb8";
    image_msg.step = frame.cols * frame.elemSize();
    image_msg.data.resize(image_msg.step * image_msg.height);
    std::memcpy(&image_msg.data[0], frame.data, image_msg.data.size());
    image_pub.publish(image_msg);         
}
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;

    // 创建发布者，发布 std_msgs::Bool 和 geometry_msgs::Point 消息
    ros::Publisher pause_pub = nh.advertise<std_msgs::Bool>("/pause_navigation", 10);
    ros::Publisher sample_points_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/raw_image", 1);

    double target_x = 0;  // 目标点的 x 坐标
    double target_y = 0;   // 目标点的 y 坐标
    double radius = 2;     // 圆的半径
    int num_points = 16;     // 采样点的数量
    std::vector<Point> sampled_points = samplePointsOnCircle(target_x, target_y, radius, num_points);
   
   for (const auto& point : sampled_points) 
   {
        pausePublish(pause_pub, true);
        ros::Duration(0.1).sleep();
        // pub(point)
        pointPublish(sample_points_pub,point);
        nav_msgs::Path::ConstPtr plan_valid = ros::topic::waitForMessage<nav_msgs::Path>("/tj_move_base/GlobalPlanner/plan",nh);
        if (plan_valid->poses.empty())
        {
            ROS_INFO("not reachable");
            continue;
        }else
        {
            ROS_INFO("reachable");
            pausePublish(pause_pub,false);
        }
        while (ros::ok())
        {
            actionlib_msgs::GoalStatusArray::ConstPtr status_msg = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", nh);
            if (!status_msg || status_msg->status_list.empty()) 
            {
                ROS_ERROR("Failed to receive move_base status message");
                break;
            }
            const auto& last_status = status_msg->status_list.back();
            if (last_status.status == 3) 
            {   
                Capopen(image_pub);
                break;
            }else
            {
                ROS_INFO("status is not 3");
            }
        }
   }
}
