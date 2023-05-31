#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/BoundingBoxes.h"
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

void Coordinate_cal(double angle)
{
    tf::TransformListener listener;
    // 等待获取变换信息
    ros::Duration(1.0).sleep();

    // 定义目标坐标系和源坐标系的名称
    std::string targetFrame = "world";
    std::string sourceFrame = "base_link";

    // 创建一个单位向量，表示直线在机器人坐标系中的方向
    tf::Vector3 direction(1.0, 0.0, 0.0);  // 假设直线沿机器人坐标系的 x 轴方向

    // 创建一个tf::StampedTransform对象，用于存储变换关系
    tf::StampedTransform transform;

    try
    {
        // 获取最近的坐标系变换信息
        listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN("无法获取坐标系变换: %s", ex.what());
    
    }

    // 将方向向量旋转到世界坐标系中
    //
    tf::Matrix3x3 rotation = transform.getBasis();
    tf::Vector3 rotatedDirection = rotation * direction;

    // 计算旋转后的方向向量
    double cosTheta = std::cos(angle);
    double sinTheta = std::sin(angle);
    double rotatedX = rotatedDirection.x() * cosTheta - rotatedDirection.y() * sinTheta;
    double rotatedY = rotatedDirection.x() * sinTheta + rotatedDirection.y() * cosTheta;

    // 打印旋转后的直线在世界坐标系中的方向
    ROS_INFO("旋转后的直线在世界坐标系中的方向: x = %f, y = %f, z = %f", rotatedX, rotatedY, rotatedDirection.z());

}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;

    // 创建发布者，发布 std_msgs::Bool 和 geometry_msgs::Point 消息
    ros::Publisher pause_pub = nh.advertise<std_msgs::Bool>("/pause_navigation", 10);
    ros::Publisher sample_points_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/raw_image", 1);
 
    yolov5_ros_msgs::BoundingBoxes::ConstPtr msg = ros::topic::waitForMessage<yolov5_ros_msgs::BoundingBoxes>("/yolov5/BoundingBoxes", nh);

    auto message =msg->bounding_boxes;
    double value = message.back().angle;
    

    Coordinate_cal(value);

    double target_x = -0.069;  // 目标点的 x 坐标
    double target_y = -0.168;   // 目标点的 y 坐标
    double radius = 0.2;     // 圆的半径
    int num_points = 16;     // 采样点的数量
    std::vector<Point> sampled_points = samplePointsOnCircle(target_x, target_y, radius, num_points);
   
    for (const auto& point : sampled_points) 
    {
        pausePublish(pause_pub, true);

        ros::Duration(0.1).sleep();
        pointPublish(sample_points_pub,point);
        actionlib_msgs::GoalStatusArray::ConstPtr plan_valid = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", nh);
        std::cout<<int (plan_valid->status_list.back().status)  << std::endl;

        if (int (plan_valid->status_list.back().status) != 1)
        {
            ROS_INFO("not reachable");
            continue;
        }else
        {
            ROS_INFO("reachable");
            pausePublish(pause_pub,false);
        }
        // if plan_result == False:
        //     continue;
        // else:
        //     pause_pub.publish(false);
        while (ros::ok())
        {
            actionlib_msgs::GoalStatusArray::ConstPtr status_msg = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", nh);
            if (!status_msg || status_msg->status_list.empty()) 
            {
                ROS_ERROR("Failed to receive move_base status message");
                break;
            }else
	    {
		ROS_INFO("valid status message");
	    }
            const auto& last_status = status_msg->status_list.back();
            if (int (last_status.status) == 3) 
            {   
                Capopen(image_pub);
                break;
            }else
            {
                std::cout<<int( status_msg->status_list.back().status)<<std::endl;
                ROS_INFO("status is not 3");
            }
        }
   }
}
