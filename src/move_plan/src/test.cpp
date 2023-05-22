#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
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


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;

    // 创建发布者，发布 std_msgs::Bool 和 geometry_msgs::Point 消息
    ros::Publisher pause_pub = nh.advertise<std_msgs::Bool>("/pause_navigation", 10);
    ros::Publisher sample_points_pub = nh.advertise<geometry_msgs::Point>("/move_base_simple/goal", 10);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/raw_image", 1);

    double target_x = 1.0;   // 目标点的 x 坐标
    double target_y = 2.0;   // 目标点的 y 坐标
    double radius = 5.0;     // 圆的半径
    int num_points = 16;     // 采样点的数量
    std::vector<Point> sampled_points = samplePointsOnCircle(target_x, target_y, radius, num_points);
    
   for (const auto& point : sampled_points) 
   {
        std_msgs::Bool bool_msg;
        bool_msg.data = true;
        pause_pub.publish(bool_msg);

        geometry_msgs::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        sample_points_pub.publish(point_msg);
      
        ros::Duration(0.1).sleep();
      
        actionlib_msgs::GoalStatusArray::ConstPtr status_msg = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", nh);
        // ROS_INFO_STREAM("status:"<<status_msg);
        
        if (!status_msg || status_msg->status_list.empty()) 
        {
         
            ROS_ERROR("Failed to receive move_base status message");
            continue;
        }
      
        const auto& last_status = status_msg->status_list.back();
        if (last_status.status != 3) 
        {
            cv::VideoCapture cap(0);
            if (!cap.isOpened()) 
            {
                ROS_ERROR("Can't open the camera");
                return 1;
            }
            
            cv::Mat frame;
             if (cap.read(frame))
            {
            // 将帧保存为图像文件
            cv::imshow("photo.jpg", frame);
          
            }
            // cv::namedWindow("侧视图", cv::WINDOW_NORMAL);

            sensor_msgs::Image image_msg;
            ros::Rate loop_rate(10);  // 设置发布图像的频率为10Hz
            while (ros::ok()) 
            {
                // 从摄像头获取图像
                cap >> frame;
                  ROS_INFO("3l");
                // 检查图像是否有效
                if (frame.empty()) 
                {
                    ROS_ERROR("Failed to capture frame from camera");
                    continue;
                }

                // 将OpenCV的图像转换为ROS图像消息
                cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);  // OpenCV默认使用BGR通道顺序，ROS使用RGB通道顺序
                image_msg = sensor_msgs::Image();
                image_msg.header.stamp = ros::Time::now();
                image_msg.height = frame.rows;
                image_msg.width = frame.cols;
                image_msg.encoding = "rgb8";
                image_msg.step = frame.cols * frame.elemSize();
                image_msg.data.resize(image_msg.step * image_msg.height);
                std::memcpy(&image_msg.data[0], frame.data, image_msg.data.size());

                // 发布图像消息
                image_pub.publish(image_msg);

                // 等待一段时间，控制发布频率
                loop_rate.sleep();
            }
        } else 
        {
            ROS_INFO("I am planning to achieve the goal");
        }
    }

    return 0;
}