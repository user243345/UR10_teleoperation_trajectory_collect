#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

namespace fs = std::filesystem;

class RGBSaver
{
public:
    RGBSaver(ros::NodeHandle& nh)
    {
        // 获取参数
        nh.param<std::string>("save_dir", save_dir_, "./saved_images");
        nh.param<std::string>("topic", topic_, "/camera/color/image_raw");

        // 创建保存目录
        if (!fs::exists(save_dir_))
        {
            fs::create_directories(save_dir_);
        }

        // 订阅图像话题
        sub_ = nh.subscribe(topic_, 10, &RGBSaver::imageCallback, this);
        ROS_INFO("Subscribed to %s. Saving images to %s", topic_.c_str(), save_dir_.c_str());
    }

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // 获取时间戳
            std::string timestamp = getCurrentTimestamp();

            // 构造文件名
            std::string filename = save_dir_ + "/frame_" + timestamp + ".png";

            // 保存图像
            cv::imwrite(filename, cv_ptr->image);
            ROS_INFO("Saved image: %s", filename.c_str());
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        catch (std::exception& e)
        {
            ROS_ERROR("Exception: %s", e.what());
        }
    }

    std::string getCurrentTimestamp()
    {
        // 获取当前时间
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        // 格式化时间为 YYYYMMDD_HHMMSS_mmm
        std::ostringstream timestamp;
        timestamp << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S")
                  << "_" << std::setfill('0') << std::setw(3) << now_ms.count();

        return timestamp.str();
    }

    ros::Subscriber sub_;
    std::string save_dir_;
    std::string topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgb_saver");
    ros::NodeHandle nh("~");

    RGBSaver saver(nh);
    ros::spin();

    return 0;
}





