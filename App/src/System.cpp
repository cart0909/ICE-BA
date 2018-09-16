#include "System.h"

System::System()
{
}

System::~System()
{
}

void System::ReadConfigYaml(const std::string& filename)
{
}

void System::TrackStereoVIO(const cv::Mat& img_left, const cv::Mat& img_right,
    double timestamp, const std::vector<XP::ImuData>& imu_data)
{
    cv::Mat img_in_smooth;
    cv::blur(img_left, img_in_smooth, cv::Size(3, 3));
}
