#pragma once
#include "IBA/IBA.h"
#include "basic_datatype.h"
#include "feature_utils.h"
#include "iba_helper.h"
#include "image_utils.h"
#include "param.h" // calib
#include "pose_viewer.h"
#include "utility.h"
#include "xp_quaternion.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class System {
public:
    System();
    ~System();

    void ReadConfigYaml(const std::string& filename);
    void TrackStereoVIO(const cv::Mat& img_left, const cv::Mat& img_right,
        double timestamp, const std::vector<XP::ImuData>& imu_data);

private:
    XP::DuoCalibParam duo_calib_param;

    // Create masks based on FOVs computed from intrinsics
    std::vector<cv::Mat_<uchar>> masks;
};
