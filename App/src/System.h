#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "IBA/IBA.h"
#include "feature_utils.h"
#include "image_utils.h"
#include "xp_quaternion.h"
#include "param.h"  // calib
#include "basic_datatype.h"
#include "iba_helper.h"
#include "pose_viewer.h"
#include "utility.h"

class System {
public:
    System() {}
    ~System() {}

    void ReadConfigYaml(const std::string& filename);
private:
    XP::DuoCalibParam duo_calib_param;

    // Create masks based on FOVs computed from intrinsics
    std::vector<cv::Mat_<uchar> > masks;
};
