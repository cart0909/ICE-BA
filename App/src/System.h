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
#include "FeatureTracking.h"
#include "LocalBA.h"
#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class System {
public:
    enum SystemState {
        SYSTEM_INIT,
        SYSTEM_TRACK,
        SYSTEM_LOST,
        SYSTEM_TOTAL_NUM_STATE
    };

    System();
    ~System();

    void ReadConfigYaml(const std::string& filename);
    void TrackStereoVIO(const cv::Mat& img_left, const cv::Mat& img_right,
        float timestamp, const std::vector<XP::ImuData>& imu_data);
private:
    SystemState mState = SYSTEM_INIT;
    SPtr<XP::DuoCalibParam> mpDuoCalibParam;
    // Create masks based on FOVs computed from intrinsics
    SPtr<std::vector<cv::Mat_<uchar>>> mpMasks;

    SPtr<FeatureTracking> mpFeatureTracking;
    SPtr<LocalBA> mpLocalBA;

    SPtr<Frame> mpCurrentFrame;
};
