#pragma once
#include <Eigen/Dense>
#include <feature_utils.h>
#include "utility.h"
#include "Config.h"
#include "Frame.h"
#include <basic_datatype.h>

class FeatureTracking {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FeatureTracking(const SPtr<XP::DuoCalibParam>& pDuoCalibParam,
                    const SPtr<std::vector<cv::Mat_<uchar>>>& pMasks);
    ~FeatureTracking();

    void Detect(const cv::Mat& img_smooth, const SPtr<Frame>& curr_frame);
    void OpticalFlowAndDetect(const cv::Mat& img_smooth, const SPtr<Frame>& curr_frame);
    void OpticalFlowAndDetectWithIMU(const cv::Mat& img_smooth, const std::vector<XP::ImuData>& imu_meas,
                                     const SPtr<Frame>& curr_frame);
    void StereoMatching(const cv::Mat& right_img_smooth, const cv::Mat& img_smooth,
                        const SPtr<Frame>& curr_frame);
private:

    SPtr<XP::DuoCalibParam> mpDuoCalibParam;
    // Create masks based on FOVs computed from intrinsics
    SPtr<std::vector<cv::Mat_<uchar>>> mpMasks;

    SPtr<XP::FeatureTrackDetector> mpFeatTrackDetector;
    SPtr<XP::ImgFeaturePropagator> mpImgFeatPropagator;

    Eigen::Matrix4f Tlr; // it can be replace by Sophus library

    SPtr<Frame> mpLastFrame;
};
