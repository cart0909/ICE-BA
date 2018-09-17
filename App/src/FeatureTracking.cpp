#include "FeatureTracking.h"
#include <xp_quaternion.h>
#include <opencv2/core/eigen.hpp>

FeatureTracking::FeatureTracking(const SPtr<XP::DuoCalibParam>& pDuoCalibParam,
                                 const SPtr<std::vector<cv::Mat_<uchar>>>& pMasks)
    :mpDuoCalibParam(pDuoCalibParam), mpMasks(pMasks)
{
    mpFeatTrackDetector = std::make_shared<XP::FeatureTrackDetector>
                                            (g_ft_len,
                                             g_ft_droprate,
                                             g_use_fast,
                                             g_uniform_radius,
                                             mpDuoCalibParam->Camera.img_size);

    mpImgFeatPropagator = std::make_shared<XP::ImgFeaturePropagator> (
                mpDuoCalibParam->Camera.cameraK_lr[1],
                mpDuoCalibParam->Camera.cameraK_lr[0],
                mpDuoCalibParam->Camera.cv_dist_coeff_lr[1],
                mpDuoCalibParam->Camera.cv_dist_coeff_lr[0],
                mpMasks->at(1),
                g_pyra_level,
                g_min_feature_distance_over_baseline_ratio,
                g_max_feature_distance_over_baseline_ratio
                );

    Tlr = mpDuoCalibParam->Camera.D_T_C_lr[0].inverse() * mpDuoCalibParam->Camera.D_T_C_lr[1];
}

FeatureTracking::~FeatureTracking() {

}

void FeatureTracking::Detect(const cv::Mat& img_smooth, const SPtr<Frame>& curr_frame) {
    mpFeatTrackDetector->detect(img_smooth, mpMasks->at(0),
                                g_max_num_per_grid * g_grid_row_num * g_grid_col_num,
                                g_pyra_level,
                                g_fast_thresh,
                                &(curr_frame->mvKeys),
                                &(curr_frame->mOrbFeat)
                                );
    mpFeatTrackDetector->build_img_pyramids(img_smooth, XP::FeatureTrackDetector::BUILD_TO_PREV);
    mpLastFrame = curr_frame;
}

void FeatureTracking::OpticalFlowAndDetect(const cv::Mat& img_smooth, const SPtr<Frame>& curr_frame) {
    const int request_feat_num = g_max_num_per_grid * g_grid_row_num * g_grid_col_num;
    mpFeatTrackDetector->build_img_pyramids(img_smooth, XP::FeatureTrackDetector::BUILD_TO_CURR);
    mpFeatTrackDetector->optical_flow_and_detect(mpMasks->at(0),
                                                 mpLastFrame->mOrbFeat,
                                                 mpLastFrame->mvKeys,
                                                 request_feat_num,
                                                 g_pyra_level,
                                                 g_fast_thresh,
                                                 &(curr_frame->mvKeys),
                                                 &(curr_frame->mOrbFeat));
    mpFeatTrackDetector->update_img_pyramids();
    mpLastFrame = curr_frame;
}

void FeatureTracking::OpticalFlowAndDetectWithIMU(const cv::Mat& img_smooth,
                                                  const std::vector<XP::ImuData>& imu_meas,
                                                  const SPtr<Frame>& curr_frame) {
    assert(imu_meas.size() > 1);
    const int request_feat_num = g_max_num_per_grid * g_grid_row_num * g_grid_col_num;
    mpFeatTrackDetector->build_img_pyramids(img_smooth, XP::FeatureTrackDetector::BUILD_TO_CURR);

    // Here we simply the transformation chain to rotation only and assume zero translation
    cv::Matx33f old_R_new;
    XP::XpQuaternion I_new_q_I_old; // The rotation between the new {I} and old {I}
    for(size_t i = 1, n = imu_meas.size(); i < n; ++i) {
        XP::XpQuaternion q_end;
        XP::IntegrateQuaternion(imu_meas[i-1].ang_v,
                                imu_meas[i].ang_v,
                                I_new_q_I_old,
                                imu_meas[i].time_stamp - imu_meas[i - 1].time_stamp,
                                &q_end);
        I_new_q_I_old = q_end;
    }
    Eigen::Matrix3f I_new_R_I_old = I_new_q_I_old.ToRotationMatrix();
    Eigen::Matrix4f I_T_C = mpDuoCalibParam->Imu.D_T_I.inverse() * mpDuoCalibParam->Camera.D_T_C_lr[0];
    Eigen::Matrix3f I_R_C = I_T_C.block<3, 3>(0, 0);
    Eigen::Matrix3f C_new_R_C_old = I_R_C.transpose() * I_new_R_I_old * I_R_C;

    old_R_new = cv::Matx33f(C_new_R_C_old.data()); // some trick

    mpFeatTrackDetector->optical_flow_and_detect(mpMasks->at(0),
                                                 mpLastFrame->mOrbFeat,
                                                 mpLastFrame->mvKeys,
                                                 request_feat_num,
                                                 g_pyra_level,
                                                 g_fast_thresh,
                                                 &(curr_frame->mvKeys),
                                                 &(curr_frame->mOrbFeat),
                                                 cv::Vec2f(0, 0), // shift init pixels
                                                 &mpDuoCalibParam->Camera.cv_camK_lr[0],
                                                 &mpDuoCalibParam->Camera.cv_dist_coeff_lr[0],
                                                 &old_R_new
                                                 );

    mpFeatTrackDetector->update_img_pyramids();
    mpLastFrame = curr_frame;
}

void FeatureTracking::StereoMatching(const cv::Mat& right_img_smooth, const cv::Mat& img_smooth,
                    const SPtr<Frame>& curr_frame) {
    mpImgFeatPropagator->PropagateFeatures(right_img_smooth, img_smooth, curr_frame->mvKeys,
                                           Tlr, &(curr_frame->mvKeysRight), &(curr_frame->mOrbFeatRight));
}
