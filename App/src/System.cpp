#include "System.h"
#include <ros/ros.h>
#include <opencv2/core/eigen.hpp>

System::System()
{
    mpDuoCalibParam = std::make_shared<XP::DuoCalibParam>();
    mpMasks = std::make_shared<std::vector<cv::Mat_<uchar>>>(2);
}

System::~System()
{
}

void System::ReadConfigYaml(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    std::vector<float> K, D;
    // camera intrinsics
    fs["intrinsics0"] >> K;
    fs["distortion_coefficients0"] >> D;

    mpDuoCalibParam->Camera.cv_camK_lr[0] << K[0], 0, K[2],
            0, K[1], K[3],
            0, 0, 1;
    mpDuoCalibParam->Camera.cameraK_lr[0] << K[0], 0, K[2],
            0, K[1], K[3],
            0, 0, 1;

    mpDuoCalibParam->Camera.cv_dist_coeff_lr[0] = (cv::Mat_<float>(4, 1) <<
                                                    D[0], D[1], D[2], D[3]);

    fs["intrinsics1"] >> K;
    fs["distortion_coefficients1"] >> D;

    mpDuoCalibParam->Camera.cv_camK_lr[1] << K[0], 0, K[2],
            0, K[1], K[3],
            0, 0, 1;
    mpDuoCalibParam->Camera.cameraK_lr[1] << K[0], 0, K[2],
            0, K[1], K[3],
            0, 0, 1;

    mpDuoCalibParam->Camera.cv_dist_coeff_lr[1] = (cv::Mat_<float>(4, 1) <<
                                                    D[0], D[1], D[2], D[3]);

    // ASL {B}ody frame is the IMU
    // {D}evice frame is the left camera
    // extrinsics
    cv::Mat cv_Tbc0, cv_Tbc1, cv_Tbi;
    fs["T_BC0"] >> cv_Tbc0;
    fs["T_BC1"] >> cv_Tbc1;
    fs["T_BI"] >> cv_Tbi;

    Eigen::Matrix4f Tbc0, Tbc1, Tbi;
    cv::cv2eigen(cv_Tbc0, Tbc0);
    cv::cv2eigen(cv_Tbc1, Tbc1);
    cv::cv2eigen(cv_Tbi, Tbi);

    Eigen::Matrix4f Tdc0, Tdb, Tdc1, Tdi;
    Tdc0 = Eigen::Matrix4f::Identity();
    Tdb = Tdc0 * Tbc0.inverse();
    Tdc1 = Tdb * Tbc1;
    Tdi = Tdb * Tdi;
    mpDuoCalibParam->Camera.D_T_C_lr[0] = Tdc0;
    mpDuoCalibParam->Camera.D_T_C_lr[1] = Tdc1;

    // image size
    int width, height;
    width = fs["image_width"];
    height = fs["image_height"];
    mpDuoCalibParam->Camera.img_size = cv::Size(width, height);

    // IMU
    mpDuoCalibParam->Imu.accel_TK = Eigen::Matrix3f::Identity();
    mpDuoCalibParam->Imu.accel_bias = Eigen::Vector3f::Zero();
    mpDuoCalibParam->Imu.gyro_TK = Eigen::Matrix3f::Identity();
    mpDuoCalibParam->Imu.gyro_bias = Eigen::Vector3f::Zero();
    mpDuoCalibParam->Imu.accel_noise_var = Eigen::Vector3f{0.0016,0.0016,0.0016};
    mpDuoCalibParam->Imu.angv_noise_var = Eigen::Vector3f{0.0001,0.0001,0.0001};
    mpDuoCalibParam->Imu.D_T_I = Tdi;
    mpDuoCalibParam->device_id = "ASL";
    mpDuoCalibParam->sensor_type = XP::DuoCalibParam::SensorType::UNKNOWN;
    fs.release();

    mpDuoCalibParam->initUndistortMap(mpDuoCalibParam->Camera.img_size);

    // Create masks based on FOVs computed from intrinsics
    for (int lr = 0; lr < 2; ++lr) {
      float fov;
      if (XP::generate_cam_mask(mpDuoCalibParam->Camera.cv_camK_lr[lr],
                                mpDuoCalibParam->Camera.cv_dist_coeff_lr[lr],
                                mpDuoCalibParam->Camera.img_size,
                                &mpMasks->at(lr),
                                &fov)) {
        ROS_INFO_STREAM("camera " << lr << " fov: " << fov << " deg");
      }
    }

    mpFeatureTracking = std::make_shared<FeatureTracking>(mpDuoCalibParam,
                                                          mpMasks);
    mpLocalBA = std::make_shared<LocalBA>(mpDuoCalibParam);
}

void System::TrackStereoVIO(const cv::Mat& img_left, const cv::Mat& img_right,
    double timestamp, const std::vector<XP::ImuData>& imu_data)
{
    SPtr<Frame> curr_frame = std::make_shared<Frame>();
    cv::Mat img_smooth, right_img_smooth;
    cv::blur(img_left, img_smooth, cv::Size(3, 3));
    cv::blur(img_right, right_img_smooth, cv::Size(3, 3));

    if(mState == SYSTEM_INIT) {
        mpFeatureTracking->Detect(img_smooth, curr_frame);
        mState = SYSTEM_TRACK;
    }
    else if(mState == SYSTEM_TRACK){
        if(imu_data.size() > 1) {
            mpFeatureTracking->OpticalFlowAndDetectWithIMU(
                        img_smooth,
                        imu_data,
                        curr_frame);
        }
        else {
            mpFeatureTracking->OpticalFlowAndDetect(
                        img_smooth,
                        curr_frame);
        }
    }
    else {
        ROS_ERROR_STREAM("not implemented.");
    }

    mpFeatureTracking->StereoMatching(right_img_smooth, img_smooth, curr_frame);
    mpLocalBA->PushCurrentFrame(curr_frame, timestamp, imu_data);
}
