#include <ros/ros.h>
#include <Eigen/Dense>
#include "LocalBA.h"
#include "iba_helper.h"
#include "Viewer.h"

// helper function
template <typename T>
void InitPOD(T& t) {
  memset(&t, 0, sizeof(t));
}

inline bool cmp_by_class_id(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs) {
    return lhs.class_id < rhs.class_id;
}

bool create_iba_frame(const std::vector<cv::KeyPoint>& kps_l,
                      const std::vector<cv::KeyPoint>& kps_r,
                      const std::vector<XP::ImuData>& imu_samples,
                      const double rig_time,
                      IBA::CurrentFrame *ptrCF,
                      IBA::KeyFrame *ptrKF) {
    ROS_ASSERT(std::is_sorted(kps_l.begin(), kps_l.end(), cmp_by_class_id));
    ROS_ASSERT(std::is_sorted(kps_r.begin(), kps_r.end(), cmp_by_class_id));
    ROS_ASSERT(std::includes(kps_l.begin(), kps_l.end(), kps_r.begin(), kps_r.end(), cmp_by_class_id));

    // IBA will handle *unknown* initial depth values
    IBA::Depth kUnknownDepth;
    kUnknownDepth.d = 0.0f;
    kUnknownDepth.s2 = 0.0f;
    static int last_added_point_id = -1;
    static int iba_iFrm = 0;
    auto kp_it_l = kps_l.cbegin(), kp_it_r = kps_r.cbegin();

    IBA::CurrentFrame& CF = *ptrCF;
    IBA::KeyFrame& KF = *ptrKF;

    CF.iFrm = iba_iFrm;
    InitPOD(CF.C); // needed to ensure the dumped frame deterministic even for unused field
    CF.C.C.R[0][0] = CF.C.v[0] = CF.C.ba[0] = CF.C.bw[0] = FLT_MAX;
    // MapPointMeasurement, process in ascending class id, left camera to right
    // Note the right keypoints is a subset of the left ones
    IBA::MapPointMeasurement mp_mea;
    InitPOD(mp_mea);
    mp_mea.x.S[0][0] = mp_mea.x.S[1][1] = 1.f;
    mp_mea.x.S[0][1] = mp_mea.x.S[1][0] = 0.f;
    for (; kp_it_l != kps_l.cend() && kp_it_l->class_id <= last_added_point_id; ++kp_it_l) {
      mp_mea.idx = kp_it_l->class_id;
      mp_mea.x.x[0] = kp_it_l->pt.x;
      mp_mea.x.x[1] = kp_it_l->pt.y;
      mp_mea.right = false;
      CF.zs.push_back(mp_mea);
      if (kp_it_r != kps_r.cend() && kp_it_r->class_id == kp_it_l->class_id) {
        mp_mea.x.x[0] = kp_it_r->pt.x;
        mp_mea.x.x[1] = kp_it_r->pt.y;
        mp_mea.right = true;
        CF.zs.push_back(mp_mea);
        ++kp_it_r;
      }
    }
    std::transform(imu_samples.begin(), imu_samples.end(), std::back_inserter(CF.us), XP::to_iba_imu);
    CF.t = rig_time;
    CF.d = kUnknownDepth;
    bool need_new_kf = std::distance(kp_it_l, kps_l.end()) >= 20 || CF.zs.size() < 20;
    if (std::distance(kp_it_l, kps_l.end()) == 0)
      need_new_kf = false;
    if (!need_new_kf) KF.iFrm = -1;
    else
      ROS_INFO_STREAM("new keyframe " << CF.iFrm);

    if (!need_new_kf) {
      KF.iFrm = -1;
      //  to make it deterministic
      InitPOD(KF.C);
      InitPOD(KF.d);
    } else {
      KF.iFrm = CF.iFrm;
      KF.C = CF.C.C;
      // MapPointMeasurement, duplication of CF
      KF.zs = CF.zs;
      // MapPoint
      for(; kp_it_l != kps_l.cend(); ++kp_it_l) {
        IBA::MapPoint mp;
        InitPOD(mp.X);
        mp.X.idx = kp_it_l->class_id;
        mp.X.X[0] = FLT_MAX;
        mp_mea.iFrm = iba_iFrm;
        mp_mea.x.x[0] = kp_it_l->pt.x;
        mp_mea.x.x[1] = kp_it_l->pt.y;
        mp_mea.right = false;
        mp.zs.push_back(mp_mea);

        if (kp_it_r != kps_r.cend() && kp_it_r->class_id == kp_it_l->class_id) {
          mp_mea.x.x[0] = kp_it_r->pt.x;
          mp_mea.x.x[1] = kp_it_r->pt.y;
          mp_mea.right = true;
          mp.zs.push_back(mp_mea);
          kp_it_r++;
        } else {
//          ROS_WARN_STREAM("add new feature point " << kp_it_l->class_id << " only found in left image");
        }
        KF.Xs.push_back(mp);
      }
      last_added_point_id = std::max(KF.Xs.back().X.idx, last_added_point_id);
      KF.d = kUnknownDepth;
    }
    ++iba_iFrm;
    return true;
}

LocalBA::LocalBA(const SPtr<XP::DuoCalibParam>& pDuoCalibParam)
    :mpDuoCalibParam(pDuoCalibParam)
{
    mSolver.Create(XP::to_iba_calibration(*mpDuoCalibParam),
                   IBA_SERIAL_LBA | IBA_SERIAL_GBA,
                   IBA_VERBOSE_NONE,
                   IBA_DEBUG_NONE,
                   IBA_HISTORY_LBA | IBA_HISTORY_GBA);

    mSolver.SetCallbackLBA([&](const int iFrm, const float ts) {
        IBA::SlidingWindow sliding_window;
        mSolver.GetSlidingWindow(&sliding_window);
        const IBA::CameraIMUState& X = sliding_window.CsLF.back();
        const IBA::CameraPose& C = X.C;
        Eigen::Matrix4f W_vio_T_S = Eigen::Matrix4f::Identity();  // W_vio_T_S
        for (int i = 0; i < 3; ++i) {
          W_vio_T_S(i, 3) = C.p[i];
          for (int j = 0; j < 3; ++j) {
            W_vio_T_S(i, j) = C.R[j][i];  // C.R is actually R_SW
          }
        }

        Eigen::Vector3f speed;
        for(int i = 0; i < 3; ++i)
            speed(i) = X.v[i];

        Viewer::Instance().PubOdometry(W_vio_T_S, speed);
#ifdef DRAW_KEYFRAME_POSE
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vec_p;
        for(int i = 0, n = sliding_window.CsKF.size(); i < n - 1; ++i) {
            Eigen::Vector3f p;
            p(0) = sliding_window.CsKF[i].p[0];
            p(1) = sliding_window.CsKF[i].p[1];
            p(2) = sliding_window.CsKF[i].p[2];
            vec_p.emplace_back(p);
        }

        if(!vec_p.empty())
            Viewer::Instance().PubKeyFramePose(vec_p);
#endif
    });
    mSolver.Start();
    mtLocalBA = std::thread(&LocalBA::Run, this);
}

LocalBA::~LocalBA() {
    mSolver.Stop();
    mSolver.Destroy();
}

void LocalBA::Run() {
    while(1) {
        std::vector<std::pair<IBA::CurrentFrame, IBA::KeyFrame>> measurements;
        std::unique_lock<std::mutex> lock(mMBuffer);
        mCvBuffer.wait(lock, [&] {
            return (measurements = GetMeasurements()).size() != 0;
        });
        lock.unlock();

        for(auto& meas : measurements) {
            auto& CF = meas.first;
            auto& KF = meas.second;
            mSolver.PushCurrentFrame(CF, KF.iFrm == -1 ? nullptr : &KF);
        }
    }
}

void LocalBA::PushCurrentFrame(const SPtr<Frame>& curr_frame, float timestamp,
                      const std::vector<XP::ImuData>& imu_meas) {
    std::vector<cv::KeyPoint> kps_l = curr_frame->mvKeys, kps_r = curr_frame->mvKeysRight;
    IBA::CurrentFrame CF;
    IBA::KeyFrame KF;

    std::sort(kps_l.begin(), kps_l.end(), cmp_by_class_id);
    std::sort(kps_r.begin(), kps_r.end(), cmp_by_class_id);

    create_iba_frame(kps_l, kps_r, imu_meas, timestamp, &CF, &KF);

    mMBuffer.lock();
    mqBuffer.emplace(CF, KF);
    mMBuffer.unlock();
    mCvBuffer.notify_one();
}

std::vector<std::pair<IBA::CurrentFrame, IBA::KeyFrame>> LocalBA::GetMeasurements() {
    std::vector<std::pair<IBA::CurrentFrame, IBA::KeyFrame>> measurements;
    while(!mqBuffer.empty()) {
        measurements.emplace_back(mqBuffer.front());
        mqBuffer.pop();
    }
    return measurements;
}
