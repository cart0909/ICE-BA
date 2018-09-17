#pragma once
#include <IBA/IBA.h>
#include <vector>
#include <thread>
#include <condition_variable>
#include "param.h" // calib
#include "utility.h"
#include "Frame.h"
#include "basic_datatype.h"

class LocalBA {
public:
    LocalBA(const SPtr<XP::DuoCalibParam>& pDuoCalibParam);
    ~LocalBA();

    void PushCurrentFrame(const SPtr<Frame>& curr_frame, double timestamp,
                          const std::vector<XP::ImuData>& imu_meas);

private:
    void Run();
    std::vector<std::pair<IBA::CurrentFrame, IBA::KeyFrame>> GetMeasurements();

    SPtr<XP::DuoCalibParam> mpDuoCalibParam;
    IBA::Solver mSolver;

    std::thread mtLocalBA;
    std::queue<std::pair<IBA::CurrentFrame, IBA::KeyFrame>> mqBuffer;
    std::mutex mMBuffer;
    std::condition_variable mCvBuffer;
};
