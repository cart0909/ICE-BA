#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

class Frame {
public:
    Frame();
    ~Frame();

    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    cv::Mat mOrbFeat, mOrbFeatRight;
};
