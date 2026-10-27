#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

// Persistent GStreamer JPEG decoder for the existing ROS compressed JPEG topic.
// No direct webcam access, and no threads are created while detection is OFF.
class FireHwJpeg
{
public:
    explicit FireHwJpeg(const std::string& requested_backend);
    ~FireHwJpeg();

    // Hardware returns CV_8UC4 BGRx; OpenCV fallback returns CV_8UC3 BGR.
    bool decode(const std::vector<uint8_t>& jpeg, cv::Mat& dst);
    std::string backend() const;

private:
    bool openHardware();
    bool decodeHardware(const std::vector<uint8_t>& jpeg, cv::Mat& dst);
    void closeHardware();
    void fallback(const std::string& reason);

    std::string backend_;
    bool first_frame_ = true;
    void* pipeline_ = nullptr;   // GstElement*; header does not expose GStreamer
    void* source_ = nullptr;     // GstAppSrc*
    void* sink_ = nullptr;       // GstAppSink*
};
