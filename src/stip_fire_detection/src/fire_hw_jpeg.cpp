#include "fire_hw_jpeg.h"

#include <ros/ros.h>
#include <opencv2/imgcodecs.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <algorithm>
#include <cstring>
#include <mutex>
#include <sstream>

namespace
{
std::once_flag gst_init_once;
}

FireHwJpeg::FireHwJpeg(const std::string& requested_backend)
    : backend_(requested_backend)
{
    if (backend_ != "nvidia" && backend_ != "opencv")
    {
        ROS_WARN_STREAM("Unknown fire decoder_backend='" << backend_
                        << "', using opencv");
        backend_ = "opencv";
    }
}

FireHwJpeg::~FireHwJpeg()
{
    closeHardware();
}

std::string FireHwJpeg::backend() const
{
    return backend_;
}

bool FireHwJpeg::openHardware()
{
    std::call_once(gst_init_once, []() { gst_init(nullptr, nullptr); });

    const char* required[] = {"appsrc", "jpegparse", "nvv4l2decoder",
                              "nvvidconv", "appsink"};
    for (const char* name : required)
    {
        GstElementFactory* factory = gst_element_factory_find(name);
        if (!factory)
        {
            ROS_WARN_STREAM("Missing GStreamer plugin: " << name);
            return false;
        }
        gst_object_unref(factory);
    }

    // Same pipeline as the successful LIVE Jetson benchmark. One JPEG in,
    // one CPU-accessible BGRx image out. The ROS camera publisher is unchanged.
    const char* pipeline_description =
        "appsrc name=jpeg_src is-live=true block=true format=time "
        "do-timestamp=true caps=\"image/jpeg,framerate=(fraction)30/1\" "
        "! jpegparse "
        "! nvv4l2decoder mjpeg=1 "
        "! nvvidconv "
        "! video/x-raw,format=BGRx "
        "! appsink name=pixel_sink emit-signals=false sync=false "
        "max-buffers=1 drop=true enable-last-sample=false";

    GError* error = nullptr;
    GstElement* pipe = gst_parse_launch(pipeline_description, &error);
    if (error)
    {
        ROS_WARN_STREAM("Cannot build NVIDIA JPEG pipeline: " << error->message);
        g_error_free(error);
        if (pipe)
            gst_object_unref(pipe);
        return false;
    }
    if (!pipe)
        return false;

    GstElement* src = gst_bin_get_by_name(GST_BIN(pipe), "jpeg_src");
    GstElement* sink = gst_bin_get_by_name(GST_BIN(pipe), "pixel_sink");
    if (!src || !sink)
    {
        ROS_WARN("NVIDIA JPEG appsrc/appsink missing");
        if (src) gst_object_unref(src);
        if (sink) gst_object_unref(sink);
        gst_object_unref(pipe);
        return false;
    }

    g_object_set(src, "max-bytes", static_cast<guint64>(2 * 1024 * 1024), nullptr);

    if (gst_element_set_state(pipe, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE)
    {
        ROS_WARN("NVIDIA JPEG pipeline cannot enter PLAYING");
        gst_element_set_state(pipe, GST_STATE_NULL);
        gst_object_unref(src);
        gst_object_unref(sink);
        gst_object_unref(pipe);
        return false;
    }

    pipeline_ = pipe;
    source_ = src;
    sink_ = sink;
    ROS_INFO("Fire JPEG decoder: NVIDIA nvv4l2decoder -> CPU BGRx -> CUDA");
    return true;
}

bool FireHwJpeg::decodeHardware(const std::vector<uint8_t>& jpeg, cv::Mat& dst)
{
    GstBuffer* input = gst_buffer_new_allocate(nullptr, jpeg.size(), nullptr);
    if (!input)
        return false;
    if (gst_buffer_fill(input, 0, jpeg.data(), jpeg.size()) != jpeg.size())
    {
        gst_buffer_unref(input);
        return false;
    }

    // gst_app_src_push_buffer takes ownership of the GstBuffer.
    const GstFlowReturn pushed =
        gst_app_src_push_buffer(GST_APP_SRC(source_), input);
    if (pushed != GST_FLOW_OK)
    {
        ROS_WARN_STREAM("NVIDIA JPEG push-buffer status " << pushed);
        return false;
    }

    GstSample* sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(sink_), (first_frame_ ? 1500 : 350) * GST_MSECOND);
    if (!sample)
    {
        GstBus* bus = gst_element_get_bus(GST_ELEMENT(pipeline_));
        GstMessage* message = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
        if (message)
        {
            GError* error = nullptr;
            gchar* detail = nullptr;
            gst_message_parse_error(message, &error, &detail);
            ROS_WARN_STREAM("NVIDIA JPEG decode: "
                            << (error ? error->message : "unknown error"));
            if (error) g_error_free(error);
            if (detail) g_free(detail);
            gst_message_unref(message);
        }
        gst_object_unref(bus);
        return false;
    }

    const GstCaps* caps = gst_sample_get_caps(sample);
    GstVideoInfo vinfo;
    gst_video_info_init(&vinfo);
    const bool caps_ok =
        caps && gst_video_info_from_caps(&vinfo, const_cast<GstCaps*>(caps));
    if (!caps_ok || GST_VIDEO_INFO_FORMAT(&vinfo) != GST_VIDEO_FORMAT_BGRx)
    {
        ROS_WARN("NVIDIA JPEG decoder output isn't CPU BGRx");
        gst_sample_unref(sample);
        return false;
    }

    GstBuffer* output = gst_sample_get_buffer(sample);
    GstMapInfo mapped;
    if (!gst_buffer_map(output, &mapped, GST_MAP_READ))
    {
        gst_sample_unref(sample);
        return false;
    }

    const int width = GST_VIDEO_INFO_WIDTH(&vinfo);
    const int height = GST_VIDEO_INFO_HEIGHT(&vinfo);
    const int stride = GST_VIDEO_INFO_PLANE_STRIDE(&vinfo, 0);
    const size_t minimum = (height - 1) * static_cast<size_t>(stride)
                           + width * 4ULL;
    bool success = false;
    if (width > 0 && height > 0 && stride >= width * 4 &&
        mapped.size >= minimum)
    {
        // Copy before GstBuffer is unmapped. Data remains valid for the
        // CUDA H2D transfer in the fire detector worker.
        cv::Mat wrapped(height, width, CV_8UC4, mapped.data,
                        static_cast<size_t>(stride));
        wrapped.copyTo(dst);
        success = !dst.empty();
        if (success)
            first_frame_ = false;
    }
    else
    {
        ROS_WARN_STREAM("Invalid NVIDIA JPEG output layout: " << width << "x"
                        << height << " stride=" << stride
                        << " size=" << mapped.size);
    }
    gst_buffer_unmap(output, &mapped);
    gst_sample_unref(sample);
    return success;
}

void FireHwJpeg::closeHardware()
{
    if (pipeline_)
        gst_element_set_state(GST_ELEMENT(pipeline_), GST_STATE_NULL);
    if (source_)
        gst_object_unref(GST_OBJECT(source_));
    if (sink_)
        gst_object_unref(GST_OBJECT(sink_));
    if (pipeline_)
        gst_object_unref(GST_OBJECT(pipeline_));
    pipeline_ = nullptr;
    source_ = nullptr;
    sink_ = nullptr;
}

void FireHwJpeg::fallback(const std::string& reason)
{
    ROS_WARN_STREAM("Fire NVIDIA JPEG decoder failed (" << reason
                    << "). Switching to OpenCV decoder until next launch.");
    closeHardware();
    backend_ = "opencv";
}

bool FireHwJpeg::decode(const std::vector<uint8_t>& jpeg, cv::Mat& dst)
{
    if (jpeg.empty())
        return false;

    if (backend_ == "nvidia")
    {
        if (!pipeline_ && !openHardware())
            fallback("pipeline initialization");
        if (pipeline_)
        {
            if (decodeHardware(jpeg, dst))
                return true;
            fallback("decode failure / timeout");
        }
    }

    // Baseline software path retained as explicit selectable fallback.
    cv::Mat encoded(1, static_cast<int>(jpeg.size()), CV_8UC1,
                    const_cast<uint8_t*>(jpeg.data()));
    dst = cv::imdecode(encoded, cv::IMREAD_COLOR);
    return !dst.empty();
}
