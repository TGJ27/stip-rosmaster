#include <ros/ros.h>

#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <ros/topic.h>

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include "fire_cuda.h"
#include "fire_hw_jpeg.h"
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;

class TrtLogger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity > Severity::kWARNING)
            return;

        if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR)
            ROS_ERROR_STREAM("[TRT] " << msg);
        else if (severity == Severity::kWARNING)
            ROS_WARN_STREAM("[TRT] " << msg);
        else
            ROS_INFO_STREAM("[TRT] " << msg);
    }
};

template <typename T>
struct TrtDestroy
{
    void operator()(T* obj) const
    {
        if (obj)
            obj->destroy();
    }
};

inline bool cudaOk(cudaError_t status, const char* what)
{
    if (status == cudaSuccess)
        return true;

    ROS_ERROR_STREAM(what << ": " << cudaGetErrorString(status));
    return false;
}

inline size_t volume(const nvinfer1::Dims& dims)
{
    size_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i)
        v *= static_cast<size_t>(std::max(1, dims.d[i]));
    return v;
}

struct Detection
{
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    float conf = 0.0f;
    int cls = -1;
};

struct ClassState
{
    bool candidate = false;
    bool active = false;
    Clock::time_point first_seen{};
    Clock::time_point last_seen{};
};

}  // namespace


class FireDetectorCpp
{
public:
    FireDetectorCpp()
        : nh_(),
          pnh_("~")
    {
        pnh_.param<std::string>(
            "camera_topic",
            camera_topic_,
            "/inspection_camera/image_raw/compressed");

        pnh_.param<std::string>(
            "engine_path",
            engine_path_,
            std::string());

        pnh_.param("input_height", input_h_, 416);
        pnh_.param("input_width", input_w_, 736);
        pnh_.param("num_classes", num_classes_, 2);

        pnh_.param("fire_class_id", fire_class_id_, 0);
        pnh_.param("smoke_class_id", smoke_class_id_, 1);

        pnh_.param("model_confidence_threshold", model_conf_threshold_, 0.10);
        pnh_.param("fire_confidence_threshold", fire_conf_threshold_, 0.25);
        pnh_.param("smoke_confidence_threshold", smoke_conf_threshold_, 0.35);
        pnh_.param("iou_threshold", nms_iou_threshold_, 0.50);
        pnh_.param("max_det", max_det_, 20);

        // No intentional frame sampling. Every incoming JPEG is eligible.
        // If processing exceeds camera FPS, the newest pending frame replaces
        // the previous pending frame to avoid an unbounded queue / stale boxes.
        pnh_.param("fire_confirm_sec", fire_confirm_sec_, 0.50);
        pnh_.param("smoke_confirm_sec", smoke_confirm_sec_, 0.50);
        pnh_.param("candidate_miss_grace_sec", candidate_miss_grace_sec_, 0.25);
        pnh_.param("clear_delay_sec", clear_delay_sec_, 1.00);

        pnh_.param<std::string>("decoder_backend", decoder_backend_, "nvidia");
        jpeg_decoder_.reset(new FireHwJpeg(decoder_backend_));
        bool initially_enabled = true;
        pnh_.param("enabled", initially_enabled, true);
        enabled_.store(initially_enabled);

        active_pub_ = nh_.advertise<std_msgs::Bool>(
            "/fire_detection/active", 1, true);
        fire_active_pub_ = nh_.advertise<std_msgs::Bool>(
            "/fire_detection/fire_active", 1, true);
        smoke_active_pub_ = nh_.advertise<std_msgs::Bool>(
            "/fire_detection/smoke_active", 1, true);

        confidence_pub_ = nh_.advertise<std_msgs::Float32>(
            "/fire_detection/confidence", 1, true);
        fire_conf_pub_ = nh_.advertise<std_msgs::Float32>(
            "/fire_detection/fire_confidence", 1, true);
        smoke_conf_pub_ = nh_.advertise<std_msgs::Float32>(
            "/fire_detection/smoke_confidence", 1, true);

        count_pub_ = nh_.advertise<std_msgs::Int32>(
            "/fire_detection/count", 1);
        fire_count_pub_ = nh_.advertise<std_msgs::Int32>(
            "/fire_detection/fire_count", 1);
        smoke_count_pub_ = nh_.advertise<std_msgs::Int32>(
            "/fire_detection/smoke_count", 1);

        boxes_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/fire_detection/boxes", 1);
        detections_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/fire_detection/detections", 1);

        status_pub_ = nh_.advertise<std_msgs::String>(
            "/fire_detection/status", 1, true);
        inference_ms_pub_ = nh_.advertise<std_msgs::Float32>(
            "/fire_detection/inference_ms", 1);
        pipeline_ms_pub_ = nh_.advertise<std_msgs::Float32>(
            "/fire_detection/pipeline_ms", 1);

        if (!loadEngine())
            throw std::runtime_error("Failed to initialize TensorRT engine");

        toggle_service_ = nh_.advertiseService(
            "/fire_detection/set_enabled", &FireDetectorCpp::setEnabled, this);
        warmup_service_ = nh_.advertiseService(
            "/fire_detection/warmup", &FireDetectorCpp::warmupOneFrame, this);
        if (!warmup_service_)
            throw std::runtime_error("Could not advertise /fire_detection/warmup");
        ROS_INFO("STIP C++ warmup service ADVERTISED: /fire_detection/warmup (v12)");
        enabled_pub_ = nh_.advertise<std_msgs::Bool>(
            "/fire_detection/enabled", 1, true);

        // Subscribe to the high-bandwidth MJPEG topic only while enabled.
        // OFF means no JPEG ROS traffic through this C++ process at all.
        if (enabled_.load())
            subscribeCamera();

        worker_ = std::thread(&FireDetectorCpp::workerLoop, this);

        publishEnabled();
        publishStatus(enabled_.load() ? "CLEAR" : "DISABLED");
        if (!enabled_.load())
            publishEmptyDetections();

        ROS_INFO_STREAM(
            "STIP C++ FIRE+SMOKE detector ready. engine=" << engine_path_);
        ROS_INFO_STREAM(
            "Input=" << input_h_ << "x" << input_w_
            << " (HxW), classes=" << num_classes_);
        ROS_INFO_STREAM(
            "Confirmation: fire=" << fire_confirm_sec_
            << "s smoke=" << smoke_confirm_sec_
            << "s; miss grace=" << candidate_miss_grace_sec_
            << "s; clear delay=" << clear_delay_sec_ << "s");
        ROS_INFO("Every camera frame is eligible; no fixed skip interval, "
                 "no smoothing and no box-hold cache.");
        ROS_INFO_STREAM("JPEG decoder backend requested: " << decoder_backend_);
        ROS_INFO("CUDA: letterbox / RGB normalization / candidate decoding; "
                 "TensorRT FP16 GPU inference; final NMS CPU.");
    }

    ~FireDetectorCpp()
    {
        stop_.store(true);

        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_ready_ = true;
        }
        pending_cv_.notify_all();

        if (worker_.joinable())
            worker_.join();

        if (stream_)
            cudaStreamDestroy(stream_);

        if (input_device_)
            cudaFree(input_device_);

        if (output_device_)
            cudaFree(output_device_);
        if (camera_bgr_device_)
            cudaFree(camera_bgr_device_);
        if (candidate_device_)
            cudaFree(candidate_device_);
        if (candidate_counter_device_)
            cudaFree(candidate_counter_device_);
    }

private:
    bool loadEngine()
    {
        if (engine_path_.empty())
        {
            ROS_ERROR("~engine_path is empty");
            return false;
        }

        std::ifstream file(engine_path_, std::ios::binary);
        if (!file)
        {
            ROS_ERROR_STREAM("Cannot open TensorRT engine: " << engine_path_);
            return false;
        }

        file.seekg(0, std::ios::end);
        const std::streamsize file_size = file.tellg();
        file.seekg(0, std::ios::beg);

        if (file_size <= 0)
        {
            ROS_ERROR("TensorRT engine file is empty");
            return false;
        }

        // Ultralytics .engine files may contain:
        // [4-byte little-endian JSON length][JSON metadata][raw TRT plan].
        // Python Ultralytics strips this prefix before deserialization.
        // Directly passing the entire file to TensorRT produces a magic-tag error.
        std::streamoff engine_offset = 0;

        if (file_size >= 4)
        {
            uint8_t len_bytes[4] = {0, 0, 0, 0};
            if (!file.read(reinterpret_cast<char*>(len_bytes), 4))
            {
                ROS_ERROR("Failed reading TensorRT file header");
                return false;
            }

            const uint32_t metadata_len =
                static_cast<uint32_t>(len_bytes[0]) |
                (static_cast<uint32_t>(len_bytes[1]) << 8) |
                (static_cast<uint32_t>(len_bytes[2]) << 16) |
                (static_cast<uint32_t>(len_bytes[3]) << 24);

            if (metadata_len > 0 &&
                static_cast<uint64_t>(metadata_len) <=
                    static_cast<uint64_t>(file_size) - 4ULL)
            {
                std::string metadata(metadata_len, '\0');

                if (!file.read(&metadata[0], metadata_len))
                {
                    ROS_ERROR("Failed reading possible Ultralytics metadata");
                    return false;
                }

                const size_t first = metadata.find_first_not_of(" \t\r\n");
                const size_t last  = metadata.find_last_not_of(" \t\r\n");

                if (first != std::string::npos &&
                    last != std::string::npos &&
                    metadata[first] == '{' &&
                    metadata[last] == '}')
                {
                    engine_offset =
                        static_cast<std::streamoff>(4ULL + metadata_len);

                    ROS_INFO_STREAM(
                        "Detected Ultralytics engine metadata: "
                        << metadata_len
                        << " bytes; TensorRT plan starts at byte "
                        << engine_offset);
                }
            }
        }

        const std::streamsize engine_size = file_size - engine_offset;

        if (engine_size <= 0)
        {
            ROS_ERROR("No TensorRT plan bytes remain after metadata header");
            return false;
        }

        file.clear();
        file.seekg(engine_offset, std::ios::beg);

        std::vector<char> bytes(static_cast<size_t>(engine_size));
        if (!file.read(bytes.data(), engine_size))
        {
            ROS_ERROR("Failed reading TensorRT plan bytes");
            return false;
        }

        // Register standard TensorRT plugin creators before loading the plan.
        if (!initLibNvInferPlugins(&trt_logger_, ""))
            ROS_WARN("initLibNvInferPlugins() returned false");

        runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
        if (!runtime_)
        {
            ROS_ERROR("createInferRuntime() failed");
            return false;
        }

        engine_.reset(
            runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
        if (!engine_)
        {
            ROS_ERROR("deserializeCudaEngine() failed");
            return false;
        }

        context_.reset(engine_->createExecutionContext());
        if (!context_)
        {
            ROS_ERROR("createExecutionContext() failed");
            return false;
        }

        if (engine_->getNbBindings() != 2)
        {
            ROS_ERROR_STREAM(
                "Expected 2 TensorRT bindings, got "
                << engine_->getNbBindings());
            return false;
        }

        for (int i = 0; i < engine_->getNbBindings(); ++i)
        {
            if (engine_->bindingIsInput(i))
                input_index_ = i;
            else
                output_index_ = i;
        }

        if (input_index_ < 0 || output_index_ < 0)
        {
            ROS_ERROR("Could not identify TensorRT input/output bindings");
            return false;
        }

        input_dims_ = engine_->getBindingDimensions(input_index_);
        output_dims_ = engine_->getBindingDimensions(output_index_);
        input_type_ = engine_->getBindingDataType(input_index_);
        output_type_ = engine_->getBindingDataType(output_index_);

        ROS_INFO_STREAM(
            "TRT input binding " << input_index_
            << " dims=" << dimsString(input_dims_)
            << " dtype=" << dtypeString(input_type_));
        ROS_INFO_STREAM(
            "TRT output binding " << output_index_
            << " dims=" << dimsString(output_dims_)
            << " dtype=" << dtypeString(output_type_));

        // Engine was exported fixed-shape. Trust binding dims when available.
        if (input_dims_.nbDims == 4 &&
            input_dims_.d[2] > 0 &&
            input_dims_.d[3] > 0)
        {
            input_h_ = input_dims_.d[2];
            input_w_ = input_dims_.d[3];
        }

        input_count_ = volume(input_dims_);
        output_count_ = volume(output_dims_);

        const size_t input_bytes =
            input_count_ * elementSize(input_type_);
        const size_t output_bytes =
            output_count_ * elementSize(output_type_);

        if (input_bytes == 0 || output_bytes == 0)
        {
            ROS_ERROR("Unsupported TensorRT binding dtype");
            return false;
        }

        if (!cudaOk(
                cudaMalloc(&input_device_, input_bytes),
                "cudaMalloc(input)"))
            return false;

        if (!cudaOk(
                cudaMalloc(&output_device_, output_bytes),
                "cudaMalloc(output)"))
            return false;

        if (!cudaOk(
                cudaStreamCreate(&stream_),
                "cudaStreamCreate"))
            return false;

        bindings_.resize(engine_->getNbBindings(), nullptr);
        bindings_[input_index_] = input_device_;
        bindings_[output_index_] = output_device_;

        if (input_type_ != nvinfer1::DataType::kFLOAT &&
            input_type_ != nvinfer1::DataType::kHALF)
        {
            ROS_ERROR("CUDA preprocessing only supports FP32 or FP16 input");
            return false;
        }

        if (output_type_ != nvinfer1::DataType::kFLOAT &&
            output_type_ != nvinfer1::DataType::kHALF)
        {
            ROS_ERROR("CUDA decoding only supports FP32 or FP16 output");
            return false;
        }

        inferOutputLayout();
        if (input_dims_.nbDims != 4 || output_dims_.nbDims != 3 ||
            output_channels_ != 4 + num_classes_ || output_candidates_ <= 0)
        {
            ROS_ERROR("Unexpected YOLOv8 engine shape; refusing unsafe CUDA decode");
            return false;
        }

        const size_t boxes_bytes =
            static_cast<size_t>(output_candidates_) * sizeof(FireCudaDetection);
        if (!cudaOk(cudaMalloc(&candidate_device_, boxes_bytes),
                    "cudaMalloc(CUDA candidates)"))
            return false;
        if (!cudaOk(cudaMalloc(&candidate_counter_device_, sizeof(int)),
                    "cudaMalloc(CUDA candidate count)"))
            return false;
        candidate_host_.resize(static_cast<size_t>(output_candidates_));

        return true;
    }

    void inferOutputLayout()
    {
        const int expected_channels = 4 + num_classes_;

        if (output_dims_.nbDims != 3)
        {
            ROS_WARN_STREAM(
                "Unexpected output rank " << output_dims_.nbDims
                << ". Parser expects a YOLOv8 [1,C,N] or [1,N,C] output.");
            return;
        }

        const int d1 = output_dims_.d[1];
        const int d2 = output_dims_.d[2];

        if (d1 == expected_channels)
        {
            output_channels_first_ = true;
            output_channels_ = d1;
            output_candidates_ = d2;
        }
        else if (d2 == expected_channels)
        {
            output_channels_first_ = false;
            output_channels_ = d2;
            output_candidates_ = d1;
        }
        else
        {
            // Fallback: choose the smaller dimension as the channel dimension.
            output_channels_first_ = d1 < d2;
            output_channels_ = output_channels_first_ ? d1 : d2;
            output_candidates_ = output_channels_first_ ? d2 : d1;

            ROS_WARN_STREAM(
                "Could not identify YOLO output channels by class count. "
                "Using heuristic. channels=" << output_channels_
                << " candidates=" << output_candidates_);
        }

        ROS_INFO_STREAM(
            "YOLO output layout: "
            << (output_channels_first_ ? "[1,C,N]" : "[1,N,C]")
            << ", C=" << output_channels_
            << ", N=" << output_candidates_);
    }

    static std::string dimsString(const nvinfer1::Dims& d)
    {
        std::ostringstream ss;
        ss << "[";
        for (int i = 0; i < d.nbDims; ++i)
        {
            if (i)
                ss << ",";
            ss << d.d[i];
        }
        ss << "]";
        return ss.str();
    }

    static std::string dtypeString(nvinfer1::DataType t)
    {
        switch (t)
        {
            case nvinfer1::DataType::kFLOAT: return "FLOAT";
            case nvinfer1::DataType::kHALF: return "HALF";
            case nvinfer1::DataType::kINT8: return "INT8";
            case nvinfer1::DataType::kINT32: return "INT32";
            case nvinfer1::DataType::kBOOL: return "BOOL";
            default: return "UNKNOWN";
        }
    }

    static size_t elementSize(nvinfer1::DataType t)
    {
        switch (t)
        {
            case nvinfer1::DataType::kFLOAT: return sizeof(float);
            case nvinfer1::DataType::kHALF: return sizeof(__half);
            case nvinfer1::DataType::kINT8: return sizeof(int8_t);
            case nvinfer1::DataType::kINT32: return sizeof(int32_t);
            case nvinfer1::DataType::kBOOL: return sizeof(bool);
            default: return 0;
        }
    }

    void publishEnabled()
    {
        std_msgs::Bool msg;
        msg.data = enabled_.load();
        enabled_pub_.publish(msg);
    }

    // Clear visualization immediately, rather than leaving the last visible
    // boxes on the dashboard when the detector is stopped.
    void publishEmptyDetections()
    {
        std_msgs::Float64MultiArray empty;
        empty.data = {0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0};
        boxes_pub_.publish(empty);
        detections_pub_.publish(empty);
    }

    void subscribeCamera()
    {
        camera_sub_ = nh_.subscribe(
            camera_topic_, 1,
            &FireDetectorCpp::cameraCallback, this,
            ros::TransportHints().tcpNoDelay());
    }

    bool warmupOneFrame(std_srvs::Trigger::Request&,
                        std_srvs::Trigger::Response& res)
    {
        // Lock inference BEFORE toggle: same order as the worker, avoiding
        // concurrent TensorRT/CUDA use during a late/manual warmup call.
        std::lock_guard<std::mutex> infer_gate(inference_mutex_);
        // The backend must remain disabled until the user presses F.
        // Use one temporary ROS subscription to obtain exactly one frame.
        // Never publish detections, change confirmation timers or move servos.
        std::lock_guard<std::mutex> gate(toggle_mutex_);
        if (enabled_.load())
        {
            res.success = false;
            res.message = "Disable fire before running one-frame warmup";
            return true;
        }
        auto msg = ros::topic::waitForMessage<sensor_msgs::CompressedImage>(
            camera_topic_, nh_, ros::Duration(6.0));
        if (!msg || msg->data.empty())
        {
            res.success = false;
            res.message = "No fresh camera JPEG received";
            return true;
        }
        cv::Mat frame;
        std::vector<uint8_t> jpeg(msg->data.begin(), msg->data.end());
        if (!jpeg_decoder_->decode(jpeg, frame) || frame.empty())
        {
            res.success = false;
            res.message = "NVIDIA JPEG decode failed";
            return true;
        }
        float scale = 1.0f;
        int pad_x = 0, pad_y = 0;
        if (!preprocessGPU(frame, scale, pad_x, pad_y) ||
            !runTensorRTAndDecodeGPU(frame.cols, frame.rows,
                                     scale, pad_x, pad_y))
        {
            res.success = false;
            res.message = "CUDA/TensorRT single-frame warmup failed";
            return true;
        }
        const auto boxes = finalNms();
        res.success = true;
        res.message = "One JPEG decoded and one TensorRT inference completed; "
                      "fire remains OFF; candidates=" +
                      std::to_string(boxes.size());
        ROS_INFO_STREAM("C++ fire warmup OK: " << res.message);
        return true;
    }

    bool setEnabled(std_srvs::SetBool::Request& req,
                    std_srvs::SetBool::Response& res)
    {
        // Serialize enabling/disabling with completed inference publication.
        // In-flight GPU inference may finish, but cannot publish an old result
        // after the disable service has returned successfully.
        std::lock_guard<std::mutex> gate(toggle_mutex_);
        const bool target = req.data;

        if (target != enabled_.load())
        {
            state_epoch_.fetch_add(1);
            enabled_.store(target);

            {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                pending_jpeg_.clear();
                pending_ready_ = false;
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                fire_state_ = ClassState{};
                smoke_state_ = ClassState{};
                status_ = target ? "CLEAR" : "DISABLED";
            }

            if (target)
                subscribeCamera();
            else
                camera_sub_.shutdown();

            publishEmptyDetections();
            publishPrimitiveTopics(false, false, false,
                                   0.0f, 0.0f, 0.0f,
                                   0, 0, 0,
                                   target ? "CLEAR" : "DISABLED");
        }
        publishEnabled();
        res.success = true;
        res.message = target ? "Fire/smoke GPU detection ON"
                             : "Fire/smoke GPU detection OFF";
        ROS_INFO_STREAM(res.message);
        return true;
    }

    void cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        if (!enabled_.load())
            return;
        const uint64_t epoch = state_epoch_.load();

        // No planned skipped frames. Only one in-flight frame can be inferred;
        // this one-slot handoff favors freshness if processing overruns 33ms.
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            if (!enabled_.load() || epoch != state_epoch_.load())
                return;
            pending_jpeg_.assign(msg->data.begin(), msg->data.end());
            pending_stamp_ = msg->header.stamp;
            pending_epoch_ = epoch;
            pending_ready_ = true;
        }
        pending_cv_.notify_one();
    }

    void workerLoop()
    {
        while (ros::ok() && !stop_.load())
        {
            std::vector<uint8_t> jpeg;
            ros::Time stamp;
            uint64_t frame_epoch = 0;
            {
                std::unique_lock<std::mutex> lock(pending_mutex_);
                pending_cv_.wait_for(
                    lock, std::chrono::milliseconds(500),
                    [&]() { return pending_ready_ || stop_.load() || !ros::ok(); });

                if (stop_.load() || !ros::ok())
                    break;
                if (!pending_ready_)
                    continue;

                jpeg.swap(pending_jpeg_);
                stamp = pending_stamp_;
                frame_epoch = pending_epoch_;
                pending_ready_ = false;
            }
            if (!enabled_.load() || frame_epoch != state_epoch_.load()
                || jpeg.empty())
                continue;

            // Serialize CUDA engine and JPEG decoder against one-shot warmup.
            std::lock_guard<std::mutex> infer_gate(inference_mutex_);
            const auto pipeline_start = Clock::now();
            cv::Mat frame;
            if (!jpeg_decoder_->decode(jpeg, frame))
            {
                ROS_WARN_THROTTLE(2.0, "C++ fire detector JPEG decode failed");
                continue;
            }
            const auto inference_start = Clock::now();
            float scale = 1.0f;
            int pad_x = 0;
            int pad_y = 0;

            if (!preprocessGPU(frame, scale, pad_x, pad_y))
            {
                publishStatus("ERROR");
                continue;
            }
            if (!runTensorRTAndDecodeGPU(frame.cols, frame.rows,
                                         scale, pad_x, pad_y))
            {
                publishStatus("ERROR");
                continue;
            }

            // GPU has already decoded and thresholded all YOLO candidates.
            // The remaining few boxes need only a cheap class-aware greedy NMS.
            const std::vector<Detection> detections = finalNms();

            // A disable/enable during inference invalidates this entire frame.
            std::lock_guard<std::mutex> gate(toggle_mutex_);
            if (!enabled_.load() || frame_epoch != state_epoch_.load())
                continue;

            const auto now = Clock::now();
            updateTemporalState(detections, now);

            const auto finished = Clock::now();
            const float inference_ms = static_cast<float>(
                std::chrono::duration<double, std::milli>(
                    finished - inference_start).count());
            const float pipeline_ms = static_cast<float>(
                std::chrono::duration<double, std::milli>(
                    finished - pipeline_start).count());

            std_msgs::Float32 ms;
            ms.data = inference_ms;
            inference_ms_pub_.publish(ms);
            ms.data = pipeline_ms;
            pipeline_ms_pub_.publish(ms);

            publishFrameDetections(stamp, detections, frame.cols, frame.rows);
        }
    }

    bool preprocessGPU(
        const cv::Mat& frame,
        float& scale, int& pad_x, int& pad_y)
    {
        const int source_w = frame.cols;
        const int source_h = frame.rows;
        const int source_channels = frame.channels();
        if (source_w <= 0 || source_h <= 0 ||
            (source_channels != 3 && source_channels != 4) ||
            frame.depth() != CV_8U)
            return false;

        const size_t input_pitch =
            static_cast<size_t>(source_w) * source_channels;
        const size_t needed_bytes = input_pitch * source_h;
        if (needed_bytes > camera_bgr_capacity_)
        {
            if (camera_bgr_device_)
                cudaFree(camera_bgr_device_);
            camera_bgr_device_ = nullptr;
            camera_bgr_capacity_ = 0;
            if (!cudaOk(cudaMalloc(&camera_bgr_device_, needed_bytes),
                        "cudaMalloc(camera BGR)"))
                return false;
            camera_bgr_capacity_ = needed_bytes;
        }

        // MJPEG decode stays on CPU (the proven USB-camera decoder).
        // This copy is needed to move decoded BGR pixels to CUDA memory.
        if (!cudaOk(cudaMemcpy2DAsync(
                camera_bgr_device_, input_pitch,
                frame.data, frame.step,
                input_pitch, source_h,
                cudaMemcpyHostToDevice, stream_), "camera H2D"))
            return false;

        scale = std::min(
            static_cast<float>(input_w_) / source_w,
            static_cast<float>(input_h_) / source_h);
        const int resized_w = static_cast<int>(std::round(source_w * scale));
        const int resized_h = static_cast<int>(std::round(source_h * scale));
        pad_x = (input_w_ - resized_w) / 2;
        pad_y = (input_h_ - resized_h) / 2;

        const cudaError_t rc = fireCudaLetterbox(
            static_cast<const uint8_t*>(camera_bgr_device_),
            source_w, source_h, static_cast<int>(input_pitch),
            source_channels, input_device_, input_w_, input_h_,
            scale, pad_x, pad_y, resized_w, resized_h,
            input_type_ == nvinfer1::DataType::kHALF,
            stream_);
        return cudaOk(rc, "CUDA letterbox/normalize");
    }

    bool runTensorRTAndDecodeGPU(int source_w, int source_h,
                                float scale, int pad_x, int pad_y)
    {
        if (!context_->enqueueV2(bindings_.data(), stream_, nullptr))
        {
            ROS_ERROR_THROTTLE(2.0, "TensorRT enqueueV2() failed");
            return false;
        }

        if (!cudaOk(cudaMemsetAsync(candidate_counter_device_, 0,
                                   sizeof(int), stream_), "CUDA reset candidates"))
            return false;

        const cudaError_t rc = fireCudaDecode(
            output_device_, output_type_ == nvinfer1::DataType::kHALF,
            output_channels_first_, output_channels_, output_candidates_,
            num_classes_, fire_class_id_, smoke_class_id_,
            static_cast<float>(model_conf_threshold_),
            static_cast<float>(fire_conf_threshold_),
            static_cast<float>(smoke_conf_threshold_),
            source_w, source_h, scale, pad_x, pad_y,
            static_cast<FireCudaDetection*>(candidate_device_),
            static_cast<int*>(candidate_counter_device_),
            output_candidates_, stream_);
        if (!cudaOk(rc, "CUDA YOLOv8 decode"))
            return false;

        // Compact candidate transfer (~24 B/candidate), NOT full YOLO tensor.
        if (!cudaOk(cudaMemcpyAsync(
                &candidate_count_host_, candidate_counter_device_, sizeof(int),
                cudaMemcpyDeviceToHost, stream_), "CUDA candidate count D2H"))
            return false;
        if (!cudaOk(cudaMemcpyAsync(
                candidate_host_.data(), candidate_device_,
                candidate_host_.size() * sizeof(FireCudaDetection),
                cudaMemcpyDeviceToHost, stream_), "CUDA candidate D2H"))
            return false;
        if (!cudaOk(cudaStreamSynchronize(stream_), "CUDA stream synchronize"))
            return false;
        candidate_count_host_ = std::max(
            0, std::min(candidate_count_host_, output_candidates_));
        return true;
    }

    std::vector<Detection> finalNms()
    {
        std::vector<Detection> proposals;
        proposals.reserve(static_cast<size_t>(candidate_count_host_));
        for (int i = 0; i < candidate_count_host_; ++i)
        {
            const FireCudaDetection& c = candidate_host_[i];
            Detection d;
            d.x1 = c.x1; d.y1 = c.y1;
            d.x2 = c.x2; d.y2 = c.y2;
            d.conf = c.conf; d.cls = c.cls;
            proposals.push_back(d);
        }
        std::sort(proposals.begin(), proposals.end(),
                  [](const Detection& a, const Detection& b) {
                      return a.conf > b.conf;
                  });
        std::vector<Detection> kept;
        kept.reserve(std::min<size_t>(max_det_, proposals.size()));
        for (const auto& d : proposals)
        {
            bool suppress = false;
            for (const auto& k : kept)
            {
                if (d.cls == k.cls && iou(d, k) > nms_iou_threshold_)
                {
                    suppress = true;
                    break;
                }
            }
            if (!suppress)
            {
                kept.push_back(d);
                if (static_cast<int>(kept.size()) >= max_det_)
                    break;
            }
        }
        return kept;
    }

    static float iou(const Detection& a, const Detection& b)
    {
        const float x1 = std::max(a.x1, b.x1);
        const float y1 = std::max(a.y1, b.y1);
        const float x2 = std::min(a.x2, b.x2);
        const float y2 = std::min(a.y2, b.y2);
        const float inter = std::max(0.0f, x2 - x1) *
                            std::max(0.0f, y2 - y1);
        const float aa = std::max(0.0f, a.x2 - a.x1) *
                         std::max(0.0f, a.y2 - a.y1);
        const float ab = std::max(0.0f, b.x2 - b.x1) *
                         std::max(0.0f, b.y2 - b.y1);
        const float denom = aa + ab - inter;
        return denom > 0.0f ? inter / denom : 0.0f;
    }

    void updateOneClassState(
        ClassState& state,
        bool positive,
        const Clock::time_point& now,
        double confirm_sec)
    {
        if (positive)
        {
            if (!state.candidate)
            {
                state.candidate = true;
                state.first_seen = now;
            }
            else if (
                state.last_seen.time_since_epoch().count() > 0 &&
                secondsSince(state.last_seen, now) >
                    candidate_miss_grace_sec_)
            {
                // Reappearance after too long a gap starts a new candidate.
                state.first_seen = now;
            }

            state.last_seen = now;

            if (!state.active &&
                secondsSince(state.first_seen, now) >= confirm_sec)
            {
                state.active = true;
            }

            return;
        }

        if (state.last_seen.time_since_epoch().count() == 0)
            return;

        const double missing_sec =
            secondsSince(state.last_seen, now);

        if (!state.active)
        {
            if (state.candidate &&
                missing_sec > candidate_miss_grace_sec_)
            {
                state.candidate = false;
            }
        }
        else
        {
            if (missing_sec > clear_delay_sec_)
            {
                state.active = false;
                state.candidate = false;
            }
        }
    }

    void updateTemporalState(
        const std::vector<Detection>& detections,
        const Clock::time_point& now)
    {
        bool fire_positive = false;
        bool smoke_positive = false;

        for (const auto& d : detections)
        {
            if (d.cls == fire_class_id_)
                fire_positive = true;
            else if (d.cls == smoke_class_id_)
                smoke_positive = true;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        updateOneClassState(
            fire_state_,
            fire_positive,
            now,
            fire_confirm_sec_);

        updateOneClassState(
            smoke_state_,
            smoke_positive,
            now,
            smoke_confirm_sec_);

        updateStatusLocked();
    }

    void updateStatusLocked()
    {
        if (fire_state_.active)
            status_ = "FIRE";
        else if (smoke_state_.active)
            status_ = "SMOKE";
        else if (fire_state_.candidate)
            status_ = "POSSIBLE_FIRE";
        else if (smoke_state_.candidate)
            status_ = "POSSIBLE_SMOKE";
        else
            status_ = "CLEAR";
    }

    static double secondsSince(
        const Clock::time_point& start,
        const Clock::time_point& end)
    {
        return std::chrono::duration<double>(end - start).count();
    }

    void publishFrameDetections(
        const ros::Time& stamp,
        const std::vector<Detection>& detections,
        int source_w, int source_h)
    {
        bool fire_active = false;
        bool smoke_active = false;
        std::string status;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            fire_active = fire_state_.active;
            smoke_active = smoke_state_.active;
            status = status_;
        }

        float max_conf = 0.0f;
        float fire_conf = 0.0f;
        float smoke_conf = 0.0f;
        int fire_count = 0;
        int smoke_count = 0;

        // Only boxes from THIS inference, no coordinate averaging/hold.
        for (const auto& d : detections)
        {
            max_conf = std::max(max_conf, d.conf);
            if (d.cls == fire_class_id_)
            {
                fire_conf = std::max(fire_conf, d.conf);
                ++fire_count;
            }
            else if (d.cls == smoke_class_id_)
            {
                smoke_conf = std::max(smoke_conf, d.conf);
                ++smoke_count;
            }
        }

        std_msgs::Float64MultiArray legacy;
        legacy.data.reserve(8 + detections.size() * 5);
        legacy.data.push_back(stamp.toSec());
        legacy.data.push_back(stamp.toSec());
        legacy.data.push_back(source_w);
        legacy.data.push_back(source_h);
        legacy.data.push_back(fire_active || smoke_active ? 1.0 : 0.0);
        legacy.data.push_back(max_conf);
        legacy.data.push_back(0.0);  // Fresh result, age = zero
        legacy.data.push_back(static_cast<double>(detections.size()));

        std_msgs::Float64MultiArray typed;
        typed.data.assign(legacy.data.begin(), legacy.data.end());
        typed.data.reserve(8 + detections.size() * 6);

        for (const auto& d : detections)
        {
            legacy.data.insert(legacy.data.end(),
                               {d.x1, d.y1, d.x2, d.y2, d.conf});
            typed.data.insert(typed.data.end(),
                              {d.x1, d.y1, d.x2, d.y2, d.conf,
                               static_cast<double>(d.cls)});
        }
        boxes_pub_.publish(legacy);
        detections_pub_.publish(typed);
        publishPrimitiveTopics(
            fire_active || smoke_active, fire_active, smoke_active,
            max_conf, fire_conf, smoke_conf,
            static_cast<int>(detections.size()),
            fire_count, smoke_count, status);
    }

    void publishPrimitiveTopics(
        bool active,
        bool fire_active,
        bool smoke_active,
        float max_conf,
        float fire_conf,
        float smoke_conf,
        int count,
        int fire_count,
        int smoke_count,
        const std::string& status)
    {
        std_msgs::Bool b;
        b.data = active;
        active_pub_.publish(b);

        b.data = fire_active;
        fire_active_pub_.publish(b);

        b.data = smoke_active;
        smoke_active_pub_.publish(b);

        std_msgs::Float32 f;
        f.data = max_conf;
        confidence_pub_.publish(f);

        f.data = fire_conf;
        fire_conf_pub_.publish(f);

        f.data = smoke_conf;
        smoke_conf_pub_.publish(f);

        std_msgs::Int32 i;
        i.data = count;
        count_pub_.publish(i);

        i.data = fire_count;
        fire_count_pub_.publish(i);

        i.data = smoke_count;
        smoke_count_pub_.publish(i);

        publishStatus(status);
    }

    void publishStatus(const std::string& s)
    {
        std::lock_guard<std::mutex> lock(status_pub_mutex_);

        if (s == last_published_status_)
            return;

        last_published_status_ = s;

        std_msgs::String msg;
        msg.data = s;
        status_pub_.publish(msg);

        ROS_INFO_STREAM("C++ fire detector status -> " << s);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber camera_sub_;
    ros::ServiceServer toggle_service_;
    ros::ServiceServer warmup_service_;
    ros::Publisher enabled_pub_;

    ros::Publisher active_pub_;
    ros::Publisher fire_active_pub_;
    ros::Publisher smoke_active_pub_;

    ros::Publisher confidence_pub_;
    ros::Publisher fire_conf_pub_;
    ros::Publisher smoke_conf_pub_;

    ros::Publisher count_pub_;
    ros::Publisher fire_count_pub_;
    ros::Publisher smoke_count_pub_;

    ros::Publisher boxes_pub_;
    ros::Publisher detections_pub_;
    ros::Publisher status_pub_;
    ros::Publisher inference_ms_pub_;
    ros::Publisher pipeline_ms_pub_;

    std::string camera_topic_;
    std::string engine_path_;
    std::string decoder_backend_ = "nvidia";
    std::unique_ptr<FireHwJpeg> jpeg_decoder_;

    int input_h_ = 416;
    int input_w_ = 736;
    int num_classes_ = 2;

    int fire_class_id_ = 0;
    int smoke_class_id_ = 1;

    double model_conf_threshold_ = 0.10;
    double fire_conf_threshold_ = 0.25;
    double smoke_conf_threshold_ = 0.35;
    double nms_iou_threshold_ = 0.50;

    int max_det_ = 20;

    double fire_confirm_sec_ = 0.50;
    double smoke_confirm_sec_ = 0.50;
    double candidate_miss_grace_sec_ = 0.25;
    double clear_delay_sec_ = 1.00;

    std::atomic<bool> enabled_{true};
    std::atomic<uint64_t> state_epoch_{0};
    std::mutex toggle_mutex_;
    std::mutex inference_mutex_;

    TrtLogger trt_logger_;

    std::unique_ptr<nvinfer1::IRuntime, TrtDestroy<nvinfer1::IRuntime>>
        runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine, TrtDestroy<nvinfer1::ICudaEngine>>
        engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, TrtDestroy<nvinfer1::IExecutionContext>>
        context_;

    int input_index_ = -1;
    int output_index_ = -1;

    nvinfer1::Dims input_dims_{};
    nvinfer1::Dims output_dims_{};
    nvinfer1::DataType input_type_ = nvinfer1::DataType::kFLOAT;
    nvinfer1::DataType output_type_ = nvinfer1::DataType::kFLOAT;

    size_t input_count_ = 0;
    size_t output_count_ = 0;

    bool output_channels_first_ = true;
    int output_channels_ = 0;
    int output_candidates_ = 0;

    void* input_device_ = nullptr;
    void* output_device_ = nullptr;
    cudaStream_t stream_ = nullptr;
    std::vector<void*> bindings_;

    void* camera_bgr_device_ = nullptr;
    size_t camera_bgr_capacity_ = 0;
    void* candidate_device_ = nullptr;
    void* candidate_counter_device_ = nullptr;
    int candidate_count_host_ = 0;
    std::vector<FireCudaDetection> candidate_host_;

    std::atomic<bool> stop_{false};

    std::thread worker_;
    std::mutex pending_mutex_;
    std::condition_variable pending_cv_;
    std::vector<uint8_t> pending_jpeg_;
    ros::Time pending_stamp_;
    bool pending_ready_ = false;
    uint64_t pending_epoch_ = 0;

    std::mutex state_mutex_;
    ClassState fire_state_;
    ClassState smoke_state_;
    std::string status_ = "STARTING";

    std::mutex status_pub_mutex_;
    std::string last_published_status_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stip_fire_detector_cpp");

    try
    {
        FireDetectorCpp node;

        ros::AsyncSpinner spinner(2);
        spinner.start();

        ros::waitForShutdown();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL_STREAM("C++ fire detector fatal error: " << e.what());
        return 1;
    }

    return 0;
}
