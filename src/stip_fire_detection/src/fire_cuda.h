#pragma once

#include <cuda_runtime_api.h>
#include <cstdint>

// Exactly six words, copied back to CPU for only the small set of candidates
// surviving CUDA class filtering.  The class-aware NMS remains CPU-side,
// as the number of survivors is small and GPU round trips would cost more.
struct FireCudaDetection
{
    float x1;
    float y1;
    float x2;
    float y2;
    float conf;
    int cls;
};

cudaError_t fireCudaLetterbox(
    const uint8_t* src_bgr, int src_w, int src_h, int src_pitch,
    int src_channels, void* dst_nchw_rgb, int dst_w, int dst_h,
    float scale, int pad_x, int pad_y, int resized_w, int resized_h,
    bool fp16, cudaStream_t stream);

cudaError_t fireCudaDecode(
    const void* output, bool fp16,
    bool channels_first, int channels, int candidates, int classes,
    int fire_class_id, int smoke_class_id,
    float model_conf, float fire_conf, float smoke_conf,
    int src_w, int src_h, float scale, int pad_x, int pad_y,
    FireCudaDetection* boxes, int* count, int capacity,
    cudaStream_t stream);
