#include "fire_cuda.h"
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <cmath>

namespace {

__device__ __forceinline__ unsigned char bgrPixel(
    const unsigned char* src, int pitch, int w, int h,
    int x, int y, int channel, int src_channels)
{
    x = max(0, min(x, w - 1));
    y = max(0, min(y, h - 1));
    return src[y * pitch + src_channels * x + channel];
}

template <typename T>
__device__ __forceinline__ void writeNorm(T* dst, int index, float value)
{
    dst[index] = static_cast<T>(value);
}

template <>
__device__ __forceinline__ void writeNorm<__half>(
    __half* dst, int index, float value)
{
    dst[index] = __float2half_rn(value);
}

// CUDA fused letterbox + bilinear scaling + BGR to planar RGB + normalization.
// Output directly fills the TensorRT input buffer; no separate conversion.
template <typename T>
__global__ void letterboxKernel(
    const unsigned char* src, int sw, int sh, int src_pitch,
    int src_channels, T* dst, int dw, int dh,
    int pad_x, int pad_y, int rw, int rh)
{
    const int tid = blockIdx.x * blockDim.x + threadIdx.x;
    const int pixels = dw * dh;
    if (tid >= pixels) return;
    const int x = tid % dw;
    const int y = tid / dw;
    float rgb[3] = {114.0f, 114.0f, 114.0f};

    if (x >= pad_x && x < pad_x + rw &&
        y >= pad_y && y < pad_y + rh)
    {
        // OpenCV's bilinear resize uses center-aligned sampling.
        const float fx = ((x - pad_x) + 0.5f) *
                         (static_cast<float>(sw) / rw) - 0.5f;
        const float fy = ((y - pad_y) + 0.5f) *
                         (static_cast<float>(sh) / rh) - 0.5f;
        const int x0 = static_cast<int>(floorf(fx));
        const int y0 = static_cast<int>(floorf(fy));
        const float ax = fx - x0;
        const float ay = fy - y0;

        // Input BGR -> output RGB.
        for (int c = 0; c < 3; ++c)
        {
            const int channel = 2 - c;
            const float a = bgrPixel(src, src_pitch, sw, sh, x0,     y0,     channel, src_channels);
            const float b = bgrPixel(src, src_pitch, sw, sh, x0 + 1, y0,     channel, src_channels);
            const float d = bgrPixel(src, src_pitch, sw, sh, x0,     y0 + 1, channel, src_channels);
            const float e = bgrPixel(src, src_pitch, sw, sh, x0 + 1, y0 + 1, channel, src_channels);
            rgb[c] = (1.0f - ay) * ((1.0f - ax) * a + ax * b)
                   + ay * ((1.0f - ax) * d + ax * e);
        }
    }

    const float inv255 = 1.0f / 255.0f;
    writeNorm(dst, tid,              rgb[0] * inv255);
    writeNorm(dst, tid + pixels,     rgb[1] * inv255);
    writeNorm(dst, tid + 2 * pixels, rgb[2] * inv255);
}

__device__ __forceinline__ float outputAt(
    const float* out, int c, int i, int nc, int n,
    bool channels_first)
{
    return out[channels_first ? c * n + i : i * nc + c];
}

__device__ __forceinline__ float outputAt(
    const __half* out, int c, int i, int nc, int n,
    bool channels_first)
{
    return __half2float(out[channels_first ? c * n + i : i * nc + c]);
}

template <typename T>
__global__ void decodeKernel(
    const T* out, bool channels_first, int channels,
    int n, int num_classes, int fire_id, int smoke_id,
    float model_conf, float fire_conf, float smoke_conf,
    int sw, int sh, float scale, int px, int py,
    FireCudaDetection* boxes, int* count, int capacity)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float score = -1.0f;
    int cls = -1;
    for (int c = 0; c < num_classes; ++c)
    {
        const float s = outputAt(out, 4 + c, i, channels, n, channels_first);
        if (s > score) { score = s; cls = c; }
    }

    if (!isfinite(score) || score < model_conf) return;
    if (cls == fire_id) { if (score < fire_conf) return; }
    else if (cls == smoke_id) { if (score < smoke_conf) return; }
    else return;

    const float cx = outputAt(out, 0, i, channels, n, channels_first);
    const float cy = outputAt(out, 1, i, channels, n, channels_first);
    const float w = outputAt(out, 2, i, channels, n, channels_first);
    const float h = outputAt(out, 3, i, channels, n, channels_first);
    if (!isfinite(cx) || !isfinite(cy) || !isfinite(w) || !isfinite(h)) return;

    const float x1 = fminf(sw - 1.0f, fmaxf(0.0f, (cx - 0.5f * w - px) / scale));
    const float y1 = fminf(sh - 1.0f, fmaxf(0.0f, (cy - 0.5f * h - py) / scale));
    const float x2 = fminf(sw - 1.0f, fmaxf(0.0f, (cx + 0.5f * w - px) / scale));
    const float y2 = fminf(sh - 1.0f, fmaxf(0.0f, (cy + 0.5f * h - py) / scale));
    if (!(x2 > x1 && y2 > y1)) return;

    const int idx = atomicAdd(count, 1);
    if (idx >= capacity) return;
    boxes[idx] = {x1, y1, x2, y2, score, cls};
}

} // namespace

cudaError_t fireCudaLetterbox(
    const uint8_t* src_bgr, int src_w, int src_h, int src_pitch,
    int src_channels, void* dst_nchw_rgb, int dst_w, int dst_h,
    float scale, int pad_x, int pad_y, int resized_w, int resized_h,
    bool fp16, cudaStream_t stream)
{
    (void)scale;  // dims are derived on the host for exact letterbox placement.
    const int threads = 256;
    const int blocks = (dst_w * dst_h + threads - 1) / threads;
    if (fp16)
        letterboxKernel<<<blocks, threads, 0, stream>>>(
            src_bgr, src_w, src_h, src_pitch, src_channels,
            reinterpret_cast<__half*>(dst_nchw_rgb), dst_w, dst_h,
            pad_x, pad_y, resized_w, resized_h);
    else
        letterboxKernel<<<blocks, threads, 0, stream>>>(
            src_bgr, src_w, src_h, src_pitch, src_channels,
            reinterpret_cast<float*>(dst_nchw_rgb), dst_w, dst_h,
            pad_x, pad_y, resized_w, resized_h);
    return cudaGetLastError();
}

cudaError_t fireCudaDecode(
    const void* output, bool fp16,
    bool channels_first, int channels, int candidates, int classes,
    int fire_class_id, int smoke_class_id,
    float model_conf, float fire_conf, float smoke_conf,
    int src_w, int src_h, float scale, int pad_x, int pad_y,
    FireCudaDetection* boxes, int* count, int capacity,
    cudaStream_t stream)
{
    const int threads = 256;
    const int blocks = (candidates + threads - 1) / threads;
    if (fp16)
        decodeKernel<<<blocks, threads, 0, stream>>>(
            reinterpret_cast<const __half*>(output), channels_first,
            channels, candidates, classes, fire_class_id, smoke_class_id,
            model_conf, fire_conf, smoke_conf,
            src_w, src_h, scale, pad_x, pad_y,
            boxes, count, capacity);
    else
        decodeKernel<<<blocks, threads, 0, stream>>>(
            reinterpret_cast<const float*>(output), channels_first,
            channels, candidates, classes, fire_class_id, smoke_class_id,
            model_conf, fire_conf, smoke_conf,
            src_w, src_h, scale, pad_x, pad_y,
            boxes, count, capacity);
    return cudaGetLastError();
}
