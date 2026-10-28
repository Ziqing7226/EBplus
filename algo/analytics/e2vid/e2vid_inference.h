// algo/analytics/e2vid/e2vid_inference.h — E2VID neural-network inference.
//
// Design §4.4.2 (E2VID DL path). Wraps the E2VID / UNet-Recurrent model
// inference for event-based grayscale reconstruction. Ported from
// rpg_e2vid (image_reconstructor.py, model/unet.py).
//
// Architecture (rpg_e2vid):
//   - Input: 1 x num_bins x H x W event voxel grid (float32)
//   - Model: UNet or UNetRecurrent (ConvLSTM/ConvGRU) with skip connections,
//     4 encoders, 2 residual blocks, 4 decoders, sigmoid output
//   - Output: 1 x 1 x H x W grayscale image in [0, 1]
//
// Backends:
//   - OpenVINO GPU (preferred when available, §4.4.2-GPU): same .onnx model,
//     compiled for the Intel GPU plugin with static effective dims. Selected
//     by the device policy (Auto/GPU); any load or runtime failure degrades
//     to the ONNX Runtime CPU path.
//   - ONNX Runtime CPU: load exported .onnx model, run inference.
//     Conditionally compiled when ONNX Runtime is found via CMake.
//   - Heuristic fallback (always available): when no model is loaded,
//     reconstructs by summing voxel bins and applying sigmoid-like mapping.
//     This produces a crude but usable preview without the neural network.
//
// The CropParameters logic (padding to power-of-2 divisible sizes) is also
// implemented to match the original rpg_e2vid preprocessing. Header-only.

#ifndef GUI_ALGO_ANALYTICS_E2VID_E2VID_INFERENCE_H
#define GUI_ALGO_ANALYTICS_E2VID_E2VID_INFERENCE_H

#include <algorithm>
#include <filesystem>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <fstream>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algo/common/event.h"
#include "algo/analytics/e2vid/event_voxel_grid.h"

// Conditional ONNX Runtime support.
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
#include <onnxruntime_cxx_api.h>
#endif

// Conditional OpenVINO support (Intel iGPU/dGPU inference, §4.4.2-GPU).
// Consumes the SAME exported .onnx models as the ONNX Runtime path — no
// model-format divergence. When the build has OpenVINO and an Intel GPU is
// present, Auto/GPU device policies route inference through the GPU plugin
// (measured ~11× faster than ONNX Runtime CPU for E2VID at 128×72 on a
// Meteor Lake iGPU); any failure falls back to ONNX Runtime CPU.
#if defined(GUI_ALGO_HAS_OPENVINO)
#include <cstring>
#include <map>
#include <openvino/openvino.hpp>
#endif

namespace gui_algo {

/// @brief Crop/padding parameters for UNet (matches rpg_e2vid CropParameters).
struct E2VIDCropParams {
    int width{0};
    int height{0};
    int crop_width{0};   ///< Padded width (divisible by 2^num_encoders).
    int crop_height{0};  ///< Padded height (divisible by 2^num_encoders).
    int pad_top{0};
    int pad_bottom{0};
    int pad_left{0};
    int pad_right{0};

    /// @brief Computes crop parameters for a given sensor size and UNet depth.
    static E2VIDCropParams compute(int width, int height, int num_encoders) {
        E2VIDCropParams p;
        p.width = width;
        p.height = height;
        const int factor = 1 << num_encoders;  // 2^num_encoders
        p.crop_width = optimal_crop_size(width, factor);
        p.crop_height = optimal_crop_size(height, factor);
        p.pad_top = (p.crop_height - height + 1) / 2;
        p.pad_bottom = (p.crop_height - height) / 2;
        p.pad_left = (p.crop_width - width + 1) / 2;
        p.pad_right = (p.crop_width - width) / 2;
        return p;
    }

    /// @brief Pads a CV_32FC1 image (HxW) to crop size using reflection.
    /// Uses BORDER_REFLECT_101 to match PyTorch's ReflectionPad2d semantics
    /// (edge sample is NOT repeated: gfedcb|abcdefgh|gfedcba).
    cv::Mat pad(const cv::Mat& img) const {
        cv::Mat padded;
        cv::copyMakeBorder(img, padded,
                           pad_top, pad_bottom, pad_left, pad_right,
                           cv::BORDER_REFLECT_101);
        return padded;
    }

    /// @brief Crops the center region back to the original sensor size.
    cv::Mat crop(const cv::Mat& img) const {
        const int cx = crop_width / 2;
        const int cy = crop_height / 2;
        const int x0 = cx - width / 2;
        const int y0 = cy - height / 2;
        return img(cv::Rect(x0, y0, width, height)).clone();
    }

private:
    static int optimal_crop_size(int max_size, int factor) {
        int crop = factor;
        while (crop < max_size) crop += factor;
        return crop;
    }
};

/// @brief E2VID model inference wrapper with ONNX Runtime and heuristic backend.
class E2VIDInference {
public:
    /// @brief Constructs the inference engine.
    /// @param width,height Sensor dimensions.
    /// @param num_bins Number of event tensor temporal bins (E2VID default: 5).
    /// @param num_encoders UNet encoder depth (default: 4).
    E2VIDInference(int width, int height, int num_bins = 5,
                   int num_encoders = 4)
        : width_(width), height_(height),
          num_bins_(clamp_bins(num_bins)),
          num_encoders_(num_encoders),
          crop_(E2VIDCropParams::compute(
              effective_dim(width), effective_dim(height), num_encoders)),
          full_crop_(E2VIDCropParams::compute(width, height, num_encoders)),
          voxel_grid_(effective_dim(width), effective_dim(height), num_bins_) {}

    /// @brief Loads an ONNX model from file.
    /// @return true if the model was loaded successfully.
    ///
    /// On success the number of input bins (num_bins_) is synchronised to the
    /// model's first-input channel dimension. This mirrors rpg_e2vid, where
    /// num_bins is a property of the model (config['num_bins'] / model.num_bins)
    /// rather than a free user parameter — see run_reconstruction.py:55 and
    /// model/model.py:14. Letting the user freely change num_bins after a model
    /// is loaded would mismatch the model's input channels and break inference.
    /// @brief Loads an ONNX model from file, selecting the runtime by device
    /// policy (§4.4.2-GPU): Auto/GPU prefer OpenVINO GPU when the build has
    /// OpenVINO and an Intel GPU is enumerated; otherwise — and on any
    /// OpenVINO load failure — ONNX Runtime CPU is used. num_bins is
    /// synchronised to the model's first-input channel dimension either way
    /// (rpg_e2vid: num_bins is a property of the model, see model.py:14).
    /// @return true if the model was loaded successfully.
    bool load_model(const std::string& model_path) {
        model_path_ = model_path;
#if defined(GUI_ALGO_HAS_OPENVINO)
        ov_release();  // drop any previous runtime state
#endif
        // Empty path = explicit unload (a fresh backend before config
        // restore, or a mode switch to a DL mode without weights): fall
        // back to the heuristic WITHOUT probing the runtimes. A missing
        // file degrades just as quietly — only a file that EXISTS but
        // fails to load prints a diagnostic (the status line already
        // shows model=heuristic in the quiet cases).
        if (model_path.empty() ||
            !std::ifstream(model_path, std::ios::binary).good()) {
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
            session_.reset();
#endif
            model_loaded_ = false;
            active_runtime_.clear();
            return false;
        }
#if defined(GUI_ALGO_HAS_OPENVINO)
        if (device_ != Device::CPU && ov_gpu_available() && ov_try_load(model_path)) {
            model_loaded_ = true;
            active_runtime_ = "gpu";
            return true;
        }
        ov_release();
#endif
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
        if (load_model_ort(model_path)) {
            model_loaded_ = true;
            active_runtime_ = "cpu";
            return true;
        }
#endif
        model_loaded_ = false;
        active_runtime_.clear();
        return false;
    }

#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
    /// ONNX Runtime CPU load path (also the live fallback for the OpenVINO
    /// GPU runtime). Returns false on any Ort::Exception.
    bool load_model_ort(const std::string& model_path) {
        try {
            env_ = std::make_unique<Ort::Env>(
                ORT_LOGGING_LEVEL_WARNING, "e2vid");
            Ort::SessionOptions session_opts;
            // Use all available CPU cores for ONNX inference (capped at 8 to
            // avoid oversubscription on high-core machines). The E2VID
            // UNetRecurrent model is compute-bound on Conv/MatMul ops, which
            // ONNX Runtime parallelises across the intra-op thread pool.
            // Single-thread (the previous setting) was the main bottleneck.
            const unsigned hw_threads = std::thread::hardware_concurrency();
            const int num_threads = static_cast<int>(
                hw_threads > 0 ? (hw_threads <= 8 ? hw_threads : 8) : 4);
            session_opts.SetIntraOpNumThreads(num_threads);
            session_opts.SetGraphOptimizationLevel(
                GraphOptimizationLevel::ORT_ENABLE_ALL);
            session_ = std::make_unique<Ort::Session>(
#ifdef _WIN32
                // ORTCHAR_T is wchar_t on Windows: the C++ API only accepts
                // a wide model path here (there is no narrow overload).
                *env_,
                std::filesystem::path(model_path).wstring().c_str(),
                session_opts);
#else
                *env_, model_path.c_str(), session_opts);
#endif
            sync_num_bins_from_model();
            // Cache MemoryInfo (constant for the session lifetime).
            mem_info_ = std::make_unique<Ort::MemoryInfo>(
                Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            // Cache input/output names once (avoids 14 string allocations
            // per frame in infer_onnx()).
            Ort::AllocatorWithDefaultOptions allocator;
            const std::size_t n_in = session_->GetInputCount();
            const std::size_t n_out = session_->GetOutputCount();
            input_name_owners_.clear();
            output_name_owners_.clear();
            cached_input_names_.clear();
            cached_output_names_.clear();
            input_name_owners_.reserve(n_in);
            cached_input_names_.reserve(n_in);
            for (std::size_t i = 0; i < n_in; ++i) {
                input_name_owners_.push_back(
                    session_->GetInputNameAllocated(i, allocator));
                cached_input_names_.push_back(input_name_owners_.back().get());
            }
            output_name_owners_.reserve(n_out);
            cached_output_names_.reserve(n_out);
            for (std::size_t i = 0; i < n_out; ++i) {
                output_name_owners_.push_back(
                    session_->GetOutputNameAllocated(i, allocator));
                cached_output_names_.push_back(
                    output_name_owners_.back().get());
            }
            return true;
        } catch (const Ort::Exception&) {
            return false;
        }
    }
#endif

    /// @brief Returns true if a model is loaded and ready for inference.
    bool is_model_loaded() const { return model_loaded_; }

    /// @brief Inference device policy (§4.4.2-GPU). Auto prefers the OpenVINO
    /// GPU runtime when available and falls back to ONNX Runtime CPU; CPU and
    /// GPU pin the respective runtime (GPU still degrades to CPU when the
    /// OpenVINO GPU plugin cannot run). Default Auto.
    enum class Device { Auto = 0, CPU = 1, GPU = 2 };

    /// @brief Applies a new device policy. Reloads the model from the cached
    /// path so the runtime selection takes effect (load-time decision).
    void set_device(Device d) {
        if (device_ == d) return;
        device_ = d;
        if (!model_path_.empty()) load_model(model_path_);
    }
    Device device() const { return device_; }

    /// @brief Runtime actually in use for the loaded model: "gpu"
    /// (OpenVINO GPU plugin), "cpu" (ONNX Runtime), or "" (no model).
    const std::string& active_runtime() const { return active_runtime_; }

    /// @brief Runs inference on a batch of events.
    /// @param events Event array.
    /// @param n Number of events.
    /// @return ONNX path: CV_32FC1 padded image in [0,1] (full_crop_h x
    ///         full_crop_w) — the caller is responsible for cropping back
    ///         to sensor size. Heuristic path: CV_8UC1 sensor-sized image.
    cv::Mat infer(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0 || width_ <= 0 || height_ <= 0) {
            return cv::Mat::zeros(height_, width_, CV_8UC1);
        }

        // 1. Build voxel grid. When downsample_ is on, keep only events whose
        //    x AND y are both even, and remap (x, y) → (x/2, y/2) into the
        //    half-size grid. For a 128×128 ROI this produces a 64×64 grid,
        //    cutting ONNX inference cost ~4×.
        if (downsample_) {
            const Event* src = events;
            std::size_t src_n = n;
            if (src_n > downsampled_events_.size()) {
                downsampled_events_.reserve(src_n / 4 + 16);
            }
            downsampled_events_.clear();
            for (std::size_t i = 0; i < src_n; ++i) {
                const auto& e = src[i];
                if ((e.x & 1u) == 0 && (e.y & 1u) == 0) {
                    downsampled_events_.push_back(
                        Event(static_cast<std::uint16_t>(e.x >> 1),
                              static_cast<std::uint16_t>(e.y >> 1),
                              e.p, e.t));
                }
            }
            voxel_grid_.build(downsampled_events_.data(),
                              downsampled_events_.size());
        } else {
            voxel_grid_.build(events, n);
        }
        if (normalize_input_) {
            voxel_grid_.normalize();
        }

#if defined(GUI_ALGO_HAS_OPENVINO)
        if (model_loaded_ && ov_active_) {
            cv::Mat result = infer_ov();  // effective crop size
            postprocess_result(result);
            return result;
        }
#endif
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
        if (model_loaded_ && session_) {
            cv::Mat result = infer_onnx();  // effective crop size
            postprocess_result(result);
            return result;
        }
#endif
        // Fallback: heuristic reconstruction from voxel grid.
        return infer_heuristic();
    }

    /// @brief Crops a padded image back to the sensor dimensions.
    /// No-op if the image already matches the sensor size (heuristic path).
    /// When downsample_ is on, the image has been upsampled to full_crop_
    /// dimensions, so use full_crop_ for cropping.
    cv::Mat crop_to_sensor(const cv::Mat& img) const {
        if (img.rows == height_ && img.cols == width_) {
            return img;
        }
        if (downsample_) {
            return full_crop_.crop(img);
        }
        return crop_.crop(img);
    }

    /// @brief Sets whether to normalize the input voxel grid.
    void set_normalize_input(bool v) { normalize_input_ = v; }
    bool normalize_input() const { return normalize_input_; }

    /// @brief Sets whether to 1/4-downsample the ROI before inference.
    /// When on, only events with even x AND even y are kept, and coordinates
    /// are halved. For 128×128 ROI this produces a 64×64 grid, ~4× faster.
    void set_downsample(bool v) {
        if (downsample_ == v) return;
        downsample_ = v;
        rebuild_effective_buffers();
    }
    bool downsample() const { return downsample_; }

    /// @brief Sets the hot-pixel mask for the voxel grid preprocessor.
    /// Accepts either effective-resolution (eff HxW) or full sensor-resolution
    /// (HxW) masks; the latter is 2x2-downsampled by the voxel grid when
    /// downsampling is active (§四-M3). Mismatched sizes are rejected — check
    /// hot_pixel_mask_rejected().
    void set_hot_pixel_mask(const std::vector<std::uint8_t>& mask) {
        hot_pixel_mask_ = mask;  // cache so it survives num_bins changes
        voxel_grid_.set_hot_pixel_mask(mask);
    }
    void clear_hot_pixel_mask() {
        hot_pixel_mask_.clear();
        voxel_grid_.clear_hot_pixel_mask();
    }
    /// @brief True if the last mask was rejected (size mismatch, §四-M3).
    bool hot_pixel_mask_rejected() const {
        return voxel_grid_.hot_mask_rejected();
    }

    void set_num_bins(int b) {
        int target = clamp_bins(b);
        // When a model is loaded, num_bins is dictated by the model's input
        // channels (rpg_e2vid: model.num_bins). Ignore the caller's value so
        // the voxel grid always matches the model — otherwise the ONNX input
        // shape would mismatch and inference would fail.
        if (model_loaded_ && model_num_bins_ > 0) {
            target = model_num_bins_;
        }
        num_bins_ = target;
        rebuild_effective_buffers();
    }
    int num_bins() const { return num_bins_; }

    const std::string& model_path() const { return model_path_; }

    void reset() {
        voxel_grid_.reset();
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
        // Clear recurrent states if applicable.
        prev_states_.clear();
        state_buffers_.clear();
        input_buffer_.clear();
#endif
#if defined(GUI_ALGO_HAS_OPENVINO)
        // Next infer re-binds the zero-state tensors (storage and compiled
        // shapes stay valid; only the recurrence sequence restarts).
        ov_have_prev_ = false;
#endif
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    static int clamp_bins(int b) {
        // Match EventVoxelGrid's clamp range: [1, 20].
        if (b < 1) return 1;
        if (b > 20) return 20;
        return b;
    }

#if defined(GUI_ALGO_HAS_ONNXRUNTIME) || defined(GUI_ALGO_HAS_OPENVINO)
    /// @brief Spatial downsampling of a recurrent-state input, read from the
    /// model's symbolic dimension NAME (§4.4.2-GPU contract shared by all
    /// export scripts): "H"/"W" = full resolution, "H2"/"H4"/"H8" = the
    /// input's 1/2, 1/4, 1/8. This covers every architecture family without
    /// heuristics — ConvLSTM h/c pairs (E2VID/E2VID+/HyperE2VID at H2/H4/H8),
    /// full-resolution ConvGRU states (FireNet+ at H) and the previous-image
    /// feedback input (HyperE2VID prev_recs at H). Unknown names fall back
    /// to the legacy rpg_e2vid pair layout ((i-1)/2 → /2^(level+1)).
    int state_divisor(std::size_t i) const {
        const std::size_t idx = i - 1;
        if (idx < state_divisors_.size() && state_divisors_[idx] > 0) {
            return state_divisors_[idx];
        }
        const int level = static_cast<int>((i - 1) / 2);
        return 1 << (level + 1);
    }

    /// Number of output channels of the loaded model (1 = grayscale image,
    /// 2 = optical flow (u, v) — EVFlowNet).
    int output_channels_{1};

    /// Parsed per-state-input divisors (size = n_inputs - 1; 0 = unknown →
    /// legacy fallback in state_divisor()).
    std::vector<int> state_divisors_;

    /// Maps a symbolic spatial dimension name to its divisor vs. the input
    /// resolution; 0 when the name carries no level information.
    static int divisor_from_dim_name(const std::string& name) {
        if (name == "H" || name == "W") return 1;
        if (name == "H2" || name == "W2") return 2;
        if (name == "H4" || name == "W4") return 4;
        if (name == "H8" || name == "W8") return 8;
        return 0;
    }

    /// Derives state_divisors_ + num_encoders_ + output_channels_ from the
    /// model I/O (shared logic; @p name_of/@p shape_of abstract the runtime
    /// API). num_encoders = log2(max state divisor); stateless models keep
    /// the constructor default (EVFlowNet: 4 encoders).
    template <typename NameOf, typename ShapeOf>
    void derive_model_layout(std::size_t n_inputs, int default_encoders,
                             int out_channels, const NameOf& name_of,
                             const ShapeOf& shape_of) {
        state_divisors_.assign(n_inputs > 1 ? n_inputs - 1 : 0, 0);
        for (std::size_t i = 1; i < n_inputs; ++i) {
            int divisor = 0;
            const auto shape = shape_of(i);
            if (shape.size() >= 3) {
                divisor = divisor_from_dim_name(name_of(i, 2));
                if (divisor == 0) {
                    divisor = divisor_from_dim_name(name_of(i, 3));
                }
            }
            state_divisors_[i - 1] = divisor;
        }
        // Fallback when the runtime exposes no symbolic names (OpenVINO's
        // ONNX front-end drops them even though they are in the file):
        // infer the layout from the channel structure of the known model
        // families. Anything unrecognized keeps divisor 0 → legacy pair
        // formula in state_divisor().
        const bool any_name = std::any_of(
            state_divisors_.begin(), state_divisors_.end(),
            [](int d) { return d > 0; });
        if (!any_name && n_inputs > 1) {
            std::vector<int> ch(n_inputs - 1, 0);
            bool channels_known = true;
            for (std::size_t i = 1; i < n_inputs; ++i) {
                const auto shape = shape_of(i);
                if (shape.size() >= 2 && shape[1] > 0) {
                    ch[i - 1] = static_cast<int>(shape[1]);
                } else {
                    channels_known = false;
                }
            }
            if (channels_known) {
                if (n_inputs == 3 && ch[0] > 0 && ch[0] == ch[1]) {
                    // FireNet family: full-resolution ConvGRU single states.
                    state_divisors_ = {1, 1};
                } else if (n_inputs == 8 && ch[6] == out_channels) {
                    // HyperE2VID family: LSTM pairs + full-res feedback.
                    state_divisors_ = {2, 2, 4, 4, 8, 8, 1};
                } else if (n_inputs == 7 && ch[0] == ch[1] &&
                           ch[2] == ch[3] && ch[4] == ch[5]) {
                    // rpg_e2vid family: ConvLSTM h/c pairs.
                    state_divisors_ = {2, 2, 4, 4, 8, 8};
                }
            }
        }
        int max_divisor = 1;
        for (int d : state_divisors_) max_divisor = std::max(max_divisor, d);
        if (n_inputs > 1) {
            int enc = 0;
            while ((1 << enc) < max_divisor) ++enc;
            num_encoders_ = enc;
        } else {
            num_encoders_ = default_encoders;
        }
        output_channels_ = out_channels > 0 ? out_channels : 1;
    }
#endif

#if defined(GUI_ALGO_HAS_ONNXRUNTIME) || defined(GUI_ALGO_HAS_OPENVINO)
    /// @brief Copies the voxel grid into input_buffer_ as a reflection-padded
    /// (BORDER_REFLECT_101) NCHW buffer shared by the ONNX Runtime and
    /// OpenVINO inference paths. Reuses the storage across frames (resize
    /// only on a bins/crop change). Returns the buffer's data pointer.
    float* fill_padded_input() {
        const int ch = crop_.crop_height;
        const int cw = crop_.crop_width;

        const std::size_t input_size =
            static_cast<std::size_t>(num_bins_) * ch * cw;
        if (cached_crop_w_ != cw || cached_crop_h_ != ch ||
            cached_num_bins_ != num_bins_ ||
            input_buffer_.size() != input_size) {
            input_buffer_.assign(input_size, 0.0f);
            cached_crop_w_ = cw;
            cached_crop_h_ = ch;
            cached_num_bins_ = num_bins_;
        } else {
            std::fill(input_buffer_.begin(), input_buffer_.end(), 0.0f);
        }

        // Copy voxel grid into padded tensor (reflection padding).
        // voxel_grid_ is at effective dimensions (possibly downsampled).
        const int ew = eff_width();
        const int eh = eff_height();
        const float* grid = voxel_grid_.data();
        const int stride_hw = ew * eh;
        for (int b = 0; b < num_bins_; ++b) {
            cv::Mat bin(eh, ew, CV_32FC1,
                        const_cast<float*>(grid + b * stride_hw));
            cv::copyMakeBorder(bin, padded_buffer_,
                               crop_.pad_top, crop_.pad_bottom,
                               crop_.pad_left, crop_.pad_right,
                               cv::BORDER_REFLECT_101);
            std::copy(padded_buffer_.begin<float>(), padded_buffer_.end<float>(),
                      input_buffer_.begin() +
                          static_cast<std::size_t>(b) * ch * cw);
        }
        return input_buffer_.data();
    }

    /// @brief Upsamples a padded inference result to the full crop
    /// dimensions after 1/4 downsampling, so crop_to_sensor() works
    /// uniformly regardless of downsample_.
    void postprocess_result(cv::Mat& result) {
        if (downsample_ &&
            (result.rows != full_crop_.crop_height ||
             result.cols != full_crop_.crop_width)) {
            cv::resize(result, result,
                       cv::Size(full_crop_.crop_width,
                                full_crop_.crop_height),
                       0, 0, cv::INTER_NEAREST);
        }
    }
#endif

#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
    /// @brief Reads num_bins and num_encoders from the loaded ONNX model.
    /// rpg_e2vid determines num_bins from the model config (model.num_bins);
    /// the ONNX equivalent is the 2nd dimension of the first input tensor
    /// (shape = [N, C, H, W]). Updates model_num_bins_ and re-syncs the voxel
    /// grid. Best-effort: on any failure keeps the existing num_bins_.
    ///
    /// Also infers num_encoders from the input count:
    ///   E2VIDRecurrent: n_inputs = 1 + 2 * num_encoders (event + h/c per level)
    ///   E2VID (non-recurrent): n_inputs = 1 (num_encoders stays at constructor default)
    /// and recomputes CropParameters to match (rpg_e2vid: model.num_encoders).
    void sync_num_bins_from_model() {
        if (!session_) return;
        try {
            auto input_shape = session_->GetInputTypeInfo(0)
                                   .GetTensorTypeAndShapeInfo()
                                   .GetShape();
            // shape = [N, C, H, W]; C is the num_bins channel dimension.
            if (input_shape.size() >= 2 && input_shape[1] > 0) {
                model_num_bins_ = static_cast<int>(input_shape[1]);
                if (model_num_bins_ != num_bins_) {
                    num_bins_ = model_num_bins_;
                }
            }
            // State layout + encoder depth + output channels from the
            // symbolic dimension names (§4.4.2-GPU contract).
            const int ctor_encoders = num_encoders_;
            derive_model_layout(
                session_->GetInputCount(), ctor_encoders,
                output_channels_from_ort(),
                [this](std::size_t i, int dim) {
                    auto info = session_->GetInputTypeInfo(i);
                    auto tinfo = info.GetTensorTypeAndShapeInfo();
                    const std::size_t nd = tinfo.GetDimensionsCount();
                    std::vector<const char*> syms(nd, nullptr);
                    if (nd > 0) {
                        tinfo.GetSymbolicDimensions(syms.data(), nd);
                    }
                    return (static_cast<std::size_t>(dim) < nd && syms[dim])
                               ? std::string(syms[dim]) : std::string();
                },
                [this](std::size_t i) {
                    return session_->GetInputTypeInfo(i)
                        .GetTensorTypeAndShapeInfo()
                        .GetShape();
                });
            // Rebuild all effective-size buffers (voxel grid, crop, states).
            rebuild_effective_buffers();
        } catch (const Ort::Exception&) {
            // Keep existing num_bins_ (best-effort).
        }
    }

    /// Declared channel count of output 0 (0 when unavailable).
    int output_channels_from_ort() const {
        try {
            auto shape = session_->GetOutputTypeInfo(0)
                             .GetTensorTypeAndShapeInfo()
                             .GetShape();
            return shape.size() >= 2 ? static_cast<int>(shape[1]) : 0;
        } catch (const Ort::Exception&) {
            return 0;
        }
    }

    /// @brief ONNX Runtime inference path.
    /// Returns the padded CV_32FC1 image in [0,1] (crop_h x crop_w).
    /// The caller crops it back to sensor size after postprocessing.
    /// Handles both plain UNet (1 input/1 output) and UNetRecurrent
    /// (N inputs/M outputs) by feeding zero-initialized states on the first
    /// call and persisting returned states across calls (matches rpg_e2vid's
    /// prev_states handling). Any Ort::Exception falls back to heuristic.
    cv::Mat infer_onnx() {
        const int ch = crop_.crop_height;
        const int cw = crop_.crop_width;

        try {
            // Recurrent states must be re-zeroed at the new shapes when the
            // effective dims change (downsample/num_bins toggles).
            const bool dims_changed =
                cached_crop_w_ != cw || cached_crop_h_ != ch ||
                cached_num_bins_ != num_bins_;
            fill_padded_input();
            if (dims_changed) {
                state_buffers_.clear();
                prev_states_.clear();
            }

            std::array<std::int64_t, 4> input_shape = {1, num_bins_, ch, cw};

            const std::size_t n_inputs = session_->GetInputCount();

            // Build input Ort::Values. First input is always the event voxel
            // grid; subsequent inputs (if any) are recurrent state tensors.
            std::vector<Ort::Value> inputs;
            inputs.reserve(n_inputs);
            inputs.push_back(Ort::Value::CreateTensor<float>(
                *mem_info_, input_buffer_.data(), input_buffer_.size(),
                input_shape.data(), input_shape.size()));

            // Allocate zero-initialized state buffers only once (or after a
            // dimension change). On subsequent frames prev_states_ holds the
            // recurrent state and state_buffers_ is skipped entirely.
            const bool need_zero_states =
                state_buffers_.empty() &&
                (prev_states_.empty() || prev_states_.size() != n_inputs - 1);
            if (need_zero_states) {
                state_buffers_.clear();
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    auto info = session_->GetInputTypeInfo(i);
                    auto tensor_info = info.GetTensorTypeAndShapeInfo();
                    auto shape = tensor_info.GetShape();
                    if (shape.size() >= 4) {
                        const int divisor = state_divisor(i);
                        shape[0] = 1;
                        shape[2] = ch / divisor;
                        shape[3] = cw / divisor;
                    }
                    std::size_t total = 1;
                    for (auto d : shape) {
                        if (d <= 0) d = 1;
                        total *= static_cast<std::size_t>(d);
                    }
                    state_buffers_.emplace_back(total, 0.0f);
                    inputs.push_back(Ort::Value::CreateTensor<float>(
                        *mem_info_, state_buffers_.back().data(),
                        state_buffers_.back().size(), shape.data(),
                        shape.size()));
                }
            } else {
                // Pad inputs with placeholder tensors (will be replaced by
                // prev_states_ below, or by existing state_buffers_).
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    inputs.push_back(Ort::Value{nullptr});
                }
            }

            // If we have prev_states_ from a previous call, replace the zero
            // state tensors with the persisted states.
            if (!prev_states_.empty() && prev_states_.size() == n_inputs - 1) {
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    inputs[i] = std::move(prev_states_[i - 1]);
                }
                prev_states_.clear();
            } else if (!state_buffers_.empty()) {
                // Rebuild Ort::Value wrappers around existing state_buffers_.
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    auto info = session_->GetInputTypeInfo(i);
                    auto tensor_info = info.GetTensorTypeAndShapeInfo();
                    auto shape = tensor_info.GetShape();
                    if (shape.size() >= 4) {
                        const int divisor = state_divisor(i);
                        shape[0] = 1;
                        shape[2] = ch / divisor;
                        shape[3] = cw / divisor;
                    }
                    inputs[i] = Ort::Value::CreateTensor<float>(
                        *mem_info_, state_buffers_[i - 1].data(),
                        state_buffers_[i - 1].size(), shape.data(),
                        shape.size());
                }
            }

            // Run inference (uses cached name pointers — no per-frame alloc).
            auto outputs = session_->Run(
                Ort::RunOptions{nullptr},
                cached_input_names_.data(), inputs.data(), inputs.size(),
                cached_output_names_.data(), cached_output_names_.size());

            // Persist recurrent states (outputs beyond the first image).
            prev_states_.clear();
            for (std::size_t i = 1; i < outputs.size(); ++i) {
                prev_states_.push_back(std::move(outputs[i]));
            }

            // Extract output: 1 x C x crop_h x crop_w (C = 1 grayscale
            // image, or 2 = flow (u, v)).
            const float* output_data = outputs[0].GetTensorData<float>();
            auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
            const int out_type =
                (out_shape.size() >= 2 && out_shape[1] == 2) ? CV_32FC2
                                                             : CV_32FC1;
            const int out_h = static_cast<int>(out_shape[2]);
            const int out_w = static_cast<int>(out_shape[3]);
            cv::Mat output(out_h, out_w, out_type,
                           const_cast<float*>(output_data));
            return output.clone();  // deep copy (Ort owns the buffer)
        } catch (const Ort::Exception& e) {
            fprintf(stderr, "[e2vid] ONNX inference failed: %s (falling back "
                    "to heuristic, will retry next batch)\n", e.what());
            prev_states_.clear();
            return infer_heuristic();
        }
    }
#endif

#if defined(GUI_ALGO_HAS_OPENVINO)
    /// Process-wide OpenVINO runtime core (lazy, thread-safe init). Device
    /// enumeration happens once; plugin discovery is relative to the
    /// libopenvino.so location (third_party/openvino/runtime/lib/intel64 or
    /// a system install).
    static ov::Core& ov_core() {
        static ov::Core core;
        return core;
    }

    /// True when the build has OpenVINO and an Intel GPU device is
    /// enumerated (cached after the first check).
    static bool ov_gpu_available() {
        static const bool available = [] {
            try {
                for (const auto& d : ov_core().get_available_devices()) {
                    if (d.rfind("GPU", 0) == 0) return true;
                }
            } catch (...) {
            }
            return false;
        }();
        return available;
    }

    /// Drops all OpenVINO state (request, compiled model, tensors). Safe to
    /// call repeatedly; never throws.
    void ov_release() {
        ov_request_ = ov::InferRequest{};
        ov_compiled_ = ov::CompiledModel{};
        ov_model_.reset();
        ov_input_tensor_ = ov::Tensor{};
        ov_zero_states_.clear();
        ov_prev_states_.clear();
        ov_have_prev_ = false;
        ov_active_ = false;
        ov_compiled_bins_ = 0;
        ov_compiled_h_ = 0;
        ov_compiled_w_ = 0;
    }

    /// Reads num_bins / state layout / output channels from the OpenVINO
    /// model (same contract as the ONNX Runtime sync) and rebuilds the
    /// effective buffers. Runs BEFORE reshape, while the symbolic dimension
    /// names (H/W, H2/W2, ...) imported from ONNX are still intact.
    void ov_sync_model_meta() {
        try {
            const ov::PartialShape ps = ov_model_->inputs()[0].get_partial_shape();
            if (ps.rank().is_static() && ps.rank().get_length() >= 2 &&
                ps[1].is_static()) {
                model_num_bins_ = static_cast<int>(ps[1].get_length());
                num_bins_ = model_num_bins_;
            }
            int out_channels = 0;
            if (!ov_model_->outputs().empty()) {
                const ov::PartialShape os =
                    ov_model_->output(0).get_partial_shape();
                if (os.rank().is_static() && os.rank().get_length() >= 2 &&
                    os[1].is_static()) {
                    out_channels = static_cast<int>(os[1].get_length());
                }
            }
            const int ctor_encoders = num_encoders_;
            const auto& model = ov_model_;
            derive_model_layout(
                model->inputs().size(), ctor_encoders, out_channels,
                [model](std::size_t i, int dim) {
                    const ov::PartialShape ps =
                        model->inputs()[i].get_partial_shape();
                    if (ps.rank().is_static() &&
                        ps.rank().get_length() >
                            static_cast<ov::Dimension::value_type>(dim)) {
                        const ov::Dimension d =
                            ps[static_cast<std::size_t>(dim)];
                        // Static dims render as numbers; dynamic (symbolic)
                        // dims render as their ONNX name ("H", "H2", ...).
                        return d.is_static() ? std::string() : d.to_string();
                    }
                    return std::string();
                },
                [model](std::size_t i) {
                    return model->inputs()[i].get_partial_shape().get_max_shape();
                });
            rebuild_effective_buffers();
        } catch (const std::exception&) {
            // Keep existing meta (best-effort, mirrors the ORT sync).
        }
    }

    /// Declared channel count of recurrent-state input @p i. Dynamic channel
    /// counts are unsupported (all our export scripts declare them static).
    std::size_t ov_state_channels(std::size_t i) const {
        const ov::PartialShape ps = ov_model_->inputs()[i].get_partial_shape();
        if (ps.rank().is_static() && ps.rank().get_length() >= 2 &&
            ps[1].is_static()) {
            return static_cast<std::size_t>(ps[1].get_length());
        }
        throw std::runtime_error("dynamic state channel count unsupported");
    }

    /// Loads the model for the OpenVINO GPU runtime: reads the SAME .onnx as
    /// the ORT path, reshapes it to the current static effective dims
    /// (compiled once per dims change — GPU scheduling needs static shapes;
    /// the ~0.3 s recompile on a downsample/ROI change is acceptable), then
    /// compiles for the GPU plugin and pre-allocates the state tensors.
    bool ov_try_load(const std::string& model_path) {
        try {
            ov_model_ = ov_core().read_model(model_path);
            ov_sync_model_meta();

            const int ch = crop_.crop_height;
            const int cw = crop_.crop_width;
            const std::size_t n_inputs = ov_model_->inputs().size();

            std::map<std::size_t, ov::PartialShape> shapes;
            shapes[0] = ov::PartialShape{
                1, static_cast<std::int64_t>(num_bins_),
                static_cast<std::int64_t>(ch), static_cast<std::int64_t>(cw)};
            for (std::size_t i = 1; i < n_inputs; ++i) {
                const int divisor = state_divisor(i);
                shapes[i] = ov::PartialShape{
                    1, static_cast<std::int64_t>(ov_state_channels(i)),
                    static_cast<std::int64_t>(ch / divisor),
                    static_cast<std::int64_t>(cw / divisor)};
            }
            ov_model_->reshape(shapes);
            ov_compiled_ = ov_core().compile_model(ov_model_, "GPU");
            ov_request_ = ov_compiled_.create_infer_request();

            // Input tensor wraps the shared padded-voxel storage (filled per
            // frame by fill_padded_input(); sizes are frozen at compile time).
            input_buffer_.assign(static_cast<std::size_t>(num_bins_) * ch * cw,
                                 0.0f);
            cached_crop_w_ = cw;
            cached_crop_h_ = ch;
            cached_num_bins_ = num_bins_;
            const ov::Shape in_shape{
                1, static_cast<std::size_t>(num_bins_),
                static_cast<std::size_t>(ch), static_cast<std::size_t>(cw)};
            ov_input_tensor_ =
                ov::Tensor(ov::element::f32, in_shape, input_buffer_.data());

            // Zero-filled recurrent-state tensors (rebound whenever no
            // previous state exists — first call or after reset()).
            ov_zero_states_.clear();
            for (std::size_t i = 1; i < n_inputs; ++i) {
                ov::Tensor t(ov::element::f32, shapes[i].get_max_shape());
                std::memset(t.data(), 0, t.get_byte_size());
                ov_zero_states_.push_back(std::move(t));
            }
            ov_prev_states_.clear();
            ov_have_prev_ = false;

            ov_compiled_bins_ = num_bins_;
            ov_compiled_h_ = ch;
            ov_compiled_w_ = cw;
            ov_active_ = true;
            return true;
        } catch (const std::exception& e) {
            fprintf(stderr, "[e2vid] OpenVINO load failed: %s\n", e.what());
            return false;  // caller falls back to the CPU runtime
        }
    }

    /// Recompiles the GPU model when the effective dims drifted after the
    /// last compile (set_downsample / set_num_bins while loaded).
    void ov_ensure_current() {
        if (ov_compiled_bins_ == num_bins_ &&
            ov_compiled_h_ == crop_.crop_height &&
            ov_compiled_w_ == crop_.crop_width) {
            return;
        }
        ov_release();
        if (!ov_try_load(model_path_)) {
            // GPU runtime unavailable after the change — degrade to CPU.
            ov_active_ = false;
            active_runtime_ = "cpu";
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
            if (!load_model_ort(model_path_)) model_loaded_ = false;
#else
            model_loaded_ = false;
#endif
        }
    }

    /// @brief OpenVINO GPU inference path (same positional recurrent-state
    /// contract as infer_onnx). On a runtime failure the engine degrades
    /// once to the ONNX Runtime CPU path (or heuristic when unavailable).
    cv::Mat infer_ov() {
        try {
            ov_ensure_current();
            fill_padded_input();

            if (!ov_have_prev_) {
                for (std::size_t i = 0; i < ov_zero_states_.size(); ++i) {
                    ov_request_.set_input_tensor(i + 1, ov_zero_states_[i]);
                }
            } else {
                for (std::size_t i = 0; i < ov_prev_states_.size(); ++i) {
                    ov_request_.set_input_tensor(i + 1, ov_prev_states_[i]);
                }
            }
            ov_request_.set_input_tensor(0, ov_input_tensor_);
            ov_request_.infer();

            // Persist recurrent states (position-mapped outputs 1..n-1).
            const std::size_t n_out = ov_compiled_.outputs().size();
            if (ov_prev_states_.size() != n_out - 1) {
                ov_prev_states_.clear();
                for (std::size_t k = 1; k < n_out; ++k) {
                    const ov::Output<const ov::Node>& port =
                        ov_compiled_.output(k);
                    ov_prev_states_.emplace_back(port.get_element_type(),
                                                 port.get_shape());
                }
            }
            for (std::size_t k = 1; k < n_out; ++k) {
                ov_request_.get_output_tensor(k).copy_to(
                    ov_prev_states_[k - 1]);
            }
            ov_have_prev_ = true;

            // Output: 1 x C x crop_h x crop_w (C = 1 grayscale image,
            // or 2 = flow (u, v)), values per the model's head.
            ov::Tensor img = ov_request_.get_output_tensor(0);
            const ov::Shape shp = img.get_shape();
            const int out_type =
                (shp.size() >= 2 && shp[1] == 2) ? CV_32FC2 : CV_32FC1;
            cv::Mat out(static_cast<int>(shp[2]), static_cast<int>(shp[3]),
                        out_type, img.data());
            return out.clone();  // deep copy (the request owns the buffer)
        } catch (const std::exception& e) {
            fprintf(stderr, "[e2vid] OpenVINO inference failed: %s — "
                    "switching to the CPU runtime\n", e.what());
            ov_release();
            active_runtime_ = "cpu";
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
            if (!load_model_ort(model_path_)) {
                model_loaded_ = false;
                return infer_heuristic();
            }
            return infer_onnx();
#else
            model_loaded_ = false;
            return infer_heuristic();
#endif
        }
    }
#endif

    /// @brief Heuristic fallback: reconstructs from voxel grid without a model.
    /// Sums bins, applies sigmoid, returns CV_8UC1 at sensor dimensions.
    cv::Mat infer_heuristic() {
        const int eh = eff_height();
        const int ew = eff_width();
        // Sum across bins to get a 2D event count map.
        cv::Mat sum_img(eh, ew, CV_32FC1, cv::Scalar(0.0f));
        const float* grid = voxel_grid_.data();
        const int stride_hw = ew * eh;
        for (int b = 0; b < num_bins_; ++b) {
            cv::Mat bin(eh, ew, CV_32FC1,
                        const_cast<float*>(grid + b * stride_hw));
            sum_img += bin;
        }
        // Apply sigmoid: out = 1 / (1 + exp(-k * sum))
        cv::Mat sig;
        const float k = 0.5f;
        cv::exp(-k * sum_img, sig);
        sig = 1.0f / (1.0f + sig);
        cv::Mat gray;
        sig.convertTo(gray, CV_8UC1, 255.0);
        // Upsample if downsampled.
        if (downsample_ && (gray.rows != height_ || gray.cols != width_)) {
            cv::resize(gray, gray, cv::Size(width_, height_), 0, 0,
                       cv::INTER_NEAREST);
        }
        return gray;
    }

    int width_;
    int height_;
    int num_bins_;
    int model_num_bins_{0};  ///< Channel count read from the loaded ONNX model.
    int num_encoders_;
    // Default true: 1/4-downsample (halve width and height) before inference.
    // For 128×128 ROI → 64×64 grid, ~4× faster inference. Events whose x OR
    // y is odd are discarded; the rest are remapped (x/2, y/2).
    // Declared before crop_/voxel_grid_ because the constructor initialiser
    // list uses effective_dim() which reads downsample_.
    bool downsample_{true};
    E2VIDCropParams crop_;       ///< Crop at effective (possibly downsampled) dims.
    E2VIDCropParams full_crop_;  ///< Crop at original sensor dims (for upsampled output).
    EventVoxelGrid voxel_grid_;
    std::vector<std::uint8_t> hot_pixel_mask_;  ///< Cached for num_bins rebuilds.
    // Default false: rpg_e2vid README says --no-normalize "will improve speed
    // a bit, but might degrade the image quality a bit". The speed gain
    // matters more for real-time GUI usage than the minor quality drop.
    bool normalize_input_{false};
    bool model_loaded_{false};
    std::string model_path_;

    /// Effective dimensions after optional 1/4 downsampling.
    int eff_width() const { return effective_dim(width_); }
    int eff_height() const { return effective_dim(height_); }

    /// Returns half the dimension when downsample_ is on, else the dimension.
    int effective_dim(int d) const {
        return downsample_ ? (d > 0 ? (d + 1) / 2 : 0) : d;
    }

    /// Rebuilds voxel_grid_, crop_, and ONNX state buffers for the current
    /// effective dimensions. Called whenever downsample_ or num_bins_ or
    /// num_encoders_ changes. Preserves the hot-pixel mask.
    void rebuild_effective_buffers() {
        const int ew = eff_width();
        const int eh = eff_height();
        voxel_grid_ = EventVoxelGrid(ew, eh, num_bins_);
        if (!hot_pixel_mask_.empty()) {
            voxel_grid_.set_hot_pixel_mask(hot_pixel_mask_);
        }
        crop_ = E2VIDCropParams::compute(ew, eh, num_encoders_);
        full_crop_ = E2VIDCropParams::compute(width_, height_, num_encoders_);
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
        state_buffers_.clear();
        prev_states_.clear();
#endif
#if defined(GUI_ALGO_HAS_ONNXRUNTIME) || defined(GUI_ALGO_HAS_OPENVINO)
        input_buffer_.clear();
        cached_crop_w_ = 0;  // force resize on next infer
#endif
#if defined(GUI_ALGO_HAS_OPENVINO)
        // Compiled GPU shapes are now stale; ov_ensure_current() recompiles
        // on the next infer, which also restarts the recurrence with fresh
        // zero states.
#endif
    }

    /// Pre-allocated buffer for downsampled events (reused across frames).
    std::vector<Event> downsampled_events_;

#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<Ort::Value> prev_states_;  ///< Recurrent states (UNetRecurrent).
    std::vector<std::vector<float>> state_buffers_;  ///< Backing storage for zero-init states.

    // Input/output name strings are fetched once at load_model() time.
    std::vector<Ort::AllocatedStringPtr> input_name_owners_;
    std::vector<Ort::AllocatedStringPtr> output_name_owners_;
    std::vector<const char*> cached_input_names_;
    std::vector<const char*> cached_output_names_;
    // MemoryInfo is constant for the lifetime of the session.
    std::unique_ptr<Ort::MemoryInfo> mem_info_;
#endif

#if defined(GUI_ALGO_HAS_ONNXRUNTIME) || defined(GUI_ALGO_HAS_OPENVINO)
    // --- Hot-path caches shared by the ORT and OpenVINO paths (avoid
    // per-frame allocations). input_buffer_ is reused across frames; resized
    // only when crop/bin dims change. Previously every infer call did a
    // 320 KB malloc+memset.
    std::vector<float> input_buffer_;
    cv::Mat padded_buffer_;  ///< Reusable padded image buffer (avoids per-bin allocation)
    // Crop dims last used to size input_buffer_ (resize only on change).
    int cached_crop_w_{0};
    int cached_crop_h_{0};
    int cached_num_bins_{0};
#endif

    // Device policy + actually-selected runtime exist in ALL builds (the
    // setter is public API; the value only steers runtime selection when
    // OpenVINO support is compiled in).
    Device device_{Device::Auto};
    std::string active_runtime_;  ///< "gpu" | "cpu" | "" (no model loaded)

#if defined(GUI_ALGO_HAS_OPENVINO)
    // OpenVINO GPU runtime state (§4.4.2-GPU). The model is compiled with
    // STATIC effective dims (GPU scheduling wants fixed shapes) and
    // recompiled lazily when downsample/num_bins changes the dims.
    std::shared_ptr<ov::Model> ov_model_;
    ov::CompiledModel ov_compiled_;
    ov::InferRequest ov_request_;
    ov::Tensor ov_input_tensor_;             ///< wraps input_buffer_
    std::vector<ov::Tensor> ov_zero_states_;  ///< state inputs after reset
    std::vector<ov::Tensor> ov_prev_states_;  ///< persisted state outputs
    bool ov_have_prev_{false};
    bool ov_active_{false};  ///< true when the active runtime is OpenVINO GPU
    int ov_compiled_bins_{0};
    int ov_compiled_h_{0};
    int ov_compiled_w_{0};
#endif
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_E2VID_E2VID_INFERENCE_H
