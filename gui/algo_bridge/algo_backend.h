// gui/algo_bridge/algo_backend.h — type-erased backend that真正调用 algo/ 类.
//
// 设计 §3.8。AlgoInstance 持有一个 AlgoBackend 实例，每次 push_events 时调用
// 真实的 algo/cv 或 algo/analytics 类的 process()/filter() 方法，pull_result 时
// 返回过滤后事件 + 叠加层（boxes/lines/points/...）+ 帧（cv::Mat）。
// 工厂 create_algo_backend() 按 algo 名字实例化对应的具体后端。

#ifndef GUI_ALGO_BRIDGE_ALGO_BACKEND_H
#define GUI_ALGO_BRIDGE_ALGO_BACKEND_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>

namespace gui {

/// @brief 叠加层矩形框（目标跟踪/团块检测输出）。
struct OverlayBox {
    int x{0}, y{0}, w{0}, h{0};
    int id{-1};
};

/// @brief 叠加层线段（霍夫直线/铰链线/ELiSeD 输出）。
struct OverlayLine {
    int x1{0}, y1{0}, x2{0}, y2{0};
};

/// @brief 叠加层点（角点/光流向量起点/XYT 点）。
struct OverlayPoint {
    int x{0}, y{0};
    float strength{1.0F};
};

/// @brief 叠加层带颜色点（光流 HSV 可视化等）。
/// r/g/b 在 [0,255]。用于光流场可视化：色相=方向，亮度=强度。
struct OverlayColoredPoint {
    int x{0}, y{0};
    std::uint8_t r{255}, g{255}, b{255};
};

/// @brief 光流箭头(端点已按 pps_scale 缩放,含起点抖动;颜色 = 方向色编码)。
struct OverlayFlowArrow {
    int x1{0}, y1{0}, x2{0}, y2{0};
    std::uint8_t r{0}, g{0}, b{0};  ///< arrow colour; all-zero = use the default
};

/// @brief 叠加层圆（霍夫圆输出）。
struct OverlayCircle {
    int cx{0}, cy{0}, r{0};
};

/// @brief 叠加层文本（统计信息/频率/计数）。
struct OverlayText {
    int x{0}, y{0};
    std::string text;
};

/// @brief 带颜色的事件（朝向/方向过滤器着色事件）。
/// 携带原始 EventCD + RGB 颜色，渲染器按颜色绘制每个事件像素。
struct ColoredEvent {
    Metavision::EventCD event;
    std::uint8_t r{255}, g{255}, b{255};
};

/// @brief 轨迹（聚类历史路径点序列）。
/// 用于 ObjectTracker/ClusterLIF 绘制聚类移动轨迹。
struct OverlayTrajectory {
    int id{-1};
    std::vector<cv::Point2i> points;
};

/// @brief 算法执行结果。由 AlgoBackend::pull_result() 返回。
struct AlgoResult {
    std::string status;                                 ///< 人类可读状态
    bool has_frame{false};                              ///< 是否产生帧
    cv::Mat frame;                                      ///< 帧产生型算法的输出帧
    // 叠加层数据（供 frame_annotator 绘制）
    std::vector<OverlayBox> boxes;
    std::vector<OverlayLine> lines;
    std::vector<OverlayPoint> points;
    std::vector<OverlayColoredPoint> colored_points;
    std::vector<OverlayFlowArrow> flow_arrows;      ///< 光流箭头(sparse/dense flow)
    std::vector<OverlayCircle> circles;
    std::vector<OverlayText> texts;
    std::vector<ColoredEvent> colored_events;       ///< 朝向/方向着色事件
    std::vector<OverlayTrajectory> trajectories;    ///< 聚类轨迹
    /// 居中提示（如 freq_detector 初始化阶段的使用说明）：非空时在主显示
    /// 帧中心绘制半透明底条 + 文字（'\n' 分行）。
    std::string hint;
    // Phase 2.6 debug D-2: has_aux_frame / aux_frame deleted with the hough
    // aux display (jAER-style debug view, never visible in baseline practice).
};

/// @brief 类型擦除的算法后端接口。
///
/// 每个具体后端包装一个 algo/ 类，将字符串参数转换为类型化 setter
/// 调用，将 EventCD 缓冲区零拷贝 reinterpret_cast 为 gui_algo::Event 后
/// 调用 process()/filter()，并将结果填充到 AlgoResult。
class AlgoBackend {
public:
    virtual ~AlgoBackend() = default;

    /// @brief 设置参数（字符串值，由 Algorithms 面板传入）。
    virtual void set_param(const std::string& key,
                           const std::string& value) = 0;

    /// @brief 读取参数当前值（字符串形式）。
    virtual std::string get_param(const std::string& key) const = 0;

    /// @brief 喂入一批事件（零拷贝：EventCD 与 gui_algo::Event 布局兼容）。
    virtual void push_events(const Metavision::EventCD* begin,
                             const Metavision::EventCD* end) = 0;

    /// @brief 取回最新结果（过滤事件 + 叠加层 + 帧）。
    virtual AlgoResult pull_result() = 0;

    /// @brief 重置内部状态。
    virtual void reset() = 0;

    /// @brief Updates the sensor dimensions and recomputes the ROI.
    /// Called when a new camera/file connects with different dimensions.
    /// This is effectively MANDATORY for every backend that filters by ROI
    /// or holds sensor-sized buffers (audit §五-D1): the default no-op leaves
    /// the backend computing its ROI at the stale construction dimensions,
    /// which silently filters out most events on a smaller sensor.
    /// Backends holding w×h algorithm state must additionally rebuild or
    /// resize the algorithm here.
    virtual void set_sensor_dimensions(int /*width*/, int /*height*/) {}

    /// @brief Per-batch OUTPUT event span for stream-filtering algorithms
    /// (Phase 7 P6-A): the display pipeline and processed recording consume
    /// this span INSTEAD of the conditioned input when non-null — so an
    /// in-place filter (hot_pixel_filter) visibly removes events and EIS
    /// displays stabilized coordinates. Zero-copy: the span points into a
    /// backend-owned buffer that stays valid until the next push_events.
    /// Default {nullptr, 0} = pass-through (the input span is unchanged);
    /// detectors return nothing on purpose — they annotate, not filter.
    virtual std::pair<const Metavision::EventCD*, std::size_t>
    output_span() const { return {nullptr, 0}; }
};

/// @brief 工厂：按算法名字创建具体后端。
/// @param name 算法标识（与 AlgoInfo::name 一致）。
/// @param width,height 传感器尺寸。
/// @return 后端实例；若 name 未知返回 nullptr。
std::unique_ptr<AlgoBackend> create_algo_backend(const std::string& name,
                                                  int width, int height);

} // namespace gui

#endif // GUI_ALGO_BRIDGE_ALGO_BACKEND_H
