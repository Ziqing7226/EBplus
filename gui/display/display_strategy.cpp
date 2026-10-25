// gui/display/display_strategy.cpp — concrete display strategies (design §3.5).
//
// The per-mode logic here is copied verbatim from the former switch branches
// of MainWindow::process_algo_results() (only the dispatch changed: member
// references annotator_/algo_windows_/camera_ and the `this` invokeMethod
// context are now routed through DisplayContext). All QMetaObject::invokeMethod
// queued calls are preserved exactly so widget touches stay marshalled to the
// GUI thread identically to the prior implementation.

#include "display/display_strategy.h"

#include <atomic>

#include <QColor>
#include <QFontMetrics>
#include <QLineF>
#include <QMetaObject>
#include <QPainter>
#include <QPoint>
#include <QPointF>
#include <QPointer>
#include <QRect>
#include <QString>
#include <QStringList>

#include <algorithm>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "algo_bridge/algo_backend.h"   // AlgoResult + Overlay* structs
#include "algo_bridge/algo_bridge.h"    // AlgoInstance, AlgoInfo
#include "app/camera_controller.h"      // CameraController
#include "display/event_display_widget.h"  // EventDisplayWidget
#include "display/frame_annotator.h"    // FrameAnnotator
#include "main_window.h"                // MainWindow (QObject invokeMethod context)
#include "widgets/algo_window.h"        // AlgoWindow

namespace gui {

// ---------------------------------------------------------------------------
// mat_to_qimage (moved from main_window.cpp anonymous namespace)
// ---------------------------------------------------------------------------

QImage mat_to_qimage(const cv::Mat& mat) {
    if (mat.empty()) return QImage();
    if (mat.channels() != 1 && mat.channels() != 3) {
        // Audit §五-G5: don't fail silently — a Replace-mode algorithm
        // producing an unsupported channel count would otherwise show a
        // black screen with no hint as to why. Throttled: first occurrence,
        // then every 300th, so a persistently-misbehaving algorithm doesn't
        // flood the log at display rate.
        static std::atomic<int> bad_channels{0};
        const int c = ++bad_channels;
        if (c == 1 || c % 300 == 0) {
            qWarning("mat_to_qimage: unsupported channel count %d (need 1 or 3) (x%d)",
                     mat.channels(), c);
        }
        return QImage();
    }
    // The QImage owns its buffer and cvtColor writes straight into it —
    // one allocation, no second full-frame copy.
    QImage img(mat.cols, mat.rows, QImage::Format_RGB888);
    if (img.isNull()) return QImage();
    cv::Mat dst_view(mat.rows, mat.cols, CV_8UC3, img.bits(),
                     static_cast<size_t>(img.bytesPerLine()));
    if (mat.channels() == 1) {
        cv::cvtColor(mat, dst_view, cv::COLOR_GRAY2RGB);
    } else {
        cv::cvtColor(mat, dst_view, cv::COLOR_BGR2RGB);
    }
    return img;
}

// ---------------------------------------------------------------------------
// PassiveStrategy
// ---------------------------------------------------------------------------

void PassiveStrategy::apply(QImage& /*frame*/, AlgoResult& result,
                            const AlgoInfo& info, DisplayContext& ctx) {
    // Passive algorithms (in-place event filters like noise_filter /
    // hot_pixel_filter) don't draw overlays or replace the frame, but if the
    // user opened an AlgoWindow for one we still update the status text so the
    // window doesn't stay stuck on "Waiting for events..." forever. The result
    // has already been pulled by the caller.
    auto wit = ctx.algo_windows->find(info.name);
    if (wit == ctx.algo_windows->end() || !wit.value()) return;
    if (!result.status.empty()) {
        QString text = QString::fromStdString(result.status);
        for (const auto& t : result.texts) {
            text += QStringLiteral("\n  ");
            text += QString::fromStdString(t.text);
        }
        QPointer<AlgoWindow> w = wit.value();
        QMetaObject::invokeMethod(ctx.window, [w, text]() {
            if (w) w->set_status_text(text);
        }, Qt::QueuedConnection);
    }
}

// ---------------------------------------------------------------------------
// OverlayStrategy
// ---------------------------------------------------------------------------

void OverlayStrategy::apply(QImage& frame, AlgoResult& r,
                            const AlgoInfo& /*info*/, DisplayContext& ctx) {
    // Phase 2.6 debug D-3: with the unified ROI active, every instance is
    // fed ROI-cropped, ROI-relative events (AlgoInstance::push_events), so
    // all overlay primitives arrive in ROI-relative coordinates. Shift them
    // back by the ROI origin so they land on the processing region of the
    // full-sensor main frame. No-op (offset 0,0) when the ROI is off.
    int ox = 0, oy = 0;
    if (ctx.camera) {
        bool roi_on = false;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        ctx.camera->unified_roi(roi_on, x0, y0, x1, y1);
        // RONI (debug D-5): events keep absolute coordinates — no shift.
        if (roi_on && !ctx.camera->unified_roi_roni()) {
            ox = x0;
            oy = y0;
        }
    }
    // Convert AlgoResult overlay primitives into FrameAnnotator calls.
    // Boxes: tracked-object boxes with optional id.
    if (!r.boxes.empty()) {
        std::vector<FrameAnnotator::Box> boxes;
        boxes.reserve(r.boxes.size());
        for (const auto& b : r.boxes) {
            FrameAnnotator::Box box;
            box.rect = QRect(b.x + ox, b.y + oy, b.w, b.h);
            box.id = b.id;
            boxes.push_back(std::move(box));
        }
        ctx.annotator->draw_boxes(frame, boxes);
    }
    // Lines.
    if (!r.lines.empty()) {
        std::vector<QLineF> lines;
        lines.reserve(r.lines.size());
        for (const auto& l : r.lines) {
            lines.emplace_back(QPointF(l.x1 + ox, l.y1 + oy),
                               QPointF(l.x2 + ox, l.y2 + oy));
        }
        ctx.annotator->draw_lines(frame, lines);
    }
    // Points.
    if (!r.points.empty()) {
        std::vector<QPointF> pts;
        pts.reserve(r.points.size());
        for (const auto& p : r.points) {
            pts.emplace_back(p.x + ox, p.y + oy);
        }
        ctx.annotator->draw_points(frame, pts);
    }
    // Colored points (optical-flow HSV visualization).
    if (!r.colored_points.empty()) {
        std::vector<std::pair<QPointF, QColor>> pts;
        pts.reserve(r.colored_points.size());
        for (const auto& p : r.colored_points) {
            pts.emplace_back(QPointF(p.x + ox, p.y + oy),
                             QColor(p.r, p.g, p.b));
        }
        ctx.annotator->draw_colored_points(frame, pts, 3.0);
    }
    // Circles.
    if (!r.circles.empty()) {
        std::vector<std::pair<QPointF, double>> circs;
        circs.reserve(r.circles.size());
        for (const auto& c : r.circles) {
            circs.emplace_back(QPointF(c.cx + ox, c.cy + oy), c.r);
        }
        ctx.annotator->draw_circles(frame, circs);
    }
    // Text labels.
    if (!r.texts.empty()) {
        for (const auto& t : r.texts) {
            ctx.annotator->draw_text(frame, QString::fromStdString(t.text),
                                     QPoint(t.x + ox, t.y + oy));
        }
    }
    // Flow arrows (sparse/dense optical flow): endpoints pre-scaled by the
    // algo's pps_scale; shift by the ROI origin like every other primitive.
    if (!r.flow_arrows.empty()) {
        std::vector<FrameAnnotator::FlowArrow> arrows;
        arrows.reserve(r.flow_arrows.size());
        for (const auto& a : r.flow_arrows) {
            arrows.push_back({QPointF(a.x1 + ox, a.y1 + oy),
                              QPointF(a.x2 + ox, a.y2 + oy),
                              (a.r || a.g || a.b) ? QColor(a.r, a.g, a.b)
                                                  : QColor()});
        }
        ctx.annotator->draw_flow_arrows(frame, arrows);
    }
    // Colored events (orientation/direction per-event coloring).
    if (!r.colored_events.empty()) {
        std::vector<std::tuple<int, int, QColor>> cevs;
        cevs.reserve(r.colored_events.size());
        for (const auto& ce : r.colored_events) {
            cevs.emplace_back(ce.event.x + ox, ce.event.y + oy,
                              QColor(ce.r, ce.g, ce.b));
        }
        ctx.annotator->draw_colored_events(frame, cevs);
    }
    // Trajectories (cluster history paths).
    if (!r.trajectories.empty()) {
        std::vector<std::pair<int, std::vector<QPointF>>> trajs;
        trajs.reserve(r.trajectories.size());
        for (const auto& t : r.trajectories) {
            std::vector<QPointF> pts;
            pts.reserve(t.points.size());
            for (const auto& pt : t.points) {
                pts.emplace_back(pt.x + ox, pt.y + oy);
            }
            trajs.emplace_back(t.id, std::move(pts));
        }
        ctx.annotator->draw_trajectories(frame, trajs, QColor(0, 255, 0));
    }
    // Centered hint (e.g. the freq_detector initialization-phase usage
    // hint): drawn on the main display frame center with a translucent
    // backdrop, one line per '\n'.
    if (!r.hint.empty()) {
        const QStringList lines =
            QString::fromStdString(r.hint).split(QLatin1Char('\n'));
        QFontMetrics fm(ctx.annotator->font());
        int text_w = 0;
        for (const QString& line : lines) text_w = std::max(text_w, fm.horizontalAdvance(line));
        const int line_h = fm.height();
        const int pad = 12;
        const int box_w = text_w + 2 * pad;
        const int box_h = lines.size() * line_h + 2 * pad;
        QRect box((frame.width() - box_w) / 2, (frame.height() - box_h) / 2,
                  box_w, box_h);
        QPainter p(&frame);
        p.fillRect(box, QColor(0, 0, 0, 160));
        p.setPen(QColor(255, 255, 255));
        p.setFont(ctx.annotator->font());
        int ty = box.top() + pad + fm.ascent();
        for (const QString& line : lines) {
            p.drawText(box.left() + pad, ty, line);
            ty += line_h;
        }
    }
    // Phase 2.6 debug D-2: the aux-frame routing (Hough θ-ρ / accumulator
    // maps) was deleted with the hough aux display — no producer remains.
    // Phase 2.6 debug D-1: the ROI zoom routing to the AlgoWindow was
    // deleted — Overlay algorithms no longer open an AlgoWindow (the main
    // display draws their overlay and the main-display Zoom-to-ROI mode
    // replaces the old window zoom view), so there is no receiver left.
}

// ---------------------------------------------------------------------------
// ReplaceStrategy
// ---------------------------------------------------------------------------

void ReplaceStrategy::apply(QImage& frame, AlgoResult& r,
                            const AlgoInfo& /*info*/, DisplayContext& ctx) {
    // Replace the main display frame with the algorithm output.
    if (r.has_frame && !r.frame.empty()) {
        QImage q = mat_to_qimage(r.frame);
        // Phase 2.6 debug D-3: with the unified ROI active the backend runs
        // at ROI dimensions, so its output frame covers only the ROI
        // window — composite it at the ROI origin instead of replacing the
        // full-sensor frame.
        bool roi_on = false;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (ctx.camera) {
            ctx.camera->unified_roi(roi_on, x0, y0, x1, y1);
        }
        // RONI (debug D-5): the backend stays full-sensor (pass-through), so
        // its output frame replaces the full frame as before.
        if (roi_on && !ctx.camera->unified_roi_roni() &&
            q.width() == x1 - x0 && q.height() == y1 - y0) {
            QPainter p(&frame);
            p.drawImage(x0, y0, q);
        } else {
            frame = q;
        }
    }
}

// ---------------------------------------------------------------------------
// StandaloneStrategy
// ---------------------------------------------------------------------------

void StandaloneStrategy::apply(QImage& /*frame*/, AlgoResult& r,
                               const AlgoInfo& info, DisplayContext& ctx) {
    // Route results to the AlgoWindow (design §5.6.6). Frame-producing algos
    // (time_surface, event_to_video, background_mask) use an
    // EventDisplayWidget; text-producing algos use the default status
    // QLabel. xyt_visualizer is handled separately via SpaceTimeDisplay.
    auto wit = ctx.algo_windows->find(info.name);
    if (wit != ctx.algo_windows->end() && wit.value()) {
        QPointer<AlgoWindow> w = wit.value();
        if (r.has_frame && !r.frame.empty()) {
            // QPointer protects against the dock being closed/undocked (and its
            // display widget destroyed) between scheduling and execution of the
            // queued call.
            QPointer<EventDisplayWidget> disp = w->frame_display();
            if (disp) {
                QImage q = mat_to_qimage(r.frame);
                QMetaObject::invokeMethod(ctx.window, [disp, q]() {
                    if (disp) disp->set_frame(q);
                }, Qt::QueuedConnection);
            }
        }
        if (!r.status.empty()) {
            QString text = QString::fromStdString(r.status);
            for (const auto& t : r.texts) {
                text += QStringLiteral("\n  ");
                text += QString::fromStdString(t.text);
            }
            QMetaObject::invokeMethod(ctx.window, [w, text]() {
                if (w) w->set_status_text(text);
            }, Qt::QueuedConnection);
        }
    }
}

} // namespace gui
