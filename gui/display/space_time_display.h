// gui/display/space_time_display.h — XYT 3D event point-cloud display.
//
// Design §4.3.25 and §1.6.6 (jAER SpaceTimeRollingEventDisplayMethod). A
// QOpenGLWidget that renders the recent event stream as a 3D point cloud:
// X = pixel column, Y = pixel row, T = time (depth axis, newest in front).
// Age-based coloring (newest=blue, oldest=red, matching jAER's fragment
// shader) or polarity coloring is provided by algo/cv/xyt_visualizer.h;
// this widget owns the VBO + GLSL rendering and the orbit camera.
// A 3D bounding box (cuboid wireframe) with axis labels is drawn to match
// jAER's SpaceTimeRollingEventDisplayMethod axes rendering.

#ifndef GUI_DISPLAY_SPACE_TIME_DISPLAY_H
#define GUI_DISPLAY_SPACE_TIME_DISPLAY_H

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPainter>
#include <QTimer>
#include <memory>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo/cv/xyt_visualizer.h"
#include "algo/common/event.h"

class QWheelEvent;
class QMouseEvent;

namespace gui {

class SpaceTimeDisplay : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit SpaceTimeDisplay(QWidget* parent = nullptr);
    ~SpaceTimeDisplay();

    /// @brief Sets the sensor geometry (used to centre the point cloud).
    void set_sensor_geometry(int width, int height);

    /// @brief Feeds a batch of events. Thread-safe via Qt::QueuedConnection
    /// if called from a non-GUI thread.
    void push_events(const Metavision::EventCD* begin, const Metavision::EventCD* end);

public slots:
    void set_time_window_ms(float ms);
    void set_point_size(float s);
    void set_color_mode(int mode);          ///< 0 = Polarity, 1 = Age
    void set_auto_rotate(bool on);
    void set_depth_shade(bool on);
    void clear();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;

private:
    void rebuild_vbo();
    /// @brief Draws a 3D bounding box (12-edge cuboid wireframe) with
    /// color gradient (blue front = newest, dark red back = oldest),
    /// plus axis labels and time-window info as a QPainter 2D overlay.
    /// Matches jAER SpaceTimeRollingEventDisplayMethod maybeRegenerateAxesDisplayList().
    void draw_axes_overlay(const QMatrix4x4& mvp);

    std::unique_ptr<QOpenGLShaderProgram> program_;
    std::unique_ptr<QOpenGLVertexArrayObject> vao_;
    std::unique_ptr<QOpenGLBuffer> vbo_;

    // Line shader for 3D bounding box (separate from point shader because
    // the point fragment shader discards non-circular gl_PointCoord pixels).
    std::unique_ptr<QOpenGLShaderProgram> line_program_;
    std::unique_ptr<QOpenGLVertexArrayObject> axis_vao_;
    std::unique_ptr<QOpenGLBuffer> axis_vbo_;

    gui_algo::XYTVisualizer viz_;
    std::vector<gui_algo::XYTPoint> points_;
    std::vector<float> vbo_data_;  ///< Reused across frames to avoid allocation
    int point_count_{0};
    int vbo_capacity_bytes_{0};
    bool gl_ready_{false};
    /// Set by every path that changes the point cloud (event pushes, time
    /// window / color / shading / geometry changes, clear); consumed by
    /// paintGL so an idle window stops paying the 60 FPS rebuild + VBO
    /// upload cost. Camera motion (auto-rotate, mouse) only changes the
    /// MVP, not the cloud, and does not set this.
    bool vbo_dirty_{true};

    /// Render timer ensures smooth 60 FPS rendering independent of event
    /// push timing (matching jAER's display-rate-driven rendering).
    QTimer* render_timer_{nullptr};

    int sensor_w_{0};
    int sensor_h_{0};

    /// Time axis aspect ratio (depth relative to max of x,y). jAER default=4.
    float time_aspect_ratio_{4.0f};

    // Orbit camera.
    float azimuth_{-35.0f};    ///< degrees
    float elevation_{20.0f};   ///< degrees
    // Scene is a unit cube scaled by (1, 1, time_aspect_ratio_=4) →
    // half-diagonal ≈ sqrt(0.25+0.25+4) ≈ 2.12; with 45° perspective we
    // need dist > ~5.6 to fit, so default to 6.0.
    float distance_{6.0f};
    bool dragging_{false};
    QPoint last_mouse_;
    bool auto_rotate_{false};
};

} // namespace gui

#endif // GUI_DISPLAY_SPACE_TIME_DISPLAY_H
