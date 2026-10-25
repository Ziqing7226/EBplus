// gui/display/space_time_display.cpp
//
// Renders events as a 3D XYT point cloud matching jAER's
// SpaceTimeRollingEventDisplayMethod:
//   X = pixel column (0..1 normalized, horizontal)
//   Y = pixel row (0..1 normalized, vertical, OpenGL Y-up)
//   Z = time (0=oldest at back, 1=newest at front, depth axis)
// The time axis is scaled by time_aspect_ratio_ (default 4, matching jAER).
// A 3D bounding box (12-edge cuboid wireframe) is drawn with a blue→dark-red
// color gradient (front/newest=blue, back/oldest=dark red), matching jAER's
// maybeRegenerateAxesDisplayList(). Axis labels ("x=sx", "y=sy", "t=0",
// "t=-Xms") are placed at the correct 3D corner positions.

#include "space_time_display.h"

#include <QMatrix4x4>
#include <QMouseEvent>
#include <QPainter>
#include <QShowEvent>
#include <QSurfaceFormat>
#include <QVector3D>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace gui {

namespace {
// Point shader: renders event points as round, depth-shaded GL_POINTS.
// aPos = (x_norm, y_norm, t_norm), each in [0,1].
// t_norm=1 at newest (front), t_norm=0 at oldest (back).
constexpr const char* kVertSrc = R"GLSL(#version 330 core
in vec3 aPos;     // (x_norm, y_norm, t_norm), each in [0,1]
in vec3 aColor;
out vec3 vColor;
out float vAge;   // t_norm: 1=newest, 0=oldest
uniform mat4 uMVP;
uniform float uPointSize;
uniform bool uDepthShade;
void main() {
    // Centre the unit cube around the origin.
    vec3 p = aPos - vec3(0.5, 0.5, 0.5);
    gl_Position = uMVP * vec4(p, 1.0);
    vAge = aPos.z;
    float ps = uPointSize;
    if (uDepthShade) {
        // Newer events (higher t) are larger, matching jAER's
        // gl_PointSize = pointSize * f1 + 1 where f1 = tn.
        ps = 1.0 + ps * vAge;
    }
    gl_PointSize = ps;
    vColor = aColor;
}
)GLSL";

constexpr const char* kFragSrc = R"GLSL(#version 330 core
in vec3 vColor;
in float vAge;
out vec4 fragColor;
uniform bool uDepthShade;
void main() {
    // Round points (discard corners) + optional depth dimming.
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) discard;
    vec3 c = vColor;
    if (uDepthShade) {
        // Match jAER SpaceTimeRollingEventDisplayMethod_Fragment.glsl:
        // brightness = 0.75*f1 + 0.25, where f1 = tn = vAge.
        c *= (0.75 * vAge + 0.25);
    }
    fragColor = vec4(c, 0.75);
}
)GLSL";

// Line shader for 3D bounding box edges (per-vertex color, no point rounding).
constexpr const char* kLineVertSrc = R"GLSL(#version 330 core
in vec3 aPos;
in vec3 aColor;
out vec3 vColor;
uniform mat4 uMVP;
void main() {
    vec3 p = aPos - vec3(0.5, 0.5, 0.5);
    gl_Position = uMVP * vec4(p, 1.0);
    vColor = aColor;
}
)GLSL";

constexpr const char* kLineFragSrc = R"GLSL(#version 330 core
in vec3 vColor;
out vec4 fragColor;
void main() {
    fragColor = vec4(vColor, 0.8);
}
)GLSL";

/// Builds the 12-edge cuboid bounding box in normalized [0,1]^3 space.
/// z=1 is the front face (newest, blue), z=0 is the back face (oldest,
/// dark red). Depth edges have a per-vertex color gradient.
/// Returns 24 vertices × 6 floats (3 pos + 3 color) = 144 floats.
constexpr std::size_t kBoxVerts = 24;
constexpr std::size_t kBoxFloats = kBoxVerts * 6;

std::array<float, kBoxFloats> make_bounding_box() {
    // Colors matching jAER maybeRegenerateAxesDisplayList().
    const float blue[3]   = {0.0f, 0.0f, 1.0f};  // front (newest)
    const float dred[3]   = {0.5f, 0.0f, 0.0f};  // back (oldest)

    auto v = [](float x, float y, float z, const float c[3]) {
        return std::array<float, 6>{x, y, z, c[0], c[1], c[2]};
    };

    std::array<float, kBoxFloats> d{};
    std::size_t i = 0;
    auto put = [&](const std::array<float, 6>& a) {
        for (std::size_t j = 0; j < 6; ++j) d[i++] = a[j];
    };

    // Front face edges (z=1, newest) — blue
    put(v(0,0,1, blue));  put(v(1,0,1, blue));   // bottom
    put(v(0,0,1, blue));  put(v(0,1,1, blue));   // left
    put(v(1,0,1, blue));  put(v(1,1,1, blue));   // right
    put(v(1,1,1, blue));  put(v(0,1,1, blue));   // top

    // Back face edges (z=0, oldest) — dark red
    put(v(0,0,0, dred));  put(v(1,0,0, dred));   // bottom
    put(v(0,0,0, dred));  put(v(0,1,0, dred));   // left
    put(v(1,0,0, dred));  put(v(1,1,0, dred));   // right
    put(v(1,1,0, dred));  put(v(0,1,0, dred));   // top

    // Depth edges (z=1→0) — gradient blue→dark red
    put(v(0,0,1, blue));  put(v(0,0,0, dred));   // bottom-left
    put(v(1,0,1, blue));  put(v(1,0,0, dred));   // bottom-right
    put(v(0,1,1, blue));  put(v(0,1,0, dred));   // top-left
    put(v(1,1,1, blue));  put(v(1,1,0, dred));   // top-right

    return d;
}
} // namespace

SpaceTimeDisplay::SpaceTimeDisplay(QWidget* parent) : QOpenGLWidget(parent) {
    setWindowTitle(tr("Space-Time 3D"));
    setMinimumSize(320, 240);
    setSizePolicy(sizePolicy().horizontalPolicy(), sizePolicy().verticalPolicy());

    QSurfaceFormat fmt = format();
    fmt.setSamples(4);  // MSAA for nicer points
    fmt.setDepthBufferSize(24);
    setFormat(fmt);
}

SpaceTimeDisplay::~SpaceTimeDisplay() {
    if (render_timer_) render_timer_->stop();
    // GL resources must be freed with a current context. Check the resources
    // themselves (not just gl_ready_) so a partial initializeGL — e.g. the
    // shader link fallback — still cleans up what it created.
    if (program_ || vao_ || vbo_ || line_program_ || axis_vao_ || axis_vbo_) {
        makeCurrent();
        axis_vbo_.reset();
        axis_vao_.reset();
        line_program_.reset();
        vbo_.reset();
        vao_.reset();
        program_.reset();
        doneCurrent();
    }
}

void SpaceTimeDisplay::set_sensor_geometry(int width, int height) {
    sensor_w_ = (width > 0) ? width : 0;
    sensor_h_ = (height > 0) ? height : 0;
    // Normalization in rebuild_vbo depends on the sensor dimensions.
    vbo_dirty_ = true;
}

void SpaceTimeDisplay::push_events(const Metavision::EventCD* begin,
                                   const Metavision::EventCD* end) {
    if (begin == nullptr || end == nullptr || begin >= end) return;
    const std::size_t n = static_cast<std::size_t>(end - begin);
    // gui_algo::Event is layout-compatible with EventCD; reinterpret-cast view.
    const gui_algo::Event* ev = reinterpret_cast<const gui_algo::Event*>(begin);
    viz_.process(ev, n);
    vbo_dirty_ = true;
    update();
}

void SpaceTimeDisplay::set_time_window_ms(float ms) {
    viz_.set_time_window_ms(ms);
    vbo_dirty_ = true;
    update();
}

void SpaceTimeDisplay::set_point_size(float s) {
    viz_.set_point_size(s);
    update();
}

void SpaceTimeDisplay::set_color_mode(int mode) {
    viz_.set_color_mode(mode == 1 ? gui_algo::XYTVisualizer::ColorMode::Age
                                  : gui_algo::XYTVisualizer::ColorMode::Polarity);
    vbo_dirty_ = true;
    update();
}

void SpaceTimeDisplay::set_auto_rotate(bool on) {
    auto_rotate_ = on;
    update();
}

void SpaceTimeDisplay::set_depth_shade(bool on) {
    viz_.set_depth_shade(on);
    vbo_dirty_ = true;
    update();
}

void SpaceTimeDisplay::clear() {
    viz_.clear();
    points_.clear();
    point_count_ = 0;
    vbo_dirty_ = true;
    update();
}

void SpaceTimeDisplay::initializeGL() {
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);

    program_ = std::make_unique<QOpenGLShaderProgram>();
    bool points_ok =
        program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertSrc) &&
        program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragSrc);
    if (points_ok) {
        program_->bindAttributeLocation("aPos", 0);
        program_->bindAttributeLocation("aColor", 1);
        points_ok = program_->link();
    }
    if (!points_ok) {
        // Shader failure should not happen on a sane GL driver; fall back to
        // a plain clear + overlay notice (paintGL) instead of drawing with a
        // broken program (audit §六-D3, matches EventDisplayWidget).
        program_.reset();
    }

    vao_ = std::make_unique<QOpenGLVertexArrayObject>();
    vao_->create();
    vao_->bind();

    vbo_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    vbo_->create();
    vbo_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vbo_->bind();

    const int kInitialVboBytes = 65536 * 6 * static_cast<int>(sizeof(float));
    vbo_->allocate(nullptr, kInitialVboBytes);
    vbo_capacity_bytes_ = kInitialVboBytes;

    if (program_) {
        // stride = 6 floats (3 pos + 3 color)
        const int stride = 6 * static_cast<int>(sizeof(float));
        program_->enableAttributeArray(0);
        program_->setAttributeBuffer(0, GL_FLOAT, 0, 3, stride);
        program_->enableAttributeArray(1);
        program_->setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, stride);
    }

    vao_->release();
    vbo_->release();

    // --- Line shader + bounding box VBO (jAER cuboid wireframe) ---
    line_program_ = std::make_unique<QOpenGLShaderProgram>();
    bool lines_ok =
        line_program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kLineVertSrc) &&
        line_program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kLineFragSrc);
    if (lines_ok) {
        line_program_->bindAttributeLocation("aPos", 0);
        line_program_->bindAttributeLocation("aColor", 1);
        lines_ok = line_program_->link();
    }
    if (!lines_ok) {
        // Bounding-box shader failed — points can still be drawn; skip the
        // axes overlay only (draw_axes_overlay guards on line_program_).
        line_program_.reset();
    }

    // 12-edge bounding box in [0,1]^3 (24 vertices, 6 floats each).
    const auto box = make_bounding_box();
    axis_vao_ = std::make_unique<QOpenGLVertexArrayObject>();
    axis_vao_->create();
    axis_vao_->bind();
    axis_vbo_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    axis_vbo_->create();
    axis_vbo_->setUsagePattern(QOpenGLBuffer::StaticDraw);
    axis_vbo_->bind();
    axis_vbo_->allocate(box.data(), static_cast<int>(box.size() * sizeof(float)));
    if (line_program_) {
        const int axis_stride = 6 * static_cast<int>(sizeof(float));
        line_program_->enableAttributeArray(0);
        line_program_->setAttributeBuffer(0, GL_FLOAT, 0, 3, axis_stride);
        line_program_->enableAttributeArray(1);
        line_program_->setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, axis_stride);
    }
    axis_vao_->release();
    axis_vbo_->release();

    glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
    // gl_ready_ requires the point shader — without it the 3D view cannot
    // render at all and paintGL shows the fallback notice instead.
    gl_ready_ = (program_ != nullptr);

    // Start a 60 FPS render timer. This decouples rendering from event push
    // timing, matching jAER's display-rate-driven rendering. Without this,
    // the FPS is limited by the event push interval (which may be variable).
    render_timer_ = new QTimer(this);
    render_timer_->setInterval(16);  // ~60 FPS
    connect(render_timer_, &QTimer::timeout, this, [this]() { update(); });
    render_timer_->start();
}

void SpaceTimeDisplay::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void SpaceTimeDisplay::showEvent(QShowEvent* event) {
    QOpenGLWidget::showEvent(event);
    // render_timer_ is created in initializeGL(); restart it on subsequent
    // shows (after a hide). The very first show triggers initializeGL which
    // creates and starts the timer itself, so render_timer_ is null here.
    if (render_timer_ && !render_timer_->isActive()) {
        render_timer_->start();
    }
}

void SpaceTimeDisplay::hideEvent(QHideEvent* event) {
    QOpenGLWidget::hideEvent(event);
    // Stop the 60 FPS render timer when the widget is not visible — avoids
    // unnecessary VBO rebuilds and GPU uploads while the dock is hidden.
    if (render_timer_) {
        render_timer_->stop();
    }
}

void SpaceTimeDisplay::rebuild_vbo() {
    viz_.render(points_);
    point_count_ = static_cast<int>(points_.size());
    if (!gl_ready_ || !vbo_) return;

    // Build a flat float buffer of normalized positions + colors.
    // X = pixel column / sensor_w, Z = t_norm (0=oldest, 1=newest).
    // Y = 1 - pixel_row / sensor_h: the box +y end is the sensor's TOP row
    // (y=0), matching the main display's top-down image rows (axis mapping
    // (t, 1-y, x)). Previously the row index went straight to +y, so the
    // cloud was vertically flipped relative to the main display.
    const float sx = (sensor_w_ > 0) ? 1.0f / static_cast<float>(sensor_w_) : 1.0f;
    const float sy = (sensor_h_ > 0) ? 1.0f / static_cast<float>(sensor_h_) : 1.0f;

    // Reuse vbo_data_ across frames to avoid per-frame heap allocation.
    vbo_data_.clear();
    vbo_data_.reserve(static_cast<std::size_t>(point_count_) * 6);
    for (const auto& p : points_) {
        vbo_data_.push_back(p.x * sx);
        vbo_data_.push_back(1.0f - p.y * sy);
        vbo_data_.push_back(p.t);
        vbo_data_.push_back(p.r);
        vbo_data_.push_back(p.g);
        vbo_data_.push_back(p.b);
    }

    vbo_->bind();
    const int needed = static_cast<int>(vbo_data_.size() * sizeof(float));
    if (needed > vbo_capacity_bytes_) {
        const int new_cap = std::max(needed, vbo_capacity_bytes_ * 2);
        vbo_->allocate(nullptr, new_cap);
        vbo_capacity_bytes_ = new_cap;
    }
    vbo_->write(0, vbo_data_.data(), needed);
    vbo_->release();
}

void SpaceTimeDisplay::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!gl_ready_ || !program_) {
        // Shader setup failed in initializeGL() — show an overlay notice
        // instead of a silent black window (audit §六-D3).
        QPainter painter(this);
        painter.setPen(QPen(QColor(220, 130, 130)));
        painter.drawText(rect(), Qt::AlignCenter,
                         tr("3D view unavailable: OpenGL shader setup failed."));
        return;
    }

    if (auto_rotate_) {
        azimuth_ += 0.4f;
        if (azimuth_ > 360.0f) azimuth_ -= 360.0f;
    }

    // Rebuild only when the point cloud actually changed. Without this the
    // 60 FPS render timer re-runs viz_.render + the full VBO upload even
    // with zero new events (pure idle burn).
    if (vbo_dirty_) {
        rebuild_vbo();
        vbo_dirty_ = false;
    }

    // Compute MVP. jAER preserves the sensor aspect ratio: the box is
    // sx × sy × (smax * timeAspectRatio). We normalize by smax so the
    // largest spatial dimension is 1.0, then scale Z by time_aspect_ratio.
    // This matches jAER's proportions exactly (e.g. 640×480 → 1.0 × 0.75 × 4).
    const int sw_raw = sensor_w_ > 0 ? sensor_w_ : 1;
    const int sh_raw = sensor_h_ > 0 ? sensor_h_ : 1;
    const float smax = static_cast<float>(std::max(sw_raw, sh_raw));
    const float sw = static_cast<float>(sw_raw) / smax;
    const float sh = static_cast<float>(sh_raw) / smax;

    const float az = azimuth_ * static_cast<float>(M_PI) / 180.0f;
    const float el = elevation_ * static_cast<float>(M_PI) / 180.0f;
    const float dist = distance_;
    QVector3D eye(dist * std::cos(el) * std::sin(az),
                  dist * std::sin(el),
                  dist * std::cos(el) * std::cos(az));
    QMatrix4x4 view;
    view.lookAt(eye, QVector3D(0, 0, 0), QVector3D(0, 1, 0));
    QMatrix4x4 proj;
    proj.perspective(45.0f,
                     static_cast<float>(width()) / static_cast<float>(std::max(height(), 1)),
                     0.05f, 100.0f);
    QMatrix4x4 model;
    model.scale(sw, sh, time_aspect_ratio_);
    const QMatrix4x4 mvp = proj * view * model;

    // Draw event points.
    if (point_count_ > 0) {
        program_->bind();
        vao_->bind();
        program_->setUniformValue("uMVP", mvp);
        program_->setUniformValue("uPointSize", viz_.point_size());
        program_->setUniformValue("uDepthShade", viz_.depth_shade());
        glDrawArrays(GL_POINTS, 0, point_count_);
        vao_->release();
        program_->release();
    }

    // Draw 3D bounding box + labels.
    draw_axes_overlay(mvp);
    // Note: no need to call update() for auto_rotate_ — the render timer
    // (render_timer_) drives continuous 60 FPS repaints.
}

void SpaceTimeDisplay::draw_axes_overlay(const QMatrix4x4& mvp) {
    // Draw 12-edge bounding box with the line shader.
    if (line_program_ && axis_vao_) {
        glLineWidth(2.0f);
        line_program_->bind();
        axis_vao_->bind();
        line_program_->setUniformValue("uMVP", mvp);
        glDrawArrays(GL_LINES, 0, 24);  // 12 edges × 2 vertices
        axis_vao_->release();
        line_program_->release();
    }

    // QPainter 2D overlay: axis labels + info text.
    // Project 3D corner positions to screen coordinates for label placement.
    // The shader centers by -0.5, and the model matrix (included in mvp)
    // handles all scaling (sw, sh, time_aspect_ratio). We must NOT
    // double-scale Z here — just center and let mvp do the rest.
    auto project = [&](float x, float y, float z) -> QPointF {
        QVector3D p(x - 0.5f, y - 0.5f, z - 0.5f);
        QVector3D clip = mvp.map(p);
        // Guard against inf/NaN projections (degenerate MVP, w≈0) — skip the
        // label instead of drawing at garbage coordinates (audit §六-D4).
        if (!std::isfinite(clip.x()) || !std::isfinite(clip.y()) ||
            !std::isfinite(clip.z())) return QPointF(-1, -1);
        if (clip.z() < -1.0f || clip.z() > 1.0f) return QPointF(-1, -1);
        return QPointF((clip.x() * 0.5f + 0.5f) * width(),
                       (1.0f - (clip.y() * 0.5f + 0.5f)) * height());
    };

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    QFont font("Monospace", 9);
    font.setBold(true);
    painter.setFont(font);

    // Label positions matching jAER maybeRegenerateAxesDisplayList():
    //   "x=sx"  at (1.05, 0, 1) — beyond right edge, bottom, front (newest)
    //   "y=sy"  at (0, -0.05, 1) — left, beyond BOTTOM, front: data Y is
    //           written as 1 - row/sensor_h, so the sensor's max row sits
    //           at the -y end of the box.
    //   "t=0"   at (-0.05, 0, 1) — beyond left, bottom, front (t=now)
    //   "t=-Xms" at (1.05, 0, 0) — beyond right, bottom, back (oldest) — RED
    const QPointF x_pos = project(1.05f, 0.0f, 1.0f);
    const QPointF y_pos = project(0.0f, -0.05f, 1.0f);
    const QPointF t0_pos = project(-0.05f, 0.0f, 1.0f);
    const QPointF t1_pos = project(1.05f, 0.0f, 0.0f);

    const QString x_label = QString("x=%1").arg(sensor_w_);
    const QString y_label = QString("y=%1").arg(sensor_h_);
    const QString t0_label = QStringLiteral("t=0");
    const float tw_ms = viz_.time_window_ms();
    const QString t1_label = QString("t=-%1ms").arg(tw_ms, 0, 'f', 1);

    auto draw_label = [&](const QPointF& pos, const QString& text, const QColor& color) {
        if (pos.x() < 0) return;
        const QRectF br = painter.fontMetrics().boundingRect(text);
        QRectF box = br.adjusted(-4, -2, 4, 2);
        box.moveCenter(pos);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0, 0, 0, 180));
        painter.drawRoundedRect(box, 3, 3);
        painter.setPen(QPen(color));
        painter.drawText(box, Qt::AlignCenter, text);
    };

    draw_label(x_pos, x_label, QColor(100, 150, 255));   // x axis — blue
    draw_label(y_pos, y_label, QColor(100, 255, 100));   // y axis — green
    draw_label(t0_pos, t0_label, QColor(100, 150, 255)); // t=0 (newest) — blue
    draw_label(t1_pos, t1_label, QColor(255, 80, 80));   // t=-Xms (oldest) — red

    // Info overlay (top-left corner).
    QString info = QString("events: %1  |  window: %2ms  |  color: %3")
                       .arg(point_count_)
                       .arg(tw_ms, 0, 'f', 1)
                       .arg(viz_.color_mode() == gui_algo::XYTVisualizer::ColorMode::Age
                                ? "Age" : "Polarity");
    painter.setPen(QPen(QColor(200, 200, 200)));
    painter.drawText(10, 15, info);

    painter.end();
}

void SpaceTimeDisplay::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        last_mouse_ = event->position().toPoint();
    }
}

void SpaceTimeDisplay::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) return;
    const QPoint cur = event->position().toPoint();
    const QPoint delta = cur - last_mouse_;
    azimuth_ += delta.x() * 0.4f;
    elevation_ -= delta.y() * 0.4f;
    if (elevation_ > 89.0f) elevation_ = 89.0f;
    if (elevation_ < -89.0f) elevation_ = -89.0f;
    last_mouse_ = cur;
    update();
}

void SpaceTimeDisplay::mouseReleaseEvent(QMouseEvent* /*event*/) {
    dragging_ = false;
}

void SpaceTimeDisplay::wheelEvent(QWheelEvent* event) {
    const int delta = event->angleDelta().y();
    if (delta > 0) {
        distance_ *= 0.9f;
    } else if (delta < 0) {
        distance_ *= 1.1f;
    }
    if (distance_ < 0.3f) distance_ = 0.3f;
    if (distance_ > 20.0f) distance_ = 20.0f;
    update();
}

} // namespace gui
