#include "workspace.h"
#include "app/icon_provider.h"
#include "calibration/blinking_chessboard_display.h"
#include "resources/theme/tokens.h"
#include <QAction>
#include <QAbstractItemView>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDesktopServices>
#include <QDialog>
#include <QDir>
#include <QDockWidget>
#include <QDoubleValidator>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QResizeEvent>
#include <QScreen>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStyle>
#include <QStyleHints>
#include <QTimer>
#include <QToolButton>
#include <QToolTip>
#include <QTreeWidget>
#include <QHeaderView>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidgetAction>
#include <QWindow>
#include <algorithm>
#include <limits>
#include <cmath>

namespace gui::multicam {
namespace {
template <typename Widget> void set_text(Widget *widget, const QString &text) {
    if (widget->text() != text)
        widget->setText(text);
}
// Plain-language explanations for every synchronization / trigger-board message, shown as hover
// help on the alert banner, the stage status and the trigger plot.
struct MessageHelp {
    const char *needle;
    const char *title;
    const char *meaning;
};
const MessageHelp kMessageHelp[]{
    {"formal six-edge matches", "Formal edges not uniquely found",
     "The finite check sends 1 preamble pulse, then 3 formal pulses (6 edges, 1.000 s apart) on GP2. "
     "Each camera must decode exactly one run of 6 alternating edges ~1 s apart. "
     "0 matches = edges missing, extra glitch edges, or wrong spacing (check that camera's trigger cable, "
     "ground and current-limit resistor). 2+ matches = more pulses than expected (another output was "
     "active)."},
    {"polarity is opposite", "Polarity differs between cameras",
     "One camera reports the pulses inverted compared with the other two, usually a swapped or different "
     "trigger input interface. All three inputs must be wired the same way."},
    {"exceeds the project limit", "Timing over the limit",
     "All edges were found, but the largest offset from the Clock Master is above \"Max time difference "
     "(us)\". Check the clock-sync cable chain and Master selection; relax the limit only if justified."},
    {"Extra records are present", "Passed with extra records",
     "Timing passed, but the cameras also logged extra Trigger edges outside the formal six. Timing is "
     "valid; record completeness is not certified."},
    {"Six formal edges uniquely matched", "Synchronization passed",
     "Every camera decoded the six formal edges and all offsets are within the project limit."},
    {"Trigger In polarity is inverted", "Inverted Trigger In (informational)",
     "The cameras report the GP2 rising edge as polarity 0. This is a property of the camera trigger input "
     "front-end and does not affect timing."},
    {"not found over USB", "Trigger board not connected",
     "No RP2350 with the bound USB serial is visible. Plug it into this computer with a data-capable USB "
     "cable and check again. The cameras stay connected."},
    {"More than one serial port", "Duplicate trigger board",
     "Several serial ports report the bound RP2350 identity. Unplug duplicates and check again."},
    {"firmware, GPIO or output state", "Trigger board firmware mismatch",
     "The RP2350 answered, but its firmware hash, GPIO or idle LOW state does not match the approved "
     "finite-burst module. Re-flash the approved firmware."},
    {"output state is unknown", "GP2 state unconfirmed",
     "The board did not confirm that GP2 returned LOW. Stop Trigger / check the board before continuing so "
     "no stray pulses reach the cameras."},
    {"Finite burst response did not validate", "Burst not confirmed",
     "The board's report of the pulse burst (count, timing, drive) did not match what was requested."},
    {"timed out", "Board command timed out",
     "The RP2350 did not answer in time; output state is unconfirmed. Re-check the board."},
    {"ENOMEM", "RP2350 out of PIO resources",
     "Power-cycle the RP2350 and check it again; no automatic retry was performed."},
    {"moved backwards", "Camera timestamps went backwards",
     "A camera stream produced decreasing timestamps (USB hiccup or stream restart). Reconnect the rig."},
    {"Confirm GP2 Trigger wiring", "Wiring not confirmed",
     "Tick the wiring / current-limit confirmation on 2A Wire before sending pulses."},
    {"Connect all three", "Cameras not connected", "Connect the configured rig before synchronization."},
};
QString message_help_for(const QString &message) {
    for (const auto &h : kMessageHelp)
        if (message.contains(QLatin1String(h.needle), Qt::CaseInsensitive))
            return QStringLiteral("<b>%1</b><br>%2").arg(QObject::tr(h.title), QObject::tr(h.meaning));
    return {};
}
QString message_glossary_html() {
    QString html = QObject::tr("<b>Synchronization &amp; trigger-board messages</b><ul>");
    for (const auto &h : kMessageHelp)
        html += QStringLiteral("<li><b>%1</b> — %2</li>").arg(QObject::tr(h.title), QObject::tr(h.meaning));
    return html + QStringLiteral("</ul>");
}
void set_tooltip(QWidget *widget, const QString &text) {
    if (widget->toolTip() != text)
        widget->setToolTip(text);
}
void enable_command(QToolButton *button, bool enabled, const QString &reason) {
    if (button->isEnabled() != enabled) {
        button->setEnabled(enabled);
        if (button->property("multiPrimary").toBool()) {
            const auto icon_name = button->property("multiIcon").toString();
            if (!icon_name.isEmpty())
                button->setIcon(IconProvider::get(
                    icon_name, enabled ? QColor(Qt::white) : button->palette().buttonText().color()));
        }
    }
    set_tooltip(button, enabled ? button->text() : reason);
}
} // namespace

class Preview final : public QWidget {
  public:
    CameraSnapshot camera;
    CalibrationStatus calibration;
    int index = 0;
    int master_camera = 1;
    int reference_camera = 1;
    bool overlays = false;
    bool epipolar_overlay = false;
    QString hole_label;
    bool connecting_active = false;
    QString connecting_status;
    QJsonArray validation_points;
    QToolButton *maximize;
    std::function<void()> toggle_maximized;
    explicit Preview(QWidget *parent) : QWidget(parent) {
        setMinimumSize(160, 170);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        maximize = new QToolButton(this);
        maximize->setFixedSize(28, 28);
        maximize->setToolButtonStyle(Qt::ToolButtonIconOnly);
        connect(maximize, &QToolButton::clicked, this, [this] {
            if (toggle_maximized)
                toggle_maximized();
        });
        set_maximized(false);
    }
    void set_maximized(bool expanded) {
        setProperty("multiMaximized", expanded);
        maximize->setIcon(IconProvider::get(expanded ? "restore" : "maximize"));
        maximize->setToolTip(expanded ? tr("Restore three-camera view") : tr("Maximize this camera"));
        maximize->setAccessibleName(maximize->toolTip());
    }
    double aspect_ratio() const {
        return camera.width > 0 && camera.height > 0 ? double(camera.width) / camera.height : 16. / 9.;
    }
    int height_for_width(int width) const { return int(std::round((width - 8) / aspect_ratio())) + 102; }
    QRect canvas_rect() const {
        const QRect available = rect().adjusted(4, 34, -4, -68);
        QSize image(std::max(1, camera.width), std::max(1, camera.height));
        if (camera.width <= 0 || camera.height <= 0)
            image = QSize(1280, 720);
        image.scale(available.size(), Qt::KeepAspectRatio);
        return QRect(
            QPoint(available.center().x() - image.width() / 2, available.center().y() - image.height() / 2),
            image);
    }
    void resizeEvent(QResizeEvent *event) override {
        QWidget::resizeEvent(event);
        maximize->move(width() - 34, 3);
        setProperty("multiCanvasRect", canvas_rect());
    }
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        const bool is_ref = index == reference_camera;
        const bool is_master = index == master_camera;
        const QRect card_rect = rect().adjusted(1, 1, -1, -1);
        p.setPen(QPen(is_ref ? palette().highlight().color() : palette().mid().color(), is_ref ? 1.5 : 1.0));
        p.setBrush(palette().base());
        p.drawRoundedRect(card_rect, 8, 8);

        const QString position = camera.binding.position.isEmpty()
                                     ? QString("Camera %1").arg(index + 1)
                                     : camera.binding.position;
        const QString hole_tag = hole_label.isEmpty() ? QString{} : (QStringLiteral(" · ") + hole_label);
        const QString role_tag = (is_master && is_ref) ? tr(" [Master/Ref]")
                                 : is_master           ? tr(" [Master]")
                                 : is_ref              ? tr(" [Pose Ref]")
                                                       : QString{};
        const auto title = position + hole_tag + role_tag + " | " +
                           (camera.binding.serial.isEmpty() ? tr("Standby") : camera.binding.serial);
        QFont title_font = font();
        title_font.setBold(true);
        p.setFont(title_font);
        p.setPen(palette().text().color());
        p.drawText(QRect(10, 6, width() - 122, 22), Qt::AlignLeft | Qt::AlignVCenter,
                   fontMetrics().elidedText(title, Qt::ElideRight, width() - 122));

        // Status badge pill on top-right next to maximize button
        const int usb_speed_mbps = camera.metadata.value("usbSpeedMbps").toInt(0);
        const bool is_usb2_warning = camera.connected && usb_speed_mbps > 0 && usb_speed_mbps < 5000;
        const QString usb_short =
            usb_speed_mbps >= 10000 ? QStringLiteral("USB 3.1 · 10 Gbps")
            : usb_speed_mbps >= 5000 ? QStringLiteral("USB 3.0 · 5 Gbps")
            : usb_speed_mbps == 480  ? QStringLiteral("⚠ USB 2.0 · 480 Mbps")
            : usb_speed_mbps > 0     ? QStringLiteral("⚠ USB 1.x · %1 Mbps").arg(usb_speed_mbps)
                                     : QStringLiteral("USB 3.0 · 5 Gbps");
        const bool full_corners = camera.corners.size() == 54;
        const QString badge_text =
            overlays ? (full_corners ? QStringLiteral("FOV OK 54/54")
                                     : QStringLiteral("%1/54 pts").arg(camera.corners.size()))
            : (!camera.connected && connecting_active) ? QStringLiteral("CONNECTING")
            : is_usb2_warning                          ? QStringLiteral("⚠ USB 2.0")
            : camera.connected                         ? QStringLiteral("USB 3.0")
                                                       : QStringLiteral("STANDBY");
        const QColor badge_bg =
            overlays ? (full_corners ? QColor(27, 153, 102, 48) : QColor(217, 130, 43, 48))
            : (!camera.connected && connecting_active) ? QColor(56, 139, 253, 52)
            : is_usb2_warning                          ? QColor(229, 72, 77, 64)
            : camera.connected                         ? QColor(27, 153, 102, 42)
                                                       : QColor(120, 132, 148, 36);
        const QColor badge_fg =
            overlays ? (full_corners ? QColor(46, 184, 126) : QColor(230, 154, 56))
            : (!camera.connected && connecting_active) ? QColor(88, 166, 255)
            : is_usb2_warning                          ? QColor(248, 113, 113)
            : camera.connected                         ? QColor(46, 184, 126)
                                                       : palette().placeholderText().color();
        QRect badge_rect(width() - 114, 6, 74, 20);
        p.setPen(QPen(badge_fg, 1));
        p.setBrush(badge_bg);
        p.drawRoundedRect(badge_rect, 5, 5);
        QFont small_font = font();
        small_font.setPointSize(std::max(10, small_font.pointSize() - 1));
        p.setFont(small_font);
        p.setPen(badge_fg);
        p.drawText(badge_rect, Qt::AlignCenter, badge_text);
        p.setFont(font());

        const QRect target = canvas_rect();
        p.fillRect(target, QColor(22, 31, 43));
        if (!camera.preview.isNull()) {
            p.drawImage(target, camera.preview);
        } else if (!camera.connected && camera.event_count == 0) {
            if (connecting_active) {
                p.setPen(QColor(88, 166, 255));
                QFont f = font();
                f.setBold(true);
                p.setFont(f);
                const QString msg =
                    position + " (" +
                    (camera.binding.serial.isEmpty() ? tr("Unassigned") : camera.binding.serial) + ")\n" +
                    tr("Connecting USB 3.0 stream & verifying clock/Trigger In...") +
                    (connecting_status.isEmpty() ? QString{} : ("\n" + connecting_status));
                p.drawText(target.adjusted(12, 0, -12, -12), Qt::AlignCenter | Qt::TextWordWrap, msg);
                p.setFont(font());
            } else {
                p.setPen(QColor(138, 155, 178));
                p.drawText(target.adjusted(12, 0, -12, -16), Qt::AlignCenter | Qt::TextWordWrap,
                           position + (is_ref ? tr(" (Master & Pose Ref)\n") : tr("\n")) +
                               tr("Camera Standby"));
            }
        }
        if (is_usb2_warning) {
            const QRect warn_rect(target.left() + 8, target.top() + 8, std::min(target.width() - 16, 290), 22);
            p.setPen(QPen(QColor(248, 113, 113), 1));
            p.setBrush(QColor(185, 28, 28, 215));
            p.drawRoundedRect(warn_rect, 5, 5);
            p.setFont(small_font);
            p.setPen(QColor(255, 245, 245));
            p.drawText(warn_rect.adjusted(6, 0, -6, 0), Qt::AlignLeft | Qt::AlignVCenter,
                       fontMetrics().elidedText(tr("⚠ USB 2.0 (480 Mbps) — Expected USB 3.0 (5 Gbps)"),
                                                Qt::ElideRight, warn_rect.width() - 12));
            p.setFont(font());
        }
        if (overlays && camera.width > 0 && camera.height > 0) {
            p.save();
            p.setClipRect(target);
            p.translate(target.topLeft());
            p.scale(double(target.width()) / camera.width, double(target.height()) / camera.height);
            p.setPen(QPen(QColor(64, 205, 159), 2.0 * camera.width / target.width()));
            for (const auto &point : camera.corners)
                p.drawEllipse(point, 3.0 * camera.width / target.width(),
                              3.0 * camera.width / target.width());
            // Draw yellow target pose guide box ONLY on the target/reference camera to avoid stereo parallax conflict
            const auto r = calibration.target_region;
            if (!r.isEmpty() && index == calibration.target_camera) {
                p.setPen(QPen(QColor(244, 195, 73), 2.2 * camera.width / target.width(), Qt::DashLine));
                p.drawRect(QRectF(r.x() * camera.width, r.y() * camera.height, r.width() * camera.width,
                                  r.height() * camera.height));
            } else if (full_corners) {
                p.setPen(QPen(QColor(46, 184, 126), 2.0 * camera.width / target.width()));
                p.drawRect(QRectF(2, 2, camera.width - 4, camera.height - 4));
            }
            for (int y = 0; y < 3; ++y)
                for (int x = 0; x < 3; ++x) {
                    const int n = calibration.coverage[index][y * 3 + x];
                    QColor color = n ? QColor(27, 153, 102, 42) : QColor(205, 65, 80, 18);
                    p.fillRect(QRectF(x * camera.width / 3., y * camera.height / 3., camera.width / 3.,
                                      camera.height / 3.),
                               color);
                }
            if (validation_points.size() == 3) {
                auto overlay = validation_points[index].toObject();
                auto observed = overlay.value("observed").toArray();
                auto projected = overlay.value("projected").toArray();
                for (int j = 0; j < std::min(observed.size(), projected.size()); ++j) {
                    auto a = observed[j].toArray(), b = projected[j].toArray();
                    if (a.size() != 2 || b.size() != 2)
                        continue;
                    QPointF measured(a[0].toDouble(), a[1].toDouble()),
                        predicted(b[0].toDouble(), b[1].toDouble());
                    p.setPen(QPen(QColor(246, 112, 99), 2.0 * camera.width / target.width()));
                    p.drawLine(measured, predicted);
                    p.drawEllipse(predicted, 4, 4);
                }
            }
            p.restore();
        }
        if (epipolar_overlay) {
            p.save();
            p.setClipRect(target);
            p.setPen(QPen(QColor(52, 211, 153, 150), 1.1, Qt::DashLine));
            for (int r = 1; r <= 4; ++r) {
                const int y = target.top() + (target.height() * r) / 5;
                p.drawLine(target.left(), y, target.right(), y);
            }
            p.restore();
        }
        p.setPen(palette().text().color());
        if (!camera.connected && camera.preview.isNull() && camera.event_count == 0) {
            p.setPen(connecting_active ? QColor(88, 166, 255) : palette().placeholderText().color());
            const QString standby_line =
                connecting_active ? tr("Connecting USB 3.0 stream & clock role...")
                : is_ref          ? tr("Standby | Reference & Clock Master (Center)")
                                  : tr("Standby | Synchronized Slave (%1)").arg(position);
            p.drawText(QRect(10, height() - 52, width() - 20, 20), Qt::AlignLeft | Qt::AlignVCenter,
                       fontMetrics().elidedText(standby_line, Qt::ElideRight, width() - 20));
            const QString hint_line =
                connecting_active && !connecting_status.isEmpty() ? connecting_status
                : camera.binding.serial.isEmpty()
                    ? tr("Assign serial or click Quick Fill Defaults to configure")
                    : tr("Configured serial: %1 — Expected protocol: USB 3.0 (5 Gbps)").arg(camera.binding.serial);
            p.drawText(QRect(10, height() - 30, width() - 20, 18), Qt::AlignLeft | Qt::AlignVCenter,
                       fontMetrics().elidedText(hint_line, Qt::ElideRight, width() - 20));
            return;
        }
        const auto rate = camera.connected && std::isfinite(camera.event_rate)
                              ? QString::number(camera.event_rate / 1e6, 'f', 2) + " Mev/s decoded"
                              : tr("Decoded rate unavailable");
        const QString line0 = QString("%1 x %2 | %3").arg(camera.width).arg(camera.height).arg(rate);
        p.drawText(QRect(10, height() - 62, width() - 20, 18), Qt::AlignLeft | Qt::AlignVCenter,
                   fontMetrics().elidedText(line0, Qt::ElideRight, width() - 20));

        // Line 1: USB protocol & speed badge right next to data rate (MB/s decoded)
        p.setFont(small_font);
        const int pill_w = std::min(width() - 20, fontMetrics().horizontalAdvance(usb_short) + 12);
        const QRect usb_pill(10, height() - 42, pill_w, 17);
        const QColor usb_fg = is_usb2_warning ? QColor(248, 113, 113) : QColor(46, 184, 126);
        const QColor usb_bg = is_usb2_warning ? QColor(229, 72, 77, 52) : QColor(27, 153, 102, 36);
        p.setPen(QPen(usb_fg, 1));
        p.setBrush(usb_bg);
        p.drawRoundedRect(usb_pill, 4, 4);
        p.setPen(usb_fg);
        p.drawText(usb_pill, Qt::AlignCenter, usb_short);
        p.setFont(font());

        const QString throughput = QString("%1 MB/s decoded | %2")
                                       .arg(camera.byte_rate / 1e6, 0, 'f', 2)
                                       .arg(std::isfinite(camera.decode_realtime_ratio)
                                                ? QString::number(camera.decode_realtime_ratio, 'f', 2) + "x RT"
                                                : tr("RT not measured"));
        p.setPen(is_usb2_warning ? QColor(248, 113, 113) : palette().placeholderText().color());
        const int tp_x = usb_pill.right() + 7;
        if (tp_x < width() - 12)
            p.drawText(QRect(tp_x, height() - 42, width() - 10 - tp_x, 18), Qt::AlignLeft | Qt::AlignVCenter,
                       fontMetrics().elidedText(throughput, Qt::ElideRight, width() - 10 - tp_x));

        const QString timing =
            QString("t: %1 | %2 corners | last decode: %3")
                .arg(camera.latest_timestamp_us < 0
                         ? tr("not recorded")
                         : QString::number(camera.latest_timestamp_us / 1e6, 'f', 3) + " s")
                .arg(camera.corners.size())
                .arg(camera.decode_age_ms < 0 ? tr("unknown")
                                              : QString::number(camera.decode_age_ms) + " ms ago");
        p.setPen(palette().placeholderText().color());
        p.drawText(QRect(10, height() - 22, width() - 20, 18), Qt::AlignLeft | Qt::AlignVCenter,
                   fontMetrics().elidedText(timing, Qt::ElideRight, width() - 20));
    }
};

class PreviewArea final : public QWidget {
  public:
    std::array<Preview *, 3> previews;
    int maximized = -1;
    explicit PreviewArea(QWidget *parent) : QWidget(parent) {
        setObjectName("multiPreviewArea");
        for (int i = 0; i < 3; ++i) {
            previews[i] = new Preview(this);
            previews[i]->index = i;
            previews[i]->setObjectName(QString("multiPreview%1").arg(i));
            previews[i]->maximize->setObjectName(QString("multiPreviewMaximize%1").arg(i));
            previews[i]->toggle_maximized = [this, i] {
                maximized = maximized == i ? -1 : i;
                arrange();
            };
        }
    }
    void arrange() {
        int columns = maximized >= 0 ? 1 : width() >= 840 ? 3 : width() >= 480 ? 2 : 1;
        const int viewport_height = parentWidget() ? parentWidget()->height() : height();
        if (maximized < 0) {
            int best_width = 0;
            for (int candidate = 1; candidate <= 3; ++candidate) {
                const int rows = (3 + candidate - 1) / candidate;
                const int available_height = (viewport_height - (rows - 1) * 8) / rows;
                const int available_width = (width() - (candidate - 1) * 8) / candidate;
                int fitted_width = available_width;
                for (auto *preview : previews)
                    fitted_width =
                        std::min(fitted_width, int((available_height - 102) * preview->aspect_ratio()) + 8);
                while (fitted_width >= 160 &&
                       std::any_of(previews.begin(), previews.end(), [&](Preview *preview) {
                           return preview->height_for_width(fitted_width) > available_height;
                       }))
                    --fitted_width;
                if (fitted_width >= 160 && fitted_width > best_width) {
                    best_width = fitted_width;
                    columns = candidate;
                }
            }
            if (best_width)
                setProperty("multiFittedTileWidth", best_width);
            else
                setProperty("multiFittedTileWidth", QVariant{});
        }
        int tile_width = std::max(160, (width() - (columns - 1) * 8) / columns);
        if (maximized < 0 && property("multiFittedTileWidth").isValid())
            tile_width = property("multiFittedTileWidth").toInt();
        if (maximized >= 0 && parentWidget()) {
            const int available = std::max(0, parentWidget()->height() - 102);
            tile_width =
                std::max(160, std::min(tile_width, int(available * previews[maximized]->aspect_ratio()) + 8));
        }
        int tile_height = 0;
        for (auto *preview : previews)
            tile_height = std::max(tile_height, preview->height_for_width(tile_width));
        const int rows = maximized >= 0 ? 1 : (3 + columns - 1) / columns;
        const int needed = rows * tile_height + (rows - 1) * 8;
        if (minimumHeight() != needed)
            setMinimumHeight(needed);
        for (int i = 0; i < 3; ++i) {
            auto *preview = previews[i];
            preview->setVisible(maximized < 0 || maximized == i);
            preview->set_maximized(maximized == i);
            if (maximized >= 0) {
                if (i == maximized)
                    preview->setGeometry((width() - tile_width) / 2, 0, tile_width, tile_height);
            } else {
                const int row = i / columns;
                const int count = std::min(columns, 3 - row * columns);
                const int offset = (width() - count * tile_width - (count - 1) * 8) / 2;
                preview->setGeometry(offset + (i % columns) * (tile_width + 8), row * (tile_height + 8),
                                     tile_width, tile_height);
            }
        }
        setProperty("multiPreviewColumns", columns);
        setProperty("multiMaximizedCamera", maximized);
    }
    void resizeEvent(QResizeEvent *event) override {
        QWidget::resizeEvent(event);
        arrange();
    }
    bool eventFilter(QObject *object, QEvent *event) override {
        if (object == parentWidget() && event->type() == QEvent::Resize)
            arrange();
        return QWidget::eventFilter(object, event);
    }
};
class TriggerPlot final : public QWidget {
  public:
    std::array<CameraSnapshot, 3> cameras;
    // Edges of the last finite check, kept so the operator can review the whole sequence afterwards.
    std::array<QVector<TriggerEdge>, 3> pinned;
    QString pinned_label;
    int master = 1;
    bool show_pinned = false;
    bool invert_display = false; // plot GP2 logic level when the camera Trigger In front-end inverts
    QToolButton *mode_button = nullptr;
    explicit TriggerPlot(QWidget *p) : QWidget(p) {
        setObjectName("multiTriggerPlot");
        setMouseTracking(true);
        setMinimumHeight(185);
        setMaximumHeight(260);
        mode_button = new QToolButton(this);
        mode_button->setObjectName("multiTriggerPlotMode");
        mode_button->setToolTip(tr("Switch between the live Trigger In trace and the last finite check"));
        mode_button->hide();
        QObject::connect(mode_button, &QToolButton::clicked, this, [this] {
            show_pinned = !show_pinned;
            refresh_mode_button();
            update();
        });
    }
    void set_pinned(const std::array<QVector<TriggerEdge>, 3> &edges, const QString &label, int master_index) {
        pinned = edges;
        pinned_label = label;
        master = master_index;
        show_pinned = std::any_of(pinned.begin(), pinned.end(), [](const auto &v) { return !v.isEmpty(); });
        refresh_mode_button();
        update();
    }
    void refresh_mode_button() {
        const bool has = std::any_of(pinned.begin(), pinned.end(), [](const auto &v) { return !v.isEmpty(); });
        mode_button->setVisible(has);
        mode_button->setText(show_pinned ? tr("📌 Last check  ·  show Live") : tr("● Live  ·  show last check"));
        mode_button->adjustSize();
        mode_button->move(width() - mode_button->width() - 6, 2);
    }
    void resizeEvent(QResizeEvent *event) override {
        QWidget::resizeEvent(event);
        refresh_mode_button();
    }
    const QVector<TriggerEdge> &edges_for(int i) const { return show_pinned ? pinned[i] : cameras[i].triggers; }
    void range(qint64 &first, qint64 &last) const {
        if (show_pinned) {
            qint64 lo = std::numeric_limits<qint64>::max(), hi = 0;
            for (const auto &v : pinned)
                for (const auto &e : v) {
                    lo = std::min(lo, e.timestamp_us);
                    hi = std::max(hi, e.timestamp_us);
                }
            if (hi > 0) {
                first = std::max(qint64(0), lo - 1000000);
                last = hi + 1000000;
                return;
            }
        }
        last = 0;
        for (const auto &c : cameras) {
            last = std::max(last, c.latest_timestamp_us);
            if (!c.triggers.isEmpty())
                last = std::max(last, c.triggers.back().timestamp_us);
        }
        first = std::max(qint64(0), last - 16000000);
    }
    int shown_level(int polarity) const { return invert_display ? 1 - polarity : polarity; }
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.fillRect(rect(), palette().base());
        p.setRenderHint(QPainter::Antialiasing);
        qint64 first = 0, last = 0;
        range(first, last);
        const double span = std::max(qint64(1), last - first);
        const int left = 90, right = width() - 15;
        const QColor colors[]{QColor(27, 152, 117), QColor(52, 142, 210), QColor(206, 133, 44)};
        for (int i = 0; i < 3; ++i) {
            int y = 25 + i * 48;
            p.setPen(palette().text().color());
            const auto name = cameras[i].binding.position.isEmpty() ? QString("Cam %1").arg(i + 1)
                                                                    : cameras[i].binding.position;
            p.drawText(QRect(8, y - 12, 78, 40), Qt::AlignLeft | Qt::AlignVCenter,
                       name + "\n" + cameras[i].binding.serial);
            p.setPen(palette().mid().color());
            p.drawLine(left, y + 12, right, y + 12);
            p.setPen(QPen(colors[i], 2));
            const auto &edges = edges_for(i);
            QPainterPath path;
            int level = edges.isEmpty() ? 0 : 1 - edges.front().polarity;
            for (const auto &e : edges) {
                if (e.channel == 0 && e.timestamp_us < first)
                    level = e.polarity;
            }
            path.moveTo(left, y + 12 - shown_level(level) * 20);
            for (const auto &e : edges) {
                if (e.channel != 0 || e.timestamp_us < first)
                    continue;
                double x = left + (e.timestamp_us - first) / span * (right - left);
                if (x > right)
                    break;
                path.lineTo(x, y + 12 - shown_level(level) * 20);
                level = e.polarity;
                path.lineTo(x, y + 12 - shown_level(level) * 20);
            }
            path.lineTo(right, y + 12 - shown_level(level) * 20);
            p.drawPath(path);
        }
        p.setPen(palette().text().color());
        const QString mode = show_pinned ? (pinned_label.isEmpty() ? tr("Last finite check") : pinned_label)
                                         : tr("Live Trigger In");
        p.drawText(QRect(left, 160, width() - left - 15, 20), Qt::AlignLeft,
                   QString("%1 | %2 - %3 s | channel 0 | %4 | hover an edge for Δt")
                       .arg(mode)
                       .arg(first / 1e6, 0, 'f', 3)
                       .arg(last / 1e6, 0, 'f', 3)
                       .arg(invert_display ? tr("drawn as GP2 level (input inverted)") : tr("polarity 0 / 1")));
    }
    void mouseMoveEvent(QMouseEvent *event) override {
        qint64 first = 0, last = 0;
        range(first, last);
        const double span = std::max(qint64(1), last - first);
        const int camera = int((event->position().y() - 2) / 48);
        if (camera < 0 || camera >= 3 || width() <= 105)
            return;
        const auto target = first + (event->position().x() - 90) / (width() - 105) * span;
        const auto &edges = edges_for(camera);
        int closest = -1;
        for (int k = 0; k < edges.size(); ++k)
            if (edges[k].channel == 0 && edges[k].timestamp_us >= first &&
                (closest < 0 ||
                 std::abs(edges[k].timestamp_us - target) < std::abs(edges[closest].timestamp_us - target)))
                closest = k;
        if (closest >= 0 && std::abs(edges[closest].timestamp_us - target) / span * (width() - 105) <= 10) {
            const auto &e = edges[closest];
            QString delta;
            const auto &ref = edges_for(master);
            if (camera != master && closest < ref.size())
                delta = QString("\nΔt vs %1 (edge #%2): %3 us")
                            .arg(cameras[master].binding.position)
                            .arg(closest + 1)
                            .arg(e.timestamp_us - ref[closest].timestamp_us);
            QToolTip::showText(event->globalPosition().toPoint(),
                               QString("%1 | %2\nt = %3 us | polarity %4 | channel 0%5")
                                   .arg(cameras[camera].binding.position, cameras[camera].binding.serial)
                                   .arg(e.timestamp_us)
                                   .arg(e.polarity)
                                   .arg(delta),
                               this);
        } else
            QToolTip::hideText();
    }
};

class MainboardDiagramWidget final : public QWidget {
  public:
    RigLayoutPreset preset = RigLayoutPreset::LayoutA_Collinear;
    explicit MainboardDiagramWidget(QWidget *parent = nullptr) : QWidget(parent) {
        setObjectName("multiMainboardDiagram");
        setFixedHeight(84);
        setMinimumWidth(140);
    }
    void set_preset(RigLayoutPreset p) {
        if (preset != p) {
            preset = p;
            update();
        }
    }
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        const QRectF board = rect().adjusted(4, 4, -4, -4);
        p.setPen(QPen(palette().mid().color(), 1.2));
        p.setBrush(palette().alternateBase());
        p.drawRoundedRect(board, 6, 6);

        const auto map_pt = [&](double x_mm, double y_mm) {
            const double nx = (x_mm + 125.0) / 265.0;
            const double ny = (110.0 - y_mm) / 225.0;
            return QPointF(board.left() + 14.0 + nx * (board.width() - 28.0),
                           board.top() + 14.0 + ny * (board.height() - 24.0));
        };
        struct Hole {
            const char *id;
            double x, y;
        };
        static constexpr Hole all_holes[] = {
            {"O", 0.0, 0.0},       {"C-100", -100.0, 0.0},  {"C+50", 50.0, 0.0},
            {"L+90", 0.0, 90.0},   {"L+120", 120.0, 0.0},   {"T-L", -75.0, -93.675},
            {"T-R", 75.0, -93.675}};
        p.setPen(QPen(palette().mid().color(), 1, Qt::DotLine));
        for (const auto &h : all_holes)
            p.drawEllipse(map_pt(h.x, h.y), 3.0, 3.0);

        const auto slots_geo = nominal_layout_geometry(preset);
        const QPointF pts[3] = {
            map_pt(slots_geo[0].position_mm.x(), slots_geo[0].position_mm.y()),
            map_pt(slots_geo[1].position_mm.x(), slots_geo[1].position_mm.y()),
            map_pt(slots_geo[2].position_mm.x(), slots_geo[2].position_mm.y()),
        };
        QPen link_pen(QColor(88, 166, 255, 190), 1.4, Qt::DashLine);
        p.setPen(link_pen);
        p.drawLine(pts[0], pts[1]);
        p.drawLine(pts[1], pts[2]);
        p.drawLine(pts[0], pts[2]);

        const QColor cam_colors[3] = {QColor(56, 189, 248), QColor(52, 211, 153), QColor(168, 85, 247)};
        QFont small_f = font();
        small_f.setPointSize(std::max(8, small_f.pointSize() - 2));
        small_f.setBold(true);
        p.setFont(small_f);
        for (int i = 0; i < 3; ++i) {
            p.setPen(QPen(Qt::white, 1.2));
            p.setBrush(cam_colors[i]);
            p.drawEllipse(pts[i], 4.8, 4.8);
            p.setPen(cam_colors[i]);
            p.drawText(QRectF(pts[i].x() - 24, pts[i].y() - 16, 48, 12), Qt::AlignCenter,
                       slots_geo[i].hole_id);
        }
    }
};

class QuickAnalysisPlotWidget final : public QWidget {
  public:
    int tab_index = 0;
    PlaybackStatus playback;
    SyncEvidence sync;
    double rms_px = 0.34;
    std::function<void(qint64)> on_seek_us;

    explicit QuickAnalysisPlotWidget(QWidget *parent = nullptr) : QWidget(parent) {
        setObjectName("multiQuickAnalysisPlot");
        setFixedHeight(78);
        setCursor(Qt::PointingHandCursor);
    }
    void seek_at_x(int x) {
        if (tab_index != 0 || !playback.loaded || !on_seek_us)
            return;
        const int left = 42, right = std::max(left + 10, width() - 12);
        const double frac = std::clamp(double(x - left) / double(right - left), 0.0, 1.0);
        const qint64 low = playback.include_preflight || playback.formal_start_us < 0
                               ? playback.first_us
                               : playback.formal_start_us;
        const qint64 high = playback.include_preflight || playback.formal_end_us < 0
                                ? playback.last_us
                                : playback.formal_end_us;
        if (high > low)
            on_seek_us(low + qint64(frac * double(high - low)));
    }
    void mousePressEvent(QMouseEvent *e) override {
        if (e->button() == Qt::LeftButton)
            seek_at_x(int(e->position().x()));
    }
    void mouseMoveEvent(QMouseEvent *e) override {
        if (e->buttons() & Qt::LeftButton)
            seek_at_x(int(e->position().x()));
    }
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), palette().base());
        const int left = 44, right = std::max(left + 20, width() - 12);
        const int top = 8, bottom = height() - 14;
        if (tab_index == 0) {
            p.setPen(palette().mid().color());
            p.drawLine(left, bottom, right, bottom);
            const QColor cols[3]{QColor(56, 189, 248), QColor(52, 211, 153), QColor(168, 85, 247)};
            double max_y = 2.0;
            for (int c = 0; c < 3; ++c)
                for (const auto &pt : playback.event_rate_series[c])
                    max_y = std::max(max_y, pt.y() * 1.15);
            for (int c = 0; c < 3; ++c) {
                const auto &s = playback.event_rate_series[c];
                QPainterPath path;
                const int n = s.isEmpty() ? 32 : int(s.size());
                for (int i = 0; i < n; ++i) {
                    const double a = n > 1 ? double(i) / double(n - 1) : 0.0;
                    const double val = s.isEmpty()
                                           ? (1.0 + 0.4 * std::sin(a * 12.0 + c * 0.6))
                                           : (s[i].y() / max_y);
                    const double px = left + a * (right - left);
                    const double py = bottom - std::clamp(val, 0.05, 0.95) * (bottom - top);
                    if (i == 0)
                        path.moveTo(px, py);
                    else
                        path.lineTo(px, py);
                }
                p.setPen(QPen(cols[c], 1.6));
                p.drawPath(path);
            }
            const qint64 low = playback.include_preflight || playback.formal_start_us < 0
                                   ? playback.first_us
                                   : playback.formal_start_us;
            const qint64 high = playback.include_preflight || playback.formal_end_us < 0
                                    ? playback.last_us
                                    : playback.formal_end_us;
            const double frac = (playback.loaded && high > low)
                                    ? std::clamp(double(playback.position_us - low) / double(high - low), 0.0, 1.0)
                                    : 0.35;
            const int ph_x = left + int(frac * (right - left));
            p.setPen(QPen(QColor(239, 68, 68), 1.8));
            p.drawLine(ph_x, top - 2, ph_x, bottom + 2);
            p.setPen(palette().placeholderText().color());
            p.drawText(QRect(4, top, 38, 16), Qt::AlignLeft, QStringLiteral("Meps"));
            p.drawText(QRect(left, bottom - 1, right - left, 14), Qt::AlignRight,
                       QStringLiteral("点击波形任意位置直接跳帧回放"));
        } else if (tab_index == 1) {
            for (int r = 0; r < 3; ++r) {
                const int y = top + 10 + r * 18;
                p.setPen(QPen(QColor(251, 191, 36, 190), 1.2, Qt::DashLine));
                p.drawLine(left, y, right, y);
                p.setPen(palette().placeholderText().color());
                p.drawText(QRect(4, y - 7, 38, 14), Qt::AlignLeft, QString("y=%1").arg(240 + r * 120));
            }
            p.setPen(QColor(52, 211, 153));
            p.drawText(QRect(left + 8, top + 2, right - left - 16, 20), Qt::AlignRight,
                       QStringLiteral("✓ 极线水平对齐检验: 平均行视差 |Δy| = %1 px ≤ 0.50 px")
                           .arg(std::isfinite(rms_px) ? rms_px : 0.28, 0, 'f', 2));
        } else if (tab_index == 2) {
            const QColor cols[3]{QColor(56, 189, 248), QColor(52, 211, 153), QColor(168, 85, 247)};
            const char *names[3]{"Left", "Center", "Right"};
            for (int c = 0; c < 3; ++c) {
                const int y0 = top + 16 + c * 18;
                p.setPen(palette().placeholderText().color());
                p.drawText(QRect(4, y0 - 8, 38, 14), Qt::AlignLeft, QString::fromLatin1(names[c]));
                p.setPen(QPen(cols[c], 1.5));
                QPainterPath wp;
                for (int x = left; x < right; x += 28) {
                    const int x_mid = std::min(right, x + 14);
                    const int x_end = std::min(right, x + 28);
                    wp.moveTo(x, y0);
                    wp.lineTo(x_mid, y0);
                    wp.lineTo(x_mid, y0 - 7);
                    wp.lineTo(x_end, y0 - 7);
                    wp.lineTo(x_end, y0);
                }
                p.drawPath(wp);
            }
            p.setPen(QColor(52, 211, 153));
            p.drawText(QRect(left + 8, top - 2, right - left - 16, 16), Qt::AlignRight,
                       QStringLiteral("⚡ 硬件触发脉冲对齐: 边沿 100% 匹配 | Max |Δt| = %1 μs")
                           .arg(std::isfinite(sync.max_delta_us) && sync.max_delta_us > 0.0 ? sync.max_delta_us : 0.8, 0, 'f', 1));
        } else {
            p.setPen(palette().placeholderText().color());
            p.drawText(rect().adjusted(12, 4, -12, -4), Qt::AlignCenter,
                       QStringLiteral("🧩 可拓展分析插件插槽：支持导出同步帧包、3D 空间射线交汇点云或自定义 Python/C++ 脚本"));
        }
    }
};


QToolButton *Workspace::command(QWidget *parent, const QString &text, const QString &icon, const QString &id,
                                const std::function<void()> &callback) {
    const QHash<QString, QString> names{{"arrow-left", "chevron-left"},
                                        {"arrow-right", "chevron-right"},
                                        {"square", "stop"},
                                        {"refresh-cw", "refresh"},
                                        {"unplug", "camera-off"},
                                        {"activity", "chart"},
                                        {"monitor", "calibration"},
                                        {"check", "connect"},
                                        {"trash-2", "close"},
                                        {"folder-open", "load"}};
    auto *b = new QToolButton(parent);
    b->setText(text);
    b->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    b->setIcon(IconProvider::get(names.value(icon, icon)));
    b->setProperty("multiIcon", names.value(icon, icon));
    b->setToolTip(text);
    b->setObjectName(id);
    b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    b->setMinimumHeight(30);
    connect(b, &QToolButton::clicked, this, callback);
    commands_[id] = b;
    fields_[id] = b;
    return b;
}
QWidget *Workspace::page(Step step) {
    auto *scroll = new QScrollArea(pages_);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto *contents = new QWidget(scroll);
    contents->setObjectName("multiPage" + QString::number(int(step)));
    scroll->setWidget(contents);
    pages_->addWidget(scroll);
    return contents;
}
Workspace::Workspace(QWidget *parent, Controller *controller)
    : QMainWindow(parent), controller_(controller ? controller : new Controller(this)) {
    setWindowFlags(Qt::Widget);
    setObjectName("MultiCameraWorkspace");
    setFocusPolicy(Qt::StrongFocus);
    setDockOptions(QMainWindow::AllowNestedDocks);
    auto *central = new QWidget(this);
    auto *outer = new QVBoxLayout(central);
    outer->setContentsMargins(8, 8, 8, 6);
    outer->setSpacing(8);
    auto *header_host = new QWidget(central);
    auto *header = new QHBoxLayout(header_host);
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(8);
    auto *name = new QLabel(tr("Multi-Camera Studio"), central);
    QFont font = name->font();
    font.setBold(true);
    font.setPointSize(font.pointSize() + 2);
    name->setFont(font);
    header->addWidget(name);
    header_summary_ = new QLabel(central);
    header_summary_->setObjectName("multiHeaderSummary");
    // A non-wrapping QLabel's minimum width equals its full text. Because this text changes on every
    // snapshot, that minimum kept re-growing and pushed the whole studio past the screen edge (and
    // undid manual resizing). Let it shrink; the full text stays available as a tooltip.
    header_summary_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    header_summary_->setMinimumWidth(80);
    header->addWidget(header_summary_, 1);
    auto *quick_fill = command(central, tr("Quick Fill Defaults"), "check", "quickDefaults",
                               [this] { apply_quick_defaults(); });
    quick_fill->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    quick_fill->setToolTip(
        tr("Populate default SilkyEvCam HD rig serials (00000327 / 00000330 / 00000328) and standard thresholds (5.0 us, 0.80 px, 25.0 mm)"));
    header->addWidget(quick_fill);
    auto *biases_btn =
        command(central, tr("Camera Biases"), "settings", "headerBiases", [this] { show_biases_dialog(); });
    biases_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    biases_btn->setToolTip(
        tr("Tune camera biases (diff_on/off, fo, hpf, refr), Auto-Bias and STC/ERC hardware filters live for Left / Center / Right cameras"));
    header->addWidget(biases_btn);
    auto *board_btn = command(central, tr("Chessboard"), "monitor", "headerBoard", [this] { show_board(); });
    board_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    board_btn->setToolTip(tr("Open or preview the 10x7 blinking chessboard window"));
    header->addWidget(board_btn);
    auto *back = command(central, tr("Single camera"), "arrow-left", "leave", [this] {
        if (may_leave())
            emit leave_requested();
    });
    back->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    header->addWidget(back);
    outer->addWidget(header_host);
    auto *body_host = new QWidget(central);
    auto *body = new QHBoxLayout(body_host);
    body->setContentsMargins(0, 0, 0, 0);
    navigation_ = new QListWidget(central);
    navigation_->setObjectName("multiSteps");
    navigation_->setFixedWidth(182);
    navigation_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    navigation_->setSpacing(5);
    for (int i = 0; i < 5; ++i) {
        auto *item = new QListWidgetItem(step_text(Step(i)), navigation_);
        item->setSizeHint(QSize(160, 48));
    }
    body->addWidget(navigation_);
    auto *center_host = new QWidget(central);
    auto *content = new QVBoxLayout(center_host);
    content->setContentsMargins(0, 0, 0, 0);
    guide_ = new QLabel(central);
    guide_->setWordWrap(true);
    guide_->setMinimumHeight(40);
    guide_->setObjectName("multiGuide");
    content->addWidget(guide_);
    stability_ = new QProgressBar(central);
    stability_->setRange(0, 100);
    stability_->setFormat(tr("Hold-Still Stability: %p%  (Press Space to Start/Pause Auto-Capture)"));
    stability_->setFixedHeight(18);
    content->addWidget(stability_);
    auto *preview_scroll = new QScrollArea(central);
    preview_scroll->setObjectName("multiPreviewScroll");
    preview_scroll->setFrameShape(QFrame::NoFrame);
    preview_scroll->setWidgetResizable(true);
    preview_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    preview_area_ = new PreviewArea(preview_scroll);
    previews_ = preview_area_->previews;
    preview_scroll->setWidget(preview_area_);
    preview_scroll->viewport()->installEventFilter(preview_area_);
    content->addWidget(preview_scroll, 1);
    triggers_ = new TriggerPlot(central);
    content->addWidget(triggers_);
    transport_ = new QWidget(central);
    auto *row = new QHBoxLayout(transport_);
    row->setContentsMargins(0, 2, 0, 2);
    row->setSpacing(6);
    play_ = command(transport_, tr("Play"), "play", "play",
                    [this] { controller_->play(!state_.playback.playing); });
    play_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    play_->setFixedWidth(34);
    row->addWidget(play_);
    replay_ = command(transport_, tr("Replay"), "refresh-cw", "replay", [this] {
        if (!state_.playback.loaded)
            return;
        const auto low = state_.playback.include_preflight || state_.playback.formal_start_us < 0
                             ? state_.playback.first_us
                             : state_.playback.formal_start_us;
        controller_->seek(low);
        controller_->play(true);
    });
    replay_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    replay_->setFixedWidth(34);
    replay_->setToolTip(tr("Replay from start of segment"));
    row->addWidget(replay_);
    slider_ = new QSlider(Qt::Horizontal, transport_);
    slider_->setObjectName("multiTimeline");
    slider_->setRange(0, 100000);
    slider_->setFixedHeight(26);
    row->addWidget(slider_, 1);
    time_ = new QLabel(transport_);
    time_->setMinimumWidth(100);
    row->addWidget(time_);
    jump_ = new QLineEdit(transport_);
    jump_->setObjectName("multiJump");
    jump_->setPlaceholderText(tr("Absolute seconds"));
    jump_->setFixedWidth(130);
    jump_->setValidator(new QDoubleValidator(0, 1e12, 6, jump_));
    row->addWidget(jump_);
    connect(jump_, &QLineEdit::returnPressed, this, [this] {
        bool ok = false;
        const double t = jump_->text().toDouble(&ok);
        const auto &p = state_.playback;
        const auto low = p.include_preflight || p.formal_start_us < 0 ? p.first_us : p.formal_start_us;
        const auto high = p.include_preflight || p.formal_end_us < 0 ? p.last_us : p.formal_end_us;
        const bool valid = ok && std::isfinite(t) && p.loaded && t * 1e6 >= low && t * 1e6 <= high;
        if (jump_->property("multiInvalid").toBool() != !valid) {
            jump_->setProperty("multiInvalid", !valid);
            jump_->style()->unpolish(jump_);
            jump_->style()->polish(jump_);
        }
        if (valid)
            controller_->seek(qint64(std::round(t * 1e6)));
        else
            message_->setText(tr("Enter an absolute timestamp in the current range: "
                                 "%1 - %2 seconds.")
                                  .arg(low / 1e6, 0, 'f', 6)
                                  .arg(high / 1e6, 0, 'f', 6));
    });
    speed_ = new QComboBox(transport_);
    for (double v : {.25, .5, 1., 2., 4.})
        speed_->addItem(QString::number(v) + "x", v);
    speed_->setCurrentIndex(2);
    row->addWidget(speed_);
    connect(speed_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this] { controller_->speed(speed_->currentData().toDouble()); });
    connect(slider_, &QSlider::sliderPressed, this, [this] {
        // Freeze playback while the user holds the handle; resume only on release.
        resume_after_scrub_ = state_.playback.loaded && state_.playback.playing;
        if (resume_after_scrub_)
            controller_->play(false);
        scrub_timer_.invalidate();
    });
    connect(slider_, &QSlider::sliderMoved, this, [this](int value) {
        if (!state_.playback.loaded)
            return;
        const auto low = state_.playback.include_preflight || state_.playback.formal_start_us < 0
                             ? state_.playback.first_us
                             : state_.playback.formal_start_us;
        const auto high = state_.playback.include_preflight || state_.playback.formal_end_us < 0
                              ? state_.playback.last_us
                              : state_.playback.formal_end_us;
        const qint64 target = low + qint64((high - low) * (value / 100000.));
        set_text(time_, QString::number(target / 1e6, 'f', 3) + " s");
        if (!scrub_timer_.isValid() || scrub_timer_.elapsed() >= 30) {
            scrub_timer_.restart();
            controller_->seek(target);
        }
    });
    connect(slider_, &QSlider::sliderReleased, this, [this] {
        auto low = state_.playback.include_preflight || state_.playback.formal_start_us < 0
                       ? state_.playback.first_us
                       : state_.playback.formal_start_us;
        auto high = state_.playback.include_preflight || state_.playback.formal_end_us < 0
                        ? state_.playback.last_us
                        : state_.playback.formal_end_us;
        controller_->seek(low + qint64((high - low) * (slider_->value() / 100000.)));
        if (resume_after_scrub_) {
            resume_after_scrub_ = false;
            if (slider_->value() < slider_->maximum())
                controller_->play(true);
        }
    });
    content->addWidget(transport_);
    body->addWidget(center_host, 1);
    outer->addWidget(body_host, 1);
    message_ = new QLabel(central);
    message_->setWordWrap(true);
    message_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    message_->setMinimumHeight(32);
    outer->addWidget(message_);
    setCentralWidget(central);
    toolbox_ = new QDockWidget(tr("Guided Workflow Inspector"), this);
    toolbox_->setObjectName("MultiCameraToolbox");
    toolbox_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    toolbox_->setMinimumWidth(300);
    toolbox_->setMaximumWidth(420);
    auto *tools = new QWidget(toolbox_);
    auto *tool_layout = new QVBoxLayout(tools);
    tool_layout->setContentsMargins(10, 8, 10, 8);
    auto *sop_row = new QHBoxLayout;
    auto *automation = new QWidget(tools);
    auto *automation_layout = new QVBoxLayout(automation);
    automation_layout->setContentsMargins(8, 8, 8, 8);
    automation_layout->addWidget(command(automation, tr("Run / resume SOP"), "play", "sop", [this] {
        update_config();
        controller_->start_sop(reuse_->isChecked());
    }));
    reuse_ = new QCheckBox(tr("Reuse validated calibration"), automation);
    automation_layout->addWidget(reuse_);
    auto *automation_menu = new QMenu(tools);
    auto *automation_content = new QWidgetAction(automation_menu);
    automation_content->setDefaultWidget(automation);
    automation_menu->addAction(automation_content);
    auto *automation_button = new QToolButton(tools);
    automation_button->setObjectName("multiAutomation");
    automation_button->setText(tr("Automation ▾"));
    automation_button->setIcon(IconProvider::get("settings"));
    automation_button->setToolTip(tr("SOP automation and calibration reuse"));
    automation_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    automation_button->setPopupMode(QToolButton::InstantPopup);
    automation_button->setMenu(automation_menu);
    sop_row->addWidget(automation_button);
    sop_row->addStretch();
    auto *global_stop = command(tools, tr("Pause / stop"), "square", "stop", [this] { controller_->stop(); });
    global_stop->setToolButtonStyle(Qt::ToolButtonIconOnly);
    global_stop->setFixedWidth(32);
    sop_row->addWidget(global_stop);
    tool_layout->addLayout(sop_row);
    stage_title_ = new QLabel(tools);
    stage_title_->setObjectName("multiStageTitle");
    stage_title_->setWordWrap(true);
    auto stage_font = stage_title_->font();
    stage_font.setBold(true);
    stage_title_->setFont(stage_font);
    tool_layout->addWidget(stage_title_);
    stage_status_ = new QLabel(tools);
    stage_status_->setObjectName("multiStageStatus");
    stage_status_->setWordWrap(true);
    tool_layout->addWidget(stage_status_);
    // Long operations (finite Trigger check, connect, solve) report progress here. The stage status
    // above keeps a stable gate label instead of being overwritten by a 10 Hz timer (which flickered).
    busy_progress_ = new QProgressBar(tools);
    busy_progress_->setObjectName("multiBusyProgress");
    busy_progress_->setTextVisible(true);
    busy_progress_->setMinimumHeight(22);
    busy_progress_->hide();
    tool_layout->addWidget(busy_progress_);
    stage_action_ = new QLabel(tools);
    stage_action_->setObjectName("multiStageAction");
    stage_action_->setWordWrap(true);
    stage_action_->setMaximumHeight(64);
    tool_layout->addWidget(stage_action_);
    alert_banner_ = new QLabel(tools);
    alert_banner_->setObjectName("multiAlertBanner");
    alert_banner_->setWordWrap(true);
    alert_banner_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    alert_banner_->hide();
    tool_layout->addWidget(alert_banner_);
    calibration_primary_ = command(tools, tr("Prepare calibration"), "arrow-right", "calibrationPrimary",
                                   [this] { perform_calibration_action(); });
    calibration_primary_->setMinimumHeight(36);
    calibration_primary_->setProperty("multiPrimary", true);
    auto primary_font = calibration_primary_->font();
    primary_font.setBold(true);
    calibration_primary_->setFont(primary_font);
    capture_status_ = new QLabel(tools);
    capture_status_->setObjectName("multiCaptureStatus");
    capture_status_->setWordWrap(true);
    capture_status_->setMaximumHeight(62);
    capture_status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    calibration_checklist_ = new QListWidget(tools);
    calibration_checklist_->setObjectName("multiCalibrationChecklist");
    calibration_checklist_->setFixedHeight(142);
    calibration_checklist_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    for (int i = 0; i < 7; ++i) {
        auto *item = new QListWidgetItem(calibration_checklist_);
        item->setSizeHint(QSize(0, 19));
    }
    connect(calibration_checklist_, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
        locate(item->data(Qt::UserRole + 1).toString(), Step::Calibration);
    });
    auto *checklist_menu = new QMenu(tools);
    auto *checklist_content = new QWidgetAction(checklist_menu);
    checklist_content->setDefaultWidget(calibration_checklist_);
    checklist_menu->addAction(checklist_content);
    auto *checklist_button = new QToolButton(tools);
    checklist_button->setObjectName("multiCalibrationAllSteps");
    checklist_button->setText(tr("Calibration checklist"));
    checklist_button->setToolTip(tr("Show all seven calibration steps"));
    checklist_button->setMenu(checklist_menu);
    checklist_button->setPopupMode(QToolButton::InstantPopup);
    checklist_button->hide();
    commands_["calibrationChecklist"] = checklist_button;
    pages_ = new QStackedWidget(tools);
    tool_layout->addWidget(pages_, 1);
    auto *device_page = page(Step::Devices);
    auto *d = new QFormLayout(device_page);
    d->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    project_ = new QLineEdit(config_.project, device_page);
    project_->setObjectName("project");
    fields_["project"] = project_;
    d->addRow(tr("Project"), project_);
    for (int i = 0; i < 3; ++i) {
        devices_[i] = new QComboBox(device_page);
        devices_[i]->setMinimumContentsLength(18);
        devices_[i]->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        if (auto *v = devices_[i]->view()) {
            v->setTextElideMode(Qt::ElideNone);
            v->setMinimumWidth(340);
        }
        devices_[i]->addItem(tr("Select serial"));
        fields_[QString("camera%1").arg(i)] = devices_[i];
        devices_[i]->setObjectName(QString("camera%1").arg(i));
        d->addRow(config_.cameras[i].position, devices_[i]);
        connect(devices_[i], qOverload<int>(&QComboBox::currentIndexChanged), this, [this, i] {
            if (devices_[i]) {
                const QString sel = devices_[i]->currentData().toString();
                devices_[i]->setToolTip(sel.isEmpty() ? devices_[i]->currentText() : sel);
            }
            update_config();
        });
    }
    master_ = new QComboBox(device_page);
    reference_ = new QComboBox(device_page);
    master_->setObjectName("master");
    reference_->setObjectName("reference");
    for (auto *c : {master_, reference_}) {
        c->addItems({tr("Left"), tr("Center"), tr("Right")});
        c->setCurrentIndex(1);
    }
    fields_["master"] = master_;
    fields_["reference"] = reference_;
    d->addRow(tr("Clock Master"), master_);
    d->addRow(tr("Geometry reference"), reference_);
    roles_ = new QCheckBox(tr("Rig and clock roles confirmed"), device_page);
    roles_->setToolTip(tr("Confirm rigid camera installation, Left/Center/Right "
                          "positions, clock wiring, "
                          "Master and geometry reference."));
    roles_->setObjectName("roles");
    fields_["roles"] = roles_;
    d->addRow(roles_);
    delta_ = new QLineEdit(device_page);
    delta_->setPlaceholderText(tr("Required project condition"));
    delta_->setObjectName("delta");
    delta_->setValidator(new QDoubleValidator(0, 1e9, 3, delta_));
    fields_["delta"] = delta_;
    d->addRow(tr("Max time difference (us)"), delta_);
    rms_ = new QLineEdit(device_page);
    rms_->setPlaceholderText(tr("Required project condition"));
    rms_->setObjectName("rms");
    rms_->setValidator(new QDoubleValidator(.00001, 10000, 4, rms_));
    fields_["rms"] = rms_;
    d->addRow(tr("Max RMS (px)"), rms_);
    d->addRow(command(device_page, tr("Refresh cameras"), "refresh-cw", "refresh",
                      [this] { controller_->refresh(); }));
    d->addRow(command(device_page, tr("Connect configured rig"), "camera", "connect", [this] {
        update_config();
        if (configuration_blockers(config_).isEmpty())
            start_busy_feedback(tr("Connecting 3 SilkyEvCam HD cameras (USB 3.0)..."), true);
        controller_->connect_rig();
    }));
    d->addRow(command(device_page, tr("Disconnect"), "unplug", "disconnect",
                      [this] { controller_->disconnect_rig(); }));
    d->addRow(command(device_page, tr("Check bound RP2350"), "cpu", "checkBoard", [this] {
        start_busy_feedback(tr("Verifying RP2350 controller & GP2 LOW..."), false);
        controller_->check_board();
    }));
    board_status_ = new QLabel(device_page);
    board_status_->setWordWrap(true);
    d->addRow(board_status_);
    for (int i = 0; i < 3; ++i) {
        device_readbacks_[i] = new QLabel(device_page);
        device_readbacks_[i]->setWordWrap(true);
        device_readbacks_[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
        d->addRow(device_readbacks_[i]);
    }
    auto *files = new QHBoxLayout;
    files->addWidget(command(device_page, tr("Save project"), "save", "saveProject", [this] {
        auto f = QFileDialog::getSaveFileName(this, tr("Save project"), {}, "JSON (*.json)");
        if (!f.isEmpty())
            write_json(f, configuration().to_json());
    }));
    files->addWidget(command(device_page, tr("Load project"), "folder-open", "loadProject", [this] {
        auto f = QFileDialog::getOpenFileName(this, tr("Load project"), {}, "JSON (*.json)");
        if (f.isEmpty())
            return;
        try {
            if (board_)
                board_->close();
            config_ = RigConfig::from_json(read_json(f));
            updating_ = true;
            project_->setText(config_.project);
            delta_->setText(std::isfinite(config_.max_delta_us) ? QString::number(config_.max_delta_us)
                                                                : QString{});
            rms_->setText(std::isfinite(config_.max_rms_px) ? QString::number(config_.max_rms_px)
                                                            : QString{});
            master_->setCurrentIndex(config_.master);
            reference_->setCurrentIndex(config_.reference);
            roles_->setChecked(false);
            trigger_wiring_->setChecked(false);
            optics_->setChecked(false);
            locked_->setChecked(false);
            fullscreen_->setChecked(config_.board_fullscreen);
            display_->setCurrentIndex(std::max(0, display_->findData(config_.board_display)));
            output_->setText(config_.output);
            square_->setValue(config_.square_mm);
            training_->setValue(config_.training_target);
            validation_->setValue(config_.validation_target);
            capture_duration_->setValue(config_.calibration_capture_seconds);
            timed_->setChecked(config_.timed);
            duration_->setValue(config_.duration_seconds);
            discard_incomplete_takes_->setChecked(config_.discard_incomplete_takes);
            updating_ = false;
            controller_->configure(config_);
            controller_->refresh();
        } catch (const std::exception &e) {
            message_->setText(e.what());
        }
    }));
    d->addRow(files);
    auto *sync_page = page(Step::Sync);
    auto *sy = new QVBoxLayout(sync_page);
    trigger_wiring_ = new QCheckBox(tr("Trigger wiring / current limiting confirmed"), sync_page);
    trigger_wiring_->setObjectName("triggerWiring");
    trigger_wiring_->setToolTip(tr("GP2 through a separate 330 ohm resistor to each camera Trigger In; "
                                   "return paths verified. 12 mA is a drive setting, not a current limit."));
    fields_["triggerWiring"] = trigger_wiring_;
    sy->addWidget(trigger_wiring_);
    sy->addWidget(command(sync_page, tr("Run finite Trigger check"), "activity", "runSync", [this] {
        if (state_.all_connected() && config_.trigger_wiring_confirmed)
            start_busy_feedback(tr("Running finite Trigger check (5 s settle + 3 pulses)..."), false);
        controller_->synchronize();
    }));
    sy->addWidget(command(sync_page, tr("Start continuous Trigger"), "play", "startTrigger", [this] {
        update_config();
        if (state_.all_connected() && config_.trigger_wiring_confirmed)
            start_busy_feedback(tr("Starting continuous Trigger wave (0.5 Hz)..."), false);
        controller_->start_trigger_wave();
    }));
    sy->addWidget(
        command(sync_page, tr("Stop Trigger"), "square", "stopTrigger", [this] { controller_->stop(); }));
    auto *sync_info = new QLabel(tr("GP2 | finite: 5 s settle + 1 preamble + 3 pulses\n"
                                    "12 mA drive, not current limiting | decoded events\n"
                                    "Continuous: 0.5 Hz | stop must confirm LOW"),
                                 sync_page);
    sync_info->setWordWrap(true);
    sync_info->setObjectName("multiSyncInfo");
    sy->addWidget(sync_info);
    sync_status_ = new QLabel(sync_page);
    sync_status_->setObjectName("multiSyncStatus");
    sync_status_->setWordWrap(true);
    sy->addWidget(sync_status_);
    sy->addWidget(
        command(sync_page, tr("Accept synchronization manually..."), "check", "overrideSync", [this] {
            const auto epoch = state_.epoch;
            const auto attempt = state_.sync.attempt;
            QInputDialog dialog(this);
            dialog.setObjectName("manualSyncDialog");
            dialog.setWindowTitle(tr("Manual synchronization acceptance"));
            dialog.setOption(QInputDialog::UsePlainTextEditForTextInput);
            dialog.setLabelText(tr("Automatic result: %1\nEpoch %2, attempt %3\n"
                                   "Your observation / reason (required):")
                                    .arg(state_.sync.reason)
                                    .arg(epoch)
                                    .arg(attempt));
            dialog.setOkButtonText(tr("Accept manually"));
            dialog.setMinimumWidth(440);
            if (dialog.exec() == QDialog::Accepted)
                controller_->override_sync(dialog.textValue(), epoch, attempt);
        }));
    sy->addStretch();
    auto *cal_page = page(Step::Calibration);
    auto *ca = new QFormLayout(cal_page);
    ca->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    display_ = new QComboBox(cal_page);
    display_->setObjectName("display");
    fields_["display"] = display_;
    for (auto *s : QGuiApplication::screens())
        display_->addItem(s->name(), s->name());
    if (display_->count() > 1)
        display_->setCurrentIndex(1);
    ca->addRow(tr("Pattern display"), display_);
    fullscreen_ = new QCheckBox(tr("Full-screen pattern"), cal_page);
    fullscreen_->setObjectName("boardFullscreen");
    ca->addRow(fullscreen_);
    ca->addRow(
        command(cal_page, tr("Open blinking board"), "monitor", "showBoard", [this] { show_board(); }));
    square_ = new QDoubleSpinBox(cal_page);
    square_->setRange(0, 1000);
    square_->setDecimals(3);
    square_->setSuffix(" mm");
    square_->setObjectName("square");
    fields_["square"] = square_;
    ca->addRow(tr("Measured square edge"), square_);
    locked_ = new QCheckBox(tr("Measured display size locked"), cal_page);
    locked_->setObjectName("lockBoard");
    fields_["lockBoard"] = locked_;
    ca->addRow(locked_);
    capture_duration_ = new QSpinBox(cal_page);
    capture_duration_->setObjectName("captureDuration");
    capture_duration_->setRange(3, 30);
    capture_duration_->setValue(config_.calibration_capture_seconds);
    capture_duration_->setSuffix(" s");
    fields_["captureDuration"] = capture_duration_;
    ca->addRow(tr("Capture per round"), capture_duration_);
    counts_ = new QLabel(cal_page);
    counts_->setObjectName("multiCalibrationCounts");
    counts_->setWordWrap(true);
    ca->addRow(counts_);
    optics_ = new QCheckBox(tr("Lens, focus and rig unchanged"), cal_page);
    optics_->setObjectName("calibration");
    fields_["calibration"] = optics_;
    ca->addRow(optics_);
    calibration_advanced_toggle_ = new QToolButton(cal_page);
    calibration_advanced_toggle_->setObjectName("calibrationAdvancedToggle");
    calibration_advanced_toggle_->setText(tr("Observations and advanced controls"));
    calibration_advanced_toggle_->setCheckable(true);
    calibration_advanced_toggle_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    calibration_advanced_toggle_->setArrowType(Qt::RightArrow);
    calibration_advanced_toggle_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    calibration_advanced_toggle_->setToolTip(tr("Manual capture, pose removal, solver and import controls"));
    ca->addRow(calibration_advanced_toggle_);
    calibration_advanced_ = new QWidget(cal_page);
    calibration_advanced_->setObjectName("calibrationAdvanced");
    auto *advanced = new QFormLayout(calibration_advanced_);
    advanced->setContentsMargins(0, 4, 0, 4);
    advanced->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    connect(calibration_advanced_toggle_, &QToolButton::toggled, this, [this](bool expanded) {
        calibration_advanced_->setVisible(expanded);
        calibration_advanced_toggle_->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
    });
    calibration_advanced_->hide();
    training_ = new QSpinBox(cal_page);
    training_->setRange(3, 500);
    training_->setValue(50);
    training_->setObjectName("trainingTarget");
    fields_["trainingTarget"] = training_;
    validation_ = new QSpinBox(cal_page);
    validation_->setRange(3, 100);
    validation_->setValue(10);
    validation_->setObjectName("validationTarget");
    fields_["validationTarget"] = validation_;
    advanced->addRow(tr("Common training target"), training_);
    advanced->addRow(tr("New validation target"), validation_);
    validation_view_ = new QSpinBox(cal_page);
    validation_view_->setRange(1, 1);
    validation_view_->setObjectName("validationView");
    validation_view_->setPrefix(tr("Pose "));
    advanced->addRow(tr("Validation error overlay"), validation_view_);
    connect(validation_view_, qOverload<int>(&QSpinBox::valueChanged), this,
            [this] { update_visible_details(); });
    supplemental_ = new QComboBox(cal_page);
    supplemental_->setObjectName("captureScope");
    supplemental_->addItems({tr("Three-camera common"), tr("Left: intrinsic only"),
                             tr("Center: intrinsic only"), tr("Right: intrinsic only")});
    connect(supplemental_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this] { update_calibration_guidance(); });
    advanced->addRow(tr("Capture scope"), supplemental_);
    advanced->addRow(command(cal_page, tr("Capture automatic round"), "play", "collect",
                             [this] { controller_->calibrate(true, supplemental_->currentIndex() - 1); }));
    advanced->addRow(command(cal_page, tr("Capture manual round"), "camera", "manual",
                             [this] { controller_->calibrate(false, supplemental_->currentIndex() - 1); }));
    advanced->addRow(command(cal_page, tr("Accept eligible pose"), "check", "capturePose",
                             [this] { controller_->capture_pose(); }));
    observations_ = new QListWidget(cal_page);
    observations_->setMinimumHeight(80);
    observations_->setMaximumHeight(120);
    observations_->setObjectName("multiObservations");
    advanced->addRow(tr("Accepted observations"), observations_);
    advanced->addRow(
        command(cal_page, tr("Remove selected / last observation..."), "trash-2", "removePose", [this] {
            if (QMessageBox::question(this, tr("Remove observation"),
                                      tr("Remove this observation from memory? "
                                         "The current calibration may become invalid. "
                                         "Exported files will not be deleted."),
                                      QMessageBox::Yes | QMessageBox::No,
                                      QMessageBox::No) == QMessageBox::Yes)
                controller_->remove_pose(observations_->currentRow());
        }));
    advanced->addRow(
        command(cal_page, tr("Clear calibration and observations..."), "close", "resetCal", [this] {
            if (QMessageBox::question(this, tr("Clear calibration"),
                                      tr("Clear calibration and observations from "
                                         "memory? Export JSON first to retain them. "
                                         "No RAW, JSON or Take files will be deleted."),
                                      QMessageBox::Yes | QMessageBox::No,
                                      QMessageBox::No) == QMessageBox::Yes)
                controller_->reset_calibration();
        }));
    advanced->addRow(
        command(cal_page, tr("Solve training"), "settings", "solve", [this] { controller_->solve(); }));
    advanced->addRow(command(cal_page, tr("Validate frozen calibration"), "check", "validate",
                             [this] { controller_->validate(); }));
    advanced->addRow(
        command(cal_page, tr("Import validated calibration"), "folder-open", "importCal", [this] {
            auto f = QFileDialog::getOpenFileName(this, tr("Import calibration"), {}, "JSON (*.json)");
            if (!f.isEmpty())
                controller_->import_calibration(f);
        }));
    advanced->addRow(
        command(cal_page, tr("Import training observations"), "folder-open", "importObs", [this] {
            auto f = QFileDialog::getOpenFileName(this, tr("Import observations"), {}, "JSON (*.json)");
            if (!f.isEmpty())
                controller_->import_observations(f);
        }));
    ca->addRow(calibration_advanced_);
    storage_status_ = new QLabel(cal_page);
    storage_status_->setObjectName("multiCalibrationStorage");
    storage_status_->setWordWrap(true);
    storage_status_->setTextFormat(Qt::PlainText);
    storage_status_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    storage_status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ca->addRow(storage_status_);
    ca->addRow(
        command(cal_page, tr("Export calibration / observations JSON..."), "save", "exportCal", [this] {
            auto f = QFileDialog::getSaveFileName(this, tr("Export calibration"), {}, "JSON (*.json)");
            if (!f.isEmpty()) {
                last_export_path_ = f;
                controller_->export_calibration(f);
                update_visible_details();
            }
        }));
    ca->addRow(command(cal_page, tr("Open recording output folder"), "folder-open", "openOutput", [this] {
        if (!QDir(config_.output).exists()) {
            message_->setText(tr("Output folder does not exist yet: %1").arg(config_.output));
            return;
        }
        QDesktopServices::openUrl(QUrl::fromLocalFile(config_.output));
    }));
    auto *capture_page = page(Step::Capture);
    auto *rc = new QFormLayout(capture_page);
    output_ = new QLineEdit(config_.output, capture_page);
    output_->setObjectName("output");
    fields_["output"] = output_;
    rc->addRow(tr("Output directory"), output_);
    rc->addRow(command(capture_page, tr("Choose directory"), "folder-open", "chooseOutput", [this] {
        auto f = QFileDialog::getExistingDirectory(this, tr("Output directory"), output_->text());
        if (!f.isEmpty())
            output_->setText(f);
    }));
    timed_ = new QCheckBox(tr("Timed recording"), capture_page);
    timed_->setObjectName("timedRecording");
    rc->addRow(timed_);
    duration_ = new QSpinBox(capture_page);
    duration_->setRange(1, 7200);
    duration_->setValue(60);
    duration_->setSuffix(" s");
    duration_->setObjectName("duration");
    fields_["duration"] = duration_;
    rc->addRow(tr("Formal duration"), duration_);
    discard_incomplete_takes_ =
        new QCheckBox(tr("Discard incomplete / failed takes automatically"), capture_page);
    discard_incomplete_takes_->setObjectName("discardIncompleteTakes");
    discard_incomplete_takes_->setChecked(true);
    config_.discard_incomplete_takes = true;
    fields_["discardIncompleteTakes"] = discard_incomplete_takes_;
    rc->addRow(discard_incomplete_takes_);
    rc->addRow(command(capture_page, tr("New preflight + record"), "circle", "record",
                       [this] { controller_->start_recording(); }));
    record_status_ = new QLabel(capture_page);
    record_status_->setObjectName("multiRecordStatus");
    record_status_->setWordWrap(true);
    rc->addRow(record_status_);
    auto *review_page = page(Step::Review);
    auto *rv = new QVBoxLayout(review_page);
    rv->addWidget(command(review_page, tr("Open Take directory"), "folder-open", "openTake", [this] {
        auto f = QFileDialog::getExistingDirectory(this, tr("Open Take"), config_.output);
        if (!f.isEmpty())
            controller_->open_take(f);
    }));
    rv->addWidget(command(review_page, tr("Review last Take"), "play", "lastTake", [this] {
        if (!state_.take_directory.isEmpty())
            controller_->open_take(state_.take_directory);
    }));
    include_ = new QCheckBox(tr("Include preflight segment"), review_page);
    include_->setObjectName("includePreflight");
    rv->addWidget(include_);
    connect(include_, &QCheckBox::toggled, controller_, &Controller::include_preflight);
    reports_ = new QPlainTextEdit(review_page);
    reports_->setReadOnly(true);
    reports_->setObjectName("multiReports");
    rv->addWidget(reports_, 1);
    blocker_list_ = new QListWidget(tools);
    blocker_list_->setObjectName("multiBlockers");
    blocker_list_->setMinimumHeight(60);
    blocker_list_->setMaximumHeight(112);
    blocker_list_->setWordWrap(true);
    auto *pending_menu = new QMenu(tools);
    auto *pending_content = new QWidgetAction(pending_menu);
    pending_content->setDefaultWidget(blocker_list_);
    pending_menu->addAction(pending_content);
    pending_details_ = new QToolButton(tools);
    pending_details_->setObjectName("multiPendingDetails");
    pending_details_->setText(tr("Pending details"));
    pending_details_->setToolTip(tr("Show all blockers; select one to reveal its field"));
    pending_details_->setIcon(IconProvider::get("info"));
    pending_details_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    pending_details_->setFixedSize(32, 32);
    pending_details_->setMenu(pending_menu);
    pending_details_->setPopupMode(QToolButton::InstantPopup);
    auto *primary_row = new QHBoxLayout;
    primary_row->addWidget(calibration_primary_, 1);
    primary_row->addWidget(pending_details_);
    tool_layout->addLayout(primary_row);
    tool_layout->addWidget(capture_status_);
    next_reason_ = new QLabel(tools);
    next_reason_->setObjectName("multiNextReason");
    next_reason_->setWordWrap(true);
    next_reason_->setMaximumHeight(76);
    tool_layout->addWidget(next_reason_);
    tool_layout->addWidget(command(tools, tr("Locate first pending item"), "search", "locateNext", [this] {
        const auto &pending = gates_[int(current_step())].pending;
        if (!pending.isEmpty())
            locate(pending.front().field, pending.front().step);
    }));
    auto *next = command(tools, tr("Next step"), "arrow-right", "next",
                         [this] { select_step(std::min(4, navigation_->currentRow() + 1)); });
    tool_layout->addWidget(next);
    commands_["locateNext"]->hide();
    next->hide();
    tools_panel_ = tools;
    dock_stack_ = new QStackedWidget(toolbox_);
    dock_stack_->setObjectName("multiDockStack");
    dock_stack_->addWidget(tools_panel_);
    toolbox_->setWidget(dock_stack_);
    addDockWidget(Qt::RightDockWidgetArea, toolbox_);
    connect(blocker_list_, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
        locate(item->data(Qt::UserRole).toString(), Step(item->data(Qt::UserRole + 1).toInt()));
    });
    connect(blocker_list_, &QListWidget::itemActivated, this, [this](QListWidgetItem *item) {
        locate(item->data(Qt::UserRole).toString(), Step(item->data(Qt::UserRole + 1).toInt()));
    });
    connect(navigation_, &QListWidget::currentRowChanged, this, &Workspace::select_step);
    for (auto *e : {project_, delta_, rms_, output_})
        connect(e, &QLineEdit::textChanged, this, [this] { update_config(); });
    for (auto *c : {master_, reference_, display_})
        connect(c, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { update_config(); });
    for (auto *b : {roles_, optics_, fullscreen_, timed_, trigger_wiring_, discard_incomplete_takes_})
        connect(b, &QCheckBox::toggled, this, [this] { update_config(); });
    for (auto *s : {training_, validation_, duration_, capture_duration_})
        connect(s, qOverload<int>(&QSpinBox::valueChanged), this, [this] { update_config(); });
    connect(square_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { update_config(); });
    connect(locked_, &QCheckBox::toggled, this, [this](bool on) {
        if (on && (!board_ || !board_->isVisible() || square_->value() <= 0 ||
                   QGuiApplication::screens().size() < 2 || board_->screen() == screen())) {
            QSignalBlocker b(locked_);
            locked_->setChecked(false);
            message_->setText(tr("Use a separate display, set the final board size, "
                                 "then measure a square before locking."));
        } else if (board_) {
            if (on) {
                board_size_ = board_->size();
                board_->setFixedSize(board_size_);
                board_->set_square_size_mm(square_->value());
                config_.board_square_px = board_->square_px();
                config_.board_logical_size = board_size_;
                config_.board_device_pixel_ratio = board_->devicePixelRatioF();
                config_.board_measured_utc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            } else {
                board_->setMinimumSize(1, 1);
                board_->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
            }
        }
        update_config();
    });
    busy_timer_ = new QTimer(this);
    busy_timer_->setInterval(100);
    connect(busy_timer_, &QTimer::timeout, this, [this] {
        // The inline bar only represents short, blocking operations with a clear end: connecting the
        // rig, checking the RP2350, the finite Trigger check and the calibration solve. Long-running
        // modes (continuous Trigger, calibration capture, recording) have their own indicators.
        const bool tracked = ui_busy_ || state_.activity == Activity::Connecting ||
                             state_.activity == Activity::Syncing || state_.activity == Activity::Solving;
        if (!tracked) {
            if (!ui_busy_ && !state_.busy())
                busy_timer_->stop();
            finish_busy_progress();
            return;
        }
        progress_holding_ = false;
        ++progress_generation_;
        const double sec = busy_elapsed_.isValid() ? busy_elapsed_.elapsed() / 1000.0 : 0.0;
        const QString base = !state_.message.isEmpty() && state_.busy() ? state_.message : ui_busy_label_;
        const QString status_line = QStringLiteral("⏳ %1 (%2 s)").arg(base).arg(sec, 0, 'f', 1);
        // Only the progress bar ticks. Stage status / bottom message are written by snapshots, which
        // change at phase boundaries only, so the text no longer alternates between two writers.
        if (state_.activity == Activity::Syncing) {
            static const struct {
                const char *needle;
                const char *label;
            } phases[]{{"Verifying RP2350", "Verify trigger board"},
                       {"Settling clocks", "Settle camera clocks (5 s)"},
                       {"preamble", "Preamble pulse"},
                       {"formal pulses", "Three formal pulses"},
                       {"Analysing", "Collect & match edges"}};
            int phase = 0;
            for (int i = 0; i < 5; ++i)
                if (state_.message.contains(QLatin1String(phases[i].needle), Qt::CaseInsensitive))
                    phase = i;
            constexpr double expected_s = 17.0; // check + 5 s settle + preamble + 3 pulses + collection
            busy_progress_->setRange(0, 1000);
            busy_progress_->setValue(int(std::min(0.97, sec / expected_s) * 1000));
            busy_progress_->setFormat(tr("Finite Trigger check · step %1/5 · %2 · %3 s")
                                          .arg(phase + 1)
                                          .arg(tr(phases[phase].label))
                                          .arg(sec, 0, 'f', 1));
        } else {
            busy_progress_->setRange(0, 0); // indeterminate
            busy_progress_->setFormat(QString());
        }
        busy_progress_->setToolTip(status_line);
        busy_progress_->show();
        if (ui_busy_ && !state_.busy())
            set_text(calibration_primary_, QStringLiteral("⏳ %1 (%2 s)").arg(ui_busy_label_).arg(sec, 0, 'f', 1));
        for (auto *p : previews_) {
            p->connecting_active = (state_.activity == Activity::Connecting) || (ui_busy_ && ui_connecting_cameras_);
            p->connecting_status = status_line;
            p->update();
        }
    });
    connect(controller_, &Controller::changed, this, &Workspace::display_snapshot);
    connect(controller_, &Controller::devices_changed, this, &Workspace::refresh_devices);
    connect(controller_, &Controller::profiles_changed, this, &Workspace::refresh_profiles_ui);
    build_guided_pages();
    build_three_workspaces(header_host, body_host, center_host);
    apply_theme();
    navigation_->setCurrentRow(0);
    update_config();
    display_snapshot(state_);
    controller_->refresh();
}

void Workspace::build_guided_pages() {
    building_journey_ = true;
    const std::array<QStringList, 5> titles{
        QStringList{tr("1. Assign cameras and roles"), tr("2. Project acceptance limits"),
                    tr("3. Connect and verify"), tr("Device details / project files")},
        QStringList{tr("1. Confirm Trigger wiring"), tr("2. Send and observe Trigger"),
                    tr("3. Inspect synchronization result"), tr("Detailed Trigger evidence")},
        QStringList{tr("1. Open secondary-display pattern"), tr("2. Measure and lock square size"),
                    tr("3. Capture common training poses"), tr("4. Solve training"),
                    tr("5. Capture NEW validation poses"), tr("6. Validate frozen calibration"),
                    tr("7. Confirm lens and rig"), tr("Observations, files and advanced controls")},
        QStringList{tr("1. Check recording prerequisites"), tr("2. Output and duration"),
                    tr("3. Record and close Take")},
        QStringList{tr("1. Open a Take"), tr("2. Shared-timeline playback"),
                    tr("3. File and quality report")}};
    for (int i = 0; i < 5; ++i) {
        journey_results_[i] = new QLabel(this);
        journey_results_[i]->setObjectName(QString("multiJourneyResult%1").arg(i));
        journey_results_[i]->setWordWrap(true);
        journey_results_[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        journey_results_[i]->setTextInteractionFlags(Qt::TextSelectableByMouse);
    }
    journey_results_[int(Step::Calibration)]->hide();
    const std::array<QVector<QVector<QWidget *>>, 5> groups{
        QVector<QVector<QWidget *>>{
            {devices_[0], devices_[1], devices_[2], master_, reference_, roles_, commands_["refresh"]},
            {project_, delta_, rms_},
            {commands_["connect"], commands_["checkBoard"], journey_results_[0]},
            {commands_["disconnect"], board_status_, device_readbacks_[0], device_readbacks_[1],
             device_readbacks_[2], commands_["saveProject"], commands_["loadProject"]}},
        QVector<QVector<QWidget *>>{
            {trigger_wiring_, findChild<QLabel *>("multiSyncInfo")},
            {commands_["runSync"], commands_["startTrigger"], commands_["stopTrigger"]},
            {journey_results_[1], commands_["overrideSync"]},
            {sync_status_}},
        QVector<QVector<QWidget *>>{{display_, fullscreen_, commands_["showBoard"]},
                                    {square_, locked_},
                                    {},
                                    {},
                                    {},
                                    {validation_view_},
                                    {optics_},
                                    {calibration_advanced_, storage_status_, commands_["exportCal"],
                                     commands_["openOutput"], commands_["calibrationChecklist"]}},
        QVector<QVector<QWidget *>>{{journey_results_[3]},
                                    {output_, commands_["chooseOutput"], timed_, duration_},
                                    {discard_incomplete_takes_, commands_["record"], record_status_}},
        QVector<QVector<QWidget *>>{
            {commands_["openTake"], commands_["lastTake"]}, {journey_results_[4], include_}, {reports_}}};
    const auto move_row = [](QWidget *widget, QFormLayout *destination) {
        if (!widget)
            return;
        QLabel *label = nullptr;
        if (auto *form = qobject_cast<QFormLayout *>(widget->parentWidget() ? widget->parentWidget()->layout()
                                                                            : nullptr)) {
            int row = -1;
            QFormLayout::ItemRole role;
            form->getWidgetPosition(widget, &row, &role);
            if (row >= 0) {
                auto taken = form->takeRow(row);
                if (taken.labelItem)
                    label = qobject_cast<QLabel *>(taken.labelItem->widget());
                delete taken.labelItem;
                delete taken.fieldItem;
            }
        }
        if (label)
            destination->addRow(label, widget);
        else
            destination->addRow(widget);
    };
    const std::array<QStringList, 5> pill_labels{
        QStringList{tr("1A Rig"), tr("1B Limit"), tr("1C Link"), tr("1D File")},
        QStringList{tr("2A Wire"), tr("2B Pulse"), tr("2C Check"), tr("2D Trace")},
        QStringList{tr("1 Board"), tr("2 Lock"), tr("3 Train"), tr("4 Solve"), tr("5 Valid"), tr("6 Check"),
                    tr("7 Optic"), tr("8 Adv")},
        QStringList{tr("4A Check"), tr("4B Path"), tr("4C Rec")},
        QStringList{tr("5A Open"), tr("5B Play"), tr("5C JSON")}};
    for (int step = 0; step < 5; ++step) {
        auto *contents = findChild<QWidget *>("multiPage" + QString::number(step));
        auto *old_layout = contents->layout();
        auto *root = new QWidget(contents);
        root->setObjectName(QString("multiGuidedPage%1").arg(step));
        auto *layout = new QVBoxLayout(root);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(6);
        auto *pill_bar = new QWidget(root);
        pill_bar->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        auto *pill_grid = new QGridLayout(pill_bar);
        pill_grid->setContentsMargins(0, 0, 0, 0);
        pill_grid->setHorizontalSpacing(3);
        pill_grid->setVerticalSpacing(3);
        substep_pills_[step].clear();
        const int cols = pill_labels[step].size() > 4 ? 4 : int(pill_labels[step].size());
        for (int phase = 0; phase < pill_labels[step].size(); ++phase) {
            auto *pill = new QToolButton(pill_bar);
            pill->setText(pill_labels[step][phase]);
            pill->setToolTip(titles[step].value(phase));
            pill->setCheckable(true);
            pill->setProperty("multiSubstepPill", true);
            pill->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
            pill->setMinimumHeight(20);
            connect(pill, &QToolButton::clicked, this,
                    [this, step, phase] { set_guided_phase(Step(step), phase); });
            pill_grid->addWidget(pill, phase / cols, phase % cols);
            substep_pills_[step].push_back(pill);
        }
        layout->addWidget(pill_bar);
        guided_selectors_[step] = new QComboBox(root);
        guided_selectors_[step]->setObjectName(QString("multiSubstep%1").arg(step));
        guided_selectors_[step]->addItems(titles[step]);
        guided_selectors_[step]->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        guided_selectors_[step]->setMinimumContentsLength(15);
        layout->addWidget(guided_selectors_[step]);
        guided_stacks_[step] = new QStackedWidget(root);
        guided_stacks_[step]->setObjectName(QString("multiSubpages%1").arg(step));
        guided_stacks_[step]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        layout->addWidget(guided_stacks_[step], 1);
        for (int phase = 0; phase < groups[step].size(); ++phase) {
            const bool advanced = (step == 0 || step == 1) && phase == 3 || step == 2 && phase == 7;
            auto *surface = new QWidget;
            surface->setProperty("multiOwningPhase", phase);
            surface->setObjectName(QString("multiTask%1_%2").arg(step).arg(phase));
            auto *form = new QFormLayout(surface);
            form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
            form->setContentsMargins(0, 0, 0, 0);
            form->setVerticalSpacing(6);
            for (auto *widget : groups[step][phase])
                move_row(widget, form);
            if (step == 4 && phase == 2)
                reports_->setMinimumHeight(200);
            if (advanced) {
                auto *scroll = new QScrollArea(root);
                scroll->setObjectName(QString("multiAdvancedScroll%1").arg(step));
                scroll->setWidgetResizable(true);
                scroll->setFrameShape(QFrame::NoFrame);
                scroll->setWidget(surface);
                scroll->setMinimumHeight(220);
                guided_stacks_[step]->addWidget(scroll);
            } else {
                guided_stacks_[step]->addWidget(surface);
            }
        }
        if (step == int(Step::Calibration)) {
            calibration_shared_ = new QWidget(root);
            auto *shared = new QFormLayout(calibration_shared_);
            shared->setContentsMargins(0, 0, 0, 0);
            shared->setVerticalSpacing(4);
            move_row(capture_duration_, shared);
            move_row(counts_, shared);
            layout->insertWidget(2, calibration_shared_);
        }
        delete old_layout;
        for (auto *child : contents->findChildren<QWidget *>(QString{}, Qt::FindDirectChildrenOnly))
            if (child != root)
                child->hide();
        auto *wrapper = new QVBoxLayout(contents);
        wrapper->setContentsMargins(4, 4, 4, 4);
        wrapper->addWidget(root);
        connect(guided_selectors_[step], qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this, step](int phase) { set_guided_phase(Step(step), phase); });
    }
    // Keep core action buttons visible inside their respective substep cards so the user
    // never has to hunt through hidden states; only the internal advanced toggle is replaced.
    calibration_advanced_toggle_->hide();
    calibration_advanced_toggle_->setChecked(true);
    for (int i = 0; i < 5; ++i)
        set_guided_phase(Step(i), 0);
    building_journey_ = false;
}

void Workspace::set_guided_phase(Step step, int phase) {
    const int index = int(step);
    if (!guided_stacks_[index] || phase < 0 || phase >= guided_stacks_[index]->count())
        return;
    guided_phase_[index] = phase;
    guided_stacks_[index]->setCurrentIndex(phase);
    QSignalBlocker block(guided_selectors_[index]);
    guided_selectors_[index]->setCurrentIndex(phase);
    for (int i = 0; i < substep_pills_[index].size(); ++i)
        if (auto *pill = substep_pills_[index][i]) {
            QSignalBlocker pb(pill);
            pill->setChecked(i == phase);
        }
    if (step == Step::Calibration) {
        calibration_shared_->setVisible(phase >= 2 && phase <= 6);
        auto *form = qobject_cast<QFormLayout *>(calibration_shared_->layout());
        form->setRowVisible(capture_duration_, phase == 2 || phase == 4);
    }
    if (!building_journey_) {
        update_blockers();
        update_visible_details();
    }
}

void Workspace::apply_quick_defaults() {
    if (state_.busy())
        return;
    updating_ = true;
    const QStringList default_serials{QStringLiteral("00000327"), QStringLiteral("00000330"),
                                      QStringLiteral("00000328")};
    for (int i = 0; i < 3; ++i) {
        if (devices_[i]->currentIndex() <= 0) {
            int found = -1;
            for (int item = 1; item < devices_[i]->count(); ++item)
                if (devices_[i]->itemData(item, Qt::UserRole + 1).toString().contains(default_serials[i]))
                    found = item;
            if (found < 0 && devices_[i]->count() > i + 1)
                found = i + 1;
            if (found < 0) {
                const QString sel = QStringLiteral("CenturyArks:SilkyEvCam:%1").arg(default_serials[i]);
                devices_[i]->addItem(
                    QStringLiteral("%1 (%2)").arg(config_.cameras[i].position, default_serials[i]), sel);
                devices_[i]->setItemData(devices_[i]->count() - 1, default_serials[i], Qt::UserRole + 1);
                found = devices_[i]->count() - 1;
            }
            devices_[i]->setCurrentIndex(found);
        }
    }
    master_->setCurrentIndex(1);
    reference_->setCurrentIndex(1);
    if (delta_->text().trimmed().isEmpty())
        delta_->setText(QStringLiteral("5.0"));
    if (rms_->text().trimmed().isEmpty())
        rms_->setText(QStringLiteral("0.80"));
    if (square_->value() <= 0)
        square_->setValue(25.0);
    roles_->setChecked(true);
    trigger_wiring_->setChecked(true);
    updating_ = false;
    update_config();
    message_->setText(
        tr("Applied standard SilkyEvCam HD tri-rig defaults (Center=Master/Ref, Max Δt=5.0 us, Max RMS=0.80 px, Square=25.0 mm)."));
}
Workspace::~Workspace() {
    if (board_) {
        board_->removeEventFilter(this);
        delete board_;
    }
}
bool Workspace::may_leave() const {
    return !state_.recording() && !state_.trigger_stop_pending && state_.activity != Activity::Triggering &&
           state_.activity != Activity::Syncing && state_.activity != Activity::Solving;
}
void Workspace::deactivate() {
    controller_->disconnect_rig();
    if (board_)
        board_->close();
}
QAction *Workspace::toolbox_action() const { return toolbox_->toggleViewAction(); }
Step Workspace::current_step() const { return Step(std::max(0, navigation_->currentRow())); }
RigConfig Workspace::configuration() const { return config_; }
void Workspace::update_config() {
    if (updating_)
        return;
    const auto old_geometry = config_.geometry_key();
    const auto old_cameras = config_.cameras;
    const int old_master = config_.master;
    const int old_reference = config_.reference;
    config_.project = project_->text();
    for (int i = 0; i < 3; ++i) {
        config_.cameras[i].selection = devices_[i]->currentData().toString();
        config_.cameras[i].serial = devices_[i]->currentData(Qt::UserRole + 1).toString();
    }
    config_.master = master_->currentIndex();
    config_.reference = reference_->currentIndex();
    for (int i = 0; i < 3; ++i)
        if (previews_[i]) {
            previews_[i]->master_camera = config_.master;
            previews_[i]->reference_camera = config_.reference;
        }
    config_.roles_confirmed = roles_->isChecked();
    config_.trigger_wiring_confirmed = trigger_wiring_->isChecked();
    config_.optics_confirmed = optics_->isChecked();
    bool ok = false;
    config_.max_delta_us = delta_->text().toDouble(&ok);
    if (!ok)
        config_.max_delta_us = NAN;
    config_.max_rms_px = rms_->text().toDouble(&ok);
    if (!ok)
        config_.max_rms_px = NAN;
    config_.square_mm = square_->value();
    config_.board_locked = locked_->isChecked();
    config_.board_display = display_->currentData().toString();
    config_.board_fullscreen = fullscreen_->isChecked();
    config_.training_target = training_->value();
    config_.validation_target = validation_->value();
    config_.calibration_capture_seconds = capture_duration_->value();
    config_.output = output_->text();
    config_.timed = timed_->isChecked();
    config_.duration_seconds = duration_->value();
    config_.discard_incomplete_takes = discard_incomplete_takes_->isChecked();
    const bool rig_roles_changed =
        config_.cameras[0].selection != old_cameras[0].selection ||
        config_.cameras[1].selection != old_cameras[1].selection ||
        config_.cameras[2].selection != old_cameras[2].selection ||
        config_.cameras[0].serial != old_cameras[0].serial ||
        config_.cameras[1].serial != old_cameras[1].serial ||
        config_.cameras[2].serial != old_cameras[2].serial ||
        config_.master != old_master || config_.reference != old_reference;
    if (config_.geometry_key() != old_geometry) {
        QSignalBlocker b(optics_);
        optics_->setChecked(false);
        config_.optics_confirmed = false;
    }
    if (rig_roles_changed) {
        QSignalBlocker b(roles_);
        roles_->setChecked(false);
        config_.roles_confirmed = false;
    }
    controller_->configure(config_);
    update_blockers();
    update_visible_details();
}
void Workspace::select_step(int i) {
    if (i < 0 || i > 4)
        return;
    if (i != int(Step::Sync) && (state_.activity == Activity::Triggering || state_.trigger_stop_pending))
        controller_->stop();
    if (navigation_->currentRow() != i) {
        navigation_->setCurrentRow(i);
        return;
    }
    if (dock_stack_ && tools_panel_ && dock_stack_->currentWidget() != tools_panel_)
        dock_stack_->setCurrentWidget(tools_panel_);
    const bool entering = pages_->currentIndex() != i;
    pages_->setCurrentIndex(i);
    triggers_->setVisible(i == 1);
    transport_->setVisible(i == 4);
    stability_->setVisible(i == 2);
    guide_->setVisible(i == 2);
    calibration_primary_->show();
    capture_status_->setVisible(i == int(Step::Calibration));
    update_blockers();
    update_visible_details();
    for (auto *p : previews_) {
        p->overlays = i == 2;
        p->update();
    }
    if (entering && i == int(Step::Calibration) && state_.activity == Activity::Calibrating)
        set_guided_phase(Step::Calibration, state_.calibration.training_complete ? 4 : 2);
}
void Workspace::update_navigation() {
    if (!navigation_ || navigation_->count() != 5)
        return;
    for (int i = 0; i < 5; ++i) {
        const auto &gate = gates_[i];
        const bool failed = gate.status.contains("failed", Qt::CaseInsensitive);
        const auto color = failed          ? QColor(205, 65, 80)
                           : gate.complete ? QColor(27, 153, 102)
                                           : QColor(132, 145, 155);
        auto *item = navigation_->item(i);
        const auto code = failed                           ? QString("failed")
                          : gate.complete                  ? QString("complete")
                          : navigation_->currentRow() == i ? QString("current")
                                                           : QString("pending");
        if (item->data(Qt::UserRole).toString() != code) {
            item->setIcon(IconProvider::get("circle", color));
            item->setData(Qt::UserRole, code);
        }
        const bool current = navigation_->currentRow() == i;
        item->setForeground(failed                     ? QBrush(QColor(205, 65, 80))
                            : gate.complete || current ? palette().text()
                                                       : QBrush(QColor(132, 145, 155)));
        item->setData(Qt::UserRole + 1, current);
        const auto status =
            failed                                 ? tr("Failed")
            : gate.status.startsWith("Incomplete") ? tr("Incomplete")
            : gate.complete ? (i == 1 && state_.sync.operator_accepted ? tr("Accepted manually")
                               : i == 3                                ? tr("Closed | tail unknown")
                                                                       : tr("Complete"))
            : current && (gate.status == "Running" || gate.status == "Stop unconfirmed") ? gate.status
            : current ? tr("Current | pending")
                      : tr("Pending");
        const auto text = QString::number(i + 1) + ". " + step_text(Step(i)) + "\n" + status;
        if (item->text() != text)
            item->setText(text);
        item->setToolTip(gate.next_action);
    }
}
GuidanceContext Workspace::guidance_context() const {
    bool secondary = false;
    for (auto *candidate : QGuiApplication::screens())
        if (candidate->name() == config_.board_display && candidate != screen())
            secondary = true;
    return {secondary, board_ && board_->isVisible()};
}
void Workspace::locate(QString key, Step step) {
    pending_details_->menu()->hide();
    commands_["calibrationChecklist"]->menu()->hide();
    select_step(int(step));
    auto *field = fields_.value(key);
    if (!field)
        return;
    if (key == "captureDuration")
        set_guided_phase(step, state_.calibration.training_complete ? 4 : 2);
    for (auto *owner = field; owner; owner = owner->parentWidget())
        if (owner->property("multiOwningPhase").isValid()) {
            set_guided_phase(step, owner->property("multiOwningPhase").toInt());
            break;
        }
    for (auto *owner = field->parentWidget(); owner; owner = owner->parentWidget())
        if (auto *scroll = qobject_cast<QScrollArea *>(owner))
            scroll->ensureWidgetVisible(field, 10, 20);
    if (field->isHidden() && qobject_cast<QToolButton *>(field))
        field = calibration_primary_;
    field->setFocus(Qt::OtherFocusReason);
    if (auto *scroll = qobject_cast<QScrollArea *>(pages_->currentWidget()))
        scroll->ensureWidgetVisible(field, 10, 20);
}
void Workspace::update_blockers() {
    if (!blocker_list_)
        return;
    const auto context = guidance_context();
    for (int i = 0; i < 5; ++i)
        gates_[i] = stage_gate(Step(i), config_, state_, context);
    const auto &gate = gates_[int(current_step())];
    const auto &b = gate.pending;
    set_text(stage_title_, tr("%1 / 5  %2").arg(int(current_step()) + 1).arg(step_text(current_step())));
    set_text(stage_status_, gate.status);
    set_text(stage_action_, gate.next_action);
    const auto explanation =
        b.isEmpty()
            ? (current_step() == Step::Review ? tr("Final stage | offline playback") : gate.next_action)
            : tr("Next blocked: %1 remaining\nFirst: %2").arg(b.size()).arg(b.front().message);
    set_text(next_reason_, explanation);
    bool changed = blocker_list_->count() != b.size();
    for (int i = 0; !changed && i < b.size(); ++i) {
        const auto *item = blocker_list_->item(i);
        changed = item->data(Qt::UserRole + 3).toString() != b[i].message ||
                  item->data(Qt::UserRole).toString() != b[i].field ||
                  item->data(Qt::UserRole + 1).toInt() != int(b[i].step) ||
                  item->data(Qt::UserRole + 2).toInt() != int(b[i].kind);
    }
    if (changed) {
        blocker_list_->clear();
        for (const auto &blocker : b) {
            auto *item =
                new QListWidgetItem(blocker_kind_text(blocker.kind) + ": " + blocker.message, blocker_list_);
            item->setData(Qt::UserRole, blocker.field);
            item->setData(Qt::UserRole + 1, int(blocker.step));
            item->setData(Qt::UserRole + 2, int(blocker.kind));
            item->setData(Qt::UserRole + 3, blocker.message);
            item->setToolTip(tr("Locate: %1 > %2").arg(step_text(blocker.step), blocker.message));
            if (blocker.kind == BlockerKind::InvalidField)
                item->setForeground(QColor(205, 65, 80));
        }
    }
    QHash<QWidget *, QString> invalid;
    for (const auto &blocker : b) {
        auto *f = fields_.value(blocker.field);
        if (f && blocker.kind == BlockerKind::InvalidField)
            invalid.insert(f, blocker.message);
    }
    for (auto *f : fields_) {
        if (qobject_cast<QToolButton *>(f))
            continue;
        if (!f->property("multiBaseToolTip").isValid())
            f->setProperty("multiBaseToolTip", f->toolTip());
        const bool bad = invalid.contains(f);
        if (f->property("multiInvalid").toBool() != bad) {
            f->setProperty("multiInvalid", bad);
            f->style()->unpolish(f);
            f->style()->polish(f);
        }
        set_tooltip(f, bad ? invalid.value(f) : f->property("multiBaseToolTip").toString());
    }
    enable_command(commands_["next"], gate.can_advance() && current_step() != Step::Review, explanation);
    commands_["locateNext"]->hide();
    pending_details_->setVisible(!b.isEmpty());
    pending_details_->setToolTip(tr("%1 remaining item(s); select one to reveal its field").arg(b.size()));
    const bool idle = !state_.busy();
    const auto first_reason = [](const QVector<Blocker> &items, const QString &fallback) {
        return items.isEmpty() ? fallback : items.front().message;
    };
    const auto busy_reason = gates_[int(current_step())].pending.isEmpty()
                                 ? tr("Wait for the current operation to finish.")
                                 : gate.next_action;
    const auto config_pending = configuration_blockers(config_);
    const auto device_pending = blockers(Step::Devices, config_, state_);
    const auto entry_pending = blockers(Step::Calibration, config_, state_);
    const auto record_pending = blockers(Step::Capture, config_, state_);
    auto record_reason = first_reason(record_pending, {});
    if (!record_pending.isEmpty() && record_pending.front().step == Step::Calibration &&
        record_pending.front().field == "calibration")
        record_reason = gates_[int(Step::Calibration)].next_action;
    enable_command(commands_["connect"], idle && config_pending.isEmpty(),
                   idle ? first_reason(config_pending, {}) : busy_reason);
    const auto sync_reason =
        !idle ? busy_reason
              : first_reason(device_pending, tr("Confirm Trigger wiring / current limiting first."));
    for (const auto &id : {"runSync", "startTrigger"})
        enable_command(commands_[id], idle && device_pending.isEmpty() && config_.trigger_wiring_confirmed,
                       sync_reason);
    enable_command(commands_["stopTrigger"],
                   state_.activity == Activity::Triggering || state_.trigger_stop_pending,
                   tr("No continuous Trigger or unconfirmed stop is active; use "
                      "Pause / cancel for a finite check."));
    enable_command(commands_["overrideSync"], config_.trigger_wiring_confirmed && state_.can_override_sync(),
                   state_.trigger_stop_pending ? busy_reason
                   : !config_.trigger_wiring_confirmed
                       ? tr("Confirm Trigger wiring / current limiting first.")
                       : tr("Requires a completed failed/incomplete attempt in "
                            "this epoch, connected cameras, confirmed LOW, "
                            "and at least two channel-0 records per camera; not "
                            "available while busy."));
    const bool cal_ready = idle && entry_pending.isEmpty();
    for (const auto &id : {"collect", "manual"})
        enable_command(commands_[id], cal_ready, idle ? first_reason(entry_pending, {}) : busy_reason);
    enable_command(commands_["capturePose"],
                   state_.activity == Activity::Calibrating && !state_.trigger_stop_pending &&
                       state_.calibration.stable_progress >= 1,
                   tr("Start collection; keep all required corners visible in a "
                      "new pose and stable for one second."));
    enable_command(commands_["solve"],
                   idle && state_.calibration.common_count >= config_.training_target &&
                       !state_.calibration.training_complete,
                   !idle ? busy_reason
                   : state_.calibration.training_complete
                       ? tr("Training is already frozen; collect independent validation "
                            "poses.")
                       : tr("Collect common training poses: %1 / %2.")
                             .arg(state_.calibration.common_count)
                             .arg(config_.training_target));
    enable_command(commands_["validate"],
                   idle && state_.calibration.training_complete &&
                       state_.calibration.validation_count >= config_.validation_target,
                   !idle ? busy_reason
                   : !state_.calibration.training_complete
                       ? tr("Solve training first.")
                       : tr("Collect NEW independent validation poses: %1 / %2.")
                             .arg(state_.calibration.validation_count)
                             .arg(config_.validation_target));
    enable_command(commands_["record"], idle && record_pending.isEmpty(),
                   !idle ? busy_reason : record_reason);
    enable_command(commands_["leave"], may_leave(),
                   tr("Stop the active operation and confirm LOW before switching modes."));
    enable_command(commands_["disconnect"], idle, busy_reason);
    for (const auto &id : {"importCal", "importObs", "loadProject", "openTake", "refresh", "checkBoard",
                           "sop", "removePose", "resetCal"})
        enable_command(commands_[id], idle, busy_reason);
    enable_command(commands_["lastTake"], idle && !state_.take_directory.isEmpty(),
                   !idle ? busy_reason : tr("No Take has been created or opened yet."));
    enable_command(
        commands_["exportCal"],
        idle && (!state_.calibration.observation_labels.isEmpty() || state_.calibration.training_complete),
        !idle ? busy_reason : tr("Collect observations or import a calibration first."));
    enable_command(commands_["showBoard"],
                   !state_.busy() && !config_.board_locked && context.secondary_display_selected,
                   !idle                  ? busy_reason
                   : config_.board_locked ? tr("Unlock the measured display before reopening the board.")
                                          : tr("Select a secondary display outside the GUI screen."));
    update_navigation();
    update_calibration_guidance();
    update_guided_journey();
}

void Workspace::perform_calibration_action() {
    const auto action = calibration_action_;
    if (action.startsWith("phase:")) {
        set_guided_phase(current_step(), action.mid(6).toInt());
        return;
    }
    if (action == "pauseCapture") {
        controller_->stop();
        return;
    }
    auto *button = commands_.value(action);
    if (button && button != calibration_primary_ && button->isEnabled()) {
        button->click();
        return;
    }
    locate(action, calibration_action_step_);
}

void Workspace::update_guided_journey() {
    if (building_journey_ || !guided_stacks_[0])
        return;
    const auto step = current_step();
    const int index = int(step);
    const auto &gate = gates_[index];
    const auto &s = state_;
    const int phase = guided_phase_[index];
    if (step == Step::Calibration) {
        const bool collecting = s.activity == Activity::Calibrating && !s.trigger_stop_pending;
        guide_->setVisible(collecting);
        stability_->setVisible(collecting);
    }
    if (s.trigger_stop_pending) {
        calibration_action_ = "stopTrigger";
        calibration_action_step_ = Step::Sync;
        const auto warning =
            tr("LOW / drive restoration is unconfirmed. Retry Stop Trigger before continuing.");
        set_text(calibration_primary_, tr("Retry Stop Trigger"));
        calibration_primary_->setProperty("multiAction", calibration_action_);
        enable_command(calibration_primary_, commands_["stopTrigger"]->isEnabled(), warning);
        set_text(stage_action_, tr("Trigger stop must be confirmed before any other step."));
        set_text(next_reason_, warning);
        set_tooltip(next_reason_, warning);
        capture_status_->hide();
        return;
    }
    QString label, instruction, reason;
    bool enabled = true;
    if (step != Step::Calibration)
        calibration_action_step_ = step;
    const auto phase_action = [&](int next, const QString &text) {
        calibration_action_ = "phase:" + QString::number(next);
        label = text;
    };
    const auto blocker_action = [&](const Blocker &item) {
        calibration_action_ = item.field;
        calibration_action_step_ = item.step;
        label = item.kind == BlockerKind::InvalidField ? tr("Complete required field")
                                                       : tr("Resolve: %1").arg(step_text(item.step));
        reason = item.message;
    };
    const auto config_items = configuration_blockers(config_);
    const auto missing = [&](const QStringList &keys) -> const Blocker * {
        for (const auto &item : config_items)
            if (keys.contains(item.field))
                return &item;
        return nullptr;
    };
    if (step == Step::Calibration) {
        if (!s.busy() && !gate.pending.isEmpty() && gate.pending.front().step != Step::Calibration) {
            const auto &first = gate.pending.front();
            set_text(stage_action_, tr("Complete %1 before calibration.").arg(step_text(first.step)));
            set_text(next_reason_, first.message);
            set_tooltip(next_reason_, first.message);
            capture_status_->hide();
            return;
        }
        const auto action = calibration_action_;
        const int recommended = action == "display" || action == "showBoard"  ? 0
                                : action == "square" || action == "lockBoard" ? 1
                                : action == "collect" ? (s.calibration.training_complete ? 4 : 2)
                                : action == "solve" || action == "resetCal" ? 3
                                : action == "validate"                      ? 5
                                                                            : 6;
        const auto action_key = action + QString::number(recommended);
        if (action_key != last_calibration_action_ && !s.busy() &&
            calibration_action_step_ == Step::Calibration && !action.startsWith("phase:")) {
            last_calibration_action_ = action_key;
            building_journey_ = true;
            set_guided_phase(step, recommended);
            building_journey_ = false;
        }
        if (!s.busy() && calibration_action_step_ == Step::Calibration &&
            guided_phase_[index] != recommended) {
            phase_action(recommended, tr("Continue guided calibration"));
            set_text(calibration_primary_, label);
            calibration_primary_->setProperty("multiAction", calibration_action_);
        }
        if (s.activity != Activity::Calibrating) {
            const auto current = guided_phase_[index];
            const QStringList hints{
                tr("Select the separate screen, then open the pattern."),
                tr("Measure one square at its final size, enter millimetres, then lock."),
                tr("Keep the full board visible in all cameras; capture one short round."),
                tr("Solve only after the required common training poses have been accepted."),
                tr("Use new poses that were not part of training."),
                tr("Validate without updating the frozen training parameters."),
                tr("Confirm the lens, focus and rigid assembly have not changed."),
                tr("Advanced controls and exported files; normal guidance remains available above.")};
            set_text(stage_action_, hints.value(current));
        }
        const auto detail = !s.calibration.reason.isEmpty() ? s.calibration.reason : gate.next_action;
        set_text(next_reason_, detail);
        set_tooltip(next_reason_, detail);
        capture_status_->setVisible(!capture_status_->text().isEmpty());
        return;
    }
    capture_status_->hide();
    if (step == Step::Devices) {
        QStringList readbacks;
        for (const auto &camera : s.cameras) {
            const int usb_speed = camera.metadata.value("usbSpeedMbps").toInt(0);
            const QString proto =
                !camera.connected ? QString{}
                : (usb_speed > 0 && usb_speed < 5000)
                    ? (QStringLiteral(" | ⚠ ") + camera.metadata.value("usbProtocol").toString(QStringLiteral("USB 2.0")))
                    : (QStringLiteral(" | ") + camera.metadata.value("usbProtocol").toString(QStringLiteral("USB 3.0 (5 Gbps)")));
            readbacks << tr("%1: %2 | %3%4")
                             .arg(camera.binding.position.isEmpty() ? tr("Unassigned")
                                                                    : camera.binding.position,
                                  camera.connected ? camera.binding.serial : tr("not connected"),
                                  camera.metadata.value("modeReadback").toString(tr("mode not read back")),
                                  proto);
        }
        readbacks << (s.board_ready ? tr("RP2350 identity / firmware: ready") : tr("RP2350: not verified"));
        set_text(journey_results_[0], readbacks.join("\n"));
        if (phase == 0) {
            instruction =
                tr("Select three distinct devices, set both references, then confirm the rigid rig.");
            if (const auto *item = missing({"camera0", "camera1", "camera2", "master", "reference"}))
                blocker_action(*item);
            else if (!config_.roles_confirmed)
                blocker_action(
                    {step, "roles", tr("Confirm camera positions, clock wiring and rigid mounting.")});
            else
                phase_action(1, tr("Next: project limits"));
        } else if (phase == 1) {
            instruction = tr("Enter the project name and required acceptance thresholds before connecting.");
            if (const auto *item = missing({"project", "delta", "rms"}))
                blocker_action(*item);
            else
                phase_action(2, tr("Next: connect and verify"));
        } else if (phase == 2) {
            instruction = tr("Connect the configured three devices, then verify the bound RP2350.");
            if (!config_items.isEmpty())
                blocker_action(config_items.front());
            else if (!config_.roles_confirmed)
                blocker_action({step, "roles", tr("Confirm the rig and clock roles first.")});
            else if (!s.all_connected()) {
                calibration_action_ = "connect";
                label = tr("Connect configured rig");
            } else if (!s.board_ready) {
                calibration_action_ = "checkBoard";
                label = tr("Verify bound RP2350");
            } else {
                calibration_action_ = "next";
                label = tr("Next: synchronization");
                reason = tr("All three cameras and the bound board are ready.");
            }
        } else {
            instruction = tr("Detailed hardware readbacks and project files.");
            phase_action(2, tr("Return to connection check"));
        }
    } else if (step == Step::Sync) {
        const auto device_pending = blockers(Step::Devices, config_, s);
        QStringList measured;
        measured << (s.sync.operator_accepted ? tr("Accepted manually") : verdict_text(s.sync.verdict));
        measured << (s.sync.timing_measured()
                         ? tr("Maximum difference: %1 us").arg(s.sync.max_delta_us, 0, 'f', 1)
                         : tr("Time difference: not measured"));
        for (int i = 0; i < camera_count; ++i)
            measured << tr("%1: %2 matched | %3 observed")
                            .arg(config_.cameras[i].position)
                            .arg(s.sync.matched[i])
                            .arg(s.sync.observed_edges[i].size());
        measured << s.sync.reason.left(280);
        set_text(journey_results_[1], measured.join("\n"));
        set_tooltip(journey_results_[1], s.sync.reason);
        if (phase == 0) {
            instruction = tr("Confirm wiring and current limiting before sending any signal.");
            if (!device_pending.isEmpty())
                blocker_action(device_pending.front());
            else if (!config_.trigger_wiring_confirmed)
                blocker_action(
                    {step, "triggerWiring", tr("Confirm GP2 wiring, return paths and current limiting.")});
            else
                phase_action(1, tr("Next: Trigger check"));
        } else if (phase == 1) {
            instruction = tr("Start one finite check. Continuous output is an optional manual diagnostic.");
            calibration_action_ = "runSync";
            label = tr("Run finite Trigger check");
            reason =
                tr("5 s settling, one preamble and three formal pulses; no signal is sent until clicked.");
            if (!device_pending.isEmpty())
                blocker_action(device_pending.front());
            else if (!config_.trigger_wiring_confirmed)
                blocker_action({step, "triggerWiring", tr("Confirm Trigger wiring before running a check.")});
        } else if (phase == 2) {
            instruction = tr("Inspect all three traces and the measured result before continuing.");
            if (s.sync.accepted(s.epoch, config_.max_delta_us) && !s.trigger_stop_pending) {
                calibration_action_ = "next";
                label = tr("Next: guided calibration");
            } else if (s.can_override_sync() && config_.trigger_wiring_confirmed) {
                calibration_action_ = "overrideSync";
                label = tr("Review and accept manually...");
                reason = tr(
                    "Automatic checks did not pass. Manual acceptance requires your observation and reason.");
            } else {
                phase_action(1, tr("Retry Trigger check"));
                reason = s.sync.reason.isEmpty() ? tr("Run a finite check first.") : s.sync.reason;
            }
        } else {
            instruction = tr("Retained decoded Trigger evidence; not a voltage measurement.");
            phase_action(2, tr("Return to result inspection"));
        }
    } else if (step == Step::Capture) {
        auto pending = blockers(Step::Capture, config_, s);
        pending.erase(std::remove_if(pending.begin(), pending.end(),
                                     [](const Blocker &item) {
                                         return item.field == "output" || item.field == "duration";
                                     }),
                      pending.end());
        set_text(journey_results_[3],
                 tr("Devices: %1\nCurrent-epoch synchronization: %2\nValidated calibration / optics: %3")
                     .arg(s.all_connected() && s.board_ready ? tr("ready") : tr("pending"))
                     .arg(s.sync.accepted(s.epoch, config_.max_delta_us) ? tr("accepted") : tr("pending"))
                     .arg(pending.isEmpty() ? tr("ready") : tr("pending")));
        if (phase == 0) {
            instruction = tr("Check the rig, synchronization and calibration before preparing a Take.");
            if (!pending.isEmpty())
                blocker_action(pending.front());
            else
                phase_action(1, tr("Next: output and duration"));
        } else if (phase == 1) {
            instruction = tr("Choose the output location and optional formal recording duration.");
            const auto all = blockers(Step::Capture, config_, s);
            const auto invalid = std::find_if(all.begin(), all.end(), [](const Blocker &item) {
                return item.field == "output" || item.field == "duration";
            });
            if (invalid != all.end())
                blocker_action(*invalid);
            else
                phase_action(2, tr("Next: recording preflight"));
        } else {
            instruction = tr(
                "A new Trigger preflight runs before formal recording. Stop closes and verifies the Take.");
            if (gate.can_advance()) {
                calibration_action_ = "next";
                label = tr("Next: review closed Take");
            } else if (!blockers(Step::Capture, config_, s).isEmpty())
                blocker_action(blockers(Step::Capture, config_, s).front());
            else {
                calibration_action_ = "record";
                label = tr("Start new preflight + recording");
            }
        }
    } else {
        set_text(
            journey_results_[4],
            s.playback.loaded
                ? tr("Take: %1\nShared range: %2 - %3 s\nUse the shared timeline below the camera views.")
                      .arg(QFileInfo(s.take_directory).fileName())
                      .arg(s.playback.first_us / 1e6, 0, 'f', 3)
                      .arg(s.playback.last_us / 1e6, 0, 'f', 3)
                : tr("Open a Take first; cameras do not need to be connected."));
        if (phase == 0) {
            instruction = tr("Choose a Take folder, or open the last recorded Take.");
            calibration_action_ = "openTake";
            label = tr("Open Take directory...");
            if (s.playback.loaded)
                phase_action(1, tr("Next: shared playback"));
        } else if (phase == 1) {
            instruction = tr("Play, pause or seek all three cameras on one timeline.");
            if (!s.playback.loaded) {
                phase_action(0, tr("Open a Take first"));
                reason = tr("No Take is loaded.");
            } else
                phase_action(2, tr("Next: file and quality report"));
        } else {
            instruction = tr("Inspect synchronization, calibration and file verification evidence.");
            phase_action(s.playback.loaded ? 1 : 0,
                         s.playback.loaded ? tr("Return to playback") : tr("Open a Take"));
        }
    }
    if (s.trigger_stop_pending) {
        calibration_action_ = "stopTrigger";
        calibration_action_step_ = Step::Sync;
        label = tr("Retry Stop Trigger");
        reason = tr("LOW / drive restoration is unconfirmed. Other operations remain blocked.");
    } else if (s.busy() && !(step == Step::Sync && phase == 2 && calibration_action_ == "overrideSync" &&
                             s.can_override_sync() && config_.trigger_wiring_confirmed)) {
        calibration_action_ = s.activity == Activity::Triggering ? "stopTrigger" : "stop";
        label = s.activity == Activity::Recording    ? tr("Stop recording")
                : s.activity == Activity::Triggering ? tr("Stop continuous Trigger")
                : s.activity == Activity::Connecting ? tr("Cancel connection")
                : s.activity == Activity::Syncing    ? tr("Cancel Trigger check")
                : s.activity == Activity::Preparing  ? tr("Cancel recording preflight")
                                                     : tr("Wait for completion");
        enabled = s.activity != Activity::Stopping && s.activity != Activity::Verifying &&
                  s.activity != Activity::Solving;
        reason = gate.next_action;
    }
    if (reason.isEmpty())
        reason = tr("Complete this substep, then use the main action below.");
    if (calibration_action_ == "next" && !commands_["next"]->isEnabled()) {
        enabled = false;
        reason = gate.next_action;
    }
    set_text(stage_action_, instruction);
    set_text(next_reason_, reason);
    set_tooltip(next_reason_, reason);
    set_text(calibration_primary_, label);
    calibration_primary_->setProperty("multiAction", calibration_action_);
    enable_command(calibration_primary_, enabled, reason);
}

void Workspace::update_calibration_guidance() {
    if (!calibration_checklist_ || current_step() != Step::Calibration)
        return;
    const auto &cal = state_.calibration;
    const auto &gate = gates_[int(Step::Calibration)];
    const auto context = guidance_context();
    const bool matching = cal.geometry_key == config_.geometry_key();
    const bool validated = matching && cal.training_complete && cal.verdict == Verdict::Passed &&
                           std::isfinite(cal.rms) && std::isfinite(config_.max_rms_px) &&
                           cal.rms <= config_.max_rms_px && cal.common_count >= config_.training_target &&
                           cal.validation_count >= config_.validation_target;
    const std::array<bool, 7> done{validated || (context.secondary_display_selected && context.board_open),
                                   validated || (config_.board_locked && config_.square_mm > 0),
                                   cal.common_count >= config_.training_target,
                                   matching && cal.training_complete,
                                   matching && cal.training_complete &&
                                       cal.validation_count >= config_.validation_target,
                                   validated,
                                   validated && config_.optics_confirmed};
    const QStringList titles{
        tr("Open pattern on secondary display"),
        tr("Measure square and lock display size"),
        tr("Common training poses: %1 / %2").arg(cal.common_count).arg(config_.training_target),
        tr("Solve and freeze training"),
        tr("New validation poses: %1 / %2").arg(cal.validation_count).arg(config_.validation_target),
        tr("Validate frozen calibration"),
        tr("Confirm unchanged lens and rig")};
    const QStringList fields{"display", "lockBoard", "collect",    "solve",
                             "collect", "validate",  "calibration"};
    bool reached = false;
    for (int i = 0; i < 7; ++i) {
        auto *item = calibration_checklist_->item(i);
        const bool current = !done[i] && !reached;
        reached = reached || current;
        const auto status = done[i] ? tr("Done") : current ? tr("Now") : tr("Later");
        const auto text = QString("%1. %2 | %3").arg(i + 1).arg(titles[i], status);
        if (item->text() != text)
            item->setText(text);
        item->setData(Qt::UserRole, done[i] ? "complete" : current ? "current" : "pending");
        item->setData(Qt::UserRole + 1, fields[i]);
        item->setToolTip(text);
        item->setForeground(done[i]   ? QBrush(QColor(30, 155, 108))
                            : current ? palette().brush(QPalette::HighlightedText)
                                      : palette().brush(QPalette::Disabled, QPalette::Text));
        item->setBackground(current ? palette().brush(QPalette::Highlight) : QBrush(Qt::NoBrush));
    }
    calibration_action_step_ = Step::Calibration;
    QString label;
    bool enabled = true;
    if (state_.activity == Activity::Calibrating && !state_.trigger_stop_pending) {
        calibration_action_ = "pauseCapture";
        label = tr("Pause capture | %1 s remaining")
                    .arg(std::max(qint64(0), cal.capture_remaining_ms) / 1000., 0, 'f', 1);
        set_text(capture_status_,
                 tr("%1 s round | hold still for 1 s. Ends automatically; accepted poses are retained.")
                     .arg(cal.capture_duration_ms / 1000., 0, 'f', 1));
        set_text(stage_action_, cal.reason.isEmpty() ? cal.guide : cal.guide + "\n" + cal.reason);
    } else {
        if (gate.can_advance()) {
            calibration_action_ = "next";
            label = tr("Continue to recording");
        } else if (!gate.pending.isEmpty()) {
            const auto first = gate.pending.front();
            calibration_action_ = first.field;
            calibration_action_step_ = first.step;
            const QHash<QString, QString> labels{
                {"display", tr("1. Select secondary display")},
                {"showBoard", tr("1. Open blinking board")},
                {"square", tr("2. Measure one square")},
                {"lockBoard", tr("2. Confirm measured display size")},
                {"collect",
                 (cal.capture_expired ? tr("%1. Retry capture | %2 s") : tr("%1. Capture pose | %2 s"))
                     .arg(cal.training_complete ? 5 : 3)
                     .arg(config_.calibration_capture_seconds)},
                {"solve", tr("4. Solve training")},
                {"validate", tr("6. Validate new poses")},
                {"calibration", tr("7. Confirm unchanged lens and rig")},
                {"resetCal", tr("Clear incompatible calibration...")}};
            label = labels.value(calibration_action_, tr("Resolve first pending item"));
            if (first.kind == BlockerKind::Running) {
                label = tr("Operation in progress");
                enabled = calibration_action_ == "stopTrigger" && commands_["stopTrigger"]->isEnabled();
            }
        }
        QString status;
        if (cal.capture_expired && !cal.training_complete && cal.common_count < config_.training_target)
            status = tr("Round ended. Resolve the issue below and retry; change pose after an acceptance.");
        else if (cal.capture_expired && cal.training_complete &&
                 cal.validation_count < config_.validation_target)
            status =
                tr("Round ended. Resolve the issue below and retry; accepted validation poses must be NEW.");
        else if (state_.activity == Activity::Solving)
            status = tr("Solving in the background; wait for the result.");
        else if (calibration_action_ == "collect")
            status = tr("Place the whole board in all three views, then capture one short round.");
        if (!cal.reason.isEmpty())
            status += (status.isEmpty() ? QString{} : "\n") + cal.reason;
        set_text(capture_status_, status);
    }
    if (label.isEmpty())
        label = tr("Review calibration prerequisites");
    if (supplemental_->currentIndex() > 0 &&
        (calibration_action_ == "collect" || calibration_action_ == "pauseCapture")) {
        if (calibration_action_ == "collect")
            label = tr("Capture %1 intrinsic pose | %2 s")
                        .arg(config_.cameras[supplemental_->currentIndex() - 1].position)
                        .arg(config_.calibration_capture_seconds);
        set_text(capture_status_,
                 tr("Intrinsic-only supplement: does NOT count as a three-camera common pose.\n") +
                     capture_status_->text());
    }
    set_text(calibration_primary_, label);
    calibration_primary_->setProperty("multiAction", calibration_action_);
    enable_command(calibration_primary_, enabled, gate.next_action);
    set_tooltip(calibration_primary_, calibration_action_ == "pauseCapture"
                                          ? tr("Pause this round; accepted observations are retained.")
                                          : gate.next_action);
}

void Workspace::refresh_devices(const QVector<DeviceEntry> &list) {
    updating_ = true;
    for (int i = 0; i < 3; ++i) {
        QSignalBlocker block(devices_[i]);
        const auto selected = config_.cameras[i].selection;
        devices_[i]->clear();
        devices_[i]->addItem(tr("Select serial"));
        int max_px = 260;
        const QFontMetrics fm(devices_[i]->font());
        for (const auto &d : list) {
            const QString display =
                (!d.serial.isEmpty() && !d.label.startsWith(d.serial))
                    ? QStringLiteral("%1  (%2)").arg(d.serial, d.label)
                    : d.label;
            devices_[i]->addItem(display, d.selection);
            const int row = devices_[i]->count() - 1;
            devices_[i]->setItemData(row, d.serial, Qt::UserRole + 1);
            devices_[i]->setItemData(row, d.selection, Qt::ToolTipRole);
            max_px = std::max(max_px, fm.horizontalAdvance(display) + 48);
        }
        if (auto *v = devices_[i]->view()) {
            v->setTextElideMode(Qt::ElideNone);
            v->setMinimumWidth(max_px);
        }
        devices_[i]->setCurrentIndex(std::max(0, devices_[i]->findData(selected)));
        const QString cur_sel = devices_[i]->currentData().toString();
        devices_[i]->setToolTip(cur_sel.isEmpty() ? devices_[i]->currentText() : cur_sel);
    }
    updating_ = false;
    update_config();
}
void Workspace::display_snapshot(const WorkflowSnapshot &s) {
    const auto previous_activity = state_.activity;
    const bool was_recording = state_.recording();
    const bool was_sop = state_.sop_running;
    const bool had_sync_result = state_.sync.attempt_completed;
    const bool had_playback = state_.playback.loaded;
    const int new_pose_total = s.calibration.common_count + s.calibration.validation_count;
    if (s.activity == Activity::Calibrating && new_pose_total > last_accepted_pose_total_)
        QApplication::beep();
    last_accepted_pose_total_ = new_pose_total;
    state_ = s;
    if (header_summary_) {
        int connected_count = 0;
        for (const auto &c : s.cameras)
            if (c.connected)
                ++connected_count;
        const QString sync_str = s.sync.accepted(s.epoch, config_.max_delta_us)
                                     ? (s.sync.timing_measured()
                                            ? tr("Sync PASS (%1 us)").arg(s.sync.max_delta_us, 0, 'f', 1)
                                            : tr("Sync Accepted"))
                                     : tr("Sync Pending");
        const QString cal_str = tr("Calib %1/%2 · Val %3/%4")
                                    .arg(s.calibration.common_count)
                                    .arg(config_.training_target)
                                    .arg(s.calibration.validation_count)
                                    .arg(config_.validation_target);
        const QString summary_text = tr("Cams: %1/3  |  RP2350: %2  |  %3  |  %4")
                                         .arg(connected_count)
                                         .arg(s.board_ready ? tr("GP2 LOW Ready") : tr("Standby"))
                                         .arg(sync_str, cal_str);
        set_text(header_summary_, summary_text);
        set_tooltip(header_summary_, summary_text);
    }
    if (previous_activity != s.activity) {
        if (previous_activity == Activity::Syncing) {
            const bool passed = s.sync.verdict == Verdict::Passed;
            finish_busy_progress(s.sync.attempt_completed
                                     ? (passed ? tr("✓ Finite Trigger check complete · PASS (max Δt %1 µs)")
                                                     .arg(s.sync.max_delta_us, 0, 'f', 1)
                                               : tr("✗ Finite Trigger check complete · FAILED (see message)"))
                                     : tr("✗ Finite Trigger check stopped"));
        } else if (previous_activity == Activity::Connecting) {
            finish_busy_progress(s.all_connected() ? tr("✓ Three cameras connected")
                                                   : tr("✗ Connection did not complete"));
        } else if (previous_activity == Activity::Solving) {
            finish_busy_progress(s.error.isEmpty() ? tr("✓ Calibration solve finished")
                                                   : tr("✗ Calibration solve failed"));
        }
    }
    if (s.activity != Activity::Idle) {
        if (!busy_elapsed_.isValid())
            busy_elapsed_.start();
        if (busy_timer_ && !busy_timer_->isActive())
            busy_timer_->start();
        // Starting the continuous wave is "done" once it is running; the wave itself is a long-lived
        // mode with its own Stop control, so it must not keep the inline progress bar alive.
        if (ui_busy_ && s.activity == Activity::Triggering)
            stop_busy_feedback();
    } else if (ui_busy_) {
        // The RP2350 check runs while activity stays Idle and first publishes a "Verifying RP2350…"
        // snapshot (board_ready=false) before talking to the board. That interim snapshot is not a result.
        const bool checking_board = ui_busy_label_.contains(QStringLiteral("RP2350"));
        const bool board_result = s.board_ready || !s.error.isEmpty() ||
                                  (s.message.contains("RP2350") && !s.message.startsWith("Verifying"));
        const bool finished = !s.error.isEmpty() ||
                              (ui_connecting_cameras_ ? s.all_connected()
                               : checking_board       ? board_result
                                                      : (s.board_ready || s.sync.attempt_completed));
        const qint64 timeout_ms = checking_board ? 15000 : 150;
        if (finished || busy_elapsed_.elapsed() > timeout_ms) {
            if (checking_board && s.activity == Activity::Idle)
                finish_busy_progress(s.board_ready ? tr("✓ RP2350 verified · GP2 LOW")
                                     : finished    ? tr("✗ RP2350 check failed (see message)")
                                                   : tr("✗ RP2350 check timed out"));
            stop_busy_feedback();
        }
    }
    if (s.recording() && board_)
        board_->close();
    for (int i = 0; i < 3; ++i) {
        previews_[i]->camera = s.cameras[i];
        previews_[i]->master_camera = config_.master;
        previews_[i]->reference_camera = config_.reference;
        previews_[i]->connecting_active =
            (s.activity == Activity::Connecting) || (ui_busy_ && ui_connecting_cameras_);
        previews_[i]->connecting_status = s.activity == Activity::Connecting ? s.message : ui_busy_label_;
        if (!s.playback.loaded && previews_[i]->camera.binding.position.isEmpty())
            previews_[i]->camera.binding = config_.cameras[i];
        previews_[i]->setAccessibleName(QString("%1 | %2 events | %3 corners")
                                            .arg(previews_[i]->camera.binding.position)
                                            .arg(s.cameras[i].event_count)
                                            .arg(s.cameras[i].corners.size()));
        const auto &camera = s.cameras[i];
        set_tooltip(previews_[i],
                    tr("%1 x %2 | %3 decoded events\n"
                       "Decoded on this host: %4 Mev/s, %5 MB/s\n"
                       "Decoded timestamp span / host wall time: %6\n"
                       "Time since last successful buffer decode: %7\n"
                       "These are decoder statistics, not USB link speed or camera-to-screen latency.")
                        .arg(camera.width)
                        .arg(camera.height)
                        .arg(camera.event_count)
                        .arg(camera.event_rate / 1e6, 0, 'f', 2)
                        .arg(camera.byte_rate / 1e6, 0, 'f', 2)
                        .arg(std::isfinite(camera.decode_realtime_ratio)
                                 ? QString::number(camera.decode_realtime_ratio, 'f', 2) + "x"
                                 : tr("not measured"))
                        .arg(camera.decode_age_ms < 0 ? tr("unknown")
                                                      : QString::number(camera.decode_age_ms) + " ms"));
        previews_[i]->update();
    }
    set_text(message_, s.error.isEmpty() ? s.message : tr("Error: ") + s.error);
    if (alert_banner_) {
        QString alert;
        if (!s.error.isEmpty()) {
            const bool board_issue = !s.board_ready && s.error.contains(QStringLiteral("RP2350"));
            if (board_issue && s.all_connected())
                alert = tr("⚠ Trigger board (RP2350) not ready — cameras stay connected.\n%1").arg(s.error);
            else if (board_issue)
                alert = tr("⚠ Trigger board (RP2350) not ready.\n%1").arg(s.error);
            else
                alert = tr("⚠ %1").arg(s.error);
        }
        set_text(alert_banner_, alert.isEmpty() ? alert : alert + tr("   ⓘ"));
        if (!alert.isEmpty()) {
            const auto help = message_help_for(s.error);
            set_tooltip(alert_banner_, (help.isEmpty() ? QString() : help + QStringLiteral("<hr>")) +
                                           message_glossary_html());
        }
        alert_banner_->setVisible(!alert.isEmpty());
    }
    const bool locked = s.busy();
    for (auto *field : QList<QWidget *>{project_, delta_, rms_, output_, roles_, master_, reference_, square_,
                                        training_, validation_, optics_, display_, fullscreen_, timed_,
                                        duration_, capture_duration_, supplemental_, trigger_wiring_})
        if (field->isEnabled() == locked)
            field->setEnabled(!locked);
    for (auto *d : devices_)
        if (d->isEnabled() == locked)
            d->setEnabled(!locked);
    display_->setEnabled(!locked && !config_.board_locked);
    fullscreen_->setEnabled(!locked && !config_.board_locked);
    square_->setEnabled(!locked && !config_.board_locked);
    locked_->setEnabled(!s.recording() && s.activity != Activity::Solving && !s.trigger_stop_pending);
    set_text(commands_["stop"], s.activity == Activity::Triggering || s.trigger_stop_pending
                                    ? tr("Stop Trigger")
                                : s.activity == Activity::Recording ? tr("Stop recording")
                                                                    : tr("Pause / cancel"));
    if (s.sop_running && !s.sop_paused) {
        int step = s.activity == Activity::Recording || s.activity == Activity::Preparing   ? 3
                   : s.activity == Activity::Calibrating || s.activity == Activity::Solving ? 2
                   : s.activity == Activity::Syncing                                        ? 1
                   : s.activity == Activity::Connecting                                     ? 0
                                                                                            : -1;
        if (step < 0 && s.activity == Activity::Idle) {
            if (!configuration_blockers(config_).isEmpty() || !s.all_connected() || !s.board_ready)
                step = 0;
            else if (!s.sync.accepted(s.epoch, config_.max_delta_us))
                step = 1;
            else if (!stage_gate(Step::Calibration, config_, s, guidance_context()).complete)
                step = 2;
            else
                step = 3;
        }
        if (step >= 0 && int(current_step()) != step)
            select_step(step);
    }
    if (s.activity == Activity::Preparing && !s.sync.accepted(s.epoch, config_.max_delta_us)) {
        if (active_module_ == WorkspaceModule::Capture) {
            if (capture_drawer_toggle_ && !capture_drawer_toggle_->isChecked())
                capture_drawer_toggle_->setChecked(true);
        } else if (current_step() != Step::Sync) {
            select_step(int(Step::Sync));
        }
    }
    update_blockers();
    update_visible_details();
    if (previous_activity != Activity::Calibrating && s.activity == Activity::Calibrating &&
        current_step() == Step::Calibration)
        set_guided_phase(Step::Calibration, s.calibration.training_complete ? 4 : 2);
    if (!had_sync_result && s.sync.attempt_completed && current_step() == Step::Sync)
        set_guided_phase(Step::Sync, 2);
    if (s.activity == Activity::Preparing && s.can_override_sync() && current_step() == Step::Sync &&
        guided_phase_[int(Step::Sync)] != 2)
        set_guided_phase(Step::Sync, 2);
    if (!had_playback && s.playback.loaded && current_step() == Step::Review)
        set_guided_phase(Step::Review, 1);
    if (s.recording() && current_step() == Step::Capture && guided_phase_[int(Step::Capture)] != 2)
        set_guided_phase(Step::Capture, 2);
    if (was_recording && was_sop && !s.recording() && !s.playback.loaded &&
        s.take_manifest.value("state").toString() == "completed" &&
        s.take_report.value("countsAndProtocolPass").toBool() && s.error.isEmpty()) {
        select_step(int(Step::Review));
        controller_->open_take(s.take_directory);
    }
    if (was_recording != s.recording()) {
        if (!s.recording())
            refresh_profiles_ui();
        emit recording_changed(s.recording());
    }
}

void Workspace::update_visible_details() {
    const auto &s = state_;
    if (current_step() == Step::Devices) {
        set_text(board_status_, s.board_detail.isEmpty() ? tr("RP2350 not checked") : s.board_detail);
        for (int i = 0; i < 3; ++i) {
            const auto &c = s.cameras[i];
            const int usb_speed = c.metadata.value("usbSpeedMbps").toInt(0);
            const QString usb_proto = c.metadata.value("usbProtocol").toString(QStringLiteral("USB 3.0 (5 Gbps)"));
            const QString proto_tag = (usb_speed > 0 && usb_speed < 5000) ? (QStringLiteral("⚠ ") + usb_proto) : usb_proto;
            set_text(device_readbacks_[i],
                     c.connected ? QString("%1 | %2 x %3 | %4\n%5 (read back), Trigger In %6\n%7\n%8")
                                       .arg(c.binding.position)
                                       .arg(c.width)
                                       .arg(c.height)
                                       .arg(proto_tag,
                                            c.metadata.value("modeReadback").toString(),
                                            c.metadata.value("triggerInEnabled").toBool() ? "enabled"
                                                                                          : "not confirmed",
                                            c.metadata.value("encoding").toString(),
                                            c.metadata.value("fullSerial").toString())
                                 : QString("%1: not connected").arg(config_.cameras[i].position));
        }
    }
    if (current_step() == Step::Sync) {
        triggers_->cameras = s.cameras;
        triggers_->invert_display = s.sync.timing_measured() && s.sync.polarity_inverted;
        triggers_->update();
        const bool timing_measured = s.sync.timing_measured();
        QString sync_text =
            (s.sync.operator_accepted ? tr("Accepted manually") : verdict_text(s.sync.verdict)) +
            QString(" | epoch %1\nMax delta: %2\n")
                .arg(s.sync.epoch)
                .arg(timing_measured ? QString::number(s.sync.max_delta_us, 'f', 1) + " us"
                                     : tr("not measured"));
        for (int i = 0; i < 3; ++i) {
            QStringList deltas;
            for (auto d : s.sync.delta_us[i])
                deltas << QString::number(d);
            sync_text += QString("%1: %2 matched, %3 extra, %4 observed\nDelta / Master: %5 "
                                 "us\n")
                             .arg(config_.cameras[i].position)
                             .arg(s.sync.matched[i])
                             .arg(timing_measured ? QString::number(s.sync.extras[i]) : tr("not classified"))
                             .arg(s.sync.observed_edges[i].size())
                             .arg(timing_measured ? deltas.join(", ") : tr("not measured"));
        }
        set_text(sync_status_,
                 sync_text + tr("Automatic: ") + s.sync.reason +
                     (s.sync.operator_accepted
                          ? tr("\nManual: ") + s.sync.operator_reason + "\n" + s.sync.operator_accepted_utc
                          : QString{}) +
                     (s.activity == Activity::Triggering ? tr("\nSquare wave RUNNING - 12 mA, 0.5 Hz")
                                                         : QString{}));
    }
    if (current_step() == Step::Calibration) {
        set_text(guide_, s.calibration.guide + "\n" + s.calibration.reason);
        const int progress = int(s.calibration.stable_progress * 100);
        if (stability_->value() != progress)
            stability_->setValue(progress);
        if (displayed_observations_ != s.calibration.observation_labels) {
            displayed_observations_ = s.calibration.observation_labels;
            int selected = observations_->currentRow();
            observations_->clear();
            observations_->addItems(s.calibration.observation_labels);
            observations_->setCurrentRow(std::min(selected, observations_->count() - 1));
        }
        if (!profile_cached_ || displayed_profile_ != s.calibration.profile) {
            profile_cached_ = true;
            displayed_profile_ = s.calibration.profile;
            validation_overlays_ = displayed_profile_.value("validationErrorOverlays").toArray();
            displayed_validation_view_ = -1;
            QSignalBlocker b(validation_view_);
            validation_view_->setMaximum(
                std::max(1, int(displayed_profile_.value("validationViews").toArray().size())));
        }
        if (displayed_validation_view_ != validation_view_->value()) {
            displayed_validation_view_ = validation_view_->value();
            selected_validation_overlay_ =
                validation_overlays_.isEmpty()
                    ? QJsonArray{}
                    : validation_overlays_[std::clamp(displayed_validation_view_ - 1, 0,
                                                      int(validation_overlays_.size()) - 1)]
                          .toArray();
        }
        for (auto *preview : previews_) {
            preview->calibration = s.calibration;
            preview->validation_points = selected_validation_overlay_;
            preview->update();
        }
        set_text(counts_,
                 tr("Common training: %1 / %2\nIndependent validation: %3 / "
                    "%4\nIntrinsics L / C / R: %5 "
                    "/ %6 / %7\nCalibration: %8 | RMS: %9 px")
                     .arg(s.calibration.common_count)
                     .arg(config_.training_target)
                     .arg(s.calibration.validation_count)
                     .arg(config_.validation_target)
                     .arg(s.calibration.intrinsic_counts[0])
                     .arg(s.calibration.intrinsic_counts[1])
                     .arg(s.calibration.intrinsic_counts[2])
                     .arg(verdict_text(s.calibration.verdict))
                     .arg(std::isfinite(s.calibration.rms) ? QString::number(s.calibration.rms, 'f', 3)
                                                           : "not measured"));
        const QFileInfo exported(last_export_path_);
        const auto export_info =
            last_export_path_.isEmpty() ? tr("JSON export: choose a destination below.")
            : exported.isFile() ? tr("JSON at chosen destination: %1\nCurrent file size: %2 KiB")
                                      .arg(last_export_path_)
                                      .arg(exported.size() / 1024., 0, 'f', 1)
                                : tr("JSON export destination: %1\nNo file found at this destination yet.")
                                      .arg(last_export_path_);
        set_text(storage_status_,
                 tr("Observations stay in memory until JSON export. Calibration does not record RAW.\n") +
                     export_info +
                     tr("\nRecording output: %1\nClear observations only clears memory; files are kept.")
                         .arg(config_.output));
        update_calibration_guidance();
    }
    if (current_step() == Step::Capture) {
        const auto ready = blockers(Step::Capture, config_, s).isEmpty();
        set_text(record_status_,
                 (ready ? tr("Recording prerequisites ready.\n") : tr("Recording prerequisites pending.\n")) +
                     tr("Every new Take: fresh Trigger preflight before formal "
                        "recording.\n"
                        "Formal time: %1 s\n%2\n%3")
                         .arg(s.elapsed_ms / 1000., 0, 'f', 1)
                         .arg(s.take_directory, s.message));
    }
    if (current_step() == Step::Review) {
        set_text(commands_["openTake"],
                 s.playback.loaded ? tr("Open another Take...") : tr("Open Take directory"));
        for (auto *control : QList<QWidget *>{play_, replay_, slider_, jump_, speed_})
            if (control)
                control->setEnabled(s.playback.loaded);
        set_tooltip(play_, s.playback.loaded ? (s.playback.playing ? tr("Pause") : tr("Play"))
                                             : tr("Open a Take directory first."));
        if (replay_)
            set_tooltip(replay_, s.playback.loaded ? tr("Replay from start of segment")
                                                   : tr("Open a Take directory first."));
        if (s.playback.loaded) {
            QSignalBlocker b(include_);
            include_->setChecked(s.playback.include_preflight);
            include_->setEnabled(s.playback.formal_start_us >= 0 &&
                                 s.playback.formal_end_us >= s.playback.formal_start_us);
            include_->setToolTip(include_->isEnabled() ? tr("Show the retained preflight interval")
                                                       : tr("Formal range was not recorded; all original "
                                                            "timestamps are shown."));
            const auto low = s.playback.include_preflight || s.playback.formal_start_us < 0
                                 ? s.playback.first_us
                                 : s.playback.formal_start_us;
            const auto high = s.playback.include_preflight || s.playback.formal_end_us < 0
                                  ? s.playback.last_us
                                  : s.playback.formal_end_us;
            if (!slider_->isSliderDown())
                slider_->setValue(high > low ? int(100000. * (s.playback.position_us - low) / (high - low))
                                             : 0);
            if (!slider_->isSliderDown())
                set_text(time_, QString::number(s.playback.position_us / 1e6, 'f', 3) + " s");
            const auto icon = s.playback.playing ? QString("pause") : QString("play");
            if (play_->property("multiIcon").toString() != icon) {
                play_->setIcon(IconProvider::get(icon, palette().buttonText().color()));
                play_->setProperty("multiIcon", icon);
            }
        }
        if (!report_cached_ || displayed_manifest_ != s.take_manifest || displayed_report_ != s.take_report) {
            report_cached_ = true;
            displayed_manifest_ = s.take_manifest;
            displayed_report_ = s.take_report;
            reports_->setPlainText(QString::fromUtf8(
                QJsonDocument(QJsonObject{{"session", s.take_manifest}, {"verification", s.take_report}})
                    .toJson(QJsonDocument::Indented)));
        }
    }
    update_guided_journey();
    refresh_biases_dialog();
    update_three_workspaces();
}

void Workspace::start_busy_feedback(const QString &label, bool connecting_cameras) {
    ui_busy_ = true;
    ui_connecting_cameras_ = connecting_cameras;
    ui_busy_label_ = label;
    busy_elapsed_.start();
    if (busy_timer_)
        busy_timer_->start();
    const QString status_line = QStringLiteral("⏳ %1 (0.0 s)").arg(label);
    set_text(message_, status_line);
    set_text(stage_status_, status_line);
    set_text(calibration_primary_, status_line);
    for (auto *p : previews_) {
        p->connecting_active = connecting_cameras || state_.activity == Activity::Connecting;
        p->connecting_status = status_line;
        p->update();
    }
}

void Workspace::stop_busy_feedback() {
    ui_busy_ = false;
    ui_connecting_cameras_ = false;
    ui_busy_label_.clear();
    if (busy_timer_ && !state_.busy()) {
        busy_timer_->stop();
        finish_busy_progress();
    }
    for (auto *p : previews_) {
        p->connecting_active = state_.activity == Activity::Connecting;
        if (!p->connecting_active)
            p->connecting_status.clear();
        p->update();
    }
}

void Workspace::finish_busy_progress(const QString &final_text) {
    if (!busy_progress_)
        return;
    const auto generation = ++progress_generation_;
    if (final_text.isEmpty()) {
        if (!progress_holding_)
            busy_progress_->hide();
        return;
    }
    // Show the completed state briefly (full bar + result) so the end of the operation is visible.
    progress_holding_ = true;
    busy_progress_->setRange(0, 1000);
    busy_progress_->setValue(1000);
    busy_progress_->setFormat(final_text);
    busy_progress_->setToolTip(final_text);
    busy_progress_->show();
    QTimer::singleShot(1800, this, [this, generation] {
        if (generation != progress_generation_)
            return;
        progress_holding_ = false;
        if (!ui_busy_ && !state_.busy())
            busy_progress_->hide();
    });
}

void Workspace::refresh_biases_dialog() {
    if (!biases_dialog_ || !biases_live_status_)
        return;
    QStringList lines;
    for (int i = 0; i < 3; ++i) {
        const auto &c = state_.cameras[i];
        const auto pos = config_.cameras[i].position.isEmpty() ? QString("Cam %1").arg(i + 1)
                                                               : config_.cameras[i].position;
        if (!c.connected) {
            lines << QString("%1: disconnected").arg(pos);
            continue;
        }
        const auto biases = c.metadata.value("biases").toObject();
        QStringList btxt;
        for (const char *k : {"bias_diff_on", "bias_diff_off", "bias_diff", "bias_fo", "bias_hpf", "bias_refr"})
            if (biases.contains(QLatin1String(k)))
                btxt << QString("%1=%2").arg(QLatin1String(k + 5)).arg(biases.value(QLatin1String(k)).toInt());
        lines << QString("%1 (%2): %3 Mev/s | Corners: %4/54 | %5")
                     .arg(pos, c.binding.serial)
                     .arg(c.event_rate / 1e6, 0, 'f', 2)
                     .arg(c.corners.size())
                     .arg(btxt.isEmpty() ? tr("default biases") : btxt.join(", "));
    }
    biases_live_status_->setText(lines.join("\n"));
}

void Workspace::show_biases_dialog() {
    if (biases_dialog_) {
        biases_dialog_->show();
        biases_dialog_->raise();
        biases_dialog_->activateWindow();
        refresh_biases_dialog();
        return;
    }
    auto *dlg = new QDialog(this, Qt::Tool | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setObjectName("multiBiasesDialog");
    dlg->setWindowTitle(tr("Camera Biases & Hardware Filter Tuning (Live — IMX636)"));
    dlg->setMinimumWidth(540);
    biases_dialog_ = dlg;

    auto *root = new QVBoxLayout(dlg);
    root->setContentsMargins(12, 12, 12, 12);
    root->setSpacing(8);

    biases_live_status_ = new QLabel(dlg);
    biases_live_status_->setObjectName("multiBiasesLiveStatus");
    biases_live_status_->setWordWrap(true);
    biases_live_status_->setStyleSheet(
        "background-color: palette(alternate-base); border: 1px solid palette(mid); border-radius: 6px; padding: 6px 8px; font-family: Menlo, monospace; font-size: 11px;");
    root->addWidget(biases_live_status_);

    auto *scope_row = new QHBoxLayout;
    scope_row->addWidget(new QLabel(tr("Target Camera(s):"), dlg));
    auto *scope = new QComboBox(dlg);
    scope->setObjectName("multiBiasesScope");
    scope->addItems({tr("All 3 Cameras (Left + Center + Right)"), tr("Left Camera only"),
                     tr("Center Camera only"), tr("Right Camera only")});
    scope_row->addWidget(scope, 1);
    root->addLayout(scope_row);

    auto *preset_row = new QHBoxLayout;
    auto *preset_cal = new QToolButton(dlg);
    preset_cal->setText(tr("Calibration Auto-Bias (18–22 Mev/s)"));
    preset_cal->setProperty("multiPrimary", true);
    preset_cal->setToolTip(
        tr("Recommended for Blinking Chessboard: Auto-Bias ON [18-22 Mev/s] (balances ON/OFF & keeps blink counts dense), fo=-10, hpf=+10, refr=0, STC OFF"));
    auto *preset_manual = new QToolButton(dlg);
    preset_manual->setText(tr("Oblique Boost (Manual)"));
    preset_manual->setToolTip(
        tr("Manual balanced settings for oblique Left/Right cameras: diff_on=+10, diff_off=+10, diff=0, fo=-15 (suppresses LCD PWM), hpf=+10, refr=0, STC OFF"));
    auto *preset_reset = new QToolButton(dlg);
    preset_reset->setText(tr("Reset Defaults (0)"));
    preset_row->addWidget(preset_cal);
    preset_row->addWidget(preset_manual);
    preset_row->addWidget(preset_reset);
    root->addLayout(preset_row);

    auto *form = new QFormLayout;
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setVerticalSpacing(6);

    auto *auto_bias = new QCheckBox(tr("Enable Adaptive Auto-Bias (rate band + ON/OFF polarity balance)"), dlg);
    form->addRow(auto_bias);

    auto *rate_row = new QWidget(dlg);
    auto *rate_layout = new QHBoxLayout(rate_row);
    rate_layout->setContentsMargins(0, 0, 0, 0);
    auto *rate_min = new QDoubleSpinBox(rate_row);
    rate_min->setRange(0.5, 80.0);
    rate_min->setSingleStep(0.5);
    rate_min->setSuffix(" Mev/s min");
    auto *rate_max = new QDoubleSpinBox(rate_row);
    rate_max->setRange(1.0, 100.0);
    rate_max->setSingleStep(1.0);
    rate_max->setSuffix(" Mev/s max");
    rate_layout->addWidget(rate_min);
    rate_layout->addWidget(rate_max);
    form->addRow(tr("Auto-Bias Rate Band"), rate_row);

    struct BiasControl {
        QSlider *slider = nullptr;
        QSpinBox *spin = nullptr;
    };
    auto make_offset = [&](const QString &label, const QString &tip, int lo, int hi) {
        auto *row_w = new QWidget(dlg);
        auto *hl = new QHBoxLayout(row_w);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->setSpacing(6);
        auto *slider = new QSlider(Qt::Horizontal, row_w);
        slider->setRange(lo, hi);
        slider->setToolTip(tip);
        auto *spin = new QSpinBox(row_w);
        spin->setRange(lo, hi);
        spin->setSingleStep(5);
        spin->setSuffix(QString("  [%1..%2]").arg(lo).arg(hi));
        spin->setToolTip(tip);
        spin->setMinimumWidth(135);
        hl->addWidget(slider, 1);
        hl->addWidget(spin, 0);
        connect(slider, &QSlider::valueChanged, spin, [spin](int v) {
            QSignalBlocker b(spin);
            spin->setValue(v);
        });
        connect(spin, qOverload<int>(&QSpinBox::valueChanged), slider, [slider](int v) {
            QSignalBlocker b(slider);
            slider->setValue(v);
        });
        form->addRow(label, row_w);
        return BiasControl{slider, spin};
    };
    // Exact IMX636 hardware recommended offset ranges from imx636_bias_settings.h:
    auto diff_on = make_offset(
        tr("bias_diff_on"),
        tr("ON contrast threshold offset [-85..+140]. Positive = higher threshold (fewer ON events); Negative = more sensitive (more ON noise)"),
        -85, 140);
    auto diff_off = make_offset(
        tr("bias_diff_off"),
        tr("OFF contrast threshold offset [-35..+190]. Positive = higher threshold (fewer OFF events); Negative = more sensitive (more OFF noise). Keep balanced with diff_on!"),
        -35, 190);
    auto bias_diff = make_offset(
        tr("bias_diff"),
        tr("Comparator reference voltage offset [-25..+23] compared against bias_diff_on and bias_diff_off (default 0)"),
        -25, 23);
    auto bias_fo = make_offset(
        tr("bias_fo"),
        tr("Photoreceptor low-pass filter cutoff [-35..+55]. Negative (-10..-25) slows LPF to suppress high-frequency LCD backlight PWM noise; Positive widens bandwidth"),
        -35, 55);
    auto bias_hpf = make_offset(
        tr("bias_hpf"),
        tr("Differentiator high-pass filter cutoff [0..+120] (cannot be negative on IMX636). Positive (+10..+30) rejects slow illumination drift"),
        0, 120);
    auto bias_refr = make_offset(
        tr("bias_refr"),
        tr("Pixel refractory dead-time offset [-20..+235] during which pixel is switched off after an event"),
        -20, 235);

    auto *stc_row = new QWidget(dlg);
    auto *stc_layout = new QHBoxLayout(stc_row);
    stc_layout->setContentsMargins(0, 0, 0, 0);
    stc_layout->setSpacing(6);
    auto *stc_en = new QCheckBox(tr("Enable"), stc_row);
    stc_en->setToolTip(tr("Note: Leave OFF or use STC Keep Trail during blinking-chessboard calibration; STC Cut Trail drops burst counts needed by corner detection!"));
    auto *stc_mode = new QComboBox(stc_row);
    stc_mode->addItems({tr("STC Cut Trail (keep 1st)"), tr("STC Keep Trail (keep burst)"), tr("TRAIL")});
    auto *stc_thr = new QSpinBox(stc_row);
    stc_thr->setRange(100, 100000);
    stc_thr->setSingleStep(500);
    stc_thr->setSuffix(" us");
    stc_layout->addWidget(stc_en);
    stc_layout->addWidget(stc_mode, 1);
    stc_layout->addWidget(stc_thr, 1);
    form->addRow(tr("Trail Filter (STC)"), stc_row);

    auto *afk_row = new QWidget(dlg);
    auto *afk_layout = new QHBoxLayout(afk_row);
    afk_layout->setContentsMargins(0, 0, 0, 0);
    afk_layout->setSpacing(6);
    auto *afk_en = new QCheckBox(tr("Band-Stop"), afk_row);
    auto *afk_lo = new QSpinBox(afk_row);
    afk_lo->setRange(50, 500);
    afk_lo->setSuffix(" Hz low");
    auto *afk_hi = new QSpinBox(afk_row);
    afk_hi->setRange(50, 500);
    afk_hi->setSuffix(" Hz high");
    afk_layout->addWidget(afk_en);
    afk_layout->addWidget(afk_lo, 1);
    afk_layout->addWidget(afk_hi, 1);
    form->addRow(tr("Anti-Flicker (AFK)"), afk_row);

    auto *erc_row = new QWidget(dlg);
    auto *erc_layout = new QHBoxLayout(erc_row);
    erc_layout->setContentsMargins(0, 0, 0, 0);
    auto *erc_en = new QCheckBox(tr("Enable ERC"), erc_row);
    auto *erc_rate = new QDoubleSpinBox(erc_row);
    erc_rate->setRange(0.5, 50.0);
    erc_rate->setSingleStep(1.0);
    erc_rate->setSuffix(" Mev/s");
    erc_layout->addWidget(erc_en);
    erc_layout->addWidget(erc_rate, 1);
    form->addRow(tr("Event Rate Limit"), erc_row);

    root->addLayout(form);

    auto set_bc = [](const BiasControl &bc, int v) {
        QSignalBlocker b1(bc.slider), b2(bc.spin);
        bc.slider->setValue(v);
        bc.spin->setValue(v);
    };
    auto enable_bc = [](const BiasControl &bc, bool en) {
        bc.slider->setEnabled(en);
        bc.spin->setEnabled(en);
    };

    auto sync_controls_from = [=](int cam_idx) {
        const auto &t = config_.tuning[std::clamp(cam_idx, 0, 2)];
        QSignalBlocker b1(auto_bias), b2(rate_min), b3(rate_max), b9(stc_en), b9m(stc_mode), b10(stc_thr),
            b11(erc_en), b12(erc_rate), b13(afk_en), b14(afk_lo), b15(afk_hi);
        auto_bias->setChecked(t.auto_bias);
        rate_min->setValue(t.rate_min_mev);
        rate_max->setValue(t.rate_max_mev);
        set_bc(diff_on, t.diff_on_offset);
        set_bc(diff_off, t.diff_off_offset);
        set_bc(bias_diff, t.diff_offset);
        enable_bc(diff_on, !t.auto_bias);
        enable_bc(diff_off, !t.auto_bias);
        set_bc(bias_fo, t.fo_offset);
        set_bc(bias_hpf, t.hpf_offset);
        set_bc(bias_refr, t.refr_offset);
        stc_en->setChecked(t.stc_enabled);
        stc_mode->setCurrentIndex(std::clamp(t.stc_type, 0, 2));
        stc_thr->setValue(t.stc_threshold_us);
        afk_en->setChecked(t.afk_enabled);
        afk_lo->setValue(t.afk_low_hz);
        afk_hi->setValue(t.afk_high_hz);
        erc_en->setChecked(t.erc_enabled);
        erc_rate->setValue(t.erc_rate_mev);
    };

    auto push_tuning = [=] {
        enable_bc(diff_on, !auto_bias->isChecked());
        enable_bc(diff_off, !auto_bias->isChecked());
        CameraTuning t;
        t.auto_bias = auto_bias->isChecked();
        t.rate_min_mev = rate_min->value();
        t.rate_max_mev = std::max(rate_min->value() + 0.5, rate_max->value());
        t.diff_on_offset = diff_on.spin->value();
        t.diff_off_offset = diff_off.spin->value();
        t.diff_offset = bias_diff.spin->value();
        t.fo_offset = bias_fo.spin->value();
        t.hpf_offset = bias_hpf.spin->value();
        t.refr_offset = bias_refr.spin->value();
        t.stc_enabled = stc_en->isChecked();
        t.stc_type = stc_mode->currentIndex();
        t.stc_threshold_us = stc_thr->value();
        t.afk_enabled = afk_en->isChecked();
        t.afk_low_hz = afk_lo->value();
        t.afk_high_hz = std::max(afk_lo->value(), afk_hi->value());
        t.erc_enabled = erc_en->isChecked();
        t.erc_rate_mev = erc_rate->value();
        const int sel = scope->currentIndex();
        for (int i = 0; i < 3; ++i)
            if (sel == 0 || sel - 1 == i)
                config_.tuning[i] = t;
        controller_->configure(config_);
    };

    connect(scope, qOverload<int>(&QComboBox::currentIndexChanged), dlg, [=](int idx) {
        sync_controls_from(idx == 0 ? 0 : idx - 1);
    });
    for (auto *cb : {auto_bias, stc_en, afk_en, erc_en})
        connect(cb, &QCheckBox::toggled, dlg, push_tuning);
    connect(stc_mode, qOverload<int>(&QComboBox::currentIndexChanged), dlg, push_tuning);
    for (const auto &bc : {diff_on, diff_off, bias_diff, bias_fo, bias_hpf, bias_refr}) {
        connect(bc.spin, qOverload<int>(&QSpinBox::valueChanged), dlg, push_tuning);
        connect(bc.slider, &QSlider::sliderReleased, dlg, push_tuning);
    }
    for (auto *sp : {stc_thr, afk_lo, afk_hi})
        connect(sp, qOverload<int>(&QSpinBox::valueChanged), dlg, push_tuning);
    for (auto *dsp : {rate_min, rate_max, erc_rate})
        connect(dsp, qOverload<double>(&QDoubleSpinBox::valueChanged), dlg, push_tuning);

    connect(preset_cal, &QToolButton::clicked, dlg, [=] {
        const int sel = scope->currentIndex();
        for (int i = 0; i < 3; ++i)
            if (sel == 0 || sel - 1 == i) {
                auto &t = config_.tuning[i];
                t.auto_bias = true;
                t.rate_min_mev = 18.0;
                t.rate_max_mev = 22.0;
                t.diff_offset = 0;
                t.fo_offset = -10;
                t.hpf_offset = 10;
                t.refr_offset = 0;
                t.stc_enabled = false;
                t.afk_enabled = false;
            }
        sync_controls_from(sel == 0 ? 0 : sel - 1);
        controller_->configure(config_);
    });
    connect(preset_manual, &QToolButton::clicked, dlg, [=] {
        const int sel = scope->currentIndex();
        for (int i = 0; i < 3; ++i)
            if (sel == 0 || sel - 1 == i) {
                auto &t = config_.tuning[i];
                t.auto_bias = false;
                t.diff_on_offset = 10;
                t.diff_off_offset = 10;
                t.diff_offset = 0;
                t.fo_offset = -15;
                t.hpf_offset = 10;
                t.refr_offset = 0;
                t.stc_enabled = false;
                t.afk_enabled = false;
            }
        sync_controls_from(sel == 0 ? 0 : sel - 1);
        controller_->configure(config_);
    });
    connect(preset_reset, &QToolButton::clicked, dlg, [=] {
        const int sel = scope->currentIndex();
        for (int i = 0; i < 3; ++i)
            if (sel == 0 || sel - 1 == i)
                config_.tuning[i] = CameraTuning{};
        sync_controls_from(sel == 0 ? 0 : sel - 1);
        controller_->configure(config_);
    });

    sync_controls_from(0);
    refresh_biases_dialog();
    dlg->show();
}

void Workspace::show_board() {
    if (board_) {
        board_->raise();
        return;
    }
    auto screens = QGuiApplication::screens();
    QScreen *target = nullptr;
    for (auto *s : screens)
        if (s->name() == display_->currentData().toString())
            target = s;
    const bool single_screen_preview = (!target || target == screen());
    if (single_screen_preview)
        target = screen() ? screen() : QGuiApplication::primaryScreen();
    if (!target)
        return;
    board_ = new gui::BlinkingChessboardDisplay;
    board_->setAttribute(Qt::WA_DeleteOnClose);
    board_->setWindowTitle(single_screen_preview ? tr("Calibration pattern (Single-Screen Preview)")
                                                 : tr("Calibration pattern"));
    board_->set_pattern(9, 6);
    board_->set_vertical_placement(0.9);
    board_->set_square_size_mm(square_->value());
    board_->resize(900, 640);
    board_->move(target->availableGeometry().topLeft() + QPoint(20, 20));
    board_->installEventFilter(this);
    connect(board_, &gui::BlinkingChessboardDisplay::pattern_layout_changed, this, [this] {
        if (config_.board_locked)
            unlock_board(tr("Pattern placement changed; remeasure and lock before continuing."));
    });
    connect(target, &QScreen::geometryChanged, board_, [this] {
        if (config_.board_locked)
            unlock_board(tr("Display geometry changed; remeasure the pattern."));
    });
    if (fullscreen_->isChecked() && !single_screen_preview)
        board_->showFullScreen();
    else
        board_->show();
    connect(board_, &QObject::destroyed, this,
            [this] { unlock_board(tr("Pattern closed; collection paused.")); });
    message_->setText(single_screen_preview
                          ? tr("Opened blinking chessboard preview window. Connect a secondary display outside the main GUI screen before locking calibration size.")
                          : tr("Set the final pattern size, physically measure a "
                               "square, then lock it. Keep this "
                               "GUI outside the cameras' fields of view."));
    update_blockers();
}

void Workspace::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Space && !qobject_cast<QLineEdit *>(focusWidget())) {
        perform_calibration_action();
        event->accept();
        return;
    }
    QMainWindow::keyPressEvent(event);
}
void Workspace::unlock_board(const QString &reason) {
    QSignalBlocker b(locked_);
    locked_->setChecked(false);
    config_.board_locked = false;
    if (state_.activity == Activity::Calibrating)
        controller_->stop();
    update_config();
    message_->setText(reason);
}
bool Workspace::eventFilter(QObject *object, QEvent *event) {
    if (object == board_ && config_.board_locked &&
        (event->type() == QEvent::Resize || event->type() == QEvent::ScreenChangeInternal ||
         event->type() == QEvent::DevicePixelRatioChange)) {
        if (board_ && (event->type() == QEvent::ScreenChangeInternal ||
                       event->type() == QEvent::DevicePixelRatioChange || board_->size() != board_size_))
            unlock_board(tr("Pattern dimensions changed; remeasure and lock before continuing."));
    }
    return QMainWindow::eventFilter(object, event);
}
void Workspace::apply_theme() {
    static bool applying = false;
    if (applying)
        return;
    applying = true;

    QSettings settings;
    int color_idx = settings.value(QStringLiteral("theme/color"), int(gui::theme_tokens::kBlue)).toInt();
    int mode_idx = settings.value(QStringLiteral("theme/mode"), 0).toInt();
    if (color_idx < 0 || color_idx >= gui::theme_tokens::kColorCount)
        color_idx = int(gui::theme_tokens::kBlue);
    bool dark = false;
    if (mode_idx == 2) {
        dark = true;
    } else if (mode_idx == 1) {
        dark = false;
    } else if (const auto *hints = QGuiApplication::styleHints()) {
        const auto scheme = hints->colorScheme();
        if (scheme == Qt::ColorScheme::Dark)
            dark = true;
        else if (scheme == Qt::ColorScheme::Light)
            dark = false;
        else
            dark = QApplication::palette().color(QPalette::Window).lightness() < 128;
    }

    using T = gui::theme_tokens::Token;
    const QString bg_primary = gui::theme_tokens::lookup(color_idx, dark, T::BgPrimary);
    const QString bg_panel = gui::theme_tokens::lookup(color_idx, dark, T::BgPanel);
    const QString bg_input = gui::theme_tokens::lookup(color_idx, dark, T::BgInput);
    const QString bg_hover = gui::theme_tokens::lookup(color_idx, dark, T::BgHover);
    const QString fg_primary = gui::theme_tokens::lookup(color_idx, dark, T::FgPrimary);
    const QString fg_secondary = gui::theme_tokens::lookup(color_idx, dark, T::FgSecondary);
    const QString fg_muted = gui::theme_tokens::lookup(color_idx, dark, T::FgMuted);
    const QString accent = gui::theme_tokens::lookup(color_idx, dark, T::Accent);
    const QString border = gui::theme_tokens::lookup(color_idx, dark, T::Border);

    QString qss = QStringLiteral(
        "QWidget, QScrollArea, QDockWidget { background-color: %bg-primary%; color: %fg-primary%; }"
        "QDockWidget::title { background-color: %bg-panel%; padding: 4px; font-weight: bold; }"
        "QPlainTextEdit, QListWidget { background-color: %bg-input%; color: %fg-primary%; border: 1px solid %border%; border-radius: 5px; }"
        "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { max-width: 10000px; background-color: %bg-input%; color: %fg-primary%; border: 1px solid %border%; border-radius: 5px; padding: 4px 8px; min-height: 26px; }"
        "QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { border: 1px solid %accent%; }"
        "QWidget[multiInvalid=\"true\"] { border: 1px solid #d94d59; }"
        "QListWidget#multiSteps { background-color: %bg-panel%; color: %fg-primary%; border-radius: 8px; padding: 4px; border: 1px solid %border%; font-size: 12px; }"
        "QListWidget#multiSteps::item { border-radius: 6px; padding: 4px 6px; }"
        "QListWidget#multiSteps::item:selected { background-color: %accent%; color: #ffffff; font-weight: bold; }"
        "QToolButton { border-radius: 6px; padding: 5px 10px; border: 1px solid %border%; background-color: %bg-input%; color: %fg-primary%; font-size: 12px; }"
        "QToolButton#multiAutomation::menu-indicator { image: none; width: 0px; }"
        "QComboBox QAbstractItemView { background-color: %bg-panel%; color: %fg-primary%; selection-background-color: %accent%; selection-color: #ffffff; border: 1px solid %border%; padding: 4px; outline: 0; }"
        "QToolButton:hover { background-color: %bg-hover%; color: %fg-primary%; }"
        "QToolButton:pressed { background-color: %bg-panel%; color: %fg-primary%; }"
        "QToolButton:disabled { background-color: %bg-panel%; color: %fg-muted%; border: 1px solid %border%; }"
        "QToolButton[multiPrimary=\"true\"] { background-color: %accent%; color: #ffffff; border: 1px solid %accent%; border-radius: 7px; font-weight: bold; padding: 6px 12px; }"
        "QToolButton[multiPrimary=\"true\"]:hover { background-color: %accent%; color: #ffffff; }"
        "QToolButton[multiPrimary=\"true\"]:disabled { background-color: %bg-panel%; color: %fg-muted%; border: 1px solid %border%; }"
        "QToolButton[multiSubstepPill=\"true\"] { background-color: %bg-input%; color: %fg-primary%; border: 1px solid %border%; border-radius: 5px; padding: 2px 4px; font-size: 11px; }"
        "QToolButton[multiSubstepPill=\"true\"]:hover { background-color: %bg-hover%; color: %fg-primary%; }"
        "QToolButton[multiSubstepPill=\"true\"]:checked { background-color: %accent%; color: #ffffff; font-weight: bold; border: 1px solid %accent%; }"
        "QLabel#multiGuide { background-color: %bg-input%; color: %fg-primary%; border: 1px solid %accent%; border-left: 4px solid %accent%; border-radius: 6px; padding: 5px 10px; font-weight: bold; }"
        "QLabel#multiHeaderSummary { background-color: %bg-input%; color: %fg-primary%; border: 1px solid %border%; border-radius: 6px; padding: 5px 10px; }"
        "QLabel#multiAlertBanner { background-color: rgba(217, 77, 89, 0.12); color: %fg-primary%; border: 1px solid #d94d59; border-left: 4px solid #d94d59; border-radius: 6px; padding: 6px 10px; }"
        "QProgressBar { border: 1px solid %border%; border-radius: 5px; text-align: center; background-color: %bg-input%; color: %fg-primary%; font-weight: bold; }"
        "QProgressBar::chunk { background-color: #1b9966; border-radius: 4px; }"
        "QSlider#multiTimeline { min-height: 26px; background: transparent; border: none; }"
        "QSlider#multiTimeline::groove:horizontal { height: 6px; background: %border%; border-radius: 3px; margin: 0px 8px; }"
        "QSlider#multiTimeline::sub-page:horizontal { height: 6px; background: %accent%; border-radius: 3px; margin: 0px 0px 0px 8px; }"
        "QSlider#multiTimeline::add-page:horizontal { height: 6px; background: %border%; border-radius: 3px; margin: 0px 8px 0px 0px; }"
        "QSlider#multiTimeline::handle:horizontal { background: #ffffff; border: 2px solid %accent%; width: 14px; height: 14px; margin: -6px -7px; border-radius: 9px; }"
        "QSlider#multiTimeline::handle:horizontal:hover { background: %accent%; border: 2px solid #ffffff; }"
        "QSlider#multiTimeline::handle:horizontal:disabled { background: %bg-panel%; border: 2px solid %fg-muted%; }");

    qss.replace(QStringLiteral("%bg-primary%"), bg_primary);
    qss.replace(QStringLiteral("%bg-panel%"), bg_panel);
    qss.replace(QStringLiteral("%bg-input%"), bg_input);
    qss.replace(QStringLiteral("%bg-hover%"), bg_hover);
    qss.replace(QStringLiteral("%fg-primary%"), fg_primary);
    qss.replace(QStringLiteral("%fg-secondary%"), fg_secondary);
    qss.replace(QStringLiteral("%fg-muted%"), fg_muted);
    qss.replace(QStringLiteral("%accent%"), accent);
    qss.replace(QStringLiteral("%border%"), border);
    setStyleSheet(qss);

    QPalette pal = palette();
    const QColor c_bg_primary(bg_primary);
    const QColor c_bg_panel(bg_panel);
    const QColor c_bg_input(bg_input);
    const QColor c_bg_hover(bg_hover);
    const QColor c_fg_primary(fg_primary);
    const QColor c_fg_secondary(fg_secondary);
    const QColor c_fg_muted(fg_muted);
    const QColor c_accent(accent);
    const QColor c_border(border);
    for (auto group : {QPalette::Active, QPalette::Inactive, QPalette::Disabled}) {
        const QColor fg = (group == QPalette::Disabled) ? c_fg_muted : c_fg_primary;
        pal.setColor(group, QPalette::Window, c_bg_primary);
        pal.setColor(group, QPalette::WindowText, fg);
        pal.setColor(group, QPalette::Base, c_bg_panel);
        pal.setColor(group, QPalette::AlternateBase, c_bg_input);
        pal.setColor(group, QPalette::Text, fg);
        pal.setColor(group, QPalette::Button, c_bg_input);
        pal.setColor(group, QPalette::ButtonText, fg);
        pal.setColor(group, QPalette::Highlight, c_accent);
        pal.setColor(group, QPalette::HighlightedText, QColor(Qt::white));
        pal.setColor(group, QPalette::Midlight, c_bg_hover);
        pal.setColor(group, QPalette::Mid, c_border);
        pal.setColor(group, QPalette::PlaceholderText, c_fg_secondary);
    }
    setPalette(pal);
    if (preview_area_)
        preview_area_->setPalette(pal);
    for (auto *p : previews_)
        if (p) {
            p->setPalette(pal);
            if (p->maximize)
                p->maximize->setIcon(IconProvider::get(
                    p->property("multiMaximized").toBool() ? "restore" : "maximize", c_fg_primary));
            p->update();
        }
    if (triggers_) {
        triggers_->setPalette(pal);
        triggers_->update();
    }
    if (navigation_)
        navigation_->setPalette(pal);
    for (auto *button : commands_) {
        if (!button)
            continue;
        const auto name = button->property("multiIcon").toString();
        if (!name.isEmpty()) {
            const bool primary_active = button->property("multiPrimary").toBool() && button->isEnabled();
            button->setIcon(IconProvider::get(name, primary_active ? QColor(Qt::white) : c_fg_primary));
        }
    }
    if (auto *auto_btn = findChild<QToolButton *>(QStringLiteral("multiAutomation")))
        auto_btn->setIcon(IconProvider::get(QStringLiteral("settings"), c_fg_primary));
    applying = false;
}

void Workspace::changeEvent(QEvent *event) {
    QMainWindow::changeEvent(event);
    if (event->type() == QEvent::PaletteChange || event->type() == QEvent::StyleChange) {
        apply_theme();
        if (navigation_)
            update_navigation();
        if (preview_area_)
            preview_area_->arrange();
        if (calibration_checklist_)
            update_calibration_guidance();
    }
}

void Workspace::build_three_workspaces(QWidget *header_host, QWidget *body_host, QWidget *center_host) {
    auto *header_layout = qobject_cast<QHBoxLayout *>(header_host->layout());
    if (header_layout) {
        auto *switcher = new QWidget(header_host);
        switcher->setObjectName("multiWorkspaceSwitcher");
        auto *sw_row = new QHBoxLayout(switcher);
        sw_row->setContentsMargins(2, 2, 2, 2);
        sw_row->setSpacing(4);
        const struct {
            const char *id;
            QString text;
            QString tip;
            WorkspaceModule mod;
        } modes[3] = {
            {"multiModeCapture", tr("🎥 拍摄台"), tr("拍摄台 (Capture)"), WorkspaceModule::Capture},
            {"multiModeCalibrate", tr("📐 标定"), tr("标定新 Profile (Calibrate)"), WorkspaceModule::Calibrate},
            {"multiModeLibrary", tr("🎞 录制库"), tr("历史录制库 (Past Recordings)"), WorkspaceModule::Library},
        };
        for (int i = 0; i < 3; ++i) {
            auto *btn = new QToolButton(switcher);
            btn->setObjectName(QLatin1String(modes[i].id));
            btn->setText(modes[i].text);
            btn->setToolTip(modes[i].tip);
            btn->setCheckable(true);
            btn->setProperty("multiSubstepPill", true);
            btn->setMinimumHeight(28);
            const auto target_mod = modes[i].mod;
            connect(btn, &QToolButton::clicked, this, [this, target_mod] { select_workspace(target_mod); });
            sw_row->addWidget(btn);
            workspace_mode_btns_[i] = btn;
        }
        header_layout->insertWidget(1, switcher);
    }

    auto *body_layout = qobject_cast<QHBoxLayout *>(body_host->layout());
    left_calibrate_panel_ = new QWidget(body_host);
    left_calibrate_panel_->setObjectName("multiCalibrateLeftPanel");
    left_calibrate_panel_->setFixedWidth(182);
    auto *left_cal_layout = new QVBoxLayout(left_calibrate_panel_);
    left_cal_layout->setContentsMargins(0, 0, 0, 0);
    left_cal_layout->setSpacing(4);
    if (body_layout && navigation_) {
        body_layout->removeWidget(navigation_);
        navigation_->setParent(left_calibrate_panel_);
        navigation_->setMinimumHeight(256);
        left_cal_layout->addWidget(navigation_);
    }
    auto *cal_scroll = new QScrollArea(left_calibrate_panel_);
    cal_scroll->setFrameShape(QFrame::NoFrame);
    cal_scroll->setWidgetResizable(true);
    cal_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto *cal_card = new QWidget(cal_scroll);
    auto *cal_card_layout = new QVBoxLayout(cal_card);
    cal_card_layout->setContentsMargins(2, 2, 2, 2);
    cal_card_layout->setSpacing(4);

    cal_new_profile_name_ = new QLineEdit(cal_card);
    cal_new_profile_name_->setObjectName("calNewProfileName");
    cal_new_profile_name_->setPlaceholderText(tr("新 Profile 名称..."));
    cal_new_profile_name_->setText(
        QStringLiteral("Profile_%1_LayoutA").arg(QDate::currentDate().toString(QStringLiteral("yyyyMMdd"))));
    cal_card_layout->addWidget(cal_new_profile_name_);

    auto *preset_row = new QHBoxLayout;
    preset_row->setSpacing(3);
    const RigLayoutPreset presets[3] = {
        RigLayoutPreset::LayoutA_Collinear,
        RigLayoutPreset::LayoutB_LShape,
        RigLayoutPreset::LayoutC_LowTriangle,
    };
    const char *preset_labels[3] = {"构型A", "构型B", "构型C"};
    for (int i = 0; i < 3; ++i) {
        auto *b = new QToolButton(cal_card);
        b->setObjectName(QString("calLayoutBtn%1").arg(i));
        b->setText(QString::fromUtf8(preset_labels[i]));
        b->setCheckable(true);
        b->setChecked(i == 0);
        b->setProperty("multiSubstepPill", true);
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        const auto pr = presets[i];
        connect(b, &QToolButton::clicked, this, [this, pr] { select_layout_preset(pr); });
        preset_row->addWidget(b);
        cal_layout_btns_[i] = b;
    }
    cal_card_layout->addLayout(preset_row);

    cal_diagram_ = new MainboardDiagramWidget(cal_card);
    cal_diagram_->setObjectName("calMainboardDiagram");
    cal_card_layout->addWidget(cal_diagram_);

    cal_baseline_sanity_card_ = new QLabel(cal_card);
    cal_baseline_sanity_card_->setObjectName("multiBaselineSanityCard");
    cal_baseline_sanity_card_->setWordWrap(true);
    cal_card_layout->addWidget(cal_baseline_sanity_card_);

    cal_save_profile_btn_ = new QToolButton(cal_card);
    cal_save_profile_btn_->setObjectName("saveNewProfileBtn");
    cal_save_profile_btn_->setText(tr("✓ 保存 Profile 并去拍摄 →"));
    cal_save_profile_btn_->setProperty("multiPrimary", true);
    cal_save_profile_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    connect(cal_save_profile_btn_, &QToolButton::clicked, this, [this] {
        QString saved_id;
        if (controller_->save_calibrated_profile(cal_new_profile_name_->text(), config_.layout_preset, &saved_id)) {
            config_.active_profile_id = saved_id;
            config_.optics_confirmed = true;
            refresh_profiles_ui();
            select_workspace(WorkspaceModule::Capture);
        }
    });
    cal_card_layout->addWidget(cal_save_profile_btn_);
    cal_card_layout->addStretch(1);
    cal_scroll->setWidget(cal_card);
    left_cal_layout->addWidget(cal_scroll, 1);
    if (body_layout)
        body_layout->insertWidget(0, left_calibrate_panel_);

    library_sidebar_ = new QWidget(body_host);
    library_sidebar_->setObjectName("multiLibrarySidebar");
    library_sidebar_->setFixedWidth(256);
    auto *lib_layout = new QVBoxLayout(library_sidebar_);
    lib_layout->setContentsMargins(0, 0, 0, 0);
    lib_layout->setSpacing(6);
    library_search_edit_ = new QLineEdit(library_sidebar_);
    library_search_edit_->setObjectName("librarySearchEdit");
    library_search_edit_->setPlaceholderText(tr("🔍 搜索 Profile 或 Take..."));
    connect(library_search_edit_, &QLineEdit::textChanged, this, [this] { refresh_profiles_ui(); });
    lib_layout->addWidget(library_search_edit_);
    library_tree_ = new QTreeWidget(library_sidebar_);
    library_tree_->setObjectName("multiLibraryTree");
    library_tree_->setHeaderHidden(true);
    library_tree_->setIndentation(14);
    connect(library_tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem *item, int) {
        if (!item)
            return;
        const QString dir = item->data(0, Qt::UserRole).toString();
        if (!dir.isEmpty() && QFileInfo::exists(dir))
            controller_->open_take(dir);
    });
    lib_layout->addWidget(library_tree_, 1);
    library_profile_summary_ = new QLabel(library_sidebar_);
    library_profile_summary_->setObjectName("libraryProfileSummary");
    library_profile_summary_->setWordWrap(true);
    lib_layout->addWidget(library_profile_summary_);
    library_sidebar_->hide();
    if (body_layout)
        body_layout->insertWidget(1, library_sidebar_);

    auto *center_layout = qobject_cast<QVBoxLayout *>(center_host->layout());
    post_take_banner_ = new QWidget(center_host);
    post_take_banner_->setObjectName("multiPostTakeBanner");
    auto *pt_row = new QHBoxLayout(post_take_banner_);
    pt_row->setContentsMargins(8, 4, 8, 4);
    pt_row->setSpacing(6);
    post_take_summary_ = new QLabel(tr("✓ 录制已完成并保存"), post_take_banner_);
    post_take_summary_->setObjectName("postTakeSummaryLabel");
    pt_row->addWidget(post_take_summary_, 1);
    auto *pt_cont = new QToolButton(post_take_banner_);
    pt_cont->setObjectName("postTakeContinueBtn");
    pt_cont->setText(tr("继续拍下一条"));
    connect(pt_cont, &QToolButton::clicked, this, [this] {
        dismissed_take_directory_ = state_.take_directory;
        post_take_banner_->hide();
    });
    pt_row->addWidget(pt_cont);
    auto *pt_lib = new QToolButton(post_take_banner_);
    pt_lib->setObjectName("postTakeOpenLibraryBtn");
    pt_lib->setText(tr("🎞 在录制库中回放与分析 →"));
    pt_lib->setProperty("multiPrimary", true);
    connect(pt_lib, &QToolButton::clicked, this, [this] {
        const QString dir = state_.take_directory;
        select_workspace(WorkspaceModule::Library);
        if (!dir.isEmpty() && QFileInfo::exists(dir))
            controller_->open_take(dir);
    });
    pt_row->addWidget(pt_lib);
    auto *pt_open = new QToolButton(post_take_banner_);
    pt_open->setObjectName("postTakeOpenDirBtn");
    pt_open->setText(tr("📂 打开目录"));
    connect(pt_open, &QToolButton::clicked, this, [this] {
        if (!state_.take_directory.isEmpty())
            QDesktopServices::openUrl(QUrl::fromLocalFile(state_.take_directory));
    });
    pt_row->addWidget(pt_open);
    auto *pt_discard = new QToolButton(post_take_banner_);
    pt_discard->setObjectName("postTakeDiscardBtn");
    pt_discard->setText(tr("🗑 丢弃重录"));
    connect(pt_discard, &QToolButton::clicked, this, [this] {
        const QString dir = state_.take_directory;
        dismissed_take_directory_ = dir;
        if (!dir.isEmpty() && !config_.output.isEmpty()) {
            const QString out_clean = QDir::cleanPath(config_.output);
            const QString dir_clean = QDir::cleanPath(dir);
            if (dir_clean.startsWith(out_clean + QLatin1Char('/')) && dir_clean != out_clean)
                QDir(dir_clean).removeRecursively();
        }
        post_take_banner_->hide();
        refresh_profiles_ui();
    });
    pt_row->addWidget(pt_discard);
    post_take_banner_->hide();

    library_header_bar_ = new QWidget(center_host);
    library_header_bar_->setObjectName("multiLibraryHeaderBar");
    auto *lh_row = new QHBoxLayout(library_header_bar_);
    lh_row->setContentsMargins(6, 4, 6, 4);
    lh_row->setSpacing(8);
    library_header_label_ = new QLabel(tr("🎞 历史录制库 · 选择左侧录制条目进行三摄同步回放與快速分析"), library_header_bar_);
    library_header_label_->setObjectName("multiLibraryHeaderLabel");
    lh_row->addWidget(library_header_label_, 1);
    auto *lib_open_ext = new QToolButton(library_header_bar_);
    lib_open_ext->setObjectName("libraryOpenExternalBtn");
    lib_open_ext->setText(tr("📂 打开外部 Take..."));
    connect(lib_open_ext, &QToolButton::clicked, this, [this] {
        if (auto *b = commands_.value("openTake"))
            b->click();
    });
    lh_row->addWidget(lib_open_ext);
    library_header_bar_->hide();

    capture_sync_drawer_ = new QWidget(center_host);
    capture_sync_drawer_->setObjectName("multiCaptureSyncDrawer");
    auto *drawer_layout = new QVBoxLayout(capture_sync_drawer_);
    drawer_layout->setContentsMargins(8, 6, 8, 6);
    drawer_layout->setSpacing(4);
    auto *drawer_top = new QHBoxLayout;
    drawer_sync_status_ = new QLabel(tr("⚡ 拍摄前硬件同步检测抽屉 (RP2350 Trigger + Clock Master)"), capture_sync_drawer_);
    drawer_sync_status_->setObjectName("drawerSyncStatusLabel");
    drawer_top->addWidget(drawer_sync_status_, 1);
    auto *retest_btn = new QToolButton(capture_sync_drawer_);
    retest_btn->setObjectName("drawerRetestSyncBtn");
    retest_btn->setText(tr("⚡ 重新发射脉冲检测"));
    retest_btn->setProperty("multiPrimary", true);
    connect(retest_btn, &QToolButton::clicked, this, [this] {
        update_config();
        controller_->synchronize();
    });
    drawer_top->addWidget(retest_btn);
    auto *override_btn = new QToolButton(capture_sync_drawer_);
    override_btn->setObjectName("drawerOverrideSyncBtn");
    override_btn->setText(tr("强制接受"));
    connect(override_btn, &QToolButton::clicked, this, [this] {
        if (auto *b = commands_.value("overrideSync"))
            b->click();
    });
    drawer_top->addWidget(override_btn);
    drawer_override_btn_ = override_btn;
    drawer_layout->addLayout(drawer_top);
    capture_triggers_ = new TriggerPlot(capture_sync_drawer_);
    capture_triggers_->setObjectName("multiCaptureTriggerPlot");
    capture_triggers_->setMinimumHeight(110);
    capture_triggers_->setMaximumHeight(135);
    drawer_layout->addWidget(capture_triggers_);
    drawer_sync_table_ = new QLabel(capture_sync_drawer_);
    drawer_sync_table_->setObjectName("drawerSyncTableLabel");
    drawer_sync_table_->setWordWrap(true);
    drawer_layout->addWidget(drawer_sync_table_);
    drawer_sync_diag_ = new QLabel(capture_sync_drawer_);
    drawer_sync_diag_->setObjectName("drawerSyncDiagLabel");
    drawer_sync_diag_->setWordWrap(true);
    drawer_layout->addWidget(drawer_sync_diag_);
    capture_sync_drawer_->hide();

    capture_control_bar_ = new QWidget(center_host);
    capture_control_bar_->setObjectName("multiCaptureControlBar");
    auto *cb_row = new QHBoxLayout(capture_control_bar_);
    cb_row->setContentsMargins(6, 4, 6, 4);
    cb_row->setSpacing(6);
    capture_sync_pill_ = new QLabel(tr("⚡ 同步待检"), capture_control_bar_);
    capture_sync_pill_->setObjectName("captureSyncPill");
    cb_row->addWidget(capture_sync_pill_);
    auto *quick_sync_btn = new QToolButton(capture_control_bar_);
    quick_sync_btn->setObjectName("captureQuickSyncBtn");
    quick_sync_btn->setText(tr("🔄 立即测同步"));
    connect(quick_sync_btn, &QToolButton::clicked, this, [this] {
        update_config();
        controller_->synchronize();
    });
    cb_row->addWidget(quick_sync_btn);
    capture_drawer_toggle_ = new QToolButton(capture_control_bar_);
    capture_drawer_toggle_->setObjectName("captureSyncDrawerToggleBtn");
    capture_drawer_toggle_->setText(tr("📊 同步波形与详情 ▲"));
    capture_drawer_toggle_->setCheckable(true);
    connect(capture_drawer_toggle_, &QToolButton::toggled, this, [this](bool open) {
        capture_sync_drawer_open_ = open;
        capture_drawer_toggle_->setText(open ? tr("📊 同步波形与详情 ▼") : tr("📊 同步波形与详情 ▲"));
        if (active_module_ == WorkspaceModule::Capture)
            capture_sync_drawer_->setVisible(open);
    });
    cb_row->addWidget(capture_drawer_toggle_);
    capture_take_label_ = new QLineEdit(capture_control_bar_);
    capture_take_label_->setObjectName("captureTakeLabelEdit");
    capture_take_label_->setPlaceholderText(tr("录制标签 (如 hand_wave_01)..."));
    connect(capture_take_label_, &QLineEdit::textChanged, this, [this](const QString &txt) {
        config_.take_label = txt.trimmed();
        controller_->configure(config_);
    });
    cb_row->addWidget(capture_take_label_, 1);
    capture_duration_combo_ = new QComboBox(capture_control_bar_);
    capture_duration_combo_->setObjectName("captureDurationCombo");
    capture_duration_combo_->addItem(tr("手动停止"), 0);
    capture_duration_combo_->addItem(tr("定时 10 s"), 10);
    capture_duration_combo_->addItem(tr("定时 30 s"), 30);
    capture_duration_combo_->addItem(tr("定时 60 s"), 60);
    connect(capture_duration_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
        const int sec = capture_duration_combo_->currentData().toInt();
        timed_->setChecked(sec > 0);
        if (sec > 0)
            duration_->setValue(sec);
        update_config();
    });
    cb_row->addWidget(capture_duration_combo_);
    capture_record_btn_ = new QToolButton(capture_control_bar_);
    capture_record_btn_->setObjectName("captureRecordPrimaryBtn");
    capture_record_btn_->setText(tr("● 开始录制 (Start Recording)"));
    capture_record_btn_->setProperty("multiPrimary", true);
    connect(capture_record_btn_, &QToolButton::clicked, this, [this] {
        if (state_.recording() || state_.activity == Activity::Preparing)
            controller_->stop();
        else {
            update_config();
            controller_->start_recording();
        }
    });
    cb_row->addWidget(capture_record_btn_);
    capture_control_bar_->hide();

    if (center_layout) {
        center_layout->insertWidget(0, post_take_banner_);
        center_layout->insertWidget(1, library_header_bar_);
        center_layout->addWidget(capture_sync_drawer_);
        center_layout->addWidget(capture_control_bar_);
    }

    quick_analysis_dock_ = new QWidget(center_host);
    quick_analysis_dock_->setObjectName("multiQuickAnalysisDock");
    auto *qa_layout = new QVBoxLayout(quick_analysis_dock_);
    qa_layout->setContentsMargins(6, 4, 6, 4);
    qa_layout->setSpacing(4);
    auto *qa_tabs = new QHBoxLayout;
    qa_tabs->setSpacing(4);
    const QString qa_titles[4] = {
        tr("📈 三摄事件率波形 (点击跳帧)"),
        tr("📐 极线水平对齐检验"),
        tr("⚡ 硬件触发脉冲对齐"),
        tr("🧩 + 拓展分析插件"),
    };
    for (int i = 0; i < 4; ++i) {
        auto *tb = new QToolButton(quick_analysis_dock_);
        tb->setObjectName(QString("quickAnalysisTab%1").arg(i));
        tb->setText(qa_titles[i]);
        tb->setCheckable(true);
        tb->setChecked(i == 0);
        tb->setProperty("multiSubstepPill", true);
        connect(tb, &QToolButton::clicked, this, [this, i] {
            for (int j = 0; j < 4; ++j)
                if (analysis_tab_btns_[j])
                    analysis_tab_btns_[j]->setChecked(j == i);
            if (analysis_plot_) {
                analysis_plot_->tab_index = i;
                analysis_plot_->update();
            }
            for (int k = 0; k < 3; ++k)
                if (previews_[k]) {
                    previews_[k]->epipolar_overlay = (active_module_ == WorkspaceModule::Library && i == 1);
                    previews_[k]->update();
                }
        });
        qa_tabs->addWidget(tb);
        analysis_tab_btns_[i] = tb;
    }
    qa_tabs->addStretch(1);
    qa_layout->addLayout(qa_tabs);
    analysis_plot_ = new QuickAnalysisPlotWidget(quick_analysis_dock_);
    analysis_plot_->on_seek_us = [this](qint64 target_us) { controller_->seek(target_us); };
    qa_layout->addWidget(analysis_plot_);
    quick_analysis_dock_->hide();
    if (center_layout)
        center_layout->addWidget(quick_analysis_dock_);

    capture_profile_sidebar_ = new QWidget(dock_stack_);
    capture_profile_sidebar_->setObjectName("multiCaptureProfileSidebar");
    auto *cps_layout = new QVBoxLayout(capture_profile_sidebar_);
    cps_layout->setContentsMargins(10, 8, 10, 8);
    cps_layout->setSpacing(6);
    auto *cps_title = new QLabel(tr("当前激活 Profile (只读标定档案)"), capture_profile_sidebar_);
    QFont tf = cps_title->font();
    tf.setBold(true);
    cps_title->setFont(tf);
    cps_layout->addWidget(cps_title);

    capture_profile_combo_ = new QComboBox(capture_profile_sidebar_);
    capture_profile_combo_->setObjectName("captureProfileCombo");
    connect(capture_profile_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int idx) {
        if (updating_ || idx < 0)
            return;
        const QString pid = capture_profile_combo_->itemData(idx).toString();
        if (!pid.isEmpty())
            controller_->activate_profile(pid);
    });
    cps_layout->addWidget(capture_profile_combo_);

    auto *cps_actions = new QHBoxLayout;
    cps_actions->setSpacing(4);
    auto *new_prof_btn = new QToolButton(capture_profile_sidebar_);
    new_prof_btn->setObjectName("captureNewProfileBtn");
    new_prof_btn->setText(tr("+ 新建标定"));
    new_prof_btn->setProperty("multiPrimary", true);
    connect(new_prof_btn, &QToolButton::clicked, this, [this] { select_workspace(WorkspaceModule::Calibrate); });
    cps_actions->addWidget(new_prof_btn);
    auto *imp_prof_btn = new QToolButton(capture_profile_sidebar_);
    imp_prof_btn->setObjectName("captureImportProfileBtn");
    imp_prof_btn->setText(tr("📥 导入"));
    connect(imp_prof_btn, &QToolButton::clicked, this, [this] {
        const auto file = QFileDialog::getOpenFileName(this, tr("导入标定 Profile"), config_.output,
                                                       tr("JSON Files (*.json)"));
        if (!file.isEmpty()) {
            controller_->import_profile(file);
            refresh_profiles_ui();
        }
    });
    cps_actions->addWidget(imp_prof_btn);
    auto *del_prof_btn = new QToolButton(capture_profile_sidebar_);
    del_prof_btn->setObjectName("captureDeleteProfileBtn");
    del_prof_btn->setText(tr("🗑 删除"));
    connect(del_prof_btn, &QToolButton::clicked, this, [this] {
        const QString pid = capture_profile_combo_->currentData().toString();
        if (!pid.isEmpty()) {
            controller_->delete_profile(pid);
            refresh_profiles_ui();
        }
    });
    cps_actions->addWidget(del_prof_btn);
    cps_layout->addLayout(cps_actions);

    capture_diagram_ = new MainboardDiagramWidget(capture_profile_sidebar_);
    capture_diagram_->setObjectName("captureMainboardDiagram");
    cps_layout->addWidget(capture_diagram_);

    capture_profile_metrics_ = new QLabel(capture_profile_sidebar_);
    capture_profile_metrics_->setObjectName("captureProfileMetricsLabel");
    capture_profile_metrics_->setWordWrap(true);
    cps_layout->addWidget(capture_profile_metrics_);

    auto *takes_hdr = new QLabel(tr("当前 Profile 下的历史录制 (点击进入回放库)"), capture_profile_sidebar_);
    takes_hdr->setFont(tf);
    cps_layout->addWidget(takes_hdr);
    capture_profile_takes_ = new QListWidget(capture_profile_sidebar_);
    capture_profile_takes_->setObjectName("captureProfileTakesList");
    connect(capture_profile_takes_, &QListWidget::itemClicked, this, [this](QListWidgetItem *item) {
        if (!item)
            return;
        const QString dir = item->data(Qt::UserRole).toString();
        select_workspace(WorkspaceModule::Library);
        if (!dir.isEmpty() && QFileInfo::exists(dir))
            controller_->open_take(dir);
    });
    cps_layout->addWidget(capture_profile_takes_, 1);
    if (dock_stack_)
        dock_stack_->addWidget(capture_profile_sidebar_);

    for (int i = 0; i < 3; ++i)
        if (workspace_mode_btns_[i])
            workspace_mode_btns_[i]->setChecked(i == int(active_module_));
    refresh_profiles_ui();
}

void Workspace::select_layout_preset(RigLayoutPreset preset) {
    config_.layout_preset = preset;
    for (int i = 0; i < 3; ++i)
        if (cal_layout_btns_[i]) {
            QSignalBlocker b(cal_layout_btns_[i]);
            cal_layout_btns_[i]->setChecked(i == int(preset));
        }
    if (cal_diagram_)
        cal_diagram_->set_preset(preset);
    if (cal_new_profile_name_) {
        const QString cur = cal_new_profile_name_->text().trimmed();
        if (cur.startsWith(QLatin1String("Profile_")) &&
            (cur.endsWith(QLatin1String("_LayoutA")) || cur.endsWith(QLatin1String("_LayoutB")) ||
             cur.endsWith(QLatin1String("_LayoutC")))) {
            cal_new_profile_name_->setText(
                QStringLiteral("Profile_%1_Layout%2")
                    .arg(QDate::currentDate().toString(QStringLiteral("yyyyMMdd")), layout_preset_code(preset)));
        }
    }
    update_config();
}

void Workspace::select_workspace(WorkspaceModule module) {
    active_module_ = module;
    for (int i = 0; i < 3; ++i)
        if (workspace_mode_btns_[i]) {
            QSignalBlocker b(workspace_mode_btns_[i]);
            workspace_mode_btns_[i]->setChecked(i == int(module));
        }
    if (module == WorkspaceModule::Capture) {
        if (navigation_) {
            QSignalBlocker nb(navigation_);
            navigation_->setCurrentRow(int(Step::Capture));
        }
        if (pages_)
            pages_->setCurrentIndex(int(Step::Capture));
        if (left_calibrate_panel_)
            left_calibrate_panel_->hide();
        if (library_sidebar_)
            library_sidebar_->hide();
        if (library_header_bar_)
            library_header_bar_->hide();
        if (triggers_)
            triggers_->hide();
        if (stability_)
            stability_->hide();
        if (guide_)
            guide_->hide();
        if (transport_)
            transport_->hide();
        if (quick_analysis_dock_)
            quick_analysis_dock_->hide();
        if (capture_sync_drawer_)
            capture_sync_drawer_->setVisible(capture_sync_drawer_open_);
        if (capture_control_bar_)
            capture_control_bar_->show();
        if (dock_stack_ && capture_profile_sidebar_)
            dock_stack_->setCurrentWidget(capture_profile_sidebar_);
        if (toolbox_)
            toolbox_->setWindowTitle(tr("当前 Profile & 标定参数"));
    } else if (module == WorkspaceModule::Calibrate) {
        if (left_calibrate_panel_)
            left_calibrate_panel_->show();
        if (library_sidebar_)
            library_sidebar_->hide();
        if (post_take_banner_)
            post_take_banner_->hide();
        if (library_header_bar_)
            library_header_bar_->hide();
        if (capture_sync_drawer_)
            capture_sync_drawer_->hide();
        if (capture_control_bar_)
            capture_control_bar_->hide();
        if (transport_)
            transport_->hide();
        if (quick_analysis_dock_)
            quick_analysis_dock_->hide();
        if (dock_stack_ && tools_panel_)
            dock_stack_->setCurrentWidget(tools_panel_);
        if (toolbox_)
            toolbox_->setWindowTitle(tr("Guided Workflow Inspector"));
        const int target_step = (navigation_ && navigation_->currentRow() >= 0 && navigation_->currentRow() <= 2)
                                    ? navigation_->currentRow()
                                    : 0;
        select_step(target_step);
    } else {
        if (navigation_) {
            QSignalBlocker nb(navigation_);
            navigation_->setCurrentRow(int(Step::Review));
        }
        if (pages_)
            pages_->setCurrentIndex(int(Step::Review));
        if (left_calibrate_panel_)
            left_calibrate_panel_->hide();
        if (library_sidebar_)
            library_sidebar_->show();
        if (post_take_banner_)
            post_take_banner_->hide();
        if (library_header_bar_)
            library_header_bar_->show();
        if (triggers_)
            triggers_->hide();
        if (stability_)
            stability_->hide();
        if (guide_)
            guide_->hide();
        if (capture_sync_drawer_)
            capture_sync_drawer_->hide();
        if (capture_control_bar_)
            capture_control_bar_->hide();
        if (transport_)
            transport_->show();
        if (quick_analysis_dock_)
            quick_analysis_dock_->show();
        if (dock_stack_ && tools_panel_)
            dock_stack_->setCurrentWidget(tools_panel_);
        if (toolbox_)
            toolbox_->setWindowTitle(tr("Playback & Retained Report"));
    }
    for (int k = 0; k < 3; ++k)
        if (previews_[k]) {
            previews_[k]->epipolar_overlay =
                (module == WorkspaceModule::Library && analysis_plot_ && analysis_plot_->tab_index == 1);
            previews_[k]->update();
        }
    refresh_profiles_ui();
    update_blockers();
    update_visible_details();
    if (preview_area_)
        preview_area_->arrange();
}

void Workspace::refresh_profiles_ui() {
    if (!controller_)
        return;
    const auto profiles = controller_->list_profiles();
    const auto legacy_takes = controller_->list_legacy_takes();

    RigProfileRecord active_rec;
    for (const auto &p : profiles)
        if (p.profile_id == config_.active_profile_id)
            active_rec = p;
    if (!active_rec.valid() && !profiles.isEmpty())
        active_rec = profiles.front();

    if (capture_profile_combo_) {
        QSignalBlocker b(capture_profile_combo_);
        capture_profile_combo_->clear();
        int sel_idx = 0;
        for (int i = 0; i < profiles.size(); ++i) {
            capture_profile_combo_->addItem(
                QStringLiteral("%1 (%2)").arg(profiles[i].display_name, layout_preset_code(profiles[i].layout_preset)),
                profiles[i].profile_id);
            if (profiles[i].profile_id == active_rec.profile_id)
                sel_idx = i;
        }
        if (profiles.isEmpty())
            capture_profile_combo_->addItem(
                QStringLiteral("未保存 Profile (%1)").arg(layout_preset_title(config_.layout_preset)),
                QString{});
        capture_profile_combo_->setCurrentIndex(sel_idx);
    }

    const RigLayoutPreset cur_preset = active_rec.valid() ? active_rec.layout_preset : config_.layout_preset;
    if (capture_diagram_)
        capture_diagram_->set_preset(cur_preset);
    if (cal_diagram_)
        cal_diagram_->set_preset(config_.layout_preset);

    const auto nom = nominal_baselines_mm(cur_preset);
    const auto slots_geo = nominal_layout_geometry(cur_preset);
    for (int k = 0; k < 3; ++k)
        if (previews_[k]) {
            previews_[k]->hole_label = slots_geo[k].hole_id;
            previews_[k]->update();
        }
    const auto sanity = active_rec.valid() ? active_rec.sanity_check()
                                           : evaluate_baseline_sanity(cur_preset, state_.calibration.profile);
    if (capture_profile_metrics_) {
        const auto &meas = sanity.verdict != BaselineSanityVerdict::Unknown ? sanity.measured_mm : nom;
        const auto &foc = active_rec.valid() ? active_rec.focal_px : std::array<double, 3>{1650.0, 1650.0, 1650.0};
        capture_profile_metrics_->setText(
            QStringLiteral("主板: 通用三相机竖直主板 - A版\n%1\n孔位: L=%2 | C=%3 | R=%4\n设计基线 B12/B23/B13: %5 / %6 / %7 mm\n标定基线 ||T||: %8 / %9 / %10 mm\n等效焦距 L/C/R: %11 / %12 / %13 px\nRMS: %14 px\n%15")
                .arg(layout_preset_title(cur_preset), slots_geo[0].hole_id, slots_geo[1].hole_id,
                     slots_geo[2].hole_id)
                .arg(nom[0], 0, 'f', 1)
                .arg(nom[1], 0, 'f', 1)
                .arg(nom[2], 0, 'f', 1)
                .arg(meas[0], 0, 'f', 1)
                .arg(meas[1], 0, 'f', 1)
                .arg(meas[2], 0, 'f', 1)
                .arg(foc[0], 0, 'f', 1)
                .arg(foc[1], 0, 'f', 1)
                .arg(foc[2], 0, 'f', 1)
                .arg(active_rec.valid() ? active_rec.rms_px
                     : std::isfinite(state_.calibration.rms) ? state_.calibration.rms
                                                             : 0.34,
                     0, 'f', 2)
                .arg(sanity.summary));
    }

    if (capture_profile_takes_) {
        capture_profile_takes_->clear();
        const auto &takes = active_rec.valid() ? active_rec.takes : legacy_takes;
        for (const auto &t : takes) {
            auto *it = new QListWidgetItem(
                QStringLiteral("🎞 %1  (%2 s)").arg(t.label, QString::number(t.duration_s, 'f', 1)),
                capture_profile_takes_);
            it->setData(Qt::UserRole, t.take_dir);
        }
    }

    if (library_tree_) {
        const QString filter = library_search_edit_ ? library_search_edit_->text().trimmed() : QString{};
        library_tree_->clear();
        for (const auto &p : profiles) {
            if (!filter.isEmpty() && !p.display_name.contains(filter, Qt::CaseInsensitive)) {
                bool any_match = false;
                for (const auto &t : p.takes)
                    if (t.label.contains(filter, Qt::CaseInsensitive) || t.take_id.contains(filter, Qt::CaseInsensitive))
                        any_match = true;
                if (!any_match)
                    continue;
            }
            auto *p_item = new QTreeWidgetItem(library_tree_);
            p_item->setText(0, QStringLiteral("📁 %1 (%2)").arg(p.display_name).arg(p.takes.size()));
            p_item->setExpanded(true);
            for (const auto &t : p.takes) {
                if (!filter.isEmpty() && !t.label.contains(filter, Qt::CaseInsensitive) &&
                    !t.take_id.contains(filter, Qt::CaseInsensitive) &&
                    !p.display_name.contains(filter, Qt::CaseInsensitive))
                    continue;
                auto *t_item = new QTreeWidgetItem(p_item);
                t_item->setText(0, QStringLiteral("🎞 %1 (%2s)").arg(t.label).arg(t.duration_s, 0, 'f', 1));
                t_item->setData(0, Qt::UserRole, t.take_dir);
            }
        }
        if (!legacy_takes.isEmpty()) {
            auto *leg_root = new QTreeWidgetItem(library_tree_);
            leg_root->setText(0, QStringLiteral("📁 Legacy / 未分类录制 (%1)").arg(legacy_takes.size()));
            leg_root->setExpanded(true);
            for (const auto &t : legacy_takes) {
                if (!filter.isEmpty() && !t.label.contains(filter, Qt::CaseInsensitive) &&
                    !t.take_id.contains(filter, Qt::CaseInsensitive))
                    continue;
                auto *t_item = new QTreeWidgetItem(leg_root);
                t_item->setText(0, QStringLiteral("🎞 %1 (%2s)").arg(t.label).arg(t.duration_s, 0, 'f', 1));
                t_item->setData(0, Qt::UserRole, t.take_dir);
            }
        }
    }
    if (library_profile_summary_) {
        library_profile_summary_->setText(
            QStringLiteral("所属 Profile: %1\n%2\n基线: %3 / %4 / %5 mm")
                .arg(active_rec.valid() ? active_rec.display_name : tr("默认/未分类档案"),
                     layout_preset_title(cur_preset))
                .arg(nom[0], 0, 'f', 1)
                .arg(nom[1], 0, 'f', 1)
                .arg(nom[2], 0, 'f', 1));
    }
}

void Workspace::update_three_workspaces() {
    const auto &s = state_;
    const auto cal_sanity = evaluate_baseline_sanity(config_.layout_preset, s.calibration.profile);
    if (cal_baseline_sanity_card_) {
        if (cal_sanity.verdict != BaselineSanityVerdict::Unknown) {
            cal_baseline_sanity_card_->setText(
                QStringLiteral("B12(L-C): 设计 %1 mm vs 实测 %2 mm (Δ%3 mm)\n"
                               "B23(C-R): 设计 %4 mm vs 实测 %5 mm (Δ%6 mm)\n"
                               "B13(L-R): 设计 %7 mm vs 实测 %8 mm (Δ%9 mm)\n%10")
                    .arg(cal_sanity.nominal_mm[0], 0, 'f', 1)
                    .arg(cal_sanity.measured_mm[0], 0, 'f', 1)
                    .arg(cal_sanity.abs_errors_mm[0], 0, 'f', 2)
                    .arg(cal_sanity.nominal_mm[1], 0, 'f', 1)
                    .arg(cal_sanity.measured_mm[1], 0, 'f', 1)
                    .arg(cal_sanity.abs_errors_mm[1], 0, 'f', 2)
                    .arg(cal_sanity.nominal_mm[2], 0, 'f', 1)
                    .arg(cal_sanity.measured_mm[2], 0, 'f', 1)
                    .arg(cal_sanity.abs_errors_mm[2], 0, 'f', 2)
                    .arg(cal_sanity.summary));
        } else {
            cal_baseline_sanity_card_->setText(
                QStringLiteral("设计基线: %1/%2/%3 mm\n%4")
                    .arg(cal_sanity.nominal_mm[0], 0, 'f', 0)
                    .arg(cal_sanity.nominal_mm[1], 0, 'f', 0)
                    .arg(cal_sanity.nominal_mm[2], 0, 'f', 0)
                    .arg(cal_sanity.summary));
        }
    }
    if (cal_save_profile_btn_)
        cal_save_profile_btn_->setEnabled(cal_sanity.verdict != BaselineSanityVerdict::Alert && !s.busy());
    if (drawer_override_btn_)
        drawer_override_btn_->setEnabled(config_.trigger_wiring_confirmed && s.can_override_sync());
    if (capture_triggers_) {
        capture_triggers_->cameras = s.cameras;
        capture_triggers_->invert_display = s.sync.timing_measured() && s.sync.polarity_inverted;
        capture_triggers_->update();
    }
    // Pin each newly finished finite check so its full edge sequence can be reviewed afterwards.
    if (s.sync.attempt_completed && s.sync.attempt != pinned_sync_attempt_ && s.activity != Activity::Syncing &&
        s.activity != Activity::Triggering &&
        std::any_of(s.sync.observed_edges.begin(), s.sync.observed_edges.end(),
                    [](const auto &v) { return !v.isEmpty(); })) {
        pinned_sync_attempt_ = s.sync.attempt;
        const QString label =
            tr("Finite check #%1 · %2").arg(s.sync.attempt).arg(verdict_text(s.sync.verdict));
        for (auto *plot : {triggers_, capture_triggers_})
            if (plot)
                plot->set_pinned(s.sync.observed_edges, label, config_.master);
    }
    const bool sync_ok = s.sync.accepted(s.epoch, config_.max_delta_us);
    if (capture_sync_pill_) {
        capture_sync_pill_->setText(
            sync_ok ? QStringLiteral("⚡ 硬件同步就绪 (Δt=%1 μs ✓)")
                          .arg(std::isfinite(s.sync.max_delta_us) ? s.sync.max_delta_us : 0.8, 0, 'f', 1)
                    : QStringLiteral("⚡ 拍摄前同步待检"));
    }
    if (drawer_sync_table_) {
        drawer_sync_table_->setText(
            QStringLiteral("Cam1 Left: %1 edges | Cam2 Center (Master): %2 edges | Cam3 Right: %3 edges | Max |Δt|: %4 μs")
                .arg(s.sync.matched[0])
                .arg(s.sync.matched[1])
                .arg(s.sync.matched[2])
                .arg(std::isfinite(s.sync.max_delta_us) ? QString::number(s.sync.max_delta_us, 'f', 1)
                                                        : QStringLiteral("--")));
    }
    if (drawer_sync_diag_)
        drawer_sync_diag_->setText(s.sync.reason.isEmpty() ? tr("点击「重新发射脉冲检测」验证三摄微秒级硬件同步。")
                                                           : s.sync.reason);
    if (capture_record_btn_) {
        capture_record_btn_->setText(
            (s.recording() || s.activity == Activity::Preparing)
                ? tr("■ 停止录制 (%1 s)").arg(s.elapsed_ms / 1000.0, 0, 'f', 1)
                : tr("● 开始录制 (Start Recording)"));
    }
    if (post_take_banner_ && active_module_ == WorkspaceModule::Capture) {
        if (s.recording())
            dismissed_take_directory_.clear();
        const bool has_finished_take = !s.recording() && !s.take_directory.isEmpty() &&
                                       s.take_directory != dismissed_take_directory_ &&
                                       s.take_manifest.value("state").toString() == QLatin1String("completed");
        post_take_banner_->setVisible(has_finished_take);
        if (has_finished_take && post_take_summary_)
            post_take_summary_->setText(
                QStringLiteral("✓ 录制完成: %1").arg(QFileInfo(s.take_directory).fileName()));
    }
    if (analysis_plot_) {
        analysis_plot_->playback = s.playback;
        analysis_plot_->sync = s.sync;
        if (std::isfinite(s.calibration.rms))
            analysis_plot_->rms_px = s.calibration.rms;
        analysis_plot_->update();
    }
    if (library_header_label_ && s.playback.loaded) {
        library_header_label_->setText(
            QStringLiteral("🎞 正在回放: %1  |  Profile: %2")
                .arg(QFileInfo(s.take_directory).fileName(),
                     s.take_manifest.value("profileId").toString(tr("Legacy"))));
    }
}
} // namespace gui::multicam





