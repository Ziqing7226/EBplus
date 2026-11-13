// gui/calibration/blinking_chessboard_display.cpp — see header.

#include "blinking_chessboard_display.h"

#include <algorithm>
#include <cmath>

#include <QPaintEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QTimer>
#include <QVector>

namespace gui {

namespace {
// Blink rate: 10 ms period between the chessboard frame and the blank frame.
// A full blink cycle is therefore 20 ms (10 ms pattern + 10 ms blank); the
// wizard's capture window must cover at least one full cycle so every toggling
// square sees both polarities (the default capture window is 100 ms).
constexpr int kBlinkIntervalMs = 10;
// Margin around the board: a constant dark gray — it never changes between the
// two frames, so it fires no events, and it keeps the screen dim (less glare /
// PWM noise for the camera). Only the area OUTSIDE the board is darkened; the
// board itself stays white with the original black squares.
const QColor kBoardBg(96, 96, 96);
} // namespace

BlinkingChessboardDisplay::BlinkingChessboardDisplay(QWidget *parent) : QWidget(parent) {
    setAutoFillBackground(false);
    // We paint every pixel ourselves (dark-gray margin + board), so Qt can
    // skip its background fill — saves a memset on each repaint.
    setAttribute(Qt::WA_OpaquePaintEvent);
    setPalette(kBoardBg);
    setMinimumSize(320, 320);
    recompute_layout();

    // 10 ms blink timer: toggles blink_frame_ (0↔1) and triggers update().
    // paintEvent blits the corresponding pre-computed pixmap — a single
    // drawPixmap call, no per-frame rendering. The timer is lightweight
    // (just a flag toggle + update); when the widget is hidden, update() is
    // a no-op so there's no wasted CPU.
    blink_timer_ = new QTimer(this);
    blink_timer_->setInterval(kBlinkIntervalMs);
    connect(blink_timer_, &QTimer::timeout, this, &BlinkingChessboardDisplay::on_blink_timeout);
    blink_timer_->start();
}

void BlinkingChessboardDisplay::set_pattern(int cols, int rows) {
    cols_ = std::max(1, cols);
    rows_ = std::max(1, rows);
    pixmap_a_ = QPixmap(); // invalidate → force re-render even if size unchanged
    pixmap_b_ = QPixmap();
    recompute_layout();
    update();
}

void BlinkingChessboardDisplay::set_square_size_mm(float mm) {
    // Perf: no update() — the mm value does not affect pixel layout.
    square_size_mm_ = std::max(0.1f, mm);
}

void BlinkingChessboardDisplay::set_vertical_placement(double placement) {
    if (!std::isfinite(placement))
        return;
    placement = std::clamp(placement, 0.0, 1.0);
    if (vertical_placement_ == placement)
        return;
    vertical_placement_ = placement;
    pixmap_a_ = QPixmap();
    pixmap_b_ = QPixmap();
    recompute_layout();
    update();
    emit pattern_layout_changed();
}

void BlinkingChessboardDisplay::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    recompute_layout();
}

void BlinkingChessboardDisplay::on_blink_timeout() {
    blink_frame_ ^= 1; // toggle 0 ↔ 1
    update();
}

void BlinkingChessboardDisplay::recompute_layout() {
    // Skip the full pixmap re-render when the widget size has not changed and
    // both caches are still valid. Layout-param changes (cols/rows) invalidate
    // the pixmaps in their setters, so a param change still re-renders.
    if (!pixmap_a_.isNull() && !pixmap_b_.isNull() && pixmap_a_.size() == size() &&
        pixmap_b_.size() == size()) {
        return;
    }
    if (width() <= 0 || height() <= 0) {
        pixmap_a_ = QPixmap();
        pixmap_b_ = QPixmap();
        return;
    }

    // Chessboard footprint: (cols+1)×(rows+1) squares for cols×rows INNER
    // corners. Pick the largest square size that fits ~92% of the widget.
    const int avail_w = std::max(0, width() - 16);
    const int avail_h = std::max(0, height() - 16);
    const int sq_cols = cols_ + 1;
    const int sq_rows = rows_ + 1;
    int sp = std::min(avail_w / std::max(1, sq_cols), avail_h / std::max(1, sq_rows));
    sp = static_cast<int>(sp * 0.92);
    square_px_ = std::max(4, sp);
    const int grid_w = sq_cols * square_px_;
    const int grid_h = sq_rows * square_px_;
    origin_x_ = (width() - grid_w) / 2;
    origin_y_ = int(std::round((height() - grid_h) * vertical_placement_));

    // The board itself is unchanged from the original pattern: BLACK squares
    // on a WHITE board area (checkerboard parity: (col+row) even = black), and
    // the blank frame shows the same board area all white — so the black
    // squares keep toggling black↔white and the non-toggling squares stay
    // white. ONLY the area OUTSIDE the board (the widget margin) is dark gray
    // instead of white — it is constant in both frames (fires no events) and
    // keeps the screen dim, reducing the glare / backlight-PWM noise an event
    // camera picks up from a bright white screen. Both rendered with batched
    // solid QPainter primitives (drawRects), keeping the QPixmap on the native
    // blit path (no QImage full-widget CPU→GPU upload — see project memory on
    // the maximized-window preview stutter).
    const QRect board_rect(origin_x_, origin_y_, grid_w, grid_h);
    QVector<QRect> black_squares;
    black_squares.reserve(static_cast<int>((sq_cols * sq_rows + 1) / 2));
    for (int r = 0; r < sq_rows; ++r) {
        for (int c = 0; c < sq_cols; ++c) {
            if (((r + c) & 1) == 0) {
                black_squares.append(
                    QRect(origin_x_ + c * square_px_, origin_y_ + r * square_px_, square_px_, square_px_));
            }
        }
    }

    {
        pixmap_a_ = QPixmap(size());
        pixmap_a_.fill(kBoardBg);
        QPainter p(&pixmap_a_);
        p.setRenderHint(QPainter::Antialiasing, false);
        p.setPen(Qt::NoPen);
        p.setBrush(Qt::white);
        p.drawRect(board_rect); // white board area on the dark-gray margin
        p.setBrush(Qt::black);
        if (!black_squares.isEmpty()) {
            p.drawRects(black_squares);
        }
    }
    {
        pixmap_b_ = QPixmap(size());
        pixmap_b_.fill(kBoardBg);
        QPainter p(&pixmap_b_);
        p.setRenderHint(QPainter::Antialiasing, false);
        p.setPen(Qt::NoPen);
        p.setBrush(Qt::white);
        p.drawRect(board_rect); // blank: whole board area white
    }

    blitted_frame_ = -1; // force blit on next paintEvent
}

void BlinkingChessboardDisplay::paintEvent(QPaintEvent *event) {
    QPainter p(this);
    const QPixmap &pm = (blink_frame_ == 0) ? pixmap_a_ : pixmap_b_;
    if (pm.isNull()) {
        p.fillRect(event->rect(), kBoardBg);
        return;
    }
    // Skip the blit if the same frame was already blitted since the last
    // toggle. With WA_OpaquePaintEvent, Qt doesn't clear the damaged region,
    // so the backing store retains the previous (correct) content. This
    // eliminates redundant blits when the WM sends expose events between
    // blinks (e.g., Mutter unredirected windows).
    if (blitted_frame_ != blink_frame_) {
        p.drawPixmap(0, 0, pm);
        blitted_frame_ = blink_frame_;
    }
}

} // namespace gui
