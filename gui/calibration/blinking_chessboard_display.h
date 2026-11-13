// gui/calibration/blinking_chessboard_display.h — on-screen blinking chessboard
// pattern for event-camera intrinsic calibration.
//
// The board BLINKS to generate events without camera motion: the BLACK squares
// of the chessboard flip black ↔ white at a 10 ms period while the white
// squares of the board stay white. Pixels on the black squares fire both ON
// and OFF events every cycle; over a capture window covering ≥ one cycle they
// form a filled checkerboard in the binary "blink frame" that the detector
// feeds to cv::findChessboardCorners (see algo/calibration/blinking_detect.h).
// The area OUTSIDE the board is a constant dark gray (it fires no events) — a
// bright all-white screen produces glare / backlight-PWM noise in the event
// camera.
//
// Two pre-computed pixmaps (frame A: black squares on white; frame B: all
// white board — both on the dark-gray margin) are toggled at 10 ms —
// paintEvent is a single drawPixmap blit, no per-frame rendering. Same
// pre-rendered-pixmap blit architecture as the previous pattern display (avoids
// the maximize-button preview stutter).

#ifndef GUI_CALIBRATION_BLINKING_CHESSBOARD_DISPLAY_H
#define GUI_CALIBRATION_BLINKING_CHESSBOARD_DISPLAY_H

#include <QPixmap>
#include <QWidget>

class QTimer;

namespace gui {

/// @brief Renders the blinking chessboard calibration pattern.
class BlinkingChessboardDisplay : public QWidget {
    Q_OBJECT
  public:
    explicit BlinkingChessboardDisplay(QWidget *parent = nullptr);

    /// @brief Sets the board dimensions (INNER corner count: patternSize for
    /// cv::findChessboardCorners). The widget draws (cols+1)×(rows+1) squares.
    /// Recomputes the pixel layout and re-renders both cached pixmaps.
    void set_pattern(int cols, int rows);

    /// @brief Sets the physical square size (mm). Stored for retrieval only —
    /// it does NOT affect pixel layout (no DPI derivation), so no repaint is
    /// triggered. Sets the real-world scale for the calibration algorithm.
    void set_square_size_mm(float mm);

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    float square_size_mm() const { return square_size_mm_; }
    /// @brief Current pixel size of one square.
    int square_px() const { return square_px_; }

    /// Places the unchanged board within the available vertical margin.
    /// 0.5 keeps the single-camera default centered; 1.0 uses the lower margin.
    void set_vertical_placement(double placement);
    double vertical_placement() const { return vertical_placement_; }
    QRect board_rect() const {
        return QRect(origin_x_, origin_y_, (cols_ + 1) * square_px_, (rows_ + 1) * square_px_);
    }

  signals:
    void pattern_layout_changed();

  protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

  private slots:
    /// @brief 10 ms blink: toggles blink_frame_ and triggers a repaint.
    void on_blink_timeout();

  private:
    void recompute_layout();

    int cols_{9};
    int rows_{6};
    int square_px_{0};
    int origin_x_{0};
    int origin_y_{0};
    double vertical_placement_{0.5};
    float square_size_mm_{20.0f};

    /// Two pre-rendered frames for the 10 ms blink. Frame A is the chessboard
    /// (BLACK squares on a WHITE board area), frame B is the same board area
    /// all white — both on the constant dark-gray margin. The black squares
    /// flip black↔white, the non-toggling squares stay white, the margin stays
    /// dark gray. recompute_layout() renders both; paintEvent() blits one — no
    /// per-frame drawing.
    QPixmap pixmap_a_; // chessboard: black squares on white board
    QPixmap pixmap_b_; // blank: all-white board
    /// Current blink frame (0 = A, 1 = B). Toggled by the blink timer.
    int blink_frame_{0};
    /// Last frame blitted to the backing store. When blink_frame_ differs,
    /// paintEvent blits the new pixmap; otherwise it skips (the backing store
    /// already has the correct content). -1 forces a blit on the first paint.
    int blitted_frame_{-1};

    /// 10 ms blink timer. Toggles blink_frame_ and calls update().
    QTimer *blink_timer_{nullptr};
};

} // namespace gui

#endif // GUI_CALIBRATION_BLINKING_CHESSBOARD_DISPLAY_H
