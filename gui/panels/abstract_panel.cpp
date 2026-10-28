// gui/panels/abstract_panel.cpp

#include "abstract_panel.h"

#include <QStyle>

#include "app/camera_controller.h"

namespace gui {

void AbstractPanel::restyle(QWidget* w) {
    w->style()->unpolish(w);
    w->style()->polish(w);
}

void AbstractPanel::bind_camera(CameraController* cam) {
    if (camera_) {
        disconnect(camera_, nullptr, this, nullptr);
    }
    camera_ = cam;
    if (camera_) {
        // Forward connected/disconnected to the panel's virtual slots.
        // The SensorInfo from `connected` is available via camera_->sensor_info()
        // (already updated before the signal is emitted), so the slot can read
        // it directly without needing the parameter.
        //
        // Capture the bound controller BY VALUE: subclasses null camera_ in
        // on_camera_disconnected(), so reading the member here at emission
        // time delivered nullptr on every connect after the first disconnect
        // — panels then replayed/populated against nullptr (the preprocessing
        // panel's flip X/Y died after one camera switch; the switch diag log
        // shows "panel connected" followed by "apply_stage SKIPPED (camera_
        // null)").
        CameraController* const bound = cam;
        connect(bound, &CameraController::connected, this,
                [this, bound](const SensorInfo&) { on_camera_connected(bound); });
        connect(bound, &CameraController::disconnected, this, [this]() {
            on_camera_disconnected();
        });
    }
}

} // namespace gui
