// gui/panels/preprocessing_panel.cpp

#include "preprocessing_panel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QVBoxLayout>

#include "algo_bridge/filter_chain.h"
#include "app/camera_controller.h"

namespace gui {

PreprocessingPanel::PreprocessingPanel(QWidget* parent) : AbstractPanel(parent) {
    build_ui();
    setEnabled(false);
}

void PreprocessingPanel::build_ui() {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    group_ = new QGroupBox(tr("Display Transform"), this);
    auto* form = new QFormLayout(group_);

    auto make_row = [&](const QString& stage, const QString& label) {
        auto* cb = new QCheckBox(label, group_);
        enables_[stage] = cb;
        form->addRow(cb);
        connect(cb, &QCheckBox::toggled, this, [this, stage](bool) { apply_stage(stage); });
    };
    auto make_enum_row = [&](const QString& stage, const QString& label,
                             const QStringList& items, const QString& param) {
        auto* cb = new QCheckBox(label, group_);
        auto* cmb = new QComboBox(group_);
        cmb->addItems(items);
        cmb->setEnabled(false);
        combos_[stage + "|" + param] = cmb;
        auto* w = new QWidget;
        auto* hl = new QHBoxLayout(w);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->addWidget(cb);
        hl->addWidget(cmb, 1);
        form->addRow(w);
        enables_[stage] = cb;
        connect(cb, &QCheckBox::toggled, this, [this, stage, cmb](bool on) {
            cmb->setEnabled(on);
            apply_stage(stage);
        });
        connect(cmb, &QComboBox::currentTextChanged, this, [this, stage](const QString&) {
            apply_stage(stage);
        });
    };

    make_enum_row("polarity_filter", tr("Polarity Filter"),
                  {"OFF (0)", "ON (1)"}, "polarity");
    make_row("polarity_invert", tr("Polarity Invert"));
    make_row("flip_x", tr("Flip X"));
    make_row("flip_y", tr("Flip Y"));
    // Phase 2.6 debug D-6: the ROI Filter stage was deleted — the unified
    // ROI (Hardware/Algorithms page checkbox + dialog, hardware I_ROI on
    // live / software crop on file) is the single ROI concept; this stage
    // duplicated it with fully independent state.
    // 2026-08-18: rotate / transpose / rescale were deleted (coordinate
    // changes without geometry propagation → OOB heap corruption).

    outer->addWidget(group_);
}

void PreprocessingPanel::apply_stage(const QString& stage) {
    if (!camera_) return;
    auto* chain = camera_->filter_chain();
    auto* cb = enables_.value(stage);
    if (!cb) return;
    // Use the thread-safe locked wrappers. Calling chain->stage()->set_enabled
    // / set_param directly would race the SDK thread's process() call which
    // reads the same fields under chain_mutex().
    chain->set_stage_enabled(stage.toStdString(), cb->isChecked());
    if (stage == "polarity_filter") {
        auto* cmb = combos_.value("polarity_filter|polarity");
        if (cmb) chain->set_stage_param(stage.toStdString(), "polarity",
                                        std::to_string(cmb->currentIndex()));
    }
    if (cb->isChecked()) {
        emit info_message(tr("Preprocess: %1 on").arg(stage));
    }
    emit stage_toggled(stage, cb->isChecked());
}

void PreprocessingPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    setEnabled(true);
    // Re-apply any stages that were enabled before the camera connected
    // (e.g. via the Preprocess menu while disconnected). Without this the
    // checkbox would show checked but the FilterChain stage stays disabled.
    for (auto it = enables_.constBegin(); it != enables_.constEnd(); ++it) {
        if (it.value() && it.value()->isChecked()) {
            apply_stage(it.key());
        }
    }
}

void PreprocessingPanel::on_camera_disconnected() {
    // Disable every stage in the FilterChain BEFORE nullifying camera_
    // (the chain would otherwise keep filtering the next camera's stream
    // with no UI indication), but KEEP the checkboxes: the user's display
    // transforms (flip X/Y, polarity filter) are a panel-level preference,
    // and on_camera_connected() replays every still-checked stage onto the
    // new camera's fresh FilterChain. Clearing the checkboxes here used to
    // make flip X / flip Y silently reset on every camera switch.
    if (camera_) {
        auto* chain = camera_->filter_chain();
        for (auto it = enables_.constBegin(); it != enables_.constEnd(); ++it) {
            chain->set_stage_enabled(it.key().toStdString(), false);
        }
    }
    camera_ = nullptr;
    setEnabled(false);
}

void PreprocessingPanel::set_stage_enabled(const QString& stage, bool on) {
    auto* cb = enables_.value(stage);
    if (!cb) return;
    // Block the toggled signal so we don't re-enter apply_stage from the
    // checkbox; apply manually so the FilterChain is still updated. The
    // stage_toggled signal emitted by apply_stage syncs the menu (whose
    // actions use 'triggered', so setChecked won't re-enter this path).
    QSignalBlocker b(cb);
    cb->setChecked(on);
    apply_stage(stage);
}

bool PreprocessingPanel::is_stage_enabled(const QString& stage) const {
    auto* cb = enables_.value(stage);
    return cb ? cb->isChecked() : false;
}

} // namespace gui
