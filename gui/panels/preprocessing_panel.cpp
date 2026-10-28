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

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>

namespace gui {

namespace {
// TEMPORARY flip-switch diagnostics (P3): the preprocessing panel owns the
// FilterChain stage enable/replay cycle across camera switches; every state
// transition is logged to %TEMP%\ebplus-open-diag.log so a reported switch
// failure shows the exact disconnect → replay sequence. Remove once the
// switch path is confirmed.
void flip_diag_line(const char* what, const char* detail = nullptr) {
    const char* tmp = std::getenv("TEMP");
    if (!tmp) return;
    std::FILE* f = std::fopen((std::string(tmp) + "\\ebplus-open-diag.log").c_str(), "a");
    if (!f) return;
    const std::time_t now = std::time(nullptr);
    std::tm tm_buf{};
    localtime_s(&tm_buf, &now);
    char stamp[16];
    std::strftime(stamp, sizeof(stamp), "%H:%M:%S", &tm_buf);
    std::fputs("[", f);
    std::fputs(stamp, f);
    std::fputs("] [flip] ", f);
    std::fputs(what, f);
    if (detail && *detail) {
        std::fputs(": ", f);
        std::fputs(detail, f);
    }
    std::fputs("\n", f);
    std::fclose(f);
}
} // namespace

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

void PreprocessingPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    setEnabled(true);
    flip_diag_line("panel connected; replaying checked stages"); // TEMP
    // Re-apply any stages that were enabled before the camera connected
    // (e.g. via the Preprocess menu while disconnected). Without this the
    // checkbox would show checked but the FilterChain stage stays disabled.
    for (auto it = enables_.constBegin(); it != enables_.constEnd(); ++it) {
        if (it.value() && it.value()->isChecked()) {
            flip_diag_line("replay", it.key().toUtf8().constData()); // TEMP
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
    flip_diag_line(camera_ ? "panel disconnected; disabling chain stages"
                           : "panel disconnected; camera_ already null"); // TEMP
    if (camera_) {
        auto* chain = camera_->filter_chain();
        for (auto it = enables_.constBegin(); it != enables_.constEnd(); ++it) {
            if (it.value() && it.value()->isChecked()) {
                flip_diag_line("disable (checkbox stays on)",
                               it.key().toUtf8().constData()); // TEMP
            }
            chain->set_stage_enabled(it.key().toStdString(), false);
        }
    }
    camera_ = nullptr;
    setEnabled(false);
}

void PreprocessingPanel::apply_stage(const QString& stage) {
    if (!camera_) {
        flip_diag_line("apply_stage SKIPPED (camera_ null)",
                       stage.toUtf8().constData()); // TEMP
        return;
    }
    auto* chain = camera_->filter_chain();
    auto* cb = enables_.value(stage);
    if (!cb) return;
    flip_diag_line(cb->isChecked() ? "apply ON" : "apply OFF",
                   stage.toUtf8().constData()); // TEMP
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
