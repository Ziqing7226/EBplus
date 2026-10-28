// gui/panels/algorithms_panel.cpp

#include "algorithms_panel.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMap>
#include <QTimer>

#include "algo_bridge/algo_bridge.h"
#include "calibration/calib_defaults.h"

namespace gui {

AlgorithmsPanel::AlgorithmsPanel(AlgoBridge* bridge, QWidget* parent)
    : AbstractPanel(parent), bridge_(bridge) {
    build_ui();
}

void AlgorithmsPanel::build_ui() {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);

    // Global Algorithm ROI selector — always visible at the top, above the
    // scrollable algorithm list. All self-developed algorithms share this ROI
    // (design §5.6.6). Per-algorithm roi_* params are no longer shown in each
    // algo's parameter editor; they're controlled exclusively here.
    build_roi_selector(outer);

    // Global Preprocessing selector (v1.1.0): stackable noise filter + 1/4
    // downsample applied AFTER the ROI (order: ROI → filter → downsample).
    // These overlay on top of any main algorithm and are NOT mutually
    // exclusive with it. Per-algorithm preproc_* params are skipped in each
    // algo's parameter editor and controlled exclusively here.
    build_preproc_selector(outer);

    // Algorithm category groups are added directly to the outer layout —
    // no inner QScrollArea. The outer SettingsPanel scroll area already
    // provides scrolling for the entire sidebar page (§12.2.5).
    auto* host = new QWidget(this);
    auto* layout = new QVBoxLayout(host);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);
    outer->addWidget(host);

    if (!bridge_) {
        auto* lbl = new QLabel(tr("Algorithm bridge unavailable."), host);
        layout->addWidget(lbl);
        layout->addStretch(1);
        return;
    }

    // Flood-guard sync (audit §五-E2): when an instance auto-disables on a
    // sustained event-rate spike, the bridge invokes this callback from the
    // SDK data thread; the queued signal unchecks the sidebar checkbox on
    // the GUI thread so the UI cannot claim the algo is still running.
    bridge_->set_overload_callback([this](const std::string& name) {
        emit algorithm_overloaded(QString::fromStdString(name));
    });
    connect(this, &AlgorithmsPanel::algorithm_overloaded, this,
            &AlgorithmsPanel::on_algorithm_overloaded, Qt::QueuedConnection);

    // Group algorithms by category. Only self-developed algorithms are shown
    // here: OpenEB-wrapped filters have no real backend in AlgoBridge and are
    // controlled via the Preprocess menu / PreprocessingPanel instead.
    // The former "cv" (Computer Vision) group is merged into Analytics
    // (2026-08-22, user decision) — the registration category field is
    // unchanged so config round-trips keep their stored values.
    algos_ = bridge_->list_algos();
    QMap<QString, std::vector<const AlgoInfo*>> by_cat;
    for (const auto& a : algos_) {
        if (a.source != "self") continue;
        QString cat = QString::fromStdString(a.category);
        if (cat == QLatin1String("cv")) cat = QStringLiteral("analytics");
        by_cat[cat].push_back(&a);
    }

    const QMap<QString, QString> cat_titles = {
        {"analytics",       tr("Analytics")},
    };

    for (auto it = by_cat.constBegin(); it != by_cat.constEnd(); ++it) {
        const QString title = cat_titles.value(it.key(), it.key());
        auto* gb = new QGroupBox(title, host);
        auto* form = new QFormLayout(gb);
        form->setContentsMargins(6, 6, 6, 6);

        for (const auto* a : it.value()) {
            // OpenEB-wrapped algorithms (source != "self") have no real
            // backend in AlgoBridge — their parameters don't take effect here.
            // They are controlled via the Preprocess menu / PreprocessingPanel
            // FilterChain. Skip them to avoid presenting dead controls.
            if (a->source != "self") continue;

            auto* cb = new QCheckBox(QString::fromStdString(a->display_name), gb);
            // Surface registered caveats in a tooltip, if any.
            if (!a->description.empty()) {
                cb->setToolTip(QString::fromStdString(a->description));
            }
            checkboxes_[a->name] = cb;
            form->addRow(cb);

            // Parameter editor (shown only when enabled).
            auto* params_host = new QWidget(gb);
            auto* pform = new QFormLayout(params_host);
            pform->setContentsMargins(20, 0, 0, 0);
            params_host->setVisible(false);

            const std::string algo_name = a->name;
            algo_panel_state_[algo_name].params_host = params_host;
            // Match a default value to an enum_values entry. Entries may be
            // "N=Label" (match on the "N" prefix) or plain values.
            auto match_enum_index = [](const std::vector<std::string>& vals,
                                       const std::string& def) -> int {
                for (size_t i = 0; i < vals.size(); ++i) {
                    const auto& v = vals[i];
                    const auto eq = v.find('=');
                    const std::string token = (eq == std::string::npos)
                        ? v : v.substr(0, eq);
                    if (token == def) return static_cast<int>(i);
                }
                return -1;
            };
            for (const auto& p : a->params) {
                // Skip per-algorithm ROI params — they're controlled by the
                // global Algorithm ROI selector at the top of the panel.
                if (p.key == "roi_enabled" || p.key == "roi_x" ||
                    p.key == "roi_y" || p.key == "roi_w" ||
                    p.key == "roi_h") continue;
                // Skip per-algorithm preproc_* params — they're controlled by
                // the global Preprocessing selector at the top of the panel.
                if (p.key.rfind("preproc_", 0) == 0) continue;

                auto* lbl = new QLabel(QString::fromStdString(p.display_name),
                                       params_host);
                const std::string param_key = p.key;
                QWidget* w = nullptr;
                if (p.type == "enum") {
                    auto* cmb = new QComboBox(params_host);
                    for (const auto& v : p.enum_values) cmb->addItem(QString::fromStdString(v));
                    const int idx = match_enum_index(p.enum_values, p.default_value);
                    if (idx >= 0) cmb->setCurrentIndex(idx);
                    w = cmb;
                    // A "mode" (or "decay") enum drives per-mode parameter
                    // visibility via mode_filter. On change, apply the param
                    // and refresh which rows show. "decay" is TimeSurface's
                    // mode-equivalent: it selects Linear (0) vs Exponential (1),
                    // and decay_time_us / tau_us each carry a mode_filter that
                    // hides the inactive one — editing a parameter the renderer
                    // ignores was the root cause of "no effect in Exp mode".
                    if (p.key == "mode" || p.key == "decay") {
                        algo_panel_state_[algo_name].mode_combo = cmb;
                        connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                                [this, algo_name, param_key, cmb](int) {
                                    apply_param(algo_name, param_key, cmb->currentText().toStdString());
                                    refresh_mode_visibility(algo_name);
                                });
                    } else {
                        connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                                [this, algo_name, param_key, cmb](int) {
                                    apply_param(algo_name, param_key, cmb->currentText().toStdString());
                                });
                    }
                } else if (p.type == "bool") {
                    auto* cmb = new QComboBox(params_host);
                    cmb->addItem("false"); cmb->addItem("true");
                    cmb->setCurrentIndex(p.default_value == "true" || p.default_value == "1" ? 1 : 0);
                    w = cmb;
                    connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                            [this, algo_name, param_key, cmb](int) {
                                apply_param(algo_name, param_key, cmb->currentText().toStdString());
                            });
                } else if (p.type == "int") {
                    auto* sp = new QSpinBox(params_host);
                    bool oklo = false, okhi = false;
                    int lo = QString::fromStdString(p.min_value).toInt(&oklo);
                    int hi = QString::fromStdString(p.max_value).toInt(&okhi);
                    sp->setRange(oklo ? lo : -100000000, okhi ? hi : 100000000);
                    sp->setValue(QString::fromStdString(p.default_value).toInt());
                    w = sp;
                    connect(sp, QOverload<int>::of(&QSpinBox::valueChanged), this,
                            [this, algo_name, param_key](int v) {
                                apply_param(algo_name, param_key, std::to_string(v));
                            });
                } else if (p.type == "float") {
                    auto* sp = new QDoubleSpinBox(params_host);
                    // Apply the registered min/max (audit §五-B1): clamping in
                    // the GUI keeps the displayed value equal to the value
                    // the algorithm actually runs with (previously the GUI
                    // allowed ±1e9 and the algo clamped silently, so the two
                    // diverged). Step is 1/100 of the registered range.
                    bool oklo = false, okhi = false;
                    const double lo = QString::fromStdString(p.min_value).toDouble(&oklo);
                    const double hi = QString::fromStdString(p.max_value).toDouble(&okhi);
                    const double rlo = oklo ? lo : -1e9;
                    const double rhi = okhi ? hi : 1e9;
                    sp->setRange(rlo, rhi);
                    sp->setDecimals(6);
                    sp->setSingleStep((rhi - rlo) / 100.0);
                    sp->setValue(QString::fromStdString(p.default_value).toDouble());
                    w = sp;
                    connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                            [this, algo_name, param_key](double v) {
                                apply_param(algo_name, param_key, std::to_string(v));
                            });
                } else {
                    auto* le = new QLineEdit(QString::fromStdString(p.default_value), params_host);
                    w = le;
                    connect(le, &QLineEdit::textChanged, this,
                            [this, algo_name, param_key](const QString& v) {
                                apply_param(algo_name, param_key, v.toStdString());
                            });
                }
                pform->addRow(lbl, w);
                algo_panel_state_[algo_name].rows.push_back({lbl, w, p.mode_filter, p.key});
            }
            // Apply initial per-mode visibility (hides params that don't apply
            // to the default mode, e.g. E2VID params when mode=BardowVariational).
            refresh_mode_visibility(algo_name);
            form->addRow(QString(), params_host);

            // Overlay algorithms no longer open an AlgoWindow (Phase 2.6
            // debug D-1): the main display draws their overlay and the
            // main-display Zoom-to-ROI mode replaces the old window zoom
            // view. Their status line (detection counts / effective params)
            // lives in this read-only label, visible while enabled.
            QLabel* status_lbl = nullptr;
            {
                // Status line for EVERY algorithm (not just Overlay): the
                // DL algorithms report model=loaded/heuristic + dev= here —
                // without it a silent heuristic fallback is invisible to
                // the user (the label above the toggle only existed for
                // Overlay-mode algorithms).
                status_lbl = new QLabel(gb);
                status_lbl->setWordWrap(true);
                status_lbl->setVisible(false);
                form->addRow(QString(), status_lbl);
                algo_status_labels_[algo_name] = status_lbl;
            }

            connect(cb, &QCheckBox::toggled, this, [this, params_host, status_lbl, cb, a, algo_name](bool on) {
                params_host->setVisible(on);
                if (status_lbl) status_lbl->setVisible(on);
                if (on) {
                    // Algorithm mutex (design §5.6.6 — exclusive mode): only
                    // one algorithm may be enabled at a time. Covers panel
                    // checkboxes AND checkbox-less instances (e.g.
                    // sensor_self_test from the Devices button) — see
                    // mutex_disable_others.
                    mutex_disable_others(algo_name);
                    // Reuse the live instance if one already exists (e.g. the
                    // user edited a parameter before enabling). create() would
                    // discard those parameters by building a fresh instance.
                    auto inst = bridge_->find_or_create(algo_name);
                    if (inst) {
                        inst->set_enabled(true);
                        live_instances_[algo_name] = inst;
                        // create() already replayed the cached preproc_* params
                        // (BUG-R4, N3) and the cached unified ROI state
                        // (Phase 2.6 step 3), so the new instance matches the
                        // sidebar state even if no widget signal fired since
                        // app start.
                        // Auto-set 1/4 downsample based on algorithm type
                        // (§11.2-I): coordinate-halving backends (ISI,
                        // TimeSurface, HoughLine, HoughCircle) default
                        // ON per project memory; all others default OFF to
                        // avoid 4× input loss (§五-F1). Only auto-set while
                        // the user has not manually toggled the checkbox.
                        // E2VID is EXCLUDED here (Phase 3): it gets the
                        // forced save/restore automation in MainWindow
                        // instead (unconditional, like the default ROI).
                        if (!preproc_downsample_user_touched_ &&
                            algo_name != "event_to_video") {
                            const bool want = algo_halves_coords(algo_name);
                            if (preproc_downsample_cb_->isChecked() != want) {
                                QSignalBlocker b(preproc_downsample_cb_);
                                preproc_downsample_cb_->setChecked(want);
                                apply_global_preproc(
                                    "preproc_downsample", want ? "true" : "false");
                            }
                        }
                        emit info_message(tr("Algorithm enabled: %1")
                                              .arg(QString::fromStdString(a->display_name)));
                        // Request MainWindow to open the AlgoWindow so
                        // Standalone algorithms have a display and Passive
                        // algorithms get their status window. Overlay
                        // algorithms no longer open one (Phase 2.6 debug
                        // D-1): the main display draws their overlay and the
                        // Zoom-to-ROI mode covers the old window zoom view.
                        // Replace algorithms (background_mask) likewise show
                        // their output in the MAIN display — the extra
                        // AlgoWindow was redundant (just a "Waiting for
                        // events..." status), so they no longer open one.
                        if (a->display_mode != AlgoDisplayMode::Overlay &&
                            a->display_mode != AlgoDisplayMode::Replace) {
                            emit open_algo_window_requested(algo_name);
                        }
                        emit algorithm_toggled(QString::fromStdString(a->name), true);
                    } else {
                        // find_or_create should never fail for registered
                        // algorithms, but revert the checkbox defensively so
                        // the UI stays consistent if it ever does.
                        QSignalBlocker b(cb);
                        cb->setChecked(false);
                        params_host->setVisible(false);
                        emit error_message(tr("Failed to create algorithm: %1")
                                               .arg(QString::fromStdString(a->display_name)));
                        // Don't emit algorithm_toggled(name, true): the algo
                        // was never enabled and the checkbox was reverted.
                        // Emitting true would mislead MainWindow into showing
                        // "enabled" in the status bar (BUG-G15). Don't emit
                        // false either: open_algo_window_requested was never
                        // emitted, so there is no AlgoWindow to close.
                    }
                } else {
                    auto it = live_instances_.find(algo_name);
                    if (it != live_instances_.end() && it->second) {
                        it->second->set_enabled(false);
                    }
                    emit algorithm_toggled(QString::fromStdString(a->name), false);
                }
            });
        }
        layout->addWidget(gb);
    }

    layout->addStretch(1);

    // Initial build complete — subsequent mode switches are user-driven and
    // must not clobber user-customised ROI/fps (BUG-14 fix).
    first_init_ = false;
}

void AlgorithmsPanel::build_roi_selector(QVBoxLayout* parent_layout) {
    auto* gb = new QGroupBox(tr("ROI (Unified)"), this);
    auto* form = new QFormLayout(gb);
    form->setContentsMargins(6, 6, 6, 6);

    // Phase 2.6 debug D-6: reduced to a single enable checkbox + a settings
    // button opening the modal UnifiedRoiDialog (rect/mode/drag live there).
    // Same unified state as the Hardware page's ROI controls; the checkbox
    // is synced via CameraController::roi_state_changed (set_roi_enabled).
    roi_enabled_cb_ = new QCheckBox(tr("Enable ROI"), gb);
    roi_enabled_cb_->setToolTip(
        tr("Single unified ROI: hardware ROI on a live camera, software crop "
           "on file playback. Heavy algorithms (E2VID/ISI/TimeSurface/Hough) "
           "enable it automatically at the center 256×144 while enabled."));
    roi_enabled_cb_->setChecked(false);
    form->addRow(roi_enabled_cb_);

    roi_settings_btn_ = new QPushButton(tr("ROI Settings..."), gb);
    form->addRow(QString(), roi_settings_btn_);

    parent_layout->addWidget(gb);

    connect(roi_enabled_cb_, &QCheckBox::toggled, this, [this](bool on) {
        // Applied by MainWindow via CameraController::set_unified_roi.
        emit roi_enable_toggled(on);
    });
    connect(roi_settings_btn_, &QPushButton::clicked, this,
            [this]() { emit roi_settings_requested(); });
}

void AlgorithmsPanel::set_roi_enabled(bool enabled) {
    QSignalBlocker b(roi_enabled_cb_);
    roi_enabled_cb_->setChecked(enabled);
}

bool AlgorithmsPanel::algo_defaults_to_roi(const std::string& algo_name) {
    // Phase 2.6 debug D-7 default list: compute-heavy algorithms that would
    // stall at full sensor auto-enable the unified ROI at the default center
    // 256×144 (and restore the prior state on disable). XYT deliberately
    // excluded (user decision — it never used ROI). freq_detector and
    // frequency_map were removed (2026-08-21, user decision): both run
    // full-sensor now — freq_detector throttles its sweep to
    // update_interval_s and frequency_map to update_interval_ms, which keeps
    // the GUI responsive without the ROI crutch.
    // Phase 7 P10: the compute-heavy full-sensor algorithms join the list
    // (sparse/dense flow, corner, hot pixel, orientation filter/cluster,
    // EIS, cluster LIF). All their backends now re-apply tuned params
    // across the set_sensor_dimensions rebuild the ROI enable triggers.
    return algo_name == "event_to_video" ||
           algo_name == "time_surface" ||
           algo_name == "hough_line" ||
           algo_name == "hough_circle" ||
           algo_name == "sparse_optical_flow" ||
           algo_name == "dense_optical_flow" ||
           algo_name == "corner_detector" ||
           algo_name == "hot_pixel_filter" ||
           algo_name == "orientation_filter" ||
           algo_name == "orientation_cluster" ||
           algo_name == "optical_gyro" ||
           algo_name == "cluster_lif" ||
           algo_name == "direction_selective" ||
           algo_name == "line_segment";
}

void AlgorithmsPanel::set_algo_status(const std::string& name, const QString& text) {
    auto it = algo_status_labels_.find(name);
    if (it != algo_status_labels_.end() && it->second) {
        it->second->setText(text);
    }
}

void AlgorithmsPanel::set_preproc_downsample(bool on) {
    if (preproc_downsample_cb_->isChecked() == on) return;
    QSignalBlocker b(preproc_downsample_cb_);
    preproc_downsample_cb_->setChecked(on);
    apply_global_preproc("preproc_downsample", on ? "true" : "false");
}

void AlgorithmsPanel::apply_global_preproc(const std::string& key,
                                           const std::string& value) {
    // Delegate to the bridge, which iterates every live self-developed
    // instance and forwards the preproc_* parameter. Each backend's
    // Preprocessor / RoiFilter member recognises the key. Preprocessing is
    // stackable and NOT mutually exclusive with the main algorithm.
    if (bridge_) bridge_->apply_global_preproc(key, value);
    // Also forward to the display-path preprocessing (Phase 2.5): the main
    // display applies the same noise filter to the rendered stream.
    emit preproc_display_param_changed(QString::fromStdString(key),
                                       QString::fromStdString(value));
}

void AlgorithmsPanel::build_preproc_selector(QVBoxLayout* parent_layout) {
    auto* gb = new QGroupBox(tr("Preprocessing"), this);
    auto* form = new QFormLayout(gb);
    form->setContentsMargins(6, 6, 6, 6);

    // Noise filter enable (default off — opt-in denoising stage).
    preproc_filter_cb_ = new QCheckBox(tr("Noise filter"), gb);
    preproc_filter_cb_->setChecked(false);
    form->addRow(preproc_filter_cb_);

    // Filter mode (9 modes; default KNoise=8 — the O(W+H)-memory,
    // fixed-cost filter, and the backend/registry default).
    preproc_filter_mode_combo_ = new QComboBox(gb);
    preproc_filter_mode_combo_->addItem("0=BAF");
    preproc_filter_mode_combo_->addItem("1=STCF");
    preproc_filter_mode_combo_->addItem("2=Refractory");
    preproc_filter_mode_combo_->addItem("3=DWF");
    preproc_filter_mode_combo_->addItem("4=AgePolarity");
    preproc_filter_mode_combo_->addItem("5=Harmonic");
    preproc_filter_mode_combo_->addItem("6=Repetitious");
    preproc_filter_mode_combo_->addItem("7=SpatialBP");
    preproc_filter_mode_combo_->addItem("8=KNoise");
    preproc_filter_mode_combo_->setCurrentIndex(8);  // KNoise
    form->addRow(tr("Filter mode"), preproc_filter_mode_combo_);

    // Mode-specific parameter rows (BUG-3 fix). All 9 modes' params are
    // pre-created and shown/hidden based on the selected filter mode.
    // Cross-mode params (mode=-1) are always visible when the filter is on.
    preproc_params_form_ = new QFormLayout();
    preproc_params_form_->setContentsMargins(6, 6, 6, 6);
    form->addRow(preproc_params_form_);

    // Parameter definitions: {key, display, type, def, lo, hi, mode}
    // type: "i"=int, "f"=float, "b"=bool
    // SYNC (audit §五-F2): this table mirrors the preproc_params() registry in
    // gui/algo_bridge/algo_bridge.cpp — the bridge registry is the single
    // source of truth; any addition/removal/default change must be mirrored
    // here. rep_period_us/rep_tolerance_us are intentionally absent (algo
    // stores but never uses them); rep_averaging_samples was registered in
    // Phase 6 (the algo gained the setter, jAER setAveragingSamples parity).
    struct PDef {
        const char* key;
        const char* disp;
        char type;
        const char* def;
        const char* lo;
        const char* hi;
        int mode;
    };
    static const PDef pdefs[] = {
        // STCF (mode 1)
        {"preproc_filter_correlation_time_s", "STCF corr (s)", 'f', "0.025", "0.001", "0.1", 1},
        {"preproc_filter_min_neighbors", "STCF min nbr", 'i', "2", "1", "8", 1},
        {"preproc_filter_require_polarity_match", "STCF pol match", 'b', "true", "", "", 1},
        {"preproc_filter_allow_coincidence", "STCF coincide", 'b', "true", "", "", 1},
        // BAF (mode 0)
        {"preproc_filter_baf_dt_us", "BAF dt (us)", 'i', "25000", "1000", "100000", 0},
        {"preproc_filter_baf_subsample_by", "BAF subsample", 'i', "0", "0", "4", 0},
        // Refractory (mode 2)
        {"preproc_filter_refractory_us", "Refractory (us)", 'i', "10000", "100", "100000", 2},
        // DWF (mode 3)
        {"preproc_filter_dwf_window_length", "DWF win len", 'i', "64", "1", "1024", 3},
        {"preproc_filter_dwf_dist_threshold", "DWF dist", 'i', "5", "1", "1024", 3},
        {"preproc_filter_dwf_min_correlated", "DWF min corr", 'i', "1", "1", "8", 3},
        {"preproc_filter_dwf_double_mode", "DWF double", 'b', "true", "", "", 3},
        // AgePolarity (mode 4)
        {"preproc_filter_agep_tau_us", "AgePol tau (us)", 'i', "10000", "1000", "100000", 4},
        {"preproc_filter_age_threshold", "AgePol thresh", 'f', "5.0", "0.0", "8.0", 4},
        {"preproc_filter_agep_radius", "AgePol radius", 'i', "1", "1", "5", 4},
        // Harmonic (mode 5) — line_freq_hz is an enum (50 or 60 Hz), not an
        // arbitrary int. Use type 'e' so the UI presents a combo box and the
        // backend's penum definition (algo_bridge.cpp) is respected.
        {"preproc_filter_line_freq_hz", "Harm Hz", 'e', "50", "50", "60", 5},
        {"preproc_filter_notch_q", "Harm Q", 'f', "3.0", "0.1", "100.0", 5},
        {"preproc_filter_harmonic_threshold", "Harm thresh", 'f', "0.1", "0.0", "1.0", 5},
        // Repetitious (mode 6). rep_period_us/rep_tolerance_us are omitted:
        // the algo stores them but never uses them (audit §7.3).
        {"preproc_filter_rep_ratio_shorter", "Rep ratio short", 'i', "2", "1", "100", 6},
        {"preproc_filter_rep_ratio_longer", "Rep ratio long", 'i', "2", "1", "100", 6},
        {"preproc_filter_rep_min_dt_to_store_us", "Rep min dt (us)", 'i', "1000", "0", "1000000", 6},
        {"preproc_filter_rep_averaging_samples", "Rep avg samples", 'i', "3", "1", "100", 6},
        // SpatialBP (mode 7)
        {"preproc_filter_sbp_center_radius_px", "SBP center", 'i', "0", "0", "10", 7},
        {"preproc_filter_sbp_surround_radius_px", "SBP surround", 'i', "1", "1", "30", 7},
        {"preproc_filter_sbp_dt_surround_us", "SBP dt (us)", 'i', "8000", "100", "1000000", 7},
        // KNoise (mode 8) — dv-processing KNoiseFilter port
        {"preproc_filter_knoise_dt_us", "KNoise dt (us)", 'i', "2000", "100", "100000", 8},
        // Cross-mode flags
        {"preproc_filter_filter_hot_pixels", "Filter hot px", 'b', "false", "", "", -1},
        {"preproc_filter_adaptive_correlation_time", "Adaptive corr", 'b', "false", "", "", -1},
    };

    for (const auto& p : pdefs) {
        auto* lbl = new QLabel(tr(p.disp), gb);
        QWidget* w = nullptr;
        const std::string pkey = p.key;
        if (p.type == 'b') {
            auto* cmb = new QComboBox(gb);
            cmb->addItem("false"); cmb->addItem("true");
            cmb->setCurrentIndex(std::string(p.def) == "true" ? 1 : 0);
            w = cmb;
            connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                    [this, pkey, cmb](int) {
                        apply_global_preproc(pkey, cmb->currentText().toStdString());
                    });
        } else if (p.type == 'e') {
            // Enum: lo and hi are the two valid string values (e.g. "50"/"60").
            auto* cmb = new QComboBox(gb);
            cmb->addItem(QString::fromUtf8(p.lo));
            cmb->addItem(QString::fromUtf8(p.hi));
            cmb->setCurrentIndex(std::string(p.def) == p.hi ? 1 : 0);
            w = cmb;
            connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                    [this, pkey, cmb](int) {
                        apply_global_preproc(pkey, cmb->currentText().toStdString());
                    });
        } else if (p.type == 'i') {
            auto* sp = new QSpinBox(gb);
            sp->setRange(std::string(p.lo).empty() ? -100000000 : std::stoi(p.lo),
                         std::string(p.hi).empty() ? 100000000 : std::stoi(p.hi));
            sp->setValue(std::stoi(p.def));
            w = sp;
            connect(sp, QOverload<int>::of(&QSpinBox::valueChanged), this,
                    [this, pkey](int v) {
                        apply_global_preproc(pkey, std::to_string(v));
                    });
        } else { // 'f'
            auto* sp = new QDoubleSpinBox(gb);
            // Apply the registered min/max (audit §五-B1); step = range/100.
            const double lo = std::string(p.lo).empty() ? -1e9 : std::stod(p.lo);
            const double hi = std::string(p.hi).empty() ? 1e9 : std::stod(p.hi);
            sp->setRange(lo, hi);
            sp->setDecimals(6);
            sp->setSingleStep((hi - lo) / 100.0);
            sp->setValue(std::stod(p.def));
            w = sp;
            connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                    [this, pkey](double v) {
                        apply_global_preproc(pkey, std::to_string(v));
                    });
        }
        preproc_params_form_->addRow(lbl, w);
        preproc_rows_.push_back({lbl, w, p.mode, pkey});
    }

    // 1/4 downsample (default OFF — audit §五-F1): for most backends this
    // only THINS events (coordinates unchanged, 3/4 of the input silently
    // discarded); only the E2VID/ISI/TimeSurface/Hough backends actually
    // halve coordinates. The checkbox is auto-set on algorithm enable
    // (§11.2-I): ON for coordinate-halving backends (project memory),
    // OFF for all others. Once the user manually toggles, auto-setting
    // stops.
    preproc_downsample_cb_ = new QCheckBox(tr("1/4 Downsample"), gb);
    preproc_downsample_cb_->setChecked(false);
    preproc_downsample_cb_->setToolTip(tr(
        "对大多数算法仅抽稀事件（坐标不变）；仅 E2VID/ISI/TimeSurface/Hough 系做坐标减半"));
    form->addRow(preproc_downsample_cb_);

    // Undistort (default off). Applied AFTER filter + downsample. Loads the
    // YAML written by Tools → Intrinsic Wizard and builds a forward event LUT
    // (cv::undistortPoints with K adjusted for ROI origin + downsample factor).
    // Default path is identical to the wizard's default export path.
    preproc_undistort_cb_ = new QCheckBox(tr("Undistort (apply calibration)"), gb);
    preproc_undistort_cb_->setChecked(false);
    form->addRow(preproc_undistort_cb_);

    preproc_undistort_path_ = new QLineEdit(gb);
    preproc_undistort_path_->setText(default_intrinsic_yml_path());
    preproc_undistort_path_->setToolTip(tr(
        "Path to the YAML written by Tools → Intrinsic Wizard. "
        "Default path is identical to the wizard's export default."));
    preproc_undistort_browse_ = new QPushButton(tr("Browse..."), gb);
    auto* undistort_row = new QWidget(gb);
    auto* undistort_layout = new QHBoxLayout(undistort_row);
    undistort_layout->setContentsMargins(0, 0, 0, 0);
    undistort_layout->addWidget(preproc_undistort_path_, 1);
    undistort_layout->addWidget(preproc_undistort_browse_, 0);
    form->addRow(tr("Calibration file"), undistort_row);

    parent_layout->addWidget(gb);

    // Wire up: any change applies the preproc setting to all live instances.
    // These checkboxes are intentionally NOT added to checkboxes_ (the
    // algorithm-mutex map) so enabling preprocessing does not disable the
    // main algorithm — preprocessing overlays on top of it.
    connect(preproc_filter_cb_, &QCheckBox::toggled, this, [this](bool) {
        // push_all starts with preproc_filter_enabled — applying it here as
        // well rebuilt the shared filter twice per toggle.
        push_all_preproc_params();
        refresh_preproc_params();
    });
    connect(preproc_downsample_cb_, &QCheckBox::toggled, this, [this](bool on) {
        apply_global_preproc("preproc_downsample", on ? "true" : "false");
        // Mark as user-touched so the auto-toggle on algorithm enable
        // (§11.2-I) stops overriding the user's explicit choice.
        preproc_downsample_user_touched_ = true;
    });
    connect(preproc_undistort_cb_, &QCheckBox::toggled, this, [this](bool on) {
        apply_global_preproc("preproc_undistort_enabled", on ? "true" : "false");
    });
    connect(preproc_undistort_path_, &QLineEdit::textChanged, this, [this](const QString& text) {
        apply_global_preproc("preproc_undistort_path", text.toStdString());
    });
    connect(preproc_undistort_browse_, &QPushButton::clicked, this, [this]() {
        const QString start = preproc_undistort_path_->text();
        const QString path = QFileDialog::getOpenFileName(
            this, tr("Select calibration YAML"),
            QFileInfo(start).absolutePath(),
            tr("YAML (*.yml *.yaml);;All Files (*)"));
        if (!path.isEmpty()) {
            preproc_undistort_path_->setText(path);
        }
    });
    connect(preproc_filter_mode_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) {
                // push_all pushes the mode too (and the rows that follow it).
                push_all_preproc_params();
                refresh_preproc_params();
            });

    // Show the rows matching the default mode (KNoise=8).
    refresh_preproc_params();

    // Forward the default undistort path ONCE so the default actually works:
    // setText() above ran before the textChanged connect, so without this the
    // backends / display path would hold an EMPTY path until the user edited
    // the field — enabling Undistort with the defaults then failed to load any
    // intrinsics. Deferred to the event loop because MainWindow connects
    // preproc_display_param_changed after this panel is constructed; the
    // bridge side (preproc_cache_) also seeds instances created later.
    QTimer::singleShot(0, this, [this]() {
        apply_global_preproc("preproc_undistort_path",
                             default_intrinsic_yml_path().toStdString());
    });
}

void AlgorithmsPanel::refresh_preproc_params() {
    if (!preproc_filter_mode_combo_) return;
    const int mode = preproc_filter_mode_combo_->currentIndex();
    const bool filter_on = preproc_filter_cb_ && preproc_filter_cb_->isChecked();
    for (auto& row : preproc_rows_) {
        const bool visible = filter_on && (row.mode < 0 || row.mode == mode);
        if (row.label) row.label->setVisible(visible);
        if (row.field) row.field->setVisible(visible);
    }
}

void AlgorithmsPanel::push_all_preproc_params() {
    // MASTER controls first — the conditioner's filter instance depends on
    // enabled/mode, and the rows below are meaningless without them. Without
    // this push a config load restored nothing (config restore drives the
    // bridge's per-instance storage directly; the conditioner route starts
    // at the panel — review fix 2026-08-21).
    apply_global_preproc("preproc_filter_enabled",
                         preproc_filter_cb_->isChecked() ? "true" : "false");
    apply_global_preproc("preproc_filter_mode",
                         std::to_string(preproc_filter_mode_combo_->currentIndex()));
    apply_global_preproc("preproc_downsample",
                         preproc_downsample_cb_->isChecked() ? "true" : "false");
    apply_global_preproc("preproc_undistort_enabled",
                         preproc_undistort_cb_->isChecked() ? "true" : "false");
    apply_global_preproc("preproc_undistort_path",
                         preproc_undistort_path_->text().toStdString());
    for (const auto& row : preproc_rows_) {
        if (!row.field) continue;
        std::string value;
        if (auto* cmb = qobject_cast<QComboBox*>(row.field)) {
            value = cmb->currentText().toStdString();
        } else if (auto* sp = qobject_cast<QSpinBox*>(row.field)) {
            value = std::to_string(sp->value());
        } else if (auto* dsp = qobject_cast<QDoubleSpinBox*>(row.field)) {
            value = std::to_string(dsp->value());
        } else {
            continue;
        }
        apply_global_preproc(row.key, value);
    }
}

void AlgorithmsPanel::apply_param(const std::string& algo_name,
                                  const std::string& param_key,
                                  const std::string& value) {
    // Lazily create the instance so parameter edits are recorded even before
    // the enable checkbox is toggled. find_or_create preserves any previously
    // set parameters instead of overwriting them with defaults.
    auto it = live_instances_.find(algo_name);
    if (it == live_instances_.end() || !it->second) {
        auto inst = bridge_ ? bridge_->find_or_create(algo_name) : nullptr;
        if (!inst) return;
        // Use operator[] (not emplace) so a pre-existing null entry is
        // replaced. std::unordered_map::emplace is a no-op if the key
        // already exists, which would leave a null shared_ptr and crash
        // on the following dereference.
        live_instances_[algo_name] = inst;
        it = live_instances_.find(algo_name);
    }
    it->second->set_param(param_key, value);

    // BUG-G2: after setting model_path, the E2VID num_bins is dictated by the
    // loaded ONNX model. Read it back and update the GUI field so the user
    // sees the actual value the algo will use (not the stale typed value).
    if (param_key == "model_path") {
        const std::string nb = it->second->get_param("num_bins");
        if (!nb.empty()) {
            auto state_it = algo_panel_state_.find(algo_name);
            if (state_it != algo_panel_state_.end()) {
                for (auto& row : state_it->second.rows) {
                    if (row.key == "num_bins" && row.field) {
                        QSignalBlocker b(row.field);
                        if (auto* sp = qobject_cast<QSpinBox*>(row.field)) {
                            sp->setValue(QString::fromStdString(nb).toInt());
                        }
                        break;
                    }
                }
            }
        }
        // §五-H1: a failed ONNX load silently falls back to the heuristic
        // path — warn once per panel session so the user does not mistake
        // the heuristic output for "E2VID quality".
        if (algo_name == "event_to_video" &&
            it->second->get_param("model_loaded") == "false" &&
            !e2vid_model_error_shown_) {
            e2vid_model_error_shown_ = true;
            emit error_message(tr("E2VID model failed to load — using the "
                                  "heuristic fallback reconstruction."));
        }
    }
}

void AlgorithmsPanel::on_algorithm_overloaded(const QString& name) {
    // Flood guard tripped (audit §五-E2): the instance already disabled
    // itself. Clear the overload flag (so MainWindow's per-frame statusBar
    // warning stops), uncheck the sidebar checkbox, and notify once.
    const std::string n = name.toStdString();
    if (bridge_) {
        if (auto inst = bridge_->find_live(n)) {
            inst->clear_overload();
        }
    }
    set_algo_enabled(n, false);
    emit algorithm_toggled(name, false);
    emit error_message(tr("Algorithm auto-disabled (event rate too high): %1. "
                          "Re-enable it to retry.").arg(name));
}

void AlgorithmsPanel::refresh_mode_visibility(const std::string& algo_name) {
    auto it = algo_panel_state_.find(algo_name);
    if (it == algo_panel_state_.end()) return;
    auto& state = it->second;
    if (!state.mode_combo) return;  // algo has no "mode" enum
    const int idx = state.mode_combo->currentIndex();
    const std::string idx_str = std::to_string(idx);
    for (auto& row : state.rows) {
        if (row.mode_filter.empty()) continue;  // common param: always visible
        // mode_filter is a comma-separated list of mode indices ("0", "1,2").
        bool visible = false;
        std::size_t pos = 0;
        while (pos < row.mode_filter.size()) {
            const auto comma = row.mode_filter.find(',', pos);
            const std::string token = (comma == std::string::npos)
                ? row.mode_filter.substr(pos)
                : row.mode_filter.substr(pos, comma - pos);
            if (token == idx_str) { visible = true; break; }
            if (comma == std::string::npos) break;
            pos = comma + 1;
        }
        if (row.label) row.label->setVisible(visible);
        if (row.field) row.field->setVisible(visible);
    }

    // Auto-set mode-appropriate ROI and output_fps only during initial build
    // (design §4.4.2): all three event_to_video modes default to a 128×128
    // center ROI. The shared 1/4 downsample defaults to OFF here; it is
    // auto-enabled on algorithm enable via algo_halves_coords() (§11.2-I),
    // NOT in this first_init_ path. 24 fps is a comfortable target across
    // all modes. Only event_to_video has a "mode" enum, so this code only
    // runs for it.
    // BUG-14 fix: skip ROI/fps reset on user-driven mode switches so
    // user-customised values are preserved.
    if (!first_init_) return;

    // Phase 2.6: the old first_init_ block used to apply a center-128 ROI to
    // EVERY future algorithm instance. With the unified ROI defaulting to
    // OFF (user decision), e2v's mode init no longer applies any ROI — the
    // default-ROI-per-algorithm auto-enable is reintroduced in step 3
    // through the unified path.
    const int target_fps = 24;

    // output_fps — find the per-algo param row by key.
    for (auto& row : state.rows) {
        if (row.key == "output_fps") {
            auto* sp = qobject_cast<QSpinBox*>(row.field);
            if (sp) { QSignalBlocker b(sp); sp->setValue(target_fps); }
            apply_param(algo_name, "output_fps", std::to_string(target_fps));
            break;
        }
    }
}

void AlgorithmsPanel::set_algo_enabled(const std::string& name, bool on) {
    auto it = checkboxes_.find(name);
    if (it == checkboxes_.end() || !it->second) {
        // Checkbox-less entries (e.g. sensor_self_test from the Devices
        // panel button): no checkbox to sync, but the algorithm mutex still
        // applies — enabling it must disable every other algorithm.
        if (on) mutex_disable_others(name);
        return;
    }
    // Block signals so this programmatic change does not re-enter the toggled
    // handler (which would create/enable instances and emit algorithm_toggled,
    // causing sync loops with the Algorithm menu / AlgoWindow).
    QSignalBlocker b(it->second);
    it->second->setChecked(on);

    // Show/hide the target's parameter editor to match the new checkbox
    // state. QSignalBlocker suppressed the toggled handler which would
    // normally do this (BUG-M2).
    auto st = algo_panel_state_.find(name);
    if (st != algo_panel_state_.end() && st->second.params_host) {
        st->second.params_host->setVisible(on);
    }

    // Algorithm mutex: when turning an algo on programmatically (e.g. from
    // on_open_algo_window), uncheck every other algo so only one is live at
    // a time. The toggled-handler path enforces mutex itself; this covers
    // the programmatic path.
    if (on) mutex_disable_others(name);
}

void AlgorithmsPanel::mutex_disable_others(const std::string& winner) {
    // Algorithm mutex (design §5.6.6 — exclusive mode): only one algorithm
    // may be enabled at a time. Covers two kinds of participants:
    //   (a) panel algorithms (checkboxes) — uncheck + hide params + disable;
    //   (b) checkbox-less instances created outside the panel (e.g.
    //       sensor_self_test from the Devices button) — disable + emit
    //       algorithm_toggled so MainWindow closes their windows.
    for (auto& [other_name, other_cb] : checkboxes_) {
        if (other_name == winner) continue;
        if (!other_cb || !other_cb->isChecked()) continue;
        QSignalBlocker ob(other_cb);
        other_cb->setChecked(false);
        // Hide the other algo's parameter editor (BUG-M2: QSignalBlocker
        // suppresses the toggled handler that would normally hide it).
        auto ost = algo_panel_state_.find(other_name);
        if (ost != algo_panel_state_.end() && ost->second.params_host) {
            ost->second.params_host->setVisible(false);
        }
        auto oi = live_instances_.find(other_name);
        if (oi != live_instances_.end() && oi->second) {
            oi->second->set_enabled(false);
        }
        emit algorithm_toggled(QString::fromStdString(other_name), false);
    }
    if (!bridge_) return;
    for (const auto& inst : bridge_->list_live()) {
        if (!inst || !inst->is_enabled()) continue;
        const auto& other = inst->info().name;
        if (other == winner) continue;
        if (checkboxes_.find(other) != checkboxes_.end()) continue;  // handled above
        inst->set_enabled(false);
        emit algorithm_toggled(QString::fromStdString(other), false);
    }
}

void AlgorithmsPanel::refresh_param_values() {
    if (!bridge_) return;
    bool masters_synced = false;
    for (auto& [algo_name, state] : algo_panel_state_) {
        auto inst = bridge_->find_live(algo_name);
        for (auto& row : state.rows) {
            // Live instance values win (registered keys always produce a
            // value, possibly a meaningful empty string); fall back to the
            // N1 cache for non-live algorithms. nullopt = not configured —
            // keep the widget's current value instead of blanking it.
            std::string val;
            bool have_value = false;
            if (inst) {
                val = inst->get_param(row.key);
                have_value = true;
            } else if (auto cv = bridge_->get_cached_algo_param(algo_name, row.key)) {
                val = std::move(*cv);
                have_value = true;
            }
            if (!have_value) continue;
            if (!row.field) continue;
            const QSignalBlocker blk(row.field);
            if (auto* cmb = qobject_cast<QComboBox*>(row.field)) {
                if (val.empty()) continue;
                // Enum/bool combos: item text may be "idx=Label" — match the
                // full text first, then the token before '='.
                int idx = -1;
                for (int i = 0; i < cmb->count(); ++i) {
                    const QString t = cmb->itemText(i);
                    if (t.toStdString() == val ||
                        t.section('=', 0, 0).toStdString() == val) {
                        idx = i;
                        break;
                    }
                }
                // Bool combos ("false"/"true") also accept legacy 0/1 from
                // hand-edited or older config files.
                if (idx < 0 && cmb->count() == 2 &&
                    cmb->itemText(0) == "false" && cmb->itemText(1) == "true") {
                    if (val == "0") idx = 0;
                    else if (val == "1") idx = 1;
                }
                if (idx >= 0) cmb->setCurrentIndex(idx);
            } else if (auto* sp = qobject_cast<QSpinBox*>(row.field)) {
                if (val.empty()) continue;
                sp->setValue(QString::fromStdString(val).toInt());
            } else if (auto* dsp = qobject_cast<QDoubleSpinBox*>(row.field)) {
                if (val.empty()) continue;
                dsp->setValue(QString::fromStdString(val).toDouble());
            } else if (auto* le = qobject_cast<QLineEdit*>(row.field)) {
                // An empty string IS a meaningful value for text params
                // (e.g. clearing model_path) — sync it too, otherwise the
                // panel keeps displaying the pre-load path while the algo
                // runs with none.
                le->setText(QString::fromStdString(val));
            }
        }
        // Row visibility follows the (possibly updated) mode combo.
        if (state.mode_combo) refresh_mode_visibility(algo_name);

        // Preproc MASTER controls (filter enabled / mode / downsample /
        // undistort): global keys mirrored on every self instance and the
        // bridge cache. Panel edits push them out immediately, so this
        // mirror only diverges on a config load (apply_algo_state writes
        // instance storage directly) — syncing from the first instance that
        // provides them re-syncs the controls after a load. Review fix
        // 2026-08-21: without this, a config with the noise filter enabled
        // restored nothing (the widgets kept the pre-load state and the
        // shared conditioners were never told).
        if (!masters_synced) {
            // Copy the structured binding: implicitly capturing it in a
            // lambda is a C++20 extension and AppleClang rejects the C++17
            // form with a warning.
            const std::string algo_name_str = algo_name;
            auto read_master = [&, algo_name_str](const char* key) -> std::string {
                if (inst) return inst->get_param(key);
                auto cv = bridge_->get_cached_algo_param(algo_name_str, key);
                return cv ? *cv : std::string();
            };
            const std::string fen = read_master("preproc_filter_enabled");
            if (fen.empty()) continue;  // this algo carries no master mirror —
                                       // try the next one (the flag must NOT
                                       // latch on an algo with no values)
            {
                QSignalBlocker b(preproc_filter_cb_);
                preproc_filter_cb_->setChecked(fen == "true");
            }
            const std::string fmode = read_master("preproc_filter_mode");
            if (!fmode.empty()) {
                int mi = QString::fromStdString(fmode).toInt();
                if (mi < 0 || mi > 8) mi = 8;
                QSignalBlocker b(preproc_filter_mode_combo_);
                preproc_filter_mode_combo_->setCurrentIndex(mi);
            }
            const std::string ds = read_master("preproc_downsample");
            if (!ds.empty()) {
                QSignalBlocker b(preproc_downsample_cb_);
                preproc_downsample_cb_->setChecked(ds == "true");
            }
            const std::string ue = read_master("preproc_undistort_enabled");
            if (!ue.empty()) {
                QSignalBlocker b(preproc_undistort_cb_);
                preproc_undistort_cb_->setChecked(ue == "true");
            }
            const std::string up = read_master("preproc_undistort_path");
            if (!up.empty()) {
                QSignalBlocker b(preproc_undistort_path_);
                preproc_undistort_path_->setText(QString::fromStdString(up));
            }
            masters_synced = true;
        }
    }
    // Mode-scoped row visibility follows the (possibly restored) mode.
    refresh_preproc_params();
}

bool AlgorithmsPanel::algo_halves_coords(const std::string& algo_name) {
    // These backends set preproc_.halve_coords_ = true in their constructor,
    // meaning 1/4 downsample halves event coordinates (correct behavior:
    // the algorithm runs at half resolution). For all other backends,
    // downsample only thins events (3/4 discarded, coordinates unchanged) —
    // a silent 4× input loss (§五-F1).
    return algo_name == "event_to_video" ||
           algo_name == "time_surface" ||
           algo_name == "hough_line" ||
           algo_name == "hough_circle";
}

} // namespace gui
