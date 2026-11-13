#pragma once
#include "controller.h"
#include <QElapsedTimer>
#include <QJsonArray>
#include <QMainWindow>
#include <QPointer>

class QComboBox;
class QDialog;
class QLineEdit;
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;
class QStackedWidget;
class QListWidget;
class QLabel;
class QDockWidget;
class QSlider;
class QProgressBar;
class QPlainTextEdit;
class QTimer;
class QToolButton;
class QTreeWidget;

namespace gui {
class BlinkingChessboardDisplay;
}
namespace gui::multicam {
class Preview;
class PreviewArea;
class TriggerPlot;
class MainboardDiagramWidget;
class QuickAnalysisPlotWidget;
class Workspace final : public QMainWindow {
    Q_OBJECT
  public:
    explicit Workspace(QWidget *parent = nullptr, Controller *controller = nullptr);
    ~Workspace() override;
    bool may_leave() const;
    void deactivate();
    QAction *toolbox_action() const;
    RigConfig configuration() const;
    Step current_step() const;
    WorkspaceModule current_workspace() const { return active_module_; }
    void select_workspace(WorkspaceModule module);
    void select_layout_preset(RigLayoutPreset preset);
    void refresh_profiles_ui();
    void display_snapshot(const WorkflowSnapshot &);
  signals:
    void leave_requested();
    void recording_changed(bool);

  protected:
    bool eventFilter(QObject *, QEvent *) override;
    void changeEvent(QEvent *) override;
    void keyPressEvent(QKeyEvent *) override;

  private:
    void apply_theme();
    void apply_quick_defaults();
    void update_config();
    void update_blockers();
    void update_navigation();
    void update_visible_details();
    void update_calibration_guidance();
    void build_guided_pages();
    void update_guided_journey();
    void set_guided_phase(Step, int);
    void perform_calibration_action();
    GuidanceContext guidance_context() const;
    void select_step(int);
    void refresh_devices(const QVector<DeviceEntry> &);
    void show_board();
    void show_biases_dialog();
    void refresh_biases_dialog();
    void start_busy_feedback(const QString &label, bool connecting_cameras = false);
    void stop_busy_feedback();
    // Ends the inline progress bar: fills it to 100 % with a final label (if given), then hides it.
    void finish_busy_progress(const QString &final_text = {});
    void unlock_board(const QString &);
    void locate(QString, Step);
    void build_three_workspaces(QWidget *header_host, QWidget *body_host, QWidget *center_host);
    void update_three_workspaces();
    QWidget *page(Step);
    QToolButton *command(QWidget *, const QString &text, const QString &icon, const QString &id,
                         const std::function<void()> &);
    Controller *controller_ = nullptr;
    RigConfig config_;
    WorkflowSnapshot state_;
    WorkspaceModule active_module_ = WorkspaceModule::Calibrate;
    QStackedWidget *pages_ = nullptr;
    QListWidget *navigation_ = nullptr;
    QDockWidget *toolbox_ = nullptr;
    QWidget *tools_panel_ = nullptr;
    QStackedWidget *dock_stack_ = nullptr;
    QWidget *left_calibrate_panel_ = nullptr;
    QWidget *library_sidebar_ = nullptr;
    QWidget *post_take_banner_ = nullptr;
    QLabel *post_take_summary_ = nullptr;
    QString dismissed_take_directory_;
    QWidget *library_header_bar_ = nullptr;
    QLabel *library_header_label_ = nullptr;
    QWidget *capture_sync_drawer_ = nullptr;
    bool capture_sync_drawer_open_ = false;
    TriggerPlot *capture_triggers_ = nullptr;
    QLabel *drawer_sync_status_ = nullptr;
    QToolButton *drawer_override_btn_ = nullptr;
    QLabel *drawer_sync_table_ = nullptr;
    QLabel *drawer_sync_diag_ = nullptr;
    QWidget *capture_control_bar_ = nullptr;
    QLabel *capture_sync_pill_ = nullptr;
    QToolButton *capture_drawer_toggle_ = nullptr;
    QLineEdit *capture_take_label_ = nullptr;
    QComboBox *capture_duration_combo_ = nullptr;
    QToolButton *capture_record_btn_ = nullptr;
    QWidget *capture_profile_sidebar_ = nullptr;
    QComboBox *capture_profile_combo_ = nullptr;
    MainboardDiagramWidget *capture_diagram_ = nullptr;
    QLabel *capture_profile_metrics_ = nullptr;
    QListWidget *capture_profile_takes_ = nullptr;
    QLineEdit *cal_new_profile_name_ = nullptr;
    std::array<QToolButton *, 3> cal_layout_btns_{};
    MainboardDiagramWidget *cal_diagram_ = nullptr;
    QLabel *cal_baseline_sanity_card_ = nullptr;
    QToolButton *cal_save_profile_btn_ = nullptr;
    QLineEdit *library_search_edit_ = nullptr;
    QTreeWidget *library_tree_ = nullptr;
    QLabel *library_profile_summary_ = nullptr;
    QWidget *quick_analysis_dock_ = nullptr;
    std::array<QToolButton *, 4> analysis_tab_btns_{};
    QuickAnalysisPlotWidget *analysis_plot_ = nullptr;
    std::array<QToolButton *, 3> workspace_mode_btns_{};
    std::array<Preview *, 3> previews_{};
    PreviewArea *preview_area_ = nullptr;
    TriggerPlot *triggers_ = nullptr;
    QLabel *message_ = nullptr, *guide_ = nullptr, *counts_ = nullptr, *board_status_ = nullptr,
           *record_status_ = nullptr, *time_ = nullptr;
    QProgressBar *stability_ = nullptr;
    QListWidget *blocker_list_ = nullptr;
    QListWidget *observations_ = nullptr;
    QPlainTextEdit *reports_ = nullptr;
    QWidget *transport_ = nullptr;
    QSlider *slider_ = nullptr;
    QToolButton *play_ = nullptr;
    QToolButton *replay_ = nullptr;
    QElapsedTimer scrub_timer_;
    bool resume_after_scrub_ = false;
    QLineEdit *project_ = nullptr, *delta_ = nullptr, *rms_ = nullptr, *output_ = nullptr, *jump_ = nullptr;
    std::array<QComboBox *, 3> devices_{};
    std::array<QLabel *, 3> device_readbacks_{};
    QComboBox *master_ = nullptr, *reference_ = nullptr, *display_ = nullptr, *supplemental_ = nullptr,
              *speed_ = nullptr;
    QDoubleSpinBox *square_ = nullptr;
    QSpinBox *training_ = nullptr, *validation_ = nullptr, *duration_ = nullptr, *capture_duration_ = nullptr;
    QSpinBox *validation_view_ = nullptr;
    QLabel *sync_status_ = nullptr;
    QLabel *stage_title_ = nullptr, *stage_status_ = nullptr, *stage_action_ = nullptr,
           *next_reason_ = nullptr, *alert_banner_ = nullptr;
    QProgressBar *busy_progress_ = nullptr;
    quint64 progress_generation_ = 0;
    bool progress_holding_ = false; // completed bar is being shown briefly; don't hide it early
    quint64 pinned_sync_attempt_ = 0;
    QLabel *capture_status_ = nullptr, *storage_status_ = nullptr;
    QListWidget *calibration_checklist_ = nullptr;
    QToolButton *calibration_primary_ = nullptr, *calibration_advanced_toggle_ = nullptr;
    QWidget *calibration_advanced_ = nullptr;
    QString calibration_action_, last_export_path_;
    Step calibration_action_step_ = Step::Calibration;
    QCheckBox *roles_ = nullptr, *optics_ = nullptr, *locked_ = nullptr, *fullscreen_ = nullptr,
              *timed_ = nullptr, *reuse_ = nullptr, *include_ = nullptr, *trigger_wiring_ = nullptr,
              *discard_incomplete_takes_ = nullptr;
    QPointer<QDialog> biases_dialog_;
    QLabel *biases_live_status_ = nullptr;
    QTimer *busy_timer_ = nullptr;
    QElapsedTimer busy_elapsed_;
    bool ui_busy_ = false;
    bool ui_connecting_cameras_ = false;
    QString ui_busy_label_;
    QHash<QString, QWidget *> fields_;
    QHash<QString, QToolButton *> commands_;
    std::array<StageGate, 5> gates_;
    std::array<QStackedWidget *, 5> guided_stacks_{};
    std::array<QComboBox *, 5> guided_selectors_{};
    std::array<QLabel *, 5> journey_results_{};
    std::array<QVector<QToolButton *>, 5> substep_pills_{};
    std::array<int, 5> guided_phase_{};
    QLabel *header_summary_ = nullptr;
    int last_accepted_pose_total_ = 0;
    QWidget *calibration_shared_ = nullptr;
    QToolButton *pending_details_ = nullptr;
    QString last_calibration_action_;
    bool building_journey_ = false;
    QStringList displayed_observations_;
    QJsonObject displayed_profile_, displayed_manifest_, displayed_report_;
    QJsonArray validation_overlays_;
    QJsonArray selected_validation_overlay_;
    int displayed_validation_view_ = -1;
    bool profile_cached_ = false;
    bool report_cached_ = false;
    QPointer<gui::BlinkingChessboardDisplay> board_;
    QSize board_size_;
    bool updating_ = false;
};
} // namespace gui::multicam

