// gui/panels/file_op_dialog.cpp

#include "file_op_dialog.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>

#include "app/file_converter.h"

namespace gui {

namespace {
// True when src and dst resolve to the same file (converting/cutting onto
// the source would overwrite the very file being read).
bool same_file(const QString& src, const QString& dst) {
    const QFileInfo s(src);
    const QString s_canon = s.canonicalFilePath();
    if (s_canon.isEmpty()) return false;
    const QFileInfo d(dst);
    QString d_canon = d.canonicalFilePath();
    if (d_canon.isEmpty()) {
        const QString cdir = d.dir().canonicalPath();
        if (cdir.isEmpty()) return false;
        d_canon = cdir + QLatin1Char('/') + d.fileName();
    }
    return s_canon == d_canon;
}
} // namespace

FileOpDialog::FileOpDialog(Mode mode, FileConverter* converter, QWidget* parent)
    : QDialog(parent), mode_(mode), converter_(converter) {
    const bool is_cut = (mode_ == Mode::Cut);
    const QString suffix = is_cut ? QStringLiteral(".raw")
                         : (mode_ == Mode::ConvertHdf5) ? QStringLiteral(".h5")
                                                        : QStringLiteral(".csv");
    setWindowTitle(is_cut ? tr("File Cutter")
                          : (mode_ == Mode::ConvertHdf5 ? tr("Convert to HDF5")
                                                        : tr("Convert to CSV")));

    auto* form = new QFormLayout(this);

    // Source row.
    auto* src_row = new QHBoxLayout;
    edt_source_ = new QLineEdit(this);
    btn_browse_src_ = new QPushButton(tr("Browse..."), this);
    src_row->addWidget(edt_source_);
    src_row->addWidget(btn_browse_src_);
    form->addRow(tr("Source:"), src_row);

    // Output row.
    auto* dst_row = new QHBoxLayout;
    edt_output_ = new QLineEdit(this);
    edt_output_->setPlaceholderText(tr("output%1").arg(suffix));
    btn_browse_dst_ = new QPushButton(tr("Browse..."), this);
    dst_row->addWidget(edt_output_);
    dst_row->addWidget(btn_browse_dst_);
    form->addRow(tr("Output:"), dst_row);

    // Cut-mode: start/end in microseconds (QDoubleSpinBox for the full
    // int64 microsecond range, 1 us precision per the user's choice).
    if (is_cut) {
        spn_start_us_ = new QDoubleSpinBox(this);
        spn_end_us_ = new QDoubleSpinBox(this);
        spn_start_us_->setDecimals(0);
        spn_end_us_->setDecimals(0);
        spn_start_us_->setRange(0, 1e15);
        spn_end_us_->setRange(0, 1e15);
        spn_start_us_->setSuffix(QStringLiteral(" us"));
        spn_end_us_->setSuffix(QStringLiteral(" us"));
        lbl_start_ = new QLabel(tr("Start:"), this);
        lbl_end_ = new QLabel(tr("End:"), this);
        form->addRow(lbl_start_, spn_start_us_);
        form->addRow(lbl_end_, spn_end_us_);
        lbl_duration_hint_ = new QLabel(this);
        lbl_duration_hint_->setProperty("class", "hint");
        form->addRow(QString(), lbl_duration_hint_);
    }

    // Convert-CSV: rough output-size warning (text is ~20 bytes/event vs
    // ~12 bytes/event raw — a large source produces a huge, slow CSV).
    if (mode_ == Mode::ConvertCsv) {
        lbl_csv_warn_ = new QLabel(this);
        lbl_csv_warn_->setStyleSheet(QStringLiteral("color: #c08020;"));
        lbl_csv_warn_->setWordWrap(true);
        form->addRow(QString(), lbl_csv_warn_);
    }

    progress_ = new QProgressBar(this);
    progress_->setRange(0, 100);
    form->addRow(progress_);

    lbl_status_ = new QLabel(tr("Ready."), this);
    lbl_status_->setWordWrap(true);
    form->addRow(lbl_status_);

    auto* btn_row = new QHBoxLayout;
    btn_start_ = new QPushButton(tr("Start"), this);
    btn_cancel_ = new QPushButton(tr("Cancel"), this);
    btn_cancel_->setEnabled(false);
    btn_close_ = new QPushButton(tr("Close"), this);
    btn_row->addWidget(btn_start_);
    btn_row->addWidget(btn_cancel_);
    btn_row->addWidget(btn_close_);
    form->addRow(btn_row);

    connect(btn_browse_src_, &QPushButton::clicked, this, &FileOpDialog::on_browse_source);
    connect(btn_browse_dst_, &QPushButton::clicked, this, &FileOpDialog::on_browse_output);
    connect(btn_start_, &QPushButton::clicked, this, &FileOpDialog::on_start);
    connect(btn_cancel_, &QPushButton::clicked, this, &FileOpDialog::on_cancel);
    connect(btn_close_, &QPushButton::clicked, this, &QDialog::reject);

    if (converter_) {
        connect(converter_, &FileConverter::completed, this, &FileOpDialog::on_completed);
        connect(converter_, &FileConverter::failed, this, &FileOpDialog::on_failed);
        connect(converter_, &FileConverter::progress, this, &FileOpDialog::on_progress);
    }
}

void FileOpDialog::set_source(const QString& path) {
    edt_source_->setText(path);
    if (edt_output_->text().isEmpty() && !path.isEmpty()) {
        const QFileInfo fi(path);
        const QString suffix = (mode_ == Mode::Cut) ? QStringLiteral("_cut.raw")
                           : (mode_ == Mode::ConvertHdf5) ? QStringLiteral(".h5")
                                                          : QStringLiteral(".csv");
        QString base = fi.completeBaseName();
        // Replacing .raw's extension directly would produce x.h5; for cuts we
        // keep the base and append _cut.raw to avoid overwriting confusion.
        if (mode_ == Mode::Cut) {
            edt_output_->setText(fi.dir().filePath(base + suffix));
        } else {
            edt_output_->setText(fi.dir().filePath(base + suffix));
        }
    }
    // Duration hint for cut mode. Prefill the REAL timeline [first, last]:
    // the cutter does not re-base output timestamps, and the old hint
    // (duration = last event timestamp) described neither the content
    // length nor where the content sits on the timeline.
    if (lbl_duration_hint_ && converter_) {
        const auto range = converter_->time_range_of(path);
        if (range.second > 0) {
            const auto content = range.second - range.first;
            lbl_duration_hint_->setText(
                tr("Content: %1 s (timeline %2 – %3 s)")
                    .arg(content / 1.0e6, 0, 'f', 3)
                    .arg(range.first / 1.0e6, 0, 'f', 3)
                    .arg(range.second / 1.0e6, 0, 'f', 3));
            if (spn_start_us_) {
                spn_start_us_->setValue(static_cast<double>(range.first));
            }
            spn_end_us_->setValue(static_cast<double>(range.second));
        } else {
            const auto dur = converter_->duration_of(path);
            if (dur > 0) {
                lbl_duration_hint_->setText(
                    tr("Source duration: %1 us (%2 s)")
                        .arg(dur)
                        .arg(dur / 1.0e6, 0, 'f', 3));
                spn_end_us_->setValue(static_cast<double>(dur));
            } else {
                lbl_duration_hint_->setText(tr("Source duration: unknown"));
            }
        }
    }
    // CSV size warning (rough: raw ~12B/event, CSV ~20B/event → ~1.7x).
    if (lbl_csv_warn_ && !path.isEmpty()) {
        const qint64 sz = QFileInfo(path).size();
        if (sz > 500LL * 1024 * 1024) {
            lbl_csv_warn_->setText(
                tr("Warning: CSV output is ~1.7x the source size (~%1 GB) and slow to write.")
                    .arg(sz * 1.7 / 1.0e9, 0, 'f', 1));
        } else {
            lbl_csv_warn_->clear();
        }
    }
}

void FileOpDialog::on_browse_source() {
    const QString src = QFileDialog::getOpenFileName(
        this, tr("Source file"), edt_source_->text(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat *.aedat4 *.alpdata);;All files (*)"));
    if (!src.isEmpty()) set_source(src);
}

void FileOpDialog::on_browse_output() {
    const QString filter = (mode_ == Mode::Cut) ? tr("RAW (*.raw);;All files (*)")
                       : (mode_ == Mode::ConvertHdf5) ? tr("HDF5 (*.h5);;All files (*)")
                                                      : tr("CSV (*.csv);;All files (*)");
    const QString dst = QFileDialog::getSaveFileName(this, windowTitle(), edt_output_->text(), filter);
    if (!dst.isEmpty()) edt_output_->setText(dst);
}

bool FileOpDialog::validate(QString& err) const {
    if (edt_source_->text().isEmpty() || edt_output_->text().isEmpty()) {
        err = tr("Source and output paths are required.");
        return false;
    }
    QString dst = edt_output_->text();
    const QString suffix = (mode_ == Mode::Cut) ? QStringLiteral(".raw")
                       : (mode_ == Mode::ConvertHdf5) ? QStringLiteral(".h5")
                                                      : QStringLiteral(".csv");
    if (!dst.endsWith(suffix, Qt::CaseInsensitive)) dst += suffix;
    if (same_file(edt_source_->text(), dst)) {
        err = tr("Output path must differ from the source file (it would be overwritten).");
        return false;
    }
    if (mode_ == Mode::Cut && spn_end_us_->value() <= spn_start_us_->value()) {
        err = tr("End time must be greater than start time.");
        return false;
    }
    return true;
}

void FileOpDialog::set_running(bool on) {
    running_ = on;
    btn_start_->setEnabled(!on);
    btn_cancel_->setEnabled(on);
    btn_browse_src_->setEnabled(!on);
    btn_browse_dst_->setEnabled(!on);
    edt_source_->setEnabled(!on);
    edt_output_->setEnabled(!on);
}

void FileOpDialog::on_start() {
    if (!converter_ || running_) return;
    QString err;
    if (!validate(err)) {
        lbl_status_->setText(err);
        return;
    }
    QString dst = edt_output_->text();
    const QString suffix = (mode_ == Mode::Cut) ? QStringLiteral(".raw")
                       : (mode_ == Mode::ConvertHdf5) ? QStringLiteral(".h5")
                                                      : QStringLiteral(".csv");
    if (!dst.endsWith(suffix, Qt::CaseInsensitive)) dst += suffix;
    edt_output_->setText(dst);

    progress_->setValue(0);
    lbl_status_->setText(tr("Working..."));
    set_running(true);
    if (mode_ == Mode::Cut) {
        converter_->cut(edt_source_->text(), dst,
                        static_cast<Metavision::timestamp>(spn_start_us_->value()),
                        static_cast<Metavision::timestamp>(spn_end_us_->value()));
    } else {
        converter_->convert(edt_source_->text(), dst,
                            mode_ == Mode::ConvertHdf5 ? FileConverter::Format::HDF5
                                                       : FileConverter::Format::CSV);
    }
}

void FileOpDialog::on_cancel() {
    if (converter_ && running_) {
        lbl_status_->setText(tr("Cancelling..."));
        converter_->cancel();
    }
}

void FileOpDialog::on_completed(const QString& out) {
    set_running(false);
    progress_->setValue(100);
    lbl_status_->setText(tr("Done: %1").arg(out));
}

void FileOpDialog::on_failed(const QString& msg) {
    set_running(false);
    lbl_status_->setText(tr("Failed: %1").arg(msg));
}

void FileOpDialog::on_progress(double r) {
    progress_->setValue(static_cast<int>(r * 100));
}

} // namespace gui
