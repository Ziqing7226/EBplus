#include "multicam/mcu_transport.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <cstdio>

int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    using namespace gui::multicam;

    const QJsonObject commands{
        {"prepare", QString::fromUtf8(mcu_prepare_command())},
        {"finite_1_default", QString::fromUtf8(finite_burst_command(1, 0))},
        {"finite_3_default", QString::fromUtf8(finite_burst_command(3, 0))},
        {"finite_1_drive12", QString::fromUtf8(finite_burst_command(1, 12))},
        {"finite_3_drive12", QString::fromUtf8(finite_burst_command(3, 12))},
        {"continuous_start", QString::fromUtf8(continuous_start_command(4))},
        {"continuous_keepalive", QString::fromUtf8(continuous_keepalive_command(4))},
        {"cleanup", QString::fromUtf8(mcu_cleanup_command(4))},
        {"state", QString::fromUtf8(mcu_state_command())},
    };
    const auto bytes = QJsonDocument(QJsonObject{{"schema_version", 1}, {"commands", commands}})
                           .toJson(QJsonDocument::Compact) + '\n';
    QFile output;
    if (!output.open(stdout, QIODevice::WriteOnly))
        return 1;
    return output.write(bytes) == bytes.size() && output.flush() ? 0 : 1;
}
