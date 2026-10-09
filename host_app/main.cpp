/** YG_MC_HOST 页面入口；预览和普通启动均默认离线，不构造设备驱动。 */
#include "gui/maintenance_window.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QPalette>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QTimer>
#include <iostream>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    app.setApplicationName("YG_MC_HOST");
    app.setApplicationVersion(QStringLiteral(YG_MC_HOST_VERSION));
    app.setOrganizationName("YG");
    QPalette palette;
    palette.setColor(QPalette::Window, QColor("#f2f5fa"));
    palette.setColor(QPalette::Base, Qt::white);
    palette.setColor(QPalette::Text, QColor("#253450"));
    palette.setColor(QPalette::Button, Qt::white);
    palette.setColor(QPalette::ButtonText, QColor("#253450"));
    palette.setColor(QPalette::Highlight, QColor("#dceaff"));
    palette.setColor(QPalette::HighlightedText, QColor("#183f79"));
    app.setPalette(palette);
    QFile theme(":/ui/theme.qss");
    if (theme.open(QIODevice::ReadOnly))
    {
        app.setStyleSheet(QString::fromUtf8(theme.readAll()));
    }
    QCommandLineParser parser;
    parser.setApplicationDescription("YG_MC_HOST 电机调试工作台 · 台架候选版");
    parser.addHelpOption();
    parser.addVersionOption();
    const QCommandLineOption preview(
        "preview-dir", "保存升级与参数页面截图后退出，不连接设备。", "directory");
    parser.addOption(preview);
    parser.process(app);
    yg::host::MaintenanceWindow window;
    window.show();
    if (parser.isSet(preview))
    {
        const auto directory = parser.value(preview);
        if (!QDir().mkpath(directory))
        {
            return 2;
        }
        QTimer::singleShot(200,
                           &window,
                           [&app, &window, directory]
                           {
                               window.resize(1366, 768);
                               bool saved = true;
                               auto save = [&](const char *filename)
                               {
                                   QApplication::processEvents();
                                   saved = window.grab().save(QDir(directory).filePath(filename)) &&
                                           saved;
                               };
                               save("connection.png");
                               window.findChild<QPushButton *>("connectDevice")->click();
                               save("home.png");
                               window.findChild<QPushButton *>("workbenchNav")->click();
                               window.findChild<QCheckBox *>("unlockWrites")->setChecked(true);
                               window.findChild<QPushButton *>("streamStart")->click();
                               window.findChild<QPushButton *>("recordStart")->click();
                               window.findChild<QPushButton *>("enableMotor")->click();
                               window.findChild<QPushButton *>("executeMotion")->click();
                               for (int i = 0; i < 50; ++i)
                               {
                                   window.Window_AdvanceDemo(0.2);
                               }
                               window.findChild<QCheckBox *>("freezeScope")->setChecked(true);
                               window.findChild<QPushButton *>("stopMotor")->click();
                               window.findChild<QPushButton *>("quickRead")->click();
                               save("workbench.png");
                               window.findChild<QPushButton *>("calibrationNav")->click();
                               window.findChild<QPushButton *>("startCalibration")->click();
                               for (int i = 0; i < 10; ++i)
                               {
                                   window.Window_AdvanceDemo(0.2);
                               }
                               save("calibration.png");
                               window.findChild<QPushButton *>("cancelCalibration")->click();
                               window.findChild<QPushButton *>("identificationNav")->click();
                               save("identification.png");
                               window.findChild<QPushButton *>("diagnosticsNav")->click();
                               save("diagnostics.png");
                               window.findChild<QPushButton *>("upgradeNav")->click();
                               window.resize(1380, 940);
                               window.findChild<QPushButton *>("mode2")->click();
                               window.findChild<QPushButton *>("loadSamples")->click();
                               QApplication::processEvents();
                               save("upgrade.png");
                               window.findChild<QPushButton *>("parameterNav")->click();
                               window.resize(1366, 768);
                               window.findChild<QPushButton *>("readParameters")->click();
                               QApplication::processEvents();
                               save("parameters.png");
                               window.findChild<QPushButton *>("disconnectDevice")->click();
                               window.findChild<QComboBox *>("connectionMode")->setCurrentIndex(1);
                               window.findChild<QPushButton *>("workbenchNav")->click();
                               save("device-workbench.png");
                               window.findChild<QPushButton *>("upgradeNav")->click();
                               window.resize(1380, 940);
                               save("device-upgrade.png");
                               app.exit(saved ? 0 : 2);
                           });
    }
    return app.exec();
}
