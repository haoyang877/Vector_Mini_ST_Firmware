/** YG_MC_HOST 页面入口；预览和普通启动均默认离线，不构造设备驱动。 */
#include "gui/maintenance_window.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QPalette>
#include <QPushButton>
#include <QTimer>
#include <iostream>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    app.setApplicationName("YG_MC_HOST");
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
    parser.setApplicationDescription("YG_MC_HOST 升级与参数页面 · 离线开发版");
    parser.addHelpOption();
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
                               window.resize(1380, 1260);
                               window.findChild<QPushButton *>("mode2")->click();
                               window.findChild<QPushButton *>("loadSamples")->click();
                               QApplication::processEvents();
                               const bool upgrade_saved =
                                   window.grab().save(QDir(directory).filePath("upgrade.png"));
                               window.findChild<QPushButton *>("parameterNav")->click();
                               window.resize(1380, 940);
                               window.findChild<QPushButton *>("readParameters")->click();
                               QApplication::processEvents();
                               const bool parameters_saved =
                                   window.grab().save(QDir(directory).filePath("parameters.png"));
                               app.exit(upgrade_saved && parameters_saved ? 0 : 2);
                           });
    }
    return app.exec();
}
