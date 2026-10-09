/** YG_MC_HOST 入口；普通启动和预览均默认断开设备。 */
#include "gui/maintenance_window.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFile>
#include <QPalette>
#include <QTimer>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    app.setApplicationName("YG_MC_HOST");
    app.setApplicationVersion(QStringLiteral(YG_MC_HOST_VERSION));
    QPalette palette;
    palette.setColor(QPalette::Window, QColor("#f2f5fa"));
    palette.setColor(QPalette::Base, Qt::white);
    palette.setColor(QPalette::Text, QColor("#253450"));
    palette.setColor(QPalette::Button, Qt::white);
    palette.setColor(QPalette::ButtonText, QColor("#253450"));
    app.setPalette(palette);
    QFile theme(":/ui/theme.qss");
    if (theme.open(QIODevice::ReadOnly))
    {
        app.setStyleSheet(QString::fromUtf8(theme.readAll()));
    }
    QCommandLineParser parser;
    parser.setApplicationDescription("YG_MC_HOST 电机 CAN FD 核心工具");
    parser.addHelpOption();
    parser.addVersionOption();
    const QCommandLineOption preview("preview-dir", "保存未连接单页截图后退出。", "directory");
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
                               window.resize(1000, 760);
                               QApplication::processEvents();
                               const bool saved =
                                   window.grab().save(QDir(directory).filePath("core.png"));
                               app.exit(saved ? 0 : 2);
                           });
    }
    return app.exec();
}
