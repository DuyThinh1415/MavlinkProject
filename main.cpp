#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <CommManager/maincomm.h>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;

    engine.rootContext()->setContextProperty("CommSingleton", MainComm::instance());
    qmlRegisterSingletonType<MainComm>("com.example", 1, 0, "MavlinkComm",
                                      [](QQmlEngine*, QJSEngine*) { return MainComm::instance(); });

    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("MavlinkConnectProject", "Main");
    // engine.load(QUrl("qrc:View/Main.qml"));

    return app.exec();
}
