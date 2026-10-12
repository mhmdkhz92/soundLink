#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "receiver.h"
#include "transmitter.h"

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    Receiver receiver;
    Transmitter transmitter;
#ifdef Q_OS_IOS
    QObject::connect(&app, &QGuiApplication::applicationStateChanged, &receiver,
        [&receiver, &transmitter](Qt::ApplicationState state) {
            if (state != Qt::ApplicationActive) {
                if (receiver.running()) receiver.stop();
                if (transmitter.running()) transmitter.stop();
            }
        });
#endif
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("receiver", &receiver);
    engine.rootContext()->setContextProperty("transmitter", &transmitter);
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("gui", "Main");

    return QGuiApplication::exec();
}
