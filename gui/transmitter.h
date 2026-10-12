#ifndef SOUNDLINK_TRANSMITTER_H
#define SOUNDLINK_TRANSMITTER_H
#include <QObject>
#include <QString>
#include <memory>

class Transmitter : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool running READ running NOTIFY updated)
    Q_PROPERTY(QString status READ status NOTIFY updated)
    Q_PROPERTY(qulonglong underrunFrames READ underrunFrames NOTIFY updated)
public:
    explicit Transmitter(QObject* parent = nullptr);
    ~Transmitter() override;
    bool running() const;
    QString status() const;
    qulonglong underrunFrames() const;
    Q_INVOKABLE bool start(double bandwidth = 5, QString rate = "1/2", int fc = 5);
    Q_INVOKABLE void stop();
signals:
    void updated();
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
#endif
