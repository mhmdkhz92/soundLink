#ifndef SOUNDLINK_RECEIVER_H
#define SOUNDLINK_RECEIVER_H
#include <QObject>
#include <QVariantList>
#include <QString>
#include <memory>

class Receiver : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool running READ running NOTIFY updated)
    Q_PROPERTY(QString status READ status NOTIFY updated)
    Q_PROPERTY(double totalBer READ totalBer NOTIFY updated)
    Q_PROPERTY(double recentBer READ recentBer NOTIFY updated)
    Q_PROPERTY(double dataRate READ dataRate NOTIFY updated)
    Q_PROPERTY(qulonglong comparedBits READ comparedBits NOTIFY updated)
    Q_PROPERTY(qulonglong errorBits READ errorBits NOTIFY updated)
    Q_PROPERTY(qulonglong droppedFrames READ droppedFrames NOTIFY updated)
    Q_PROPERTY(qulonglong clippedSamples READ clippedSamples NOTIFY updated)
    Q_PROPERTY(double inputLevel READ inputLevel NOTIFY updated)
    Q_PROPERTY(QVariantList spectrum READ spectrum NOTIFY updated)
    Q_PROPERTY(QVariantList constellation READ constellation NOTIFY updated)
    Q_PROPERTY(QVariantList rbSnr READ rbSnr NOTIFY updated)
public:
    explicit Receiver(QObject* parent = nullptr);
    ~Receiver() override;
    bool running() const;
    QString status() const;
    double totalBer() const;
    double recentBer() const;
    double dataRate() const;
    qulonglong comparedBits() const;
    qulonglong errorBits() const;
    qulonglong droppedFrames() const;
    qulonglong clippedSamples() const;
    double inputLevel() const;
    QVariantList spectrum() const;
    QVariantList constellation() const;
    QVariantList rbSnr() const;
    // Default microphone; bandwidth in kHz, center frequency in integer kHz.
    Q_INVOKABLE bool start(double bandwidth = 5, QString rate = "1/2", int fc = 5);
    Q_INVOKABLE void stop();
    // 0: spectrum, 1: constellation, 2: SNR per RB, -1: no visible plot.
    Q_INVOKABLE void setDisplay(int plot);
signals:
    void updated();
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
    void refresh();
};
#endif
