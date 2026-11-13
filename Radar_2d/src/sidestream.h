#pragma once

#include <QObject>
#include <QUdpSocket>
#include <QVector>
#include <cstdint>

// One decoded CFAR detection, already in PPI polar coordinates.
struct SectorPoint {
    float azimuthDeg = 0.0f;   // 0 = forward (top), increases clockwise
    float rangeM = 0.0f;       // slant range (m)
    float snr = 0.0f;          // 0..255
};

// Decoded 32-byte header + the raw sector points that survive filtering.
struct SideFrame {
    bool valid = false;
    uint16_t sectorsDecoded = 0;
    uint16_t pointCountTotal = 0;
    uint16_t frameToken = 0;
    uint16_t rotAngleA = 0, rotAngleB = 0, rotAngleC = 0;
    uint8_t cfarError = 0;
    QVector<SectorPoint> points;
};

// Receives the on-radar `radar_streamer side` UDP stream (port 9877),
// registers with it, and decodes the latest frame. Emits frameReady() for
// every decoded frame (single-threaded, all on the Qt event loop).
class SideStream : public QObject {
    Q_OBJECT
public:
    explicit SideStream(const QString &host, quint16 port,
                        float azimuthOffsetDeg, int minSnr, float minRangeM,
                        QObject *parent = nullptr);

    void start();                       // bind + register with the streamer
    const SideFrame &latest() const { return mLatest; }

    // Cumulative UDP packet-loss estimate (%%), derived from the streamer's
    // per-frame token counter. 0.0 until at least two frames are seen.
    double packetLossPct() const { return mPacketLossPct; }

signals:
    void frameReady();

private slots:
    void onReadyRead();

private:
    static SideFrame decode(const QByteArray &datagram, float offsetDeg,
                            int minSnr, float minRangeM);

    QUdpSocket mSocket;
    QString mHost;
    quint16 mPort;
    float mOffsetDeg;
    int mMinSnr;
    float mMinRangeM;
    SideFrame mLatest;

    // Packet-loss accounting via the monotonic frame token (raw+26).
    uint16_t mLastToken = 0;        // token of the previous decoded frame
    bool mHaveToken = false;        // false until the first valid frame
    qulonglong mExpected = 0;       // frames the streamer is believed to have sent
    qulonglong mReceived = 0;       // frames we actually received
    double mPacketLossPct = 0.0;    // 100 * (expected - received) / expected
};
