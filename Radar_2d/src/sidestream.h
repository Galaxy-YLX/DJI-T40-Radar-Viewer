#pragma once

#include <QObject>
#include <QUdpSocket>
#include <QVector>
#include <cstdint>

// The radar covers +/-120 deg around forward; the rear 120 deg wedge has no
// returns. Grid, ticks and blips are all clipped to that forward sector.
inline constexpr double kBlindStartDeg = 120.0;
inline constexpr double kBlindEndDeg = 240.0;

// One decoded detection, already in PPI polar coordinates.
struct SectorPoint {
    float azimuthDeg = 0.0f;   // 0 = forward (top), increases clockwise
    float rangeM = 0.0f;       // horizontal range (m)
    float snr = 0.0f;          // 0..255 (0x00F0 c3 graded)
};

// One decoded 0x00F0 point-cloud frame, projected into PPI polar coordinates.
struct SideFrame {
    bool valid = false;
    uint16_t entryCount = 0;    // raw records in this frame
    uint32_t frameToken = 0;    // 0x00F0 container seq
    bool sumOk = true;          // 0x00F0 header checksums passed
    double rpm = 0.0;           // 0x00F0 rpm inferred from c5 (0 = unknown)
    double blindStartDeg = kBlindStartDeg;  // azimuth wedge with no data
    double blindEndDeg = kBlindEndDeg;
    QVector<SectorPoint> points;
};

// Receives the radar stream on UDP 40204 and decodes only the 0x00F0 point
// cloud. Other messages on that port (e.g. 1E61 perception) are ignored.
// Emits frameReady() for every decoded frame (all on the Qt event loop).
class SideStream : public QObject {
    Q_OBJECT
public:
    explicit SideStream(const QString &host, quint16 port,
                        float azimuthOffsetDeg, int minAmp,
                        float minRangeM, QObject *parent = nullptr);

    void start();                       // bind UDP port
    void stop() { mSocket.close(); }
    const SideFrame &latest() const { return mLatest; }

    // Cumulative 0x00F0 frame-loss estimate (%), from container seq gaps.
    double packetLossPct() const { return mPacketLossPct; }

signals:
    void frameReady();

private slots:
    void onReadyRead();

private:
    QUdpSocket mSocket;
    QString mHost;
    quint16 mPort;
    float mOffsetDeg;
    int mMinAmp;
    float mMinRangeM;
    SideFrame mLatest;

    // Packet-loss accounting via 0x00F0 container sequence numbers.
    uint32_t mLastSeq = 0;
    bool mHaveSeq = false;
    qulonglong mExpected = 0;
    qulonglong mReceived = 0;
    double mPacketLossPct = 0.0;
};
