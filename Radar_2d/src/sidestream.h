#pragma once

#include <QObject>
#include <QUdpSocket>
#include <QVector>
#include <cstdint>

// Which point-cloud stream the current frame came from.
enum class PointSource {
    E61,     // DJI 1E61 v2 perception entries (UDP 40204)
    F0Udp,   // 0x00F0 black-box points, straight from radar UDP 40204
    F0Link,  // 0x00F0 points forwarded by the t40-radar-link board (TCP 46000)
};

// One decoded detection, already in PPI polar coordinates.
struct SectorPoint {
    float azimuthDeg = 0.0f;   // 0 = forward (top), increases clockwise
    float rangeM = 0.0f;       // horizontal range (m)
    float snr = 0.0f;          // 0..255 (1E61 metric or 0x00F0 c3 graded)
};

// One decoded radar frame, projected into PPI polar coordinates. Either the
// 1E61 entries or the 0x00F0 point cloud from t40-radar-link fills it.
struct SideFrame {
    bool valid = false;
    PointSource source = PointSource::E61;
    uint8_t subtype = 0;        // 1E61 only: 0=ground, 1=wire, 2=pcla
    uint16_t entryCount = 0;    // raw records in this frame
    uint32_t frameToken = 0;    // 1E61 V2 seq / 0x00F0 container seq
    bool sumOk = true;          // 0x00F0 header checksums passed
    double rpm = 0.0;           // 0x00F0 rpm inferred from c5 (0 = unknown)
    double blindStartDeg = 120.0;  // azimuth wedge with no data
    double blindEndDeg = 240.0;
    QVector<SectorPoint> points;
};

// Receives the radar's perception stream on UDP 40204 and decodes both the
// 0x00F0 point cloud (t40-radar-link format) and legacy 1E61 v2 entries.
// Emits frameReady() for every decoded frame (all on the Qt event loop).
class SideStream : public QObject {
    Q_OBJECT
public:
    explicit SideStream(const QString &host, quint16 port,
                        float azimuthOffsetDeg, int minSnr, int minAmp,
                        float minRangeM, QObject *parent = nullptr);

    void start();                       // bind UDP port
    void stop() { mSocket.close(); }
    const SideFrame &latest() const { return mLatest; }

    // Cumulative UDP packet-loss estimate (%%), derived from V2 seq gaps.
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
    int mMinAmp;
    float mMinRangeM;
    SideFrame mLatest;

    // The 1E61 stream interleaves three subtypes (0=ground, 1=wire, 2=pcla)
    // in separate frames. Keep the latest points per subtype and combine them
    // so the PPI shows a stable picture instead of flickering one subtype.
    QVector<SectorPoint> mAccum[3];

    // Packet-loss accounting via 1E61 V2 / 0x00F0 sequence numbers.
    uint32_t mLastSeq = 0;
    bool mHaveSeq = false;
    qulonglong mExpected = 0;
    qulonglong mReceived = 0;
    double mPacketLossPct = 0.0;
};
