#pragma once

#include <QObject>
#include <QUdpSocket>
#include <QVector>
#include <cstdint>

// One decoded detection, already in PPI polar coordinates.
struct SectorPoint {
    float azimuthDeg = 0.0f;   // 0 = forward (top), increases clockwise
    float rangeM = 0.0f;       // horizontal range (m)
    float snr = 0.0f;          // 0..255 (DJI 1E61 metric)
};

// One decoded DJI 1E61 perception frame (post-CFAR point cloud). This is the
// radar's real point cloud emitted over UDP 40204 (V2); unlike the on-radar
// omni snapshot it works on bench units whose encoder->FPGA wire is severed.
struct SideFrame {
    bool valid = false;
    uint8_t subtype = 0;        // 0=ground, 1=wire, 2=pcla
    uint16_t entryCount = 0;    // raw 1E61 entries in this frame
    uint32_t frameToken = 0;    // V2 sequence number
    QVector<SectorPoint> points;
};

// Receives the DJI 1E61 v2 UDP perception stream (port 40204) and decodes
// the latest frame. Emits frameReady() for every decoded frame (all on the
// Qt event loop). The radar broadcasts this stream to the host, so no
// registration handshake is required.
class SideStream : public QObject {
    Q_OBJECT
public:
    explicit SideStream(const QString &host, quint16 port,
                        float azimuthOffsetDeg, int minSnr, float minRangeM,
                        QObject *parent = nullptr);

    void start();                       // bind UDP port
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
    float mMinRangeM;
    SideFrame mLatest;

    // The 1E61 stream interleaves three subtypes (0=ground, 1=wire, 2=pcla)
    // in separate frames. Keep the latest points per subtype and combine them
    // so the PPI shows a stable picture instead of flickering one subtype.
    QVector<SectorPoint> mAccum[3];

    // Packet-loss accounting via V2 sequence numbers.
    uint32_t mLastSeq = 0;
    bool mHaveSeq = false;
    qulonglong mExpected = 0;
    qulonglong mReceived = 0;
    double mPacketLossPct = 0.0;
};
