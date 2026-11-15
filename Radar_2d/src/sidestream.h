#pragma once

#include <QElapsedTimer>
#include <QMetaType>
#include <QObject>
#include <QUdpSocket>
#include <QVector>
#include <cstdint>

// The radar covers +/-120 deg around forward; the rear 120 deg wedge has no
// returns. Grid, ticks and blips are all clipped to that forward sector.
inline constexpr double kBlindStartDeg = 120.0;
inline constexpr double kBlindEndDeg = 240.0;

// One unfiltered 0x00F0 record, exactly the six u16 words on the wire.
// Keeping the raw words (instead of the projected points) is what lets the
// recorder replay a session under different minAmp / minRange / offset.
struct RawPoint {
    quint16 azRaw = 0;   // azimuth, mrad
    quint16 elRaw = 0;   // elevation + 2*pi, mrad
    quint16 rRaw = 0;    // slant range, mm
    quint16 c3 = 0;      // echo amplitude
    quint16 c4 = 0;      // second intensity / SNR candidate
    quint16 c5Us = 0;    // detection timestamp, us (16-bit wrap)
};

// One decoded detection, already in PPI polar coordinates.
struct SectorPoint {
    float azimuthDeg = 0.0f;   // 0 = forward (top), increases clockwise
    float elevationDeg = 0.0f; // + = above horizon, deg
    float rangeM = 0.0f;       // horizontal range (m)
    float snr = 0.0f;          // 0..255 (0x00F0 c3 graded)
    float amp = 0.0f;          // raw c3, before the snr grading
    float c4 = 0.0f;           // raw second intensity
    quint16 c5Us = 0;          // sensor timestamp, raw (wraps every 65.536 ms)
    quint16 azRaw = 0;         // native mrad value, for scan-time compensation
};

// One decoded 0x00F0 point-cloud frame, projected into PPI polar coordinates.
struct SideFrame {
    bool valid = false;
    uint16_t entryCount = 0;    // raw records in this frame
    uint32_t frameToken = 0;    // 0x00F0 container seq (16-bit wire value)
    bool sumOk = true;          // 0x00F0 header checksums passed
    double rpm = 0.0;           // 0x00F0 rpm inferred from c5 (0 = unknown)
    double blindStartDeg = kBlindStartDeg;  // azimuth wedge with no data
    double blindEndDeg = kBlindEndDeg;
    QVector<SectorPoint> points;   // filtered + projected, for display
    QVector<RawPoint> rawPoints;   // unfiltered wire words, for recording

    // Frame header: three independent identifiers, never interchangeable.
    quint32 frameIndex = 0;     // local monotonic (renumbered on replay)
    qint64 recvMonoUs = 0;      // QElapsedTimer, primary algorithm time base
    qint64 tsWallMs = 0;        // wall clock, for human alignment only
    double dtMs = 0.0;          // measured since previous frame (0 for first)
    double revUs = 0.0;         // 60e6 / rpm, one revolution (0 if unknown)
};

Q_DECLARE_METATYPE(SideFrame)

// Receives the radar stream on UDP 40204 and decodes only the 0x00F0 point
// cloud. Other messages on that port (e.g. 1E61 perception) are ignored.
// Every decoded frame is emitted on its own, even when several datagrams
// arrive in one readyRead burst; no frame is ever overwritten.
// Emits frameReady() on the Qt event loop.
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
    void frameReady(const SideFrame &fr);

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
    QElapsedTimer mClock;
    quint32 mFrameIndex = 0;
    qint64 mLastMonoUs = 0;

    // Packet-loss accounting via 0x00F0 container sequence numbers.
    uint32_t mLastSeq = 0;
    bool mHaveSeq = false;
    qulonglong mExpected = 0;
    qulonglong mReceived = 0;
    double mPacketLossPct = 0.0;
};
