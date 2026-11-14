#pragma once

#include <QByteArray>
#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QTimer>

#include "sidestream.h"

// A subset of the board's STATUS (0x30) payload, for the HUD.
struct LinkStatus {
    bool connected = false;
    quint32 radarPkts = 0;
    quint32 dropped = 0;
    bool wifi = false;
    quint16 statusAge = 0;
};

// T40R client for t40-radar-link/bridge/t40_bridge.py (TCP 46000).
// Parses RADAR_UDP 0x20 frames, decodes their 0x00F0 point cloud and emits
// the same SideFrame the UDP source produces. Also sends heartbeats and the
// CMD 0x40 motor commands used by the 开机/关机 buttons.
class LinkStream : public QObject {
    Q_OBJECT
public:
    LinkStream(const QString &host, quint16 port, float offsetDeg, int minAmp,
               float minRangeM, QObject *parent = nullptr);

    void start();
    void stop();
    bool isConnected() const;
    const SideFrame &latest() const { return mLatest; }
    const LinkStatus &status() const { return mStatus; }
    double packetLossPct() const { return mPacketLossPct; }

    // CMD 0x40 op codes, see t40-radar-link/protocol/PROTOCOL.md.
    static constexpr quint8 kOpSetRpm = 0x01;
    static constexpr quint8 kOpStop = 0x02;
    static constexpr quint8 kOpMotorEnable = 0x03;
    void sendCmd(quint8 op, const QByteArray &args = QByteArray());

signals:
    void frameReady();
    void stateChanged();

private slots:
    void onConnected();
    void onReadyRead();
    void onDisconnected();
    void onError(QAbstractSocket::SocketError err);
    void onReconnect();
    void onHeartbeat();

private:
    void parse();
    void handleFrame(quint8 type, const char *payload, int len);
    void noteLoss(quint16 seq);
    QByteArray encodeFrame(quint8 type, const QByteArray &payload);

    QTcpSocket mSocket;
    QTimer mReconnectTimer;
    QTimer mHeartbeatTimer;
    QElapsedTimer mClock;
    QByteArray mBuf;
    QString mHost;
    quint16 mPort;
    float mOffsetDeg;
    int mMinAmp;
    float mMinRangeM;
    SideFrame mLatest;
    LinkStatus mStatus;
    bool mRunning = false;
    int mBackoffMs = 1000;
    quint32 mTxSeq = 0;

    quint16 mLastSeq = 0;
    bool mHaveSeq = false;
    qulonglong mExpected = 0;
    qulonglong mReceived = 0;
    double mPacketLossPct = 0.0;
};
