#include "linkstream.h"

#include <QAbstractSocket>
#include <QNetworkProxy>

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "f0decoder.h"

namespace {
constexpr int kHdr = 24;
constexpr quint8 kTypeHeartbeat = 0x02;
constexpr quint8 kTypeRadarUdp = 0x20;
constexpr quint8 kTypeStatus = 0x30;
constexpr quint8 kTypeCmd = 0x40;
constexpr quint16 kRadarPort = 40204;

inline quint16 u16(const char *p) {
    const auto *b = reinterpret_cast<const uint8_t *>(p);
    return quint16(b[0] | (b[1] << 8));
}
inline quint32 u32(const char *p) {
    const auto *b = reinterpret_cast<const uint8_t *>(p);
    return quint32(b[0]) | (quint32(b[1]) << 8) | (quint32(b[2]) << 16)
         | (quint32(b[3]) << 24);
}
inline void putU32(char *p, quint32 v) {
    auto *b = reinterpret_cast<uint8_t *>(p);
    b[0] = uint8_t(v);
    b[1] = uint8_t(v >> 8);
    b[2] = uint8_t(v >> 16);
    b[3] = uint8_t(v >> 24);
}
inline void putU64(char *p, quint64 v) {
    auto *b = reinterpret_cast<uint8_t *>(p);
    for (int i = 0; i < 8; ++i) {
        b[i] = uint8_t(v >> (8 * i));
    }
}

// CRC32/IEEE (zlib.crc32), covering version..payload.
quint32 crc32(const char *data, int len) {
    static quint32 table[256] = {};
    static const bool ready = [] {
        for (quint32 i = 0; i < 256; ++i) {
            quint32 c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        return true;
    }();
    (void)ready;
    quint32 c = 0xFFFFFFFFu;
    const auto *b = reinterpret_cast<const uint8_t *>(data);
    for (int i = 0; i < len; ++i) {
        c = table[(c ^ b[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}
}  // namespace

LinkStream::LinkStream(const QString &host, quint16 port, float offsetDeg,
                       int minAmp, float minRangeM, QObject *parent)
    : QObject(parent),
      mHost(host),
      mPort(port),
      mOffsetDeg(offsetDeg),
      mMinAmp(minAmp),
      mMinRangeM(minRangeM) {
    connect(&mSocket, &QTcpSocket::connected, this, &LinkStream::onConnected);
    connect(&mSocket, &QTcpSocket::readyRead, this, &LinkStream::onReadyRead);
    connect(&mSocket, &QTcpSocket::disconnected, this,
            &LinkStream::onDisconnected);
    connect(&mSocket, &QAbstractSocket::errorOccurred, this,
            &LinkStream::onError);
    mReconnectTimer.setSingleShot(true);
    connect(&mReconnectTimer, &QTimer::timeout, this,
            &LinkStream::onReconnect);
    connect(&mHeartbeatTimer, &QTimer::timeout, this,
            &LinkStream::onHeartbeat);
}

void LinkStream::start() {
    if (mRunning) {
        return;
    }
    mRunning = true;
    mClock.start();
    mHeartbeatTimer.start(1000);
    mSocket.setProxy(QNetworkProxy::NoProxy);
    mSocket.connectToHost(mHost, mPort);
    std::fprintf(stderr, "[link] connecting %s:%u (0x00F0 point cloud)\n",
                 qPrintable(mHost), mPort);
}

void LinkStream::stop() {
    mRunning = false;
    mHeartbeatTimer.stop();
    mReconnectTimer.stop();
    mSocket.abort();
}

bool LinkStream::isConnected() const {
    return mSocket.state() == QAbstractSocket::ConnectedState;
}

void LinkStream::onConnected() {
    mBackoffMs = 1000;
    mBuf.clear();
    mHaveSeq = false;
    mStatus.connected = true;
    std::fprintf(stderr, "[link] connected %s:%u\n", qPrintable(mHost), mPort);
    onHeartbeat();
    emit stateChanged();
}

void LinkStream::onDisconnected() {
    mStatus.connected = false;
    emit stateChanged();
    if (!mRunning) {
        return;
    }
    mReconnectTimer.start(mBackoffMs);
    mBackoffMs = std::min(mBackoffMs * 17 / 10, 5000);
}

void LinkStream::onError(QAbstractSocket::SocketError) {
    if (mSocket.state() == QAbstractSocket::ConnectedState) {
        return;
    }
    std::fprintf(stderr, "[link] %s (retry in %d ms)\n",
                 qPrintable(mSocket.errorString()), mBackoffMs);
    mStatus.connected = false;
    emit stateChanged();
    if (mRunning) {
        mReconnectTimer.start(mBackoffMs);
        mBackoffMs = std::min(mBackoffMs * 17 / 10, 5000);
    }
}

void LinkStream::onReconnect() {
    if (mRunning && mSocket.state() == QAbstractSocket::UnconnectedState) {
        mSocket.connectToHost(mHost, mPort);
    }
}

void LinkStream::onHeartbeat() {
    if (!mRunning || !isConnected()) {
        return;
    }
    QByteArray payload(4, '\0');
    putU32(payload.data(), quint32(mClock.elapsed() / 1000));
    mSocket.write(encodeFrame(kTypeHeartbeat, payload));
}

void LinkStream::onReadyRead() {
    mBuf += mSocket.readAll();
    parse();
}

void LinkStream::parse() {
    for (;;) {
        const int magicAt = int(mBuf.indexOf("T40R"));
        if (magicAt < 0) {
            // Keep a possible partial magic at the tail.
            if (mBuf.size() > 3) {
                mBuf = mBuf.right(3);
            }
            return;
        }
        if (magicAt > 0) {
            mBuf.remove(0, magicAt);
        }
        if (mBuf.size() < kHdr) {
            return;
        }
        const quint32 len = u32(mBuf.constData() + 20);
        if (len > 65536) {
            std::fprintf(stderr, "[link] bad frame length %u, reconnecting\n", len);
            mBuf.clear();
            mSocket.disconnectFromHost();
            return;
        }
        const int total = kHdr + int(len) + 4;
        if (mBuf.size() < total) {
            return;
        }
        const quint32 crc = u32(mBuf.constData() + kHdr + int(len));
        if (crc32(mBuf.constData() + 4, 20 + int(len)) == crc) {
            handleFrame(quint8(mBuf.at(5)), mBuf.constData() + kHdr, int(len));
        }
        mBuf.remove(0, total);
    }
}

void LinkStream::handleFrame(quint8 type, const char *payload, int len) {
    if (type == kTypeRadarUdp && len > 2) {
        // RADAR_UDP payload = u16 source port + untouched UDP datagram.
        if (u16(payload) != kRadarPort) {
            return;
        }
        const F0Frame f0 = F0::decode(
            reinterpret_cast<const uint8_t *>(payload) + 2, len - 2);
        if (!f0.valid) {
            return;
        }
        noteLoss(quint16(f0.seq));
        mLatest = F0::toSideFrame(f0, PointSource::F0Link, mOffsetDeg,
                                  mMinAmp, mMinRangeM);
        emit frameReady();
    } else if (type == kTypeStatus && len >= 34) {
        mStatus.radarPkts = u32(payload + 16);
        mStatus.dropped = u32(payload + 24);
        mStatus.wifi = payload[28] != 0;
        mStatus.statusAge = len >= 40 ? u16(payload + 38) : 0;
        emit stateChanged();
    }
    // Unknown types are skipped, per the protocol's forward-compat rule.
}

void LinkStream::noteLoss(quint16 seq) {
    if (mHaveSeq) {
        const quint16 delta = quint16(seq - mLastSeq);
        if (delta >= 1 && delta <= 1000) {
            mExpected += delta;
            ++mReceived;
            if (mExpected > 0) {
                mPacketLossPct = 100.0 * double(mExpected - mReceived)
                               / double(mExpected);
            }
        }
    }
    mLastSeq = seq;
    mHaveSeq = true;
}

QByteArray LinkStream::encodeFrame(quint8 type, const QByteArray &payload) {
    QByteArray frame(kHdr + payload.size() + 4, '\0');
    char *p = frame.data();
    std::memcpy(p, "T40R", 4);
    p[4] = char(0x01);       // version
    p[5] = char(type);
    p[6] = p[7] = char(0);   // flags
    putU32(p + 8, ++mTxSeq);
    putU64(p + 12, quint64(mClock.nsecsElapsed()));
    putU32(p + 20, quint32(payload.size()));
    if (!payload.isEmpty()) {
        std::memcpy(p + kHdr, payload.constData(), size_t(payload.size()));
    }
    putU32(p + kHdr + payload.size(), crc32(p + 4, 20 + payload.size()));
    return frame;
}

void LinkStream::sendCmd(quint8 op, const QByteArray &args) {
    if (!mRunning || !isConnected()) {
        std::fprintf(stderr, "[link] command 0x%02X dropped (link down)\n", op);
        return;
    }
    QByteArray payload(1, char(op));
    payload += args;
    mSocket.write(encodeFrame(kTypeCmd, payload));
}
