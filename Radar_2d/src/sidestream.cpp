#include "sidestream.h"

#include <QHostAddress>
#include <QNetworkProxy>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

// Wire constants — must match radar_viewer.py / radar_streamer.c.
namespace {
constexpr int SIDE_FRAME_SIZE = 60632;      // 32 header + 360*160 + 250*12
constexpr int SIDE_HEADER_SIZE = 32;
constexpr int SIDE_N_SECTORS = 360;
constexpr int SIDE_SECTOR_DUMP = 160;
constexpr int SIDE_MAX_PTS_PER_SEC = 7;     // (160 - 15) / 20
constexpr float SIDE_RANGE_MIN_M = 0.1f;
constexpr float SIDE_RANGE_MAX_M = 120.0f;
constexpr float SIDE_DEG_PER_SECTOR = 1.0f;

uint16_t readU16(const uint8_t *p) {
    return uint16_t(p[0] | (p[1] << 8));
}

float readF32(const uint8_t *p) {
    uint32_t v = uint32_t(p[0]) | (uint32_t(p[1]) << 8)
               | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    float f;
    std::memcpy(&f, &v, sizeof(f));
    return f;
}
}  // namespace

SideStream::SideStream(const QString &host, quint16 port, float azimuthOffsetDeg,
                       int minSnr, float minRangeM, QObject *parent)
    : QObject(parent),
      mHost(host),
      mPort(port),
      mOffsetDeg(azimuthOffsetDeg),
      mMinSnr(minSnr),
      mMinRangeM(minRangeM) {
    connect(&mSocket, &QUdpSocket::readyRead, this, &SideStream::onReadyRead);
}

void SideStream::start() {
    mSocket.setProxy(QNetworkProxy::NoProxy);  // raw UDP, never via a proxy
    if (!mSocket.bind(QHostAddress::AnyIPv4, 0)) {
        std::fprintf(stderr, "[side] bind failed: %s\n",
                     mSocket.errorString().toUtf8().constData());
        return;
    }
    // Send a registration byte so the streamer learns our address and
    // starts streaming 60632-byte SIDE frames back to us.
    const QByteArray reg(1, '\0');
    mSocket.writeDatagram(reg, QHostAddress(mHost), mPort);
    std::fprintf(stderr, "[side] registered with %s:%u\n",
                 mHost.toUtf8().constData(), mPort);
}

void SideStream::onReadyRead() {
    QByteArray latest;
    uint16_t latestTok = 0;
    int burstValid = 0;   // valid frames pulled from the socket this burst
    while (mSocket.hasPendingDatagrams()) {
        const qint64 size = mSocket.pendingDatagramSize();
        if (size <= 0) {
            continue;
        }
        QByteArray d;
        d.resize(int(size));
        mSocket.readDatagram(d.data(), d.size());
        if (d.size() == SIDE_FRAME_SIZE) {
            latest = std::move(d);
            const auto *raw =
                reinterpret_cast<const uint8_t *>(latest.constData());
            latestTok = readU16(raw + 26);   // per-frame token (raw+26)
            ++burstValid;
        }
    }
    if (!latest.isEmpty()) {
        mLatest = decode(latest, mOffsetDeg, mMinSnr, mMinRangeM);
        // Packet-loss estimate: the token is a monotonic per-frame counter
        // maintained by the streamer. Across bursts the number of frames the
        // streamer should have emitted equals (token - lastToken); the number
        // we actually received is the datagram count pulled this burst.
        if (mHaveToken) {
            const uint16_t delta = uint16_t(latestTok - mLastToken);
            // Ignore implausible jumps (stream restart / token wrap noise).
            if (delta >= 1 && delta <= 60000) {
                mExpected += delta;
                mReceived += burstValid;
                if (mExpected > 0) {
                    mPacketLossPct =
                        100.0 * double(mExpected - mReceived) / double(mExpected);
                }
            }
        }
        mLastToken = latestTok;
        mHaveToken = true;
        emit frameReady();
    }
}

SideFrame SideStream::decode(const QByteArray &datagram, float offsetDeg,
                             int minSnr, float minRangeM) {
    SideFrame fr;
    if (datagram.size() != SIDE_FRAME_SIZE) {
        return fr;
    }
    const auto *raw = reinterpret_cast<const uint8_t *>(datagram.constData());
    if (raw[0] != 'O' || raw[1] != 'M' || raw[2] != 'N' || raw[3] != 'I') {
        return fr;  // bad magic
    }
    fr.valid = true;
    fr.rotAngleA = readU16(raw + 8);
    fr.rotAngleB = readU16(raw + 10);
    fr.sectorsDecoded = readU16(raw + 12);
    fr.rotAngleC = readU16(raw + 14);
    fr.pointCountTotal = readU16(raw + 24);
    fr.frameToken = readU16(raw + 26);
    fr.cfarError = raw[28];

    const uint8_t *body = raw + SIDE_HEADER_SIZE;
    for (int i = 0; i < SIDE_N_SECTORS; ++i) {
        const uint8_t *s = body + i * SIDE_SECTOR_DUMP;
        const int count = s[1];
        const int n = std::min(count, SIDE_MAX_PTS_PER_SEC);
        for (int pi = 0; pi < n; ++pi) {
            const int off = 15 + pi * 20;
            const float r = readF32(s + off);
            if (!std::isfinite(r) || r < SIDE_RANGE_MIN_M || r > SIDE_RANGE_MAX_M) {
                continue;
            }
            if (r < minRangeM) {
                continue;
            }
            const float snr = float(readU16(s + off + 16));
            if (snr < minSnr) {
                continue;
            }
            SectorPoint p;
            p.rangeM = r;
            p.snr = snr;
            p.azimuthDeg = float(std::fmod(i * SIDE_DEG_PER_SECTOR + offsetDeg,
                                           360.0));
            fr.points.push_back(p);
        }
    }
    return fr;
}
