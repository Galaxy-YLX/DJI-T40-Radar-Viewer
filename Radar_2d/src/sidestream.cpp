#include "sidestream.h"

#include <QHostAddress>
#include <QNetworkProxy>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "f0decoder.h"

// Wire constants — must match radar_min_client.py's V2 parser and
// decode_perception_1e61_entries().
namespace {
constexpr int V2_MSG_OFFSET = 24;      // msg_id (u16 BE) at byte 24
constexpr int V2_PAYLOAD_OFFSET = 27;  // payload starts at byte 27
constexpr int V2_SEQ_OFFSET = 7;       // seq (u32 LE) at byte 7
constexpr int E61_MSG_ID = 0x1E61;
constexpr int E61_HEADER_LEN = 69;     // 1E61 payload header
constexpr int E61_SUBTYPE_OFFSET = 65; // subtype byte
constexpr int E61_ENTRY_LEN = 16;      // <HfffH> per entry
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

uint16_t readU16(const uint8_t *p) {
    return uint16_t(p[0] | (p[1] << 8));
}

uint32_t readU32(const uint8_t *p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8)
         | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

float readF32(const uint8_t *p) {
    uint32_t v = readU32(p);
    float f;
    std::memcpy(&f, &v, sizeof(f));
    return f;
}
}  // namespace

SideStream::SideStream(const QString &host, quint16 port, float azimuthOffsetDeg,
                       int minSnr, int minAmp, float minRangeM, QObject *parent)
    : QObject(parent),
      mHost(host),
      mPort(port),
      mOffsetDeg(azimuthOffsetDeg),
      mMinSnr(minSnr),
      mMinAmp(minAmp),
      mMinRangeM(minRangeM) {
    connect(&mSocket, &QUdpSocket::readyRead, this, &SideStream::onReadyRead);
}

void SideStream::start() {
    mSocket.setProxy(QNetworkProxy::NoProxy);  // raw UDP, never via a proxy
    // The radar broadcasts the 1E61 stream to the host's port 40204; no
    // registration byte is needed (unlike the omni "side" streamer).
    if (!mSocket.bind(QHostAddress::AnyIPv4, mPort)) {
        std::fprintf(stderr, "[e61] bind UDP :%u failed: %s\n", mPort,
                     mSocket.errorString().toUtf8().constData());
        return;
    }
    std::fprintf(stderr, "[udp] listening on UDP :%u (0x00F0 point cloud / 1E61)\n",
                 mPort);
}

void SideStream::onReadyRead() {
    SideFrame latest;
    uint32_t latestSeq = 0;
    int burstValid = 0;   // valid 1E61 frames pulled from the socket this burst
    while (mSocket.hasPendingDatagrams()) {
        const qint64 size = mSocket.pendingDatagramSize();
        if (size <= 0) {
            continue;
        }
        QByteArray d;
        d.resize(int(size));
        mSocket.readDatagram(d.data(), d.size());
        SideFrame fr = decode(d, mOffsetDeg, mMinSnr, mMinRangeM);
        if (!fr.valid) {
            // The same UDP port also carries the 0x00F0 black-box point
            // cloud decoded by t40-radar-link.
            const F0Frame f0 = F0::decode(
                reinterpret_cast<const uint8_t *>(d.constData()), d.size());
            if (f0.valid) {
                fr = F0::toSideFrame(f0, PointSource::F0Udp, mOffsetDeg,
                                     mMinAmp, mMinRangeM);
            }
        }
        if (!fr.valid) {
            continue;
        }
        if (fr.source == PointSource::E61 && fr.subtype <= 2) {
            mAccum[fr.subtype] = fr.points;
        }
        latest = std::move(fr);
        latestSeq = latest.frameToken;
        ++burstValid;
    }
    if (!latest.valid) {
        return;
    }

    if (latest.source == PointSource::E61) {
        // Combine the three subtypes into one stable display frame.
        latest.points.clear();
        for (int s = 0; s < 3; ++s) {
            latest.points += mAccum[s];
        }
    }
    mLatest = std::move(latest);

    // Packet-loss estimate from sequence gaps (u32 for 1E61, u16 for 0x00F0).
    if (mHaveSeq) {
        const uint32_t delta = latest.source == PointSource::E61
            ? latestSeq - mLastSeq
            : uint32_t(uint16_t(latestSeq - mLastSeq));
        if (delta >= 1 && delta <= 100000) {
            mExpected += delta;
            mReceived += burstValid;
            if (mExpected > 0) {
                mPacketLossPct =
                    100.0 * double(mExpected - mReceived) / double(mExpected);
            }
        }
    }
    mLastSeq = latestSeq;
    mHaveSeq = true;
    emit frameReady();
}

SideFrame SideStream::decode(const QByteArray &datagram, float offsetDeg,
                             int minSnr, float minRangeM) {
    SideFrame fr;
    const int len = datagram.size();
    if (len < V2_PAYLOAD_OFFSET + 2) {
        return fr;
    }
    const auto *raw = reinterpret_cast<const uint8_t *>(datagram.constData());

    // V2 frame sanity: SOF + version + type, then the full length.
    if (raw[0] != 0x55) {
        return fr;
    }
    if ((raw[1] & 0x3F) != 0x18) {
        return fr;
    }
    if ((raw[2] & 0x08) != 0x08) {
        return fr;
    }
    const uint16_t totalLen = uint16_t(raw[3] | (raw[4] << 8));
    if (totalLen != len) {
        return fr;
    }

    const uint16_t msgId =
        uint16_t((raw[V2_MSG_OFFSET] << 8) | raw[V2_MSG_OFFSET + 1]);
    if (msgId != E61_MSG_ID) {
        return fr;
    }

    const uint32_t seq = readU32(raw + V2_SEQ_OFFSET);
    const int payloadLen = len - V2_PAYLOAD_OFFSET - 2;  // minus CRC16 tail
    if (payloadLen < E61_HEADER_LEN) {
        return fr;
    }

    const uint8_t *payload = raw + V2_PAYLOAD_OFFSET;
    const int bodyLen = payloadLen - E61_HEADER_LEN;
    if (bodyLen % E61_ENTRY_LEN != 0) {
        return fr;
    }
    const int count = bodyLen / E61_ENTRY_LEN;
    const uint8_t subtype = payload[E61_SUBTYPE_OFFSET];

    fr.valid = true;
    fr.subtype = subtype;
    fr.entryCount = uint16_t(count);
    fr.frameToken = seq;

    const uint8_t *body = payload + E61_HEADER_LEN;
    for (int i = 0; i < count; ++i) {
        const uint8_t *e = body + i * E61_ENTRY_LEN;
        // Entry layout <HfffH>: id(u16) x(f32) y(f32) z(f32) metric(u16).
        const float x = readF32(e + 2);
        const float y = readF32(e + 6);
        const float metric = float(readU16(e + 14));
        const float r = std::sqrt(x * x + y * y);
        if (!std::isfinite(r) || r < minRangeM) {
            continue;
        }
        const float snr = std::clamp(metric, 0.0f, 255.0f);
        if (snr < minSnr) {
            continue;
        }
        // Radar frame: x = forward, y = right. atan2(y,x) already yields
        // 0 = forward and increases clockwise, matching the PPI convention.
        float az = float(std::atan2(y, x) * kRadToDeg);
        az = float(std::fmod(az + offsetDeg, 360.0));
        if (az < 0.0f) {
            az += 360.0f;
        }
        SectorPoint p;
        p.azimuthDeg = az;
        p.rangeM = r;
        p.snr = snr;
        fr.points.push_back(p);
    }
    return fr;
}
