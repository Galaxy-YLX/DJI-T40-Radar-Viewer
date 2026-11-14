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
    // UDP 40204 interleaves 0x00F0 point-cloud frames with 1E61 perception
    // packets. The 1E61 stream currently arrives as empty frames, and its
    // no-data wedge (120..240 deg, lower half) is opposite the 0x00F0 wedge
    // (~298..59 deg, upper half). Replacing the display frame on every empty
    // packet flips both the blips and the grid up/down at packet rate, so
    // prefer the point cloud and keep the last good frame otherwise.
    SideFrame display;
    bool haveDisplay = false;
    SideFrame e61;
    int burstValid = 0;

    // Packet loss is tracked on the 0x00F0 container sequence only; mixing in
    // the 1E61 u32 sequence would make the estimate meaningless.
    auto noteF0Loss = [this](uint32_t seq) {
        if (mHaveSeq) {
            const uint32_t delta = uint32_t(uint16_t(seq - mLastSeq));
            if (delta >= 1 && delta <= 1000) {
                mExpected += delta;
                ++mReceived;
                if (mExpected > 0) {
                    mPacketLossPct =
                        100.0 * double(mExpected - mReceived) / double(mExpected);
                }
            }
        }
        mLastSeq = seq;
        mHaveSeq = true;
    };

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
            // The same UDP port also carries the 0x00F0 black-box point cloud.
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
        ++burstValid;
        if (fr.source == PointSource::F0Udp) {
            mSeenF0 = true;
            display = std::move(fr);
            haveDisplay = true;
            noteF0Loss(display.frameToken);
        } else if (!mSeenF0 && fr.source == PointSource::E61) {
            // Only fall back to 1E61 when no point cloud has ever arrived.
            // Its sparse detections and opposite no-data wedge would make
            // the PPI flip up/down whenever it interleaves with 0x00F0.
            // An empty subtype frame must not wipe the accumulated points.
            if (fr.subtype <= 2 && !fr.points.isEmpty()) {
                mAccum[fr.subtype] = fr.points;
            }
            e61 = std::move(fr);
        }
    }
    if (burstValid == 0) {
        return;
    }

    if (!haveDisplay) {
        if (!e61.valid) {
            return;
        }
        // Combine the three 1E61 subtypes, but only display them when there
        // are real detections: an empty burst keeps the previous point cloud.
        QVector<SectorPoint> pts;
        for (int s = 0; s < 3; ++s) {
            pts += mAccum[s];
        }
        if (pts.isEmpty()) {
            return;
        }
        e61.points = pts;
        display = std::move(e61);
    }

    mLatest = std::move(display);
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
