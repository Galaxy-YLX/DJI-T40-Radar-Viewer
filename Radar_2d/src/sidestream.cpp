#include "sidestream.h"

#include <QDateTime>
#include <QHostAddress>
#include <QNetworkProxy>
#include <cmath>
#include <cstdio>
#include <utility>

#include "f0decoder.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxMatchSpeedMps = 15.0;  // association gate, not a display gate
constexpr double kMatchSlackM = 0.25;       // tolerates small assay jitter
constexpr double kSmoothAlpha = 0.35;
}  // namespace

void CrossFrameVelocity::reset() {
    mTracks.clear();
    mHaveLast = false;
}

void CrossFrameVelocity::estimate(SideFrame &fr) {
    const double dtSec = fr.dtMs / 1000.0;
    const bool continuous = mHaveLast
                            && fr.frameIndex == mLastFrame + 1
                            && dtSec > 0.0 && dtSec < 0.5;
    if (!continuous) {
        reset();
    }

    const int n = fr.points.size();
    QVector<double> xs(n), ys(n);
    for (int i = 0; i < n; ++i) {
        const double rad = double(fr.points.at(i).azimuthDeg) * kPi / 180.0;
        const double r = double(fr.points.at(i).rangeM);
        xs[i] = r * std::sin(rad);
        ys[i] = r * std::cos(rad);
    }

    const int m = mTracks.size();
    QVector<int> prevForCur(n, -1);
    QVector<int> curForPrev(m, -1);
    QVector<double> dist2(n, 0.0);
    if (continuous && m > 0 && n > 0) {
        const double radius = kMatchSlackM + kMaxMatchSpeedMps * dtSec;
        const double radius2 = radius * radius;
        for (int i = 0; i < n; ++i) {
            double best = radius2;
            for (int j = 0; j < m; ++j) {
                const double dx = xs[i] - mTracks.at(j).xM;
                const double dy = ys[i] - mTracks.at(j).yM;
                const double d2 = dx * dx + dy * dy;
                if (d2 < best) {
                    best = d2;
                    prevForCur[i] = j;
                    dist2[i] = d2;
                }
            }
        }
        // Mutual nearest neighbour suppresses matches to a different static
        // blob that merely happens to sit inside the association radius.
        for (int j = 0; j < m; ++j) {
            double best = radius2;
            for (int i = 0; i < n; ++i) {
                const double dx = xs[i] - mTracks.at(j).xM;
                const double dy = ys[i] - mTracks.at(j).yM;
                const double d2 = dx * dx + dy * dy;
                if (d2 < best) {
                    best = d2;
                    curForPrev[j] = i;
                }
            }
        }
    }

    QVector<bool> usedPrev(m, false);
    QVector<Tracklet> next;
    next.reserve(n);
    for (int i = 0; i < n; ++i) {
        const int j = prevForCur.at(i);
        Tracklet tr;
        tr.xM = xs.at(i);
        tr.yM = ys.at(i);
        if (j >= 0 && curForPrev.at(j) == i && !usedPrev.at(j)) {
            usedPrev[j] = true;
            const double instant = std::sqrt(dist2.at(i)) / dtSec;
            tr.speedMps = float((1.0 - kSmoothAlpha) * mTracks.at(j).speedMps
                                + kSmoothAlpha * instant);
        }
        SectorPoint pt = fr.points.at(i);
        pt.speedMps = tr.speedMps;
        fr.points[i] = pt;
        next.push_back(tr);
    }

    mTracks = next;
    mLastFrame = fr.frameIndex;
    mHaveLast = true;
}

SideStream::SideStream(const QString &host, quint16 port, float azimuthOffsetDeg,
                       int minAmp, float minRangeM, QObject *parent)
    : QObject(parent),
      mHost(host),
      mPort(port),
      mOffsetDeg(azimuthOffsetDeg),
      mMinAmp(minAmp),
      mMinRangeM(minRangeM) {
    mClock.start();
    connect(&mSocket, &QUdpSocket::readyRead, this, &SideStream::onReadyRead);
}

void SideStream::start() {
    mSocket.setProxy(QNetworkProxy::NoProxy);  // raw UDP, never via a proxy
    if (!mSocket.bind(QHostAddress::AnyIPv4, mPort)) {
        std::fprintf(stderr, "[udp] bind UDP :%u failed: %s\n", mPort,
                     mSocket.errorString().toUtf8().constData());
        return;
    }
    std::fprintf(stderr, "[udp] listening on UDP :%u (0x00F0 point cloud only)\n",
                 mPort);
}

void SideStream::onReadyRead() {
    // UDP 40204 also carries 1E61 perception packets. The 2D PPI consumes
    // only the 0x00F0 point cloud, so every other datagram is ignored.
    // Every valid datagram is emitted as its own frame: a burst of N packets
    // must produce N frames, not just the last one, otherwise dt and the
    // frame sequence fed to a tracker are both wrong.
    while (mSocket.hasPendingDatagrams()) {
        const qint64 size = mSocket.pendingDatagramSize();
        if (size <= 0) {
            continue;
        }
        QByteArray d;
        d.resize(int(size));
        mSocket.readDatagram(d.data(), d.size());
        const F0Frame f0 = F0::decode(
            reinterpret_cast<const uint8_t *>(d.constData()), d.size());
        if (!f0.valid) {
            continue;
        }
        SideFrame fr = F0::toSideFrame(f0, mOffsetDeg, mMinAmp, mMinRangeM);

        // Frame header. recvMonoUs is the primary time base: c5 wraps every
        // 65.536 ms, which is the same order as one revolution (50 ms at
        // 1200 rpm), so it cannot be used as an absolute timestamp here.
        const qint64 nowUs = mClock.nsecsElapsed() / 1000;
        fr.frameIndex = mFrameIndex++;
        fr.recvMonoUs = nowUs;
        fr.tsWallMs = QDateTime::currentMSecsSinceEpoch();
        fr.dtMs = mLastMonoUs > 0 ? double(nowUs - mLastMonoUs) / 1000.0 : 0.0;
        fr.revUs = fr.rpm > 0.0 ? 60e6 / fr.rpm : 0.0;
        mLastMonoUs = nowUs;

        // Packet loss is tracked on the 0x00F0 container sequence only,
        // once per frame (not once per readyRead burst).
        const uint32_t seq = fr.frameToken;
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

        mLatest = fr;
        emit frameReady(fr);
    }
}
