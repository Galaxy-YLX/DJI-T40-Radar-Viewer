#include "sidestream.h"

#include <QHostAddress>
#include <QNetworkProxy>
#include <cstdio>
#include <utility>

#include "f0decoder.h"

SideStream::SideStream(const QString &host, quint16 port, float azimuthOffsetDeg,
                       int minAmp, float minRangeM, QObject *parent)
    : QObject(parent),
      mHost(host),
      mPort(port),
      mOffsetDeg(azimuthOffsetDeg),
      mMinAmp(minAmp),
      mMinRangeM(minRangeM) {
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
    SideFrame display;
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
        display = F0::toSideFrame(f0, mOffsetDeg, mMinAmp, mMinRangeM);

        // Packet loss is tracked on the 0x00F0 container sequence only.
        const uint32_t seq = display.frameToken;
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
    }
    if (!display.valid) {
        return;
    }
    mLatest = std::move(display);
    emit frameReady();
}
