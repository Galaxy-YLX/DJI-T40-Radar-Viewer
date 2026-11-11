#include "mainwindow.h"

#include <QStatusBar>
#include <QTimer>
#include <cmath>

#include "ppiwidget.h"
#include "sidestream.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
}

MainWindow::MainWindow(const Config &cfg, QWidget *parent)
    : QMainWindow(parent), mOffsetDeg(cfg.offsetDeg) {
    setWindowTitle("DJI T40 — 2D radar (Qt6)");
    resize(720, 720);

    mPpi = new PpiWidget(this);
    mPpi->setRangeLimit(cfg.rangeM);
    mPpi->setPointRadius(cfg.pointRadius);
    setCentralWidget(mPpi);
    statusBar()->showMessage("waiting for SIDE frames (UDP 9877) ...");

    mClock.start();
    if (cfg.demo) {
        mDemoTimer = new QTimer(this);
        connect(mDemoTimer, &QTimer::timeout, this, &MainWindow::onDemoTick);
        mDemoTimer->start(80);
    } else {
        mStream = new SideStream(cfg.host, cfg.port, float(cfg.offsetDeg),
                                 cfg.minSnr, cfg.minRangeM, this);
        connect(mStream, &SideStream::frameReady, this,
                &MainWindow::onFrameReady);
        mStream->start();
    }
}

void MainWindow::applyFrame(const SideFrame &fr, const QString &header) {
    mFrames++;
    const double fps = mFrames / (mClock.elapsed() / 1000.0);
    mPpi->setFrame(fr);
    const QString body =
        QString("blips=%1  frames=%2  fps=%3  range=%4m  size=%5 ([/])")
            .arg(fr.points.size())
            .arg(mFrames)
            .arg(fps, 0, 'f', 1)
            .arg(mPpi->rangeLimit(), 0, 'f', 1)
            .arg(mPpi->pointRadius(), 0, 'f', 1);
    mPpi->setStatusText(header + "\n" + body);
    statusBar()->showMessage(header);
}

void MainWindow::onFrameReady() {
    applyFrame(mStream->latest(), QString("token=%1  sectors=%2  "
                                          "rot=%3/%4/%5  cfar=%6")
                                     .arg(mStream->latest().frameToken)
                                     .arg(mStream->latest().sectorsDecoded)
                                     .arg(mStream->latest().rotAngleA)
                                     .arg(mStream->latest().rotAngleB)
                                     .arg(mStream->latest().rotAngleC)
                                     .arg(mStream->latest().cfarError));
}

void MainWindow::onDemoTick() {
    ++mDemoN;
    QVector<SectorPoint> pts;
    const double phase = mDemoN * 0.02;
    for (int i = 0; i < 360; ++i) {
        const double az = i * 1.0;
        const double r = 9.0 + 2.5 * std::sin(3.0 * az * kPi / 180.0 + phase);
        const double snr = 60.0 + 90.0 * std::fabs(std::sin(2.0 * az * kPi / 180.0));
        SectorPoint p;
        p.azimuthDeg = float(std::fmod(az + mOffsetDeg, 360.0));
        p.rangeM = float(r);
        p.snr = float(snr);
        pts.push_back(p);
    }
    SideFrame fr;
    fr.valid = true;
    fr.sectorsDecoded = 360;
    fr.frameToken = uint16_t(mDemoN & 0xFFFF);
    fr.rotAngleA = uint16_t(mDemoN % 360);
    fr.points = pts;
    applyFrame(fr, QString("token=%1  sectors=360  rot=%2/0/0  [demo]")
                       .arg(mDemoN % 360)
                       .arg(mDemoN % 360));
}
