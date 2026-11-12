#include "mainwindow.h"

#include <QComboBox>
#include <QLabel>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

#include "ppiwidget.h"
#include "sidestream.h"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kRangeMinM = 5;
constexpr int kRangeMaxM = 50;
constexpr int kRangeStepM = 5;
constexpr int kFpsWindowMs = 2000;  // sliding window for the instant frame rate
constexpr int kFpsStaleMs = 500;    // no frame this long -> 0 Hz
constexpr int kHudRefreshMs = 200;  // status text refresh, decoupled from frames
}

MainWindow::MainWindow(const Config &cfg, QWidget *parent)
    : QMainWindow(parent), mOffsetDeg(cfg.offsetDeg) {
    setWindowTitle("DJI T40 — 2D radar (Qt6)");
    resize(720, 780);

    mPpi = new PpiWidget(this);
    mPpi->setPointRadius(cfg.pointRadius);

    auto *topBar = new QToolBar("Range", this);
    topBar->setMovable(false);
    topBar->setFloatable(false);
    topBar->setStyleSheet(
        "QToolBar { background: #000000; border: none; }"
        "QToolBar QLabel { color: #e6e6e6; }");
    addToolBar(Qt::TopToolBarArea, topBar);

    topBar->addWidget(new QLabel("range", topBar));
    mRangeCombo = new QComboBox(topBar);
    mRangeCombo->setMinimumWidth(100);
    mRangeCombo->setStyleSheet(
        "QComboBox { background-color: #000000; color: #e6e6e6;"
        " border: 1px solid #3a3a3a; border-radius: 3px; padding: 2px 6px; }"
        "QComboBox:hover { border-color: #6a6a6a; }"
        "QComboBox::drop-down { subcontrol-origin: padding;"
        " subcontrol-position: top right; width: 18px;"
        " border-left: 1px solid #3a3a3a; }"
        "QComboBox::down-arrow {"
        " border-left: 4px solid transparent;"
        " border-right: 4px solid transparent;"
        " border-top: 5px solid #e6e6e6; }"
        "QComboBox QAbstractItemView { background-color: #000000;"
        " color: #e6e6e6; border: 1px solid #3a3a3a;"
        " selection-background-color: #1f4e79;"
        " selection-color: #ffffff; }");
    mRangeCombo->setToolTip("scope range limit (wheel / i, o keys also work)");
    for (int v = kRangeMinM; v <= kRangeMaxM; v += kRangeStepM) {
        mRangeCombo->addItem(QString::number(v) + " m", v);
    }
    topBar->addWidget(mRangeCombo);

    auto *central = new QWidget(this);
    auto *vbox = new QVBoxLayout(central);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(0);
    vbox->addWidget(mPpi, 1);
    setCentralWidget(central);

    mPpi->setRangeLimit(cfg.rangeM);
    syncRangeCombo(mPpi->rangeLimit());

    connect(mRangeCombo, &QComboBox::activated, this, [this](int idx) {
        const QVariant v = mRangeCombo->itemData(idx);
        if (v.isValid()) {
            mPpi->setRangeLimit(v.toInt());
        }
    });
    connect(mPpi, &PpiWidget::rangeChanged, this,
            &MainWindow::syncRangeCombo);

    mClock.start();
    mHudTimer = new QTimer(this);
    connect(mHudTimer, &QTimer::timeout, this, &MainWindow::refreshHud);
    mHudTimer->start(kHudRefreshMs);
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

void MainWindow::syncRangeCombo(double m) {
    const int nearest = std::clamp(
        int(std::lround(m / kRangeStepM)) * kRangeStepM, kRangeMinM, kRangeMaxM);
    const int idx = std::abs(m - nearest) < 0.05
                        ? mRangeCombo->findData(nearest) : -1;
    const int last = mRangeCombo->count() - 1;
    if (idx >= 0) {
        if (mCustomRangeItem) {
            mRangeCombo->removeItem(last);
            mCustomRangeItem = false;
        }
        mRangeCombo->setCurrentIndex(idx);
        return;
    }
    const QString text = QString::number(m, 'f', 1) + " m";
    if (mCustomRangeItem) {
        mRangeCombo->setItemText(last, text);
    } else {
        mRangeCombo->addItem(text);
        mCustomRangeItem = true;
    }
    mRangeCombo->setCurrentIndex(mRangeCombo->count() - 1);
}

void MainWindow::applyFrame(const SideFrame &fr, const QString &header) {
    mFrames++;
    mFrameTimes.push_back(mClock.elapsed());
    mHeader = header;
    mLastBlips = int(fr.points.size());
    mPpi->setFrame(fr);
}

void MainWindow::refreshHud() {
    const qint64 now = mClock.elapsed();
    while (!mFrameTimes.empty() && mFrameTimes.front() < now - kFpsWindowMs) {
        mFrameTimes.pop_front();
    }
    double fps = 0.0;
    if (mFrameTimes.size() >= 2 && now - mFrameTimes.back() <= kFpsStaleMs) {
        const double span =
            double(mFrameTimes.back() - mFrameTimes.front()) / 1000.0;
        if (span > 0.0) {
            fps = double(mFrameTimes.size() - 1) / span;
        }
    }
    const QString text =
        mHeader.isEmpty()
            ? QString("waiting for SIDE frames (UDP 9877) ...")
            : mHeader + "\n" +
                  QString("blips=%1  frames=%2  fps=%3Hz  range=%4m  size=%5 ([/])")
                      .arg(mLastBlips, 4)
                      .arg(mFrames, 6)
                      .arg(fps, 5, 'f', 1)
                      .arg(mPpi->rangeLimit(), 0, 'f', 1)
                      .arg(mPpi->pointRadius(), 0, 'f', 1);
    if (text != mLastStatus) {
        mLastStatus = text;
        mPpi->setStatusText(text);
    }
}

void MainWindow::onFrameReady() {
    const SideFrame &fr = mStream->latest();
    applyFrame(fr, QString("token=%1  sectors=%2  rot=%3/%4/%5  cfar=%6")
                       .arg(fr.frameToken)
                       .arg(fr.sectorsDecoded)
                       .arg(fr.rotAngleA)
                       .arg(fr.rotAngleB)
                       .arg(fr.rotAngleC)
                       .arg(fr.cfarError));
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
