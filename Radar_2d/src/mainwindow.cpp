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
    setWindowTitle("大疆 T40 — 侧向雷达 (Qt6)");
    resize(720, 780);

    mPpi = new PpiWidget(this);
    mPpi->setPointRadius(cfg.pointRadius);
    mDemoMode = cfg.demo;
    mHud.rangeM = cfg.rangeM;
    mHud.pointRadius = cfg.pointRadius;
    mPpi->setHud(mHud);

    auto *topBar = new QToolBar("量程", this);
    topBar->setMovable(false);
    topBar->setFloatable(false);
    topBar->setStyleSheet(
        "QToolBar { background: #000000; border: none;"
        " font-family: 'Noto Sans CJK SC', monospace; }"
        "QToolBar QLabel { color: #00ff66; }");
    addToolBar(Qt::TopToolBarArea, topBar);

    topBar->addWidget(new QLabel("量程", topBar));
    mRangeCombo = new QComboBox(topBar);
    mRangeCombo->setMinimumWidth(100);
    mRangeCombo->setStyleSheet(
        "QComboBox { background-color: #000000; color: #00ff66;"
        " border: 1px solid #0b5c2c; padding: 2px 6px;"
        " font-family: 'Noto Sans CJK SC', monospace; }"
        "QComboBox:hover { border-color: #00ff66; }"
        "QComboBox::drop-down { subcontrol-origin: padding;"
        " subcontrol-position: top right; width: 18px;"
        " border-left: 1px solid #0b5c2c; }"
        "QComboBox::down-arrow {"
        " border-left: 4px solid transparent;"
        " border-right: 4px solid transparent;"
        " border-top: 5px solid #00ff66; }"
        "QComboBox QAbstractItemView { background-color: #000000;"
        " color: #00ff66; border: 1px solid #0b5c2c;"
        " selection-background-color: #0b3d20;"
        " selection-color: #00ff66; }");
    mRangeCombo->setToolTip("显示量程（滚轮或 i / o 键调整）");
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

void MainWindow::applyFrame(const SideFrame &fr) {
    mFrames++;
    mFrameTimes.push_back(mClock.elapsed());
    mHud.token = fr.frameToken;
    mHud.sectors = fr.sectorsDecoded;
    mHud.rotA = fr.rotAngleA;
    mHud.rotB = fr.rotAngleB;
    mHud.rotC = fr.rotAngleC;
    mHud.cfarError = fr.cfarError;
    mHud.blips = int(fr.points.size());
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
    mHud.live = mFrames > 0;
    mHud.demo = mDemoMode;
    mHud.fps = std::round(fps * 10.0) / 10.0;
    mHud.frames = mFrames;
    mHud.rangeM = mPpi->rangeLimit();
    mHud.pointRadius = mPpi->pointRadius();
    mPpi->setHud(mHud);
}

void MainWindow::onFrameReady() {
    applyFrame(mStream->latest());
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
    applyFrame(fr);
}
