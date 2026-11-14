#include "mainwindow.h"

#include <QLabel>
#include <QPalette>
#include <QPushButton>
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
    mPpi->setFlipLeftRight(cfg.flipLeftRight);
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
        "QToolBar QLabel { color: #00ff66; }"
        "QToolBar QPushButton { background-color: #000000; color: #00ff66;"
        " border: 1px solid #0b5c2c; min-width: 24px; min-height: 22px;"
        " margin-left: 2px; font-size: 14px; font-weight: bold; }"
        "QToolBar QPushButton:hover:enabled { border-color: #00ff66; }"
        "QToolBar QPushButton:pressed { background-color: #0b3d20; }"
        "QToolBar QPushButton:disabled { color: #0b5c2c;"
        " border-color: #062d16; }");
    QPalette barPal = topBar->palette();
    barPal.setColor(QPalette::Window, Qt::black);
    topBar->setPalette(barPal);
    topBar->setAutoFillBackground(true);
    addToolBar(Qt::TopToolBarArea, topBar);

    topBar->addWidget(new QLabel("量程", topBar));
    mRangeValue = new QLabel(topBar);
    mRangeValue->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    mRangeValue->setMinimumWidth(40);
    mRangeValue->setStyleSheet("font-size: 15px; font-weight: bold;");
    mRangeValue->setToolTip("显示量程（i / o 键或 + / - 按钮调整）");
    topBar->addWidget(mRangeValue);
    auto *rangeUnit = new QLabel("m", topBar);
    rangeUnit->setStyleSheet("margin-left: 4px; margin-right: 4px;");
    topBar->addWidget(rangeUnit);

    mRangePlus = new QPushButton("+", topBar);
    mRangePlus->setToolTip("量程 +5 m");
    mRangePlus->setFocusPolicy(Qt::NoFocus);
    topBar->addWidget(mRangePlus);

    mRangeMinus = new QPushButton("−", topBar);
    mRangeMinus->setToolTip("量程 -5 m");
    mRangeMinus->setFocusPolicy(Qt::NoFocus);
    topBar->addWidget(mRangeMinus);

    auto *central = new QWidget(this);
    auto *vbox = new QVBoxLayout(central);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(0);
    vbox->addWidget(mPpi, 1);
    setCentralWidget(central);

    mPpi->setRangeLimit(cfg.rangeM);
    syncRangeDisplay(mPpi->rangeLimit());

    connect(mRangePlus, &QPushButton::clicked, this, [this] {
        mPpi->setRangeLimit(mPpi->rangeLimit() + kRangeStepM);
    });
    connect(mRangeMinus, &QPushButton::clicked, this, [this] {
        mPpi->setRangeLimit(mPpi->rangeLimit() - kRangeStepM);
    });
    connect(mPpi, &PpiWidget::rangeChanged, this,
            &MainWindow::syncRangeDisplay);

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

void MainWindow::syncRangeDisplay(double m) {
    const QString text = std::abs(m - std::round(m)) < 0.05
                             ? QString::number(int(std::lround(m)))
                             : QString::number(m, 'f', 1);
    mRangeValue->setText(text);
    mRangeMinus->setEnabled(m > kRangeMinM + 0.05);
    mRangePlus->setEnabled(m < kRangeMaxM - 0.05);
}

void MainWindow::applyFrame(const SideFrame &fr) {
    mFrames++;
    mFrameTimes.push_back(mClock.elapsed());
    mHud.token = int(fr.frameToken);
    mHud.subtype = int(fr.subtype);
    mHud.entries = int(fr.entryCount);
    mHud.blips = int(fr.points.size());
    mHud.packetLossPct = mStream ? mStream->packetLossPct() : 0.0;
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
    fr.subtype = 2;  // pcla
    fr.entryCount = 360;
    fr.frameToken = uint32_t(mDemoN);
    fr.points = pts;
    applyFrame(fr);
}
