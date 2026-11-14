#include "mainwindow.h"

#include <QCoreApplication>
#include <QDir>
#include <QLabel>
#include <QPalette>
#include <QProcess>
#include <QPushButton>
#include <QSplitter>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "flatview.h"
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
constexpr int kPowerButtonWidth = 120;
constexpr int kPowerButtonHeight = 52;

constexpr const char *kToolBarStyle =
    "QToolBar { background: #000000; border: none;"
    " font-family: 'Noto Sans CJK SC', monospace; }"
    "QToolBar QLabel { color: #00ff66; }"
    "QToolBar QPushButton { background-color: #000000; color: #00ff66;"
    " border: 1px solid #0b5c2c; min-width: 24px; min-height: 22px;"
    " margin-left: 2px; font-size: 14px; font-weight: bold; }"
    "QToolBar QPushButton:hover:enabled { border-color: #00ff66; }"
    "QToolBar QPushButton:pressed { background-color: #0b3d20; }"
    "QToolBar QPushButton:disabled { color: #0b5c2c;"
    " border-color: #062d16; }"
    "QPushButton#powerButton { font-size: 20px; }";
}

MainWindow::MainWindow(const Config &cfg, QWidget *parent)
    : QMainWindow(parent), mOffsetDeg(cfg.offsetDeg) {
    setWindowTitle("DJI AESA Digital Radar");
    // Side by side: give the PPI enough width that its HUD card does not
    // collide with the azimuth ring, and keep the unrolled view wide.
    resize(1500, 780);

    mPpi = new PpiWidget(this);
    mPpi->setPointRadius(cfg.pointRadius);
    mPpi->setFlipLeftRight(cfg.flipLeftRight);
    mFlat = new FlatView(this);
    mFlat->setRangeLimit(cfg.rangeM);
    mFlat->setPointRadius(std::max(2.0, cfg.pointRadius * 3.0));
    mDemoMode = cfg.demo;
    mHud.rangeM = cfg.rangeM;
    mHud.source = cfg.demo ? QStringLiteral("演示") : QStringLiteral("00F0/UDP");
    mHud.pointRadius = cfg.pointRadius;
    mPpi->setHud(mHud);

    mRadarIp = cfg.host;
    mCanIface = cfg.canIface;
    mFcClientPath = cfg.fcClient.isEmpty()
        ? QDir::cleanPath(QCoreApplication::applicationDirPath()
                          + "/../../radar_client/radar_min_client.py")
        : cfg.fcClient;

    auto makeBar = [this](const QString &title) {
        auto *bar = new QToolBar(title, this);
        bar->setMovable(false);
        bar->setFloatable(false);
        bar->setStyleSheet(QString::fromUtf8(kToolBarStyle));
        QPalette pal = bar->palette();
        pal.setColor(QPalette::Window, Qt::black);
        bar->setPalette(pal);
        bar->setAutoFillBackground(true);
        addToolBar(Qt::TopToolBarArea, bar);
        return bar;
    };

    // Power row sits above the range row.
    auto *powerBar = makeBar("电源");
    mPowerOn = new QPushButton("开机", powerBar);
    mPowerOn->setObjectName("powerButton");
    mPowerOn->setFixedSize(kPowerButtonWidth, kPowerButtonHeight);
    mPowerOn->setFocusPolicy(Qt::NoFocus);
    mPowerOn->setToolTip("启动 FC 保活与 CAN 电机，使雷达进入工作状态");
    powerBar->addWidget(mPowerOn);

    mPowerOff = new QPushButton("关机", powerBar);
    mPowerOff->setObjectName("powerButton");
    mPowerOff->setFixedSize(kPowerButtonWidth, kPowerButtonHeight);
    mPowerOff->setFocusPolicy(Qt::NoFocus);
    mPowerOff->setToolTip("停止 FC 保活并发送停电机命令");
    powerBar->addWidget(mPowerOff);
    addToolBarBreak(Qt::TopToolBarArea);

    auto *topBar = makeBar("量程");

    auto *rangeLabel = new QLabel("量程", topBar);
    topBar->addWidget(rangeLabel);
    mPpi->setHudAnchor(rangeLabel);
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

    topBar->addSeparator();
    auto *viewToggle = new QPushButton("展开视图", topBar);
    viewToggle->setCheckable(true);
    viewToggle->setChecked(true);
    viewToggle->setFocusPolicy(Qt::NoFocus);
    viewToggle->setToolTip(
        "显示/隐藏方位-俯仰平铺视图（横轴 240→120，纵轴 +45→−45）");
    viewToggle->setStyleSheet(
        "QPushButton:checked { background-color: #0b3d20;"
        " border-color: #00ff66; }");
    topBar->addWidget(viewToggle);
    connect(viewToggle, &QPushButton::toggled, mFlat, &QWidget::setVisible);

    auto *central = new QWidget(this);
    auto *vbox = new QVBoxLayout(central);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(0);
    auto *split = new QSplitter(Qt::Horizontal, central);
    split->setChildrenCollapsible(false);
    split->addWidget(mPpi);
    split->addWidget(mFlat);
    split->setStretchFactor(0, 8);
    split->setStretchFactor(1, 7);
    split->setSizes(QList<int>{800, 700});
    vbox->addWidget(split, 1);
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
    // The distance colour scale follows the PPI's range rings.
    connect(mPpi, &PpiWidget::rangeChanged, mFlat,
            &FlatView::setRangeLimit);

    // "开机" starts the FC stand-in that keeps the radar in its working
    // state; "关机" sends SIGTERM so its close() can emit motor_stop and
    // the idle CAN setpoint before exiting.
    mFcProcess = new QProcess(this);
    mFcProcess->setProcessChannelMode(QProcess::ForwardedChannels);
    connect(mFcProcess, &QProcess::finished, this,
            [this](int, QProcess::ExitStatus) { setPowerRunning(false); });
    connect(mFcProcess, &QProcess::errorOccurred, this,
            [this](QProcess::ProcessError err) {
        if (err == QProcess::FailedToStart) {
            std::fprintf(stderr, "[power] FC client failed to start: %s\n",
                         qPrintable(mFcProcess->errorString()));
            setPowerRunning(false);
        }
    });
    connect(mPowerOn, &QPushButton::clicked, this, [this] { startRadar(); });
    connect(mPowerOff, &QPushButton::clicked, this, [this] { stopRadar(); });
    setPowerRunning(false);

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
                                 cfg.minAmp, cfg.minRangeM, this);
        connect(mStream, &SideStream::frameReady, this,
                &MainWindow::onFrameReady);
        mStream->start();
    }
}

MainWindow::~MainWindow() {
    if (mFcProcess && mFcProcess->state() != QProcess::NotRunning) {
        mFcProcess->terminate();
        if (!mFcProcess->waitForFinished(1000)) {
            mFcProcess->kill();
            mFcProcess->waitForFinished(1000);
        }
    }
}

void MainWindow::startRadar() {
    if (mFcProcess->state() != QProcess::NotRunning) {
        return;
    }
    setPowerRunning(true);
    mFcProcess->start(QStringLiteral("python3"),
                      {mFcClientPath, QStringLiteral("--can-bridge"), mCanIface,
                       QStringLiteral("--radar-ip"), mRadarIp,
                       QStringLiteral("--no-v2-listen")});
}

void MainWindow::stopRadar() {
    if (mFcProcess->state() == QProcess::NotRunning) {
        return;
    }
    mPowerOff->setEnabled(false);
    mFcProcess->terminate();
    if (!mFcProcess->waitForFinished(3000)) {
        mFcProcess->kill();
        mFcProcess->waitForFinished(1000);
    }
}

void MainWindow::setPowerRunning(bool running) {
    mPowerOn->setEnabled(!mDemoMode && !running);
    mPowerOff->setEnabled(!mDemoMode && running);
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
    mHud.entries = int(fr.entryCount);
    mHud.blips = int(fr.points.size());
    mHud.sumOk = fr.sumOk;
    mHud.rpm = fr.rpm;
    mHud.source = mDemoMode ? QStringLiteral("演示")
                            : QStringLiteral("00F0/UDP");
    mHud.packetLossPct = mStream ? mStream->packetLossPct() : 0.0;
    mPpi->setFrame(fr);
    mFlat->setFrame(fr);
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
    mFlat->setPointRadius(std::max(2.0, mPpi->pointRadius() * 3.0));
    mPpi->setHud(mHud);
}

void MainWindow::onFrameReady() {
    if (mStream) {
        applyFrame(mStream->latest());
    }
}

void MainWindow::onDemoTick() {
    ++mDemoN;
    QVector<SectorPoint> pts;
    const double phase = mDemoN * 0.02;
    for (int i = -120; i <= 120; ++i) {
        const double az = i * 1.0;
        // Sweep the full range limit so the demo exercises red -> green ->
        // black, and keep every elevation inside the +/-45 deg window.
        const double r = 2.5 + 21.0 * (i + 120) / 240.0
                         + 1.2 * std::sin(3.0 * az * kPi / 180.0 + phase);
        const double el = 32.0 * std::sin(2.0 * az * kPi / 180.0 + phase);
        const double snr = 60.0 + 90.0 * std::fabs(std::sin(2.0 * az * kPi / 180.0));
        SectorPoint p;
        p.azimuthDeg = float(std::fmod(az + mOffsetDeg + 360.0, 360.0));
        p.elevationDeg = float(el);
        p.rangeM = float(r);
        p.snr = float(snr);
        pts.push_back(p);
    }
    SideFrame fr;
    fr.valid = true;
    fr.entryCount = uint16_t(pts.size());
    fr.frameToken = uint32_t(mDemoN);
    fr.points = pts;
    applyFrame(fr);
}
