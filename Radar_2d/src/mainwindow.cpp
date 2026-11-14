#include "mainwindow.h"

#include <QCoreApplication>
#include <QDir>
#include <QLabel>
#include <QPalette>
#include <QProcess>
#include <QPushButton>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "linkstream.h"
#include "ppiwidget.h"
#include "sidestream.h"

namespace {
QString sourceLabel(PointSource s) {
    switch (s) {
    case PointSource::F0Link: return QStringLiteral("00F0/链路");
    case PointSource::F0Udp:  return QStringLiteral("00F0/UDP");
    case PointSource::E61:    return QStringLiteral("1E61/UDP");
    }
    return QStringLiteral("?");
}

constexpr double kPi = 3.14159265358979323846;
constexpr int kRangeMinM = 5;
constexpr int kRangeMaxM = 50;
constexpr int kRangeStepM = 5;
constexpr int kFpsWindowMs = 2000;  // sliding window for the instant frame rate
constexpr int kFpsStaleMs = 500;    // no frame this long -> 0 Hz
constexpr int kHudRefreshMs = 200;  // status text refresh, decoupled from frames

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
    " border-color: #062d16; }";
}

MainWindow::MainWindow(const Config &cfg, QWidget *parent)
    : QMainWindow(parent), mOffsetDeg(cfg.offsetDeg) {
    setWindowTitle("大疆 T40 — 侧向雷达 (Qt6)");
    resize(720, 780);

    mPpi = new PpiWidget(this);
    mPpi->setPointRadius(cfg.pointRadius);
    mPpi->setFlipLeftRight(cfg.flipLeftRight);
    mDemoMode = cfg.demo;
    mRpm = cfg.rpm;
    mHud.rangeM = cfg.rangeM;
    mHud.source = cfg.demo ? QStringLiteral("演示")
                : (cfg.source == Source::Link ? QStringLiteral("00F0/链路")
                                              : QStringLiteral("UDP/自动"));
    mHud.linkUp = cfg.demo || cfg.source != Source::Link;
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
    mPowerOn->setFixedWidth(60);
    mPowerOn->setFocusPolicy(Qt::NoFocus);
    mPowerOn->setToolTip("启动 FC 保活与 CAN 电机，使雷达进入工作状态");
    powerBar->addWidget(mPowerOn);

    mPowerOff = new QPushButton("关机", powerBar);
    mPowerOff->setFixedWidth(60);
    mPowerOff->setFocusPolicy(Qt::NoFocus);
    mPowerOff->setToolTip("停止 FC 保活并发送停电机命令");
    powerBar->addWidget(mPowerOff);
    addToolBarBreak(Qt::TopToolBarArea);

    auto *topBar = makeBar("量程");

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
    } else if (cfg.source == Source::Link) {
        mLink = new LinkStream(cfg.linkHost, cfg.linkPort,
                               float(cfg.offsetDeg), cfg.minAmp,
                               cfg.minRangeM, this);
        connect(mLink, &LinkStream::frameReady, this,
                &MainWindow::onFrameReady);
        connect(mLink, &LinkStream::stateChanged, this,
                &MainWindow::refreshHud);
        mLink->start();
    } else {
        mStream = new SideStream(cfg.host, cfg.port, float(cfg.offsetDeg),
                                 cfg.minSnr, cfg.minAmp, cfg.minRangeM, this);
        connect(mStream, &SideStream::frameReady, this,
                &MainWindow::onFrameReady);
        mStream->start();
    }
}

MainWindow::~MainWindow() {
    if (mLink) {
        mLink->stop();
    }
    if (mFcProcess && mFcProcess->state() != QProcess::NotRunning) {
        mFcProcess->terminate();
        if (!mFcProcess->waitForFinished(1000)) {
            mFcProcess->kill();
            mFcProcess->waitForFinished(1000);
        }
    }
}

void MainWindow::startRadar() {
    if (mLink) {
        // Over the link the board owns the motor CAN commands (CMD 0x40);
        // starting the local FC client would fight with it over the radar.
        if (!mLink->isConnected()) {
            std::fprintf(stderr, "[power] link is down, power-on skipped\n");
            return;
        }
        QByteArray args(2, '\0');
        args[0] = char(quint8(mRpm & 0xFF));
        args[1] = char(quint8((mRpm >> 8) & 0xFF));
        mLink->sendCmd(LinkStream::kOpMotorEnable);
        mLink->sendCmd(LinkStream::kOpSetRpm, args);
        setPowerRunning(true);
        return;
    }
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
    if (mLink) {
        mLink->sendCmd(LinkStream::kOpStop);
        setPowerRunning(false);
        return;
    }
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
    mHud.subtype = int(fr.subtype);
    mHud.entries = int(fr.entryCount);
    mHud.blips = int(fr.points.size());
    mHud.sumOk = fr.sumOk;
    mHud.rpm = fr.rpm;
    mHud.source = mDemoMode ? QStringLiteral("演示") : sourceLabel(fr.source);
    mHud.packetLossPct = mLink ? mLink->packetLossPct()
                               : (mStream ? mStream->packetLossPct() : 0.0);
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
    mHud.linkUp = mLink ? mLink->isConnected() : true;
    mHud.fps = std::round(fps * 10.0) / 10.0;
    mHud.frames = mFrames;
    mHud.rangeM = mPpi->rangeLimit();
    mHud.pointRadius = mPpi->pointRadius();
    mPpi->setHud(mHud);
}

void MainWindow::onFrameReady() {
    if (mLink) {
        applyFrame(mLink->latest());
    } else if (mStream) {
        applyFrame(mStream->latest());
    }
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
