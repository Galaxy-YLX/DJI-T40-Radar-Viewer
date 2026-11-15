#include "mainwindow.h"

#include <QCoreApplication>
#include <QDateTime>
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

#include "f0decoder.h"
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

// Every toolbar button follows the power buttons' current spec: 18px label
// with 1px vertical / 12px horizontal padding, width following its text.
constexpr const char *kToolBarStyle =
    "QToolBar { background: #000000; border: none;"
    " font-family: 'Noto Sans CJK SC', monospace; }"
    "QToolBar QLabel { color: #00ff66; }"
    "QToolBar QPushButton { background-color: #000000; color: #00ff66;"
    " border: 1px solid #0b5c2c; min-width: 24px; min-height: 22px;"
    " margin-left: 2px; font-size: 18px; padding: 1px 12px;"
    " font-weight: bold; }"
    "QToolBar QPushButton:hover:enabled { border-color: #00ff66; }"
    "QToolBar QPushButton:pressed { background-color: #0b3d20; }"
    "QToolBar QPushButton:disabled { color: #0b5c2c;"
    " border-color: #062d16; }"
    // The +/- range buttons keep their original compact 14px design.
    "QPushButton#rangeButton { font-size: 14px; padding: 0px; }";
}

MainWindow::MainWindow(const Config &cfg, QWidget *parent)
    : QMainWindow(parent), mCfg(cfg), mOffsetDeg(cfg.offsetDeg) {
    setWindowTitle("DJI AESA Digital Radar");
    // Toggling the flat view off briefly exposes its parents' background.
    // Keep every parent surface black so that repaint never flashes white.
    QPalette blackPal = palette();
    blackPal.setColor(QPalette::Window, Qt::black);
    setPalette(blackPal);
    setAutoFillBackground(true);
    resize(1500, 780);

    mMeta.offsetDeg = cfg.offsetDeg;
    mMeta.minAmp = cfg.minAmp;
    mMeta.minRangeM = cfg.minRangeM;
    mMeta.rangeM = cfg.rangeM;
    mMeta.flipLeftRight = cfg.flipLeftRight;
    mMeta.source = cfg.demo ? QStringLiteral("demo")
                            : QStringLiteral("udp:40204");

    mPpi = new PpiWidget(this);
    mPpi->setPointRadius(cfg.pointRadius);
    mPpi->setFlipLeftRight(cfg.flipLeftRight);
    mFlat = new FlatView(this);
    mFlat->setFlipLeftRight(cfg.flipLeftRight);
    mFlat->setRangeLimit(cfg.rangeM);
    mFlat->setPointRadius(std::max(2.0, cfg.pointRadius * 3.0));
    mDemoMode = cfg.demo;
    mHud.rangeM = cfg.rangeM;
    mHud.source = cfg.demo ? QStringLiteral("演示") : QStringLiteral("00F0/UDP");
    mHud.mode = QStringLiteral("实时");
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
    mPowerOn->setFocusPolicy(Qt::NoFocus);
    mPowerOn->setToolTip("启动 FC 保活与 CAN 电机，使雷达进入工作状态");
    powerBar->addWidget(mPowerOn);

    mPowerOff = new QPushButton("关机", powerBar);
    mPowerOff->setObjectName("powerButton");
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
    mRangePlus->setObjectName("rangeButton");
    mRangePlus->setToolTip("量程 +5 m");
    mRangePlus->setFocusPolicy(Qt::NoFocus);
    topBar->addWidget(mRangePlus);

    mRangeMinus = new QPushButton("−", topBar);
    mRangeMinus->setObjectName("rangeButton");
    mRangeMinus->setToolTip("量程 -5 m");
    mRangeMinus->setFocusPolicy(Qt::NoFocus);
    topBar->addWidget(mRangeMinus);

    // View toggle gets its own toolbar row, directly below the range row.
    addToolBarBreak(Qt::TopToolBarArea);
    auto *viewBar = makeBar("视图");
    auto *viewToggle = new QPushButton("展开视图", viewBar);
    viewToggle->setCheckable(true);
    viewToggle->setChecked(true);
    viewToggle->setFocusPolicy(Qt::NoFocus);
    viewToggle->setToolTip(
        "显示/隐藏方位-俯仰平铺视图（横轴 240→120，纵轴 +45→−45）");
    viewToggle->setStyleSheet(
        "QPushButton { background-color: #000000; }"
        "QPushButton:checked { background-color: #0b3d20;"
        " border-color: #00ff66; }");
    viewBar->addWidget(viewToggle);
    connect(viewToggle, &QPushButton::toggled, mFlat, &QWidget::setVisible);

    // Recording and replay have no toolbar row; they are driven purely by
    // --record / --replay / --speed / --loop on the command line.

    auto *central = new QWidget(this);
    central->setPalette(blackPal);
    central->setAutoFillBackground(true);
    auto *vbox = new QVBoxLayout(central);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->setSpacing(0);
    auto *split = new QSplitter(Qt::Horizontal, central);
    split->setChildrenCollapsible(false);
    split->setPalette(blackPal);
    split->setAutoFillBackground(true);
    split->setStyleSheet(
        "QSplitter { background-color: #000000; }"
        "QSplitter::handle { background-color: #0b3d20; }");
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

    if (!cfg.replayPath.isEmpty()) {
        setupReplay();
        if (!mReplay) {
            startLiveSource();   // load failed: fall back to the live path
        }
    } else {
        startLiveSource();
    }

    if (!cfg.recordPath.isEmpty()) {
        if (mState == PlayState::Replay) {
            std::fprintf(stderr,
                         "[record] ignored: --replay and --record are exclusive\n");
        } else {
            startRecording();
        }
    }
    if (mReplay) {
        mReplay->setLoop(cfg.loop);
        mReplay->setSpeed(cfg.speed);
        mReplay->play();
    }
}

MainWindow::~MainWindow() {
    if (mReplay) {
        mReplay->pause();
    }
    mRecorder.close();
    if (mFcProcess && mFcProcess->state() != QProcess::NotRunning) {
        mFcProcess->terminate();
        if (!mFcProcess->waitForFinished(1000)) {
            mFcProcess->kill();
            mFcProcess->waitForFinished(1000);
        }
    }
}

void MainWindow::startLiveSource() {
    if (mStream || mDemoTimer) {
        return;
    }
    if (mDemoMode) {
        mDemoTimer = new QTimer(this);
        connect(mDemoTimer, &QTimer::timeout, this, &MainWindow::onDemoTick);
        mDemoTimer->start(80);
        return;
    }
    mStream = new SideStream(mCfg.host, mCfg.port, float(mCfg.offsetDeg),
                             mCfg.minAmp, mCfg.minRangeM, this);
    connect(mStream, &SideStream::frameReady, this,
            &MainWindow::onFrameReady);
    mStream->start();
}

void MainWindow::setupReplay() {
    auto *src = new ReplaySource(mMeta, this);
    QString err;
    if (!src->load(mCfg.replayPath, &err)) {
        std::fprintf(stderr, "[replay] %s — falling back to live source\n",
                     qPrintable(err));
        delete src;
        return;
    }
    mReplay = src;
    mState = PlayState::Replay;
    mMeta.source = QStringLiteral("replay");
    connect(mReplay, &ReplaySource::frameReady, this,
            &MainWindow::onReplayFrame);
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

// Single display entry point: live UDP and file replay both reach the PPI
// and the unrolled view through here, with identical frame objects.
void MainWindow::applyFrame(const SideFrame &fr) {
    mHud.token = int(fr.frameToken);
    mHud.entries = int(fr.entryCount);
    mHud.blips = int(fr.points.size());
    mHud.sumOk = fr.sumOk;
    mHud.rpm = fr.rpm;
    mHud.frameIndex = fr.frameIndex;
    mHud.dtMs = fr.dtMs;
    mHud.source = mState == PlayState::Replay ? QStringLiteral("回放")
                  : mDemoMode ? QStringLiteral("演示")
                              : QStringLiteral("00F0/UDP");
    mHud.packetLossPct = mStream ? mStream->packetLossPct() : 0.0;
    mCursorFrameIndex = fr.frameIndex;
    mPpi->setFrame(fr);
    mFlat->setFrame(fr);
}

void MainWindow::onFrameReady(const SideFrame &fr) {
    // Every decoded frame is buffered and recorded; none is dropped.
    mStore.append(fr);
    if (mRecording) {
        mRecorder.write(fr);
    }
    ++mFrames;
    mFrameTimes.push_back(mClock.elapsed());
    if (mState == PlayState::Live) {
        mCursor = mStore.size() - 1;
        applyFrame(fr);
    }
}

void MainWindow::onReplayFrame(const SideFrame &fr) {
    if (mState != PlayState::Replay) {
        return;
    }
    ++mFrames;
    mFrameTimes.push_back(mClock.elapsed());
    applyFrame(fr);
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
    // Two-state status: a frame within the stale window means the viewer is
    // working, otherwise it is idle. Independent of the FPS estimate so a
    // very low frame rate does not flicker the status.
    mHud.working = !mFrameTimes.empty()
                   && now - mFrameTimes.back() <= kFpsStaleMs;
    mHud.demo = mDemoMode;
    mHud.fps = std::round(fps * 10.0) / 10.0;
    mHud.frames = mFrames;
    mHud.rangeM = mPpi->rangeLimit();
    mHud.pointRadius = mPpi->pointRadius();
    mHud.recording = mRecording;
    mHud.buffered = mStore.size();
    mHud.mode = mState == PlayState::Live ? QStringLiteral("实时")
                                          : QStringLiteral("回放");
    mFlat->setPointRadius(std::max(2.0, mPpi->pointRadius() * 3.0));
    mPpi->setHud(mHud);
}

void MainWindow::startRecording() {
    if (mRecording) {
        return;
    }
    QString path = mCfg.recordPath;
    if (path.isEmpty()) {
        path = QDir::current().filePath(
            QStringLiteral("radar2d_")
            + QDateTime::currentDateTime()
                  .toString(QStringLiteral("yyyyMMdd_HHmmss"))
            + QStringLiteral(".jsonl"));
    }
    QString err;
    if (!mRecorder.open(path, mMeta, &err)) {
        std::fprintf(stderr, "[record] %s\n", qPrintable(err));
        return;
    }
    mRecording = true;
    std::fprintf(stderr,
                 "[record] writing %s.part (renamed to %s on clean exit)\n",
                 qPrintable(path), qPrintable(path));
}

void MainWindow::onDemoTick() {
    ++mDemoN;
    const double phase = mDemoN * 0.02;
    QVector<RawPoint> raw;
    raw.reserve(241);
    for (int i = -120; i <= 120; ++i) {
        const double az = i * 1.0;
        const double r = 2.5 + 21.0 * (i + 120) / 240.0
                         + 1.2 * std::sin(3.0 * az * kPi / 180.0 + phase);
        const double el = 32.0 * std::sin(2.0 * az * kPi / 180.0 + phase);
        const double snr = 60.0 + 90.0
                           * std::fabs(std::sin(2.0 * az * kPi / 180.0));
        const double elRad = el * kPi / 180.0;
        double azRad = std::fmod((az - 180.0) * kPi / 180.0, 2.0 * kPi);
        if (azRad < 0.0) {
            azRad += 2.0 * kPi;
        }
        // Synthesise wire words whose re-projection yields this demo point.
        RawPoint rp;
        rp.azRaw = quint16(std::lround(azRad * 1000.0));
        rp.elRaw = quint16(std::lround((elRad + 2.0 * kPi) * 1000.0));
        rp.rRaw = quint16(std::lround(r / std::cos(elRad) * 1000.0));
        rp.c3 = quint16(std::lround(snr + mCfg.minAmp));
        rp.c4 = 0;
        rp.c5Us = quint16(mDemoN * 1000);
        raw.push_back(rp);
    }

    SideFrame fr;
    fr.valid = true;
    fr.frameToken = uint32_t(mDemoN);
    fr.rpm = 1200.0;
    fr.sumOk = true;
    fr.rawPoints = raw;
    fr.entryCount = uint16_t(raw.size());
    F0::projectRaw(raw, fr, float(mOffsetDeg), mCfg.minAmp, mCfg.minRangeM);

    const qint64 nowUs = mClock.nsecsElapsed() / 1000;
    fr.frameIndex = mDemoIndex++;
    fr.recvMonoUs = nowUs;
    fr.tsWallMs = QDateTime::currentMSecsSinceEpoch();
    fr.dtMs = mDemoLastUs > 0 ? double(nowUs - mDemoLastUs) / 1000.0 : 0.0;
    fr.revUs = 60e6 / fr.rpm;
    mDemoLastUs = nowUs;
    onFrameReady(fr);
}
