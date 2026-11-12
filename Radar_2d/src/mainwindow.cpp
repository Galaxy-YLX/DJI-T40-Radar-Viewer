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
