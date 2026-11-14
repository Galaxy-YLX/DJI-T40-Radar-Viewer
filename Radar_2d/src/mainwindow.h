#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QString>
#include <deque>

#include "config.h"
#include "ppiwidget.h"

class SideStream;
class LinkStream;
class QTimer;
class QLabel;
class QProcess;
class QPushButton;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(const Config &cfg, QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onFrameReady();
    void onDemoTick();
    void refreshHud();

private:
    void applyFrame(const SideFrame &fr);
    void syncRangeDisplay(double m);
    void startRadar();
    void stopRadar();
    void setPowerRunning(bool running);

    PpiWidget *mPpi;
    SideStream *mStream = nullptr;
    LinkStream *mLink = nullptr;
    QTimer *mDemoTimer = nullptr;
    QTimer *mHudTimer = nullptr;
    QProcess *mFcProcess = nullptr;
    QLabel *mRangeValue = nullptr;
    QPushButton *mRangePlus = nullptr;
    QPushButton *mRangeMinus = nullptr;
    QPushButton *mPowerOn = nullptr;
    QPushButton *mPowerOff = nullptr;
    QString mRadarIp;
    QString mCanIface;
    QString mFcClientPath;
    bool mDemoMode = false;
    QElapsedTimer mClock;
    std::deque<qint64> mFrameTimes;
    HudState mHud;
    int mFrames = 0;
    int mDemoN = 0;
    int mRpm = 1200;
    double mOffsetDeg;
};
