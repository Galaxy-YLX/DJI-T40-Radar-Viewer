#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QString>
#include <deque>

#include "config.h"
#include "frameio.h"
#include "ppiwidget.h"

class SideStream;
class FlatView;
class ReplaySource;
class QTimer;
class QLabel;
class QProcess;
class QPushButton;
class QUdpSocket;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(const Config &cfg, QWidget *parent = nullptr);
    ~MainWindow() override;

private:
    enum class PlayState { Live, Replay };

private slots:
    void onFrameReady(const SideFrame &fr);
    void onReplayFrame(const SideFrame &fr);
    void onDemoTick();
    void refreshHud();

private:
    void applyFrame(const SideFrame &fr);
    void refreshFrameDisplay();
    void startLiveSource();
    void setupReplay();
    void startRecording();
    void syncRangeDisplay(double m);
    void setSpeedGate(double mps);
    void syncSpeedGateDisplay(double mps);
    void setMinAmp(int n);
    void syncMinAmpDisplay(int n);
    void setMotorRpm(int rpm);
    void syncMotorRpmDisplay(int rpm);
    void sendMotorRpm();
    void startRadar();
    void stopRadar();
    void setPowerRunning(bool running);

    PpiWidget *mPpi;
    FlatView *mFlat = nullptr;
    SideStream *mStream = nullptr;
    ReplaySource *mReplay = nullptr;
    QTimer *mDemoTimer = nullptr;
    QTimer *mHudTimer = nullptr;
    QProcess *mFcProcess = nullptr;
    QLabel *mRangeValue = nullptr;
    QPushButton *mRangePlus = nullptr;
    QPushButton *mRangeMinus = nullptr;
    QLabel *mSpeedGateValue = nullptr;
    QPushButton *mSpeedGatePlus = nullptr;
    QPushButton *mSpeedGateMinus = nullptr;
    QLabel *mMinAmpValue = nullptr;
    QPushButton *mMinAmpPlus = nullptr;
    QPushButton *mMinAmpMinus = nullptr;
    QLabel *mMotorRpmValue = nullptr;
    QPushButton *mMotorRpmPlus = nullptr;
    QPushButton *mMotorRpmMinus = nullptr;
    QPushButton *mPowerOn = nullptr;
    QPushButton *mPowerOff = nullptr;

    Config mCfg;
    FrameMeta mMeta;
    FrameStore mStore;
    FrameRecorder mRecorder;
    PlayState mState = PlayState::Live;
    int mCursor = -1;                 // cursor into mStore (latest frame)
    quint32 mCursorFrameIndex = 0;    // for the phase-1 detection store
    bool mRecording = false;

    CrossFrameVelocity mVelocity;
    SideFrame mLastFrame;      // latest frame with per-point speeds, before gating
    QString mRadarIp;
    QString mCanIface;
    QString mFcClientPath;
    bool mDemoMode = false;
    QElapsedTimer mClock;
    std::deque<qint64> mFrameTimes;
    HudState mHud;
    int mFrames = 0;
    int mDemoN = 0;
    quint32 mDemoIndex = 0;
    qint64 mDemoLastUs = 0;
    double mOffsetDeg;
    double mSpeedGateMps = 0.0;
    int mMinAmp = 0;
    QUdpSocket *mMotorControl = nullptr;
    int mMotorRpm = 900;
};
