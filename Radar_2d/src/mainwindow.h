#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QString>
#include <deque>

#include "config.h"
#include "ppiwidget.h"

class SideStream;
class QTimer;
class QLabel;
class QPushButton;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(const Config &cfg, QWidget *parent = nullptr);

private slots:
    void onFrameReady();
    void onDemoTick();
    void refreshHud();

private:
    void applyFrame(const SideFrame &fr);
    void syncRangeDisplay(double m);

    PpiWidget *mPpi;
    SideStream *mStream = nullptr;
    QTimer *mDemoTimer = nullptr;
    QTimer *mHudTimer = nullptr;
    QLabel *mRangeValue = nullptr;
    QPushButton *mRangePlus = nullptr;
    QPushButton *mRangeMinus = nullptr;
    bool mDemoMode = false;
    QElapsedTimer mClock;
    std::deque<qint64> mFrameTimes;
    HudState mHud;
    int mFrames = 0;
    int mDemoN = 0;
    double mOffsetDeg;
};
