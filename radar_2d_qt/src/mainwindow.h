#pragma once

#include <QElapsedTimer>
#include <QMainWindow>

#include "config.h"

class PpiWidget;
class SideStream;
struct SideFrame;
class QTimer;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(const Config &cfg, QWidget *parent = nullptr);

private slots:
    void onFrameReady();
    void onDemoTick();

private:
    void applyFrame(const SideFrame &fr, const QString &header);

    PpiWidget *mPpi;
    SideStream *mStream = nullptr;
    QTimer *mDemoTimer = nullptr;
    QElapsedTimer mClock;
    int mFrames = 0;
    int mDemoN = 0;
    double mOffsetDeg;
};
