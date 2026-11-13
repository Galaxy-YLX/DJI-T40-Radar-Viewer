#pragma once

#include <QColor>
#include <QWidget>
#include <QVector>

#include "sidestream.h"

class QPainter;

// Everything shown in the top-left status card. Keeping it structured lets
// the widget align the fields and highlight abnormal values instead of
// painting one flat green string.
struct HudState {
    bool live = false;
    bool demo = false;
    double fps = 0.0;
    int frames = 0;
    int blips = 0;
    int token = 0;
    int sectors = 0;
    int rotA = 0, rotB = 0, rotC = 0;
    int cfarError = 0;
    double rangeM = 0.0;
    double pointRadius = 0.0;
    double packetLossPct = 0.0;   // UDP packet loss, %% since stream start

    bool operator==(const HudState &o) const {
        return live == o.live && demo == o.demo && fps == o.fps &&
               frames == o.frames && blips == o.blips && token == o.token &&
               sectors == o.sectors && rotA == o.rotA && rotB == o.rotB &&
               rotC == o.rotC && cfarError == o.cfarError &&
               rangeM == o.rangeM && pointRadius == o.pointRadius &&
               packetLossPct == o.packetLossPct;
    }
    bool operator!=(const HudState &o) const { return !(*this == o); }
};

// Plan-position-indicator: azimuth around the circle, range along the
// radius. Paints range rings, bearing lines, the forward marker, and the
// latest blips coloured by SNR.
class PpiWidget : public QWidget {
    Q_OBJECT
public:
    explicit PpiWidget(QWidget *parent = nullptr);

    void setFrame(const SideFrame &fr);
    void setHud(const HudState &hud);

    double rangeLimit() const { return mRangeLimit; }
    double pointRadius() const { return mPointRadius; }
    void setRangeLimit(double m);
    void setPointRadius(double px);

signals:
    void rangeChanged(double m);

protected:
    void paintEvent(QPaintEvent *ev) override;
    void keyPressEvent(QKeyEvent *ev) override;

private:
    static QColor snrColor(float snr);
    void drawHud(QPainter &p);

    QVector<SectorPoint> mPoints;
    HudState mHud;
    double mRangeLimit = 25.0;
    double mPointRadius = 0.5;
};
