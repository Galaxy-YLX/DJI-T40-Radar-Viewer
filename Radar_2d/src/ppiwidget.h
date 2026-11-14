#pragma once

#include <QColor>
#include <QString>
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
    int subtype = 0;        // 1E61 subtype: 0=ground 1=wire 2=pcla
    int entries = 0;        // raw records in the latest frame
    double rangeM = 0.0;
    double pointRadius = 0.0;
    double packetLossPct = 0.0;   // packet loss, %% since stream start
    QString source;         // point-cloud source (1E61 / 00F0 ...)
    bool linkUp = true;     // TCP link to the board is connected
    bool sumOk = true;      // 0x00F0 header checksums passed
    double rpm = 0.0;       // motor rpm inferred from c5 (0 = unknown)

    bool operator==(const HudState &o) const {
        return live == o.live && demo == o.demo && fps == o.fps &&
               frames == o.frames && blips == o.blips && token == o.token &&
               subtype == o.subtype && entries == o.entries &&
               rangeM == o.rangeM && pointRadius == o.pointRadius &&
               packetLossPct == o.packetLossPct && source == o.source &&
               linkUp == o.linkUp && sumOk == o.sumOk && rpm == o.rpm;
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
    bool flipLeftRight() const { return mFlipLeftRight; }
    void setRangeLimit(double m);
    void setPointRadius(double px);
    void setFlipLeftRight(bool flip);

signals:
    void rangeChanged(double m);

protected:
    void paintEvent(QPaintEvent *ev) override;
    void keyPressEvent(QKeyEvent *ev) override;

private:
    static QColor snrColor(float snr);
    void drawHud(QPainter &p);
    bool inBlind(double azDeg) const;
    double arcStartAngle() const;  // Qt arc angles, 1/16 degree
    double arcSpanAngle() const;

    QVector<SectorPoint> mPoints;
    HudState mHud;
    double mRangeLimit = 25.0;
    double mPointRadius = 0.5;
    bool mFlipLeftRight = false;
    // No-data wedge described by the current point-cloud source. 1E61 has no
    // rear coverage (120..240); 0x00F0 has no forward coverage (~298..59).
    double mBlindStartDeg = 120.0;
    double mBlindEndDeg = 240.0;
};
