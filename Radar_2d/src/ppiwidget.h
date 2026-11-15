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
    int entries = 0;        // raw records in the latest frame
    double rangeM = 0.0;
    double pointRadius = 0.0;
    double packetLossPct = 0.0;   // packet loss, %% since stream start
    QString source;         // point-cloud source (00F0/UDP / 演示)
    bool sumOk = true;      // 0x00F0 header checksums passed
    double rpm = 0.0;       // motor rpm inferred from c5 (0 = unknown)
    quint32 frameIndex = 0; // local monotonic frame number
    double dtMs = 0.0;      // measured inter-frame interval
    int buffered = 0;       // frames held in the ring buffer
    bool recording = false; // JSONL recorder active
    QString mode;           // 实时 / 暂停 / 回放

    bool operator==(const HudState &o) const {
        return live == o.live && demo == o.demo && fps == o.fps &&
               frames == o.frames && blips == o.blips && token == o.token &&
               entries == o.entries &&
               rangeM == o.rangeM && pointRadius == o.pointRadius &&
               packetLossPct == o.packetLossPct && source == o.source &&
               sumOk == o.sumOk && rpm == o.rpm &&
               frameIndex == o.frameIndex && dtMs == o.dtMs &&
               buffered == o.buffered && recording == o.recording &&
               mode == o.mode;
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
    void setHudAnchor(QWidget *anchor);

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
    QWidget *mHudAnchor = nullptr;  // info card left edge follows this widget
    double mRangeLimit = 25.0;
    double mPointRadius = 0.5;
    bool mFlipLeftRight = false;
    // Rear no-data wedge; the PPI shows +/-120 deg around forward.
    double mBlindStartDeg = kBlindStartDeg;
    double mBlindEndDeg = kBlindEndDeg;
};
