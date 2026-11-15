#pragma once

#include <QColor>
#include <QVector>
#include <QWidget>

#include "sidestream.h"

class QPainter;
class QPaintEvent;

// Azimuth/elevation "unrolled" view: the +/-120 deg forward sector is laid
// out left -> right (240, 270, 300, 330, 000, 030, 060, 090, 120) and
// elevation runs +45 deg (top) -> -45 deg (bottom). Distance is encoded as
// a discrete red -> yellow -> green colour scale instead of a position.
class FlatView : public QWidget {
    Q_OBJECT
public:
    explicit FlatView(QWidget *parent = nullptr);

    void setFrame(const SideFrame &fr);
    void setRangeLimit(double m);
    void setPointRadius(double px);
    void setFlipLeftRight(bool flip);   // mirror blips only, keep the scale
    double rangeLimit() const { return mRangeLimit; }

protected:
    void paintEvent(QPaintEvent *ev) override;

private:
    static QColor rangeColor(double t);      // 0 = near, 1 = far
    static double unrolledAz(double azDeg);  // 240..360/0..120 -> 0..240
    void drawGrid(QPainter &p, const QRectF &plot) const;
    void drawPoints(QPainter &p, const QRectF &plot);
    void drawAzimuthScale(QPainter &p, const QRectF &plot) const;
    void drawElevationScale(QPainter &p, const QRectF &plot) const;
    void drawLegend(QPainter &p, const QRectF &plot) const;

    QVector<SectorPoint> mPoints;
    double mRangeLimit = 25.0;
    double mPointRadius = 2.2;
    bool mFlipLeftRight = false;
    int mShown = 0;
    int mOutOfFov = 0;
};
