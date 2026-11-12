#pragma once

#include <QWidget>
#include <QVector>

#include "sidestream.h"

// Plan-position-indicator: azimuth around the circle, range along the
// radius. Paints range rings, bearing lines, the forward marker, and the
// latest blips coloured by SNR.
class PpiWidget : public QWidget {
    Q_OBJECT
public:
    explicit PpiWidget(QWidget *parent = nullptr);

    void setFrame(const SideFrame &fr);
    void setStatusText(const QString &text);

    double rangeLimit() const { return mRangeLimit; }
    double pointRadius() const { return mPointRadius; }
    void setRangeLimit(double m);
    void setPointRadius(double px);

signals:
    void rangeChanged(double m);

protected:
    void paintEvent(QPaintEvent *ev) override;
    void wheelEvent(QWheelEvent *ev) override;
    void keyPressEvent(QKeyEvent *ev) override;

private:
    static QColor snrColor(float snr);

    QVector<SectorPoint> mPoints;
    QString mStatus;
    double mRangeLimit = 25.0;
    double mPointRadius = 2.0;
};
