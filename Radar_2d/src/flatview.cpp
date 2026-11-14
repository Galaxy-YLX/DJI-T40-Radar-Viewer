#include "flatview.h"

#include <QColor>
#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QString>
#include <algorithm>
#include <cmath>

namespace {
// The unrolled forward sector: 240 -> 360/000 -> 120, i.e. +/-120 deg.
constexpr double kAzStartDeg = 240.0;
constexpr double kAzSpanDeg = 240.0;
constexpr double kElTopDeg = 45.0;
constexpr double kElBottomDeg = -45.0;
constexpr double kElSpanDeg = kElTopDeg - kElBottomDeg;   // 90 deg
constexpr int kBins = 12;              // discrete distance colour steps

constexpr double kLeft = 58.0;
constexpr double kRight = 26.0;    // room for the rightmost "120" label
constexpr double kTop = 46.0;
constexpr double kBottom = 90.0;   // caption + colour bar + distance ticks

// Black phosphor canvas, matching the PPI. The far end of the requested
// colour scale is black as well, so those points simply fade into the canvas.
const QColor kBg(0, 0, 0);
const QColor kGridMinor(0, 58, 30);
const QColor kGridMajor(0, 100, 52);
const QColor kGridZero(0, 160, 82);
const QColor kFrame(0, 120, 62);
const QColor kText(0, 255, 102);
const QColor kTextDim(0, 175, 88);

const int kAzLabels[] = {240, 270, 300, 330, 0, 30, 60, 90, 120};
constexpr int kAzLabelCount = 9;
const int kElLabels[] = {45, 30, 15, 0, -15, -30, -45};

QString azLabel(int deg) {
    return QString::number(deg).rightJustified(3, QLatin1Char('0'));
}

QString elLabel(int deg) {
    return deg > 0 ? QStringLiteral("+") + QString::number(deg)
                   : QString::number(deg);
}
}  // namespace

FlatView::FlatView(QWidget *parent) : QWidget(parent) {
    setMinimumSize(520, 320);
}

void FlatView::setFrame(const SideFrame &fr) {
    mPoints = fr.points;
    update();
}

void FlatView::setRangeLimit(double m) {
    mRangeLimit = std::clamp(m, 5.0, 50.0);
    update();
}

void FlatView::setPointRadius(double px) {
    const double r = std::clamp(px, 1.0, 12.0);
    if (r == mPointRadius) {
        return;   // refreshed from the HUD timer: skip needless repaints
    }
    mPointRadius = r;
    update();
}

QColor FlatView::rangeColor(double t) {
    t = std::clamp(t, 0.0, 1.0);
    const QColor red(226, 48, 42);
    const QColor yellow(242, 205, 38);
    const QColor green(38, 190, 96);
    const QColor black(10, 10, 10);
    // Yellow sits halfway between red and green, so green still marks the
    // middle of the range limit (0.5) and black the far end (1.0).
    QColor a, b;
    double u;
    if (t <= 0.25) {
        a = red;    b = yellow; u = t / 0.25;
    } else if (t <= 0.5) {
        a = yellow; b = green;  u = (t - 0.25) / 0.25;
    } else {
        a = green;  b = black;  u = (t - 0.5) / 0.5;
    }
    auto lerp = [u](int from, int to) {
        return int(from + (to - from) * u + 0.5);
    };
    return QColor(lerp(a.red(), b.red()),
                  lerp(a.green(), b.green()),
                  lerp(a.blue(), b.blue()));
}

double FlatView::unrolledAz(double azDeg) {
    double a = std::fmod(azDeg - kAzStartDeg, 360.0);
    if (a < 0.0) {
        a += 360.0;
    }
    return a;   // 0..240 is the forward sector; >240 is the rear blind wedge
}

void FlatView::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.fillRect(rect(), kBg);
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRectF plot(kLeft, kTop,
                      width() - kLeft - kRight,
                      height() - kTop - kBottom);
    if (plot.width() < 40.0 || plot.height() < 40.0) {
        return;
    }
    drawGrid(p, plot);
    drawPoints(p, plot);
    drawAzimuthScale(p, plot);
    drawElevationScale(p, plot);
    drawLegend(p, plot);
}

void FlatView::drawGrid(QPainter &p, const QRectF &plot) const {
    p.save();
    for (int a = 0; a <= int(kAzSpanDeg); a += 10) {
        const double x = plot.left() + a / kAzSpanDeg * plot.width();
        const bool zero = (a == 120);   // 000 deg, straight ahead
        p.setPen(QPen(zero ? kGridZero
                           : (a % 30 == 0 ? kGridMajor : kGridMinor),
                      zero ? 1.6 : 1.0));
        p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    }
    for (int e = -45; e <= 45; e += 5) {
        const double y = plot.top()
                         + (kElTopDeg - e) / kElSpanDeg * plot.height();
        const bool zero = (e == 0);     // horizon
        p.setPen(QPen(zero ? kGridZero
                           : (e % 15 == 0 ? kGridMajor : kGridMinor),
                      zero ? 1.6 : 1.0));
        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }
    p.setPen(QPen(kFrame, 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawRect(plot);
    p.restore();
}

void FlatView::drawPoints(QPainter &p, const QRectF &plot) {
    mShown = 0;
    mOutOfFov = 0;
    p.save();
    p.setClipRect(plot);
    p.setPen(Qt::NoPen);
    for (const SectorPoint &pt : mPoints) {
        if (!std::isfinite(pt.elevationDeg) || !std::isfinite(pt.azimuthDeg)) {
            continue;
        }
        if (pt.elevationDeg > kElTopDeg || pt.elevationDeg < kElBottomDeg) {
            ++mOutOfFov;   // outside the +/-45 deg elevation window
            continue;
        }
        const double a = unrolledAz(pt.azimuthDeg);
        if (a < 0.0 || a > kAzSpanDeg) {
            continue;      // rear no-data sector
        }
        const double x = plot.left() + a / kAzSpanDeg * plot.width();
        const double y = plot.top()
                         + (kElTopDeg - pt.elevationDeg) / kElSpanDeg
                         * plot.height();
        const double t = std::isfinite(pt.rangeM)
            ? std::clamp(double(pt.rangeM) / mRangeLimit, 0.0, 1.0)
            : 1.0;
        // Snap to one of kBins discrete distance steps ("分度").
        const int bin = std::min(kBins - 1, int(t * kBins));
        p.setBrush(rangeColor((bin + 0.5) / kBins));
        p.drawEllipse(QPointF(x, y), mPointRadius, mPointRadius);
        ++mShown;
    }
    p.restore();
}

void FlatView::drawAzimuthScale(QPainter &p, const QRectF &plot) const {
    QFont small = font();
    small.setPointSizeF(8.0);
    for (int i = 0; i < kAzLabelCount; ++i) {
        const double a = i * 30.0;
        const double x = plot.left() + a / kAzSpanDeg * plot.width();
        const bool centre = (kAzLabels[i] == 0);
        p.setPen(QPen(centre ? kGridZero : kGridMajor, 1.0));
        p.drawLine(QPointF(x, plot.top() - 6.0), QPointF(x, plot.top()));
        QFont labelFont = small;
        labelFont.setBold(centre);
        labelFont.setPointSizeF(centre ? 9.5 : 8.5);
        p.setFont(labelFont);
        p.setPen(kText);
        p.drawText(QRectF(x - 22.0, plot.top() - 26.0, 44.0, 18.0),
                   Qt::AlignCenter, azLabel(kAzLabels[i]));
    }
}

void FlatView::drawElevationScale(QPainter &p, const QRectF &plot) const {
    QFont small = font();
    small.setPointSizeF(8.0);
    p.setFont(small);
    for (int e : kElLabels) {
        const double y = plot.top()
                         + (kElTopDeg - e) / kElSpanDeg * plot.height();
        p.setPen(QPen(e == 0 ? kGridZero : kGridMajor, 1.0));
        p.drawLine(QPointF(plot.left() - 5.0, y), QPointF(plot.left(), y));
        p.setPen(e == 0 ? kText : kTextDim);
        p.drawText(QRectF(2.0, y - 9.0, plot.left() - 10.0, 18.0),
                   Qt::AlignRight | Qt::AlignVCenter, elLabel(e));
    }
}

void FlatView::drawLegend(QPainter &p, const QRectF &plot) const {
    QFont small = font();
    small.setPointSizeF(7.5);
    p.setFont(small);
    p.setPen(kTextDim);
    p.drawText(QPointF(plot.left(), plot.bottom() + 16.0),
               QString::fromUtf8("方位 × 俯仰 平铺   |   横轴 方位角(°) 000=正前"
                                 "   |   纵轴 俯仰角(°) +45 → −45"));

    const double barW = std::min(320.0, plot.width() * 0.62);
    constexpr double barH = 12.0;
    const double barX = plot.left();
    const double barY = plot.bottom() + 34.0;
    for (int i = 0; i < kBins; ++i) {
        p.fillRect(QRectF(barX + i * barW / kBins, barY,
                          barW / kBins + 1.0, barH),
                   rangeColor((i + 0.5) / kBins));
    }
    // Let the dark end of the bar fade into the canvas; only a dim outer
    // frame marks where the scale starts and ends.
    p.setPen(QPen(kFrame, 1.0));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(barX, barY, barW, barH));

    p.setPen(kText);
    p.drawText(QRectF(barX, barY + barH + 1.0, 60.0, 15.0),
               Qt::AlignLeft | Qt::AlignTop, QStringLiteral("0 m"));
    p.drawText(QRectF(barX + barW / 2.0 - 30.0, barY + barH + 1.0,
                      60.0, 15.0),
               Qt::AlignCenter | Qt::AlignTop,
               QString::number(mRangeLimit / 2.0, 'f', 1) + " m");
    p.drawText(QRectF(barX + barW - 60.0, barY + barH + 1.0, 60.0, 15.0),
               Qt::AlignRight | Qt::AlignTop,
               QString::number(mRangeLimit, 'f', 1) + " m");

    const double infoX = barX + barW + 18.0;
    p.setPen(kTextDim);
    p.drawText(QPointF(infoX, barY + 3.0),
               QString::fromUtf8("距离分度：红 近 → 黄 → 绿 中 → 黑 远/超量程"));
    p.drawText(QPointF(infoX, barY + 20.0),
               QString::fromUtf8("点数 %1   俯仰越界 %2")
                   .arg(mShown).arg(mOutOfFov));
}
