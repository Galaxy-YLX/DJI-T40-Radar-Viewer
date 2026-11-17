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
constexpr double kBottom = 90.0;   // colour bar + distance ticks

// Black phosphor canvas, matching the PPI. The colour scale ends at green,
// so even the farthest points stay visible.
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

QRectF legendBarRect(const QRectF &plot) {
    const double barW = std::min(320.0, plot.width() * 0.62);
    constexpr double barH = 12.0;
    return QRectF(plot.left(), plot.bottom() + 34.0, barW, barH);
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

void FlatView::setFlipLeftRight(bool flip) {
    if (flip == mFlipLeftRight) {
        return;
    }
    mFlipLeftRight = flip;
    update();
}

QColor FlatView::rangeColor(double t) {
    t = std::clamp(t, 0.0, 1.0);
    const QColor red(226, 48, 42);
    const QColor yellow(242, 205, 38);
    const QColor green(38, 190, 96);
    // Near = red, half range = yellow, far = green. The scale no longer has
    // a dark end, so even the farthest points stay visible on the black canvas.
    const bool nearHalf = t <= 0.5;
    const QColor &a = nearHalf ? red : yellow;
    const QColor &b = nearHalf ? yellow : green;
    const double u = nearHalf ? t / 0.5 : (t - 0.5) / 0.5;
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

bool FlatView::backgroundCacheValid() const {
    return !mBackgroundCache.isNull()
        && mCachedWidgetSize == size()
        && mCachedRangeLimit == mRangeLimit
        && mCachedDevicePixelRatio == devicePixelRatioF()
        && mCachedFontValid
        && mCachedFont == font();
}

void FlatView::rebuildBackgroundCache() {
    const QSize sz = size();
    if (sz.isEmpty()) {
        mBackgroundCache = QPixmap();
        return;
    }

    const qreal dpr = devicePixelRatioF();
    QPixmap cache(QSizeF(sz.width() * dpr, sz.height() * dpr).toSize());
    cache.setDevicePixelRatio(dpr);
    cache.fill(kBg);

    QPainter p(&cache);
    p.scale(dpr, dpr);
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRectF plot(kLeft, kTop,
                      sz.width() - kLeft - kRight,
                      sz.height() - kTop - kBottom);
    if (plot.width() >= 40.0 && plot.height() >= 40.0) {
        drawGrid(p, plot);
        drawAzimuthScale(p, plot);
        drawElevationScale(p, plot);
        drawLegendStatic(p, plot);
    }

    p.end();
    mBackgroundCache = cache;
    mCachedWidgetSize = sz;
    mCachedRangeLimit = mRangeLimit;
    mCachedDevicePixelRatio = dpr;
    mCachedFont = font();
    mCachedFontValid = true;
}

void FlatView::paintEvent(QPaintEvent *) {
    if (!backgroundCacheValid()) {
        rebuildBackgroundCache();
    }

    QPainter p(this);
    if (mBackgroundCache.isNull()) {
        p.fillRect(rect(), kBg);
    } else {
        p.drawPixmap(0, 0, mBackgroundCache);
    }

    const QRectF plot(kLeft, kTop,
                      width() - kLeft - kRight,
                      height() - kTop - kBottom);
    if (plot.width() < 40.0 || plot.height() < 40.0) {
        return;
    }
    p.setRenderHint(QPainter::Antialiasing, true);
    drawPoints(p, plot);
    drawLegendCounts(p, plot);
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
        double a = unrolledAz(pt.azimuthDeg);
        if (a < 0.0 || a > kAzSpanDeg) {
            continue;      // rear no-data sector
        }
        // The 240 -> 120 scale stays in place; mirror the blips alone when
        // the PPI mirrors its returns, so a target lands on the same side
        // of both views (about the centre 000 line).
        if (mFlipLeftRight) {
            a = kAzSpanDeg - a;
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

void FlatView::drawLegendStatic(QPainter &p, const QRectF &plot) const {
    QFont small = font();
    small.setPointSizeF(7.5);
    p.setFont(small);

    const QRectF bar = legendBarRect(plot);
    for (int i = 0; i < kBins; ++i) {
        p.fillRect(QRectF(bar.left() + i * bar.width() / kBins, bar.top(),
                          bar.width() / kBins + 1.0, bar.height()),
                   rangeColor((i + 0.5) / kBins));
    }
    // A dim outer frame marks where the scale starts and ends.
    p.setPen(QPen(kFrame, 1.0));
    p.setBrush(Qt::NoBrush);
    p.drawRect(bar);

    p.setPen(kText);
    p.drawText(QRectF(bar.left(), bar.bottom() + 1.0, 60.0, 15.0),
               Qt::AlignLeft | Qt::AlignTop, QStringLiteral("0 m"));
    p.drawText(QRectF(bar.left() + bar.width() / 2.0 - 30.0,
                      bar.bottom() + 1.0, 60.0, 15.0),
               Qt::AlignCenter | Qt::AlignTop,
               QString::number(mRangeLimit / 2.0, 'f', 1) + " m");
    p.drawText(QRectF(bar.right() - 60.0, bar.bottom() + 1.0,
                      60.0, 15.0),
               Qt::AlignRight | Qt::AlignTop,
               QString::number(mRangeLimit, 'f', 1) + " m");
}

void FlatView::drawLegendCounts(QPainter &p, const QRectF &plot) const {
    QFont small = font();
    small.setPointSizeF(7.5);
    p.setFont(small);

    const QRectF bar = legendBarRect(plot);
    const double infoX = bar.right() + 18.0;
    p.setPen(kTextDim);
    // Share the colour scale's row, vertically centred against the bar.
    p.drawText(QRectF(infoX, bar.top(), 260.0, bar.height()),
               Qt::AlignLeft | Qt::AlignVCenter,
               QString::fromUtf8("点数 %1   俯仰越界 %2")
                   .arg(mShown).arg(mOutOfFov));
}
