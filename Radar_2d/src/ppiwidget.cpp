#include "ppiwidget.h"

#include "coords.h"

#include <QKeyEvent>
#include <QPainter>
#include <QPaintEvent>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
constexpr double kRangeStepM = 5.0;   // one key press = one toolbar step

// Classic monochrome radar: pure black canvas, green phosphor only.
const QColor kGrid(0, 96, 48, 170);      // bearing lines
const QColor kRing(0, 120, 60, 150);     // range rings
const QColor kScale(0, 205, 105);        // ring / bearing labels
const QColor kTick(0, 150, 80, 190);     // minor azimuth ticks
const QColor kPhosphor(0, 255, 102);     // primary text, strong returns
const QColor kPhosphorDim(0, 185, 88);   // labels, secondary text
}  // namespace

PpiWidget::PpiWidget(QWidget *parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(420, 420);
}

void PpiWidget::setFrame(const SideFrame &fr) {
    mBlindStartDeg = fr.blindStartDeg;
    mBlindEndDeg = fr.blindEndDeg;
    // Clip blips to the +/-120 deg forward view (the demo frame is 360 deg).
    mPoints.clear();
    mPoints.reserve(fr.points.size());
    for (const SectorPoint &pt : fr.points) {
        if (!inBlind(pt.azimuthDeg)) {
            mPoints.push_back(pt);
        }
    }
    if (std::getenv("RADAR2D_PPI_DEBUG")) {
        std::fprintf(stderr,
                     "[ppi] pts=%d blind=%.1f..%.1f\n",
                     int(mPoints.size()), mBlindStartDeg, mBlindEndDeg);
    }
    update();
}

bool PpiWidget::inBlind(double azDeg) const {
    if (mBlindStartDeg < mBlindEndDeg) {
        return azDeg > mBlindStartDeg && azDeg < mBlindEndDeg;
    }
    // A wrapped wedge (start > end) straddles 0 degrees.
    return azDeg > mBlindStartDeg || azDeg < mBlindEndDeg;
}

double PpiWidget::arcStartAngle() const {
    // Screen azimuth 0 is up; Qt arcs start at 3 o'clock and count CCW.
    return (90.0 - mBlindEndDeg) * 16.0;
}

double PpiWidget::arcSpanAngle() const {
    const double covered = mBlindStartDeg - mBlindEndDeg
                         + (mBlindStartDeg < mBlindEndDeg ? 360.0 : 0.0);
    return -covered * 16.0;
}

void PpiWidget::setHud(const HudState &hud) {
    if (mHud == hud) {
        return;
    }
    mHud = hud;
    update();
}

void PpiWidget::setHudAnchor(QWidget *anchor) {
    mHudAnchor = anchor;
    update();
}

void PpiWidget::setRangeLimit(double m) {
    mRangeLimit = std::clamp(m, 5.0, 50.0);
    emit rangeChanged(mRangeLimit);
    update();
}

void PpiWidget::setPointRadius(double px) {
    mPointRadius = std::clamp(px, 0.5, 20.0);
    update();
}

void PpiWidget::setFlipLeftRight(bool flip) {
    mFlipLeftRight = flip;
    update();
}

QColor PpiWidget::snrColor(float snr) {
    // Weak returns are dim phosphor, strong returns approach full green.
    const float t = std::clamp(snr / 255.0f, 0.0f, 1.0f);
    const int g = int(70.0f + 185.0f * t + 0.5f);
    return QColor(0, g, int(g * 0.4f + 0.5f));
}

void PpiWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.fillRect(rect(), Qt::black);
    p.setRenderHint(QPainter::Antialiasing, true);

    const double w = width(), h = height();
    const double cx = w / 2.0, cy = h / 2.0;
    // Extra margin leaves room for the outer azimuth scale and its numerals.
    const double radiusPx = std::min(w, h) * 0.5 - 52.0;

    // Bearing lines (every 30 deg), skipping the source's no-data wedge.
    p.setPen(QPen(kGrid, 1));
    for (int a = 0; a < 360; a += 30) {
        if (inBlind(a)) {
            continue;
        }
        const Coord::ScreenPoint tip =
            Coord::ppiPolarToScreen(a, radiusPx, cx, cy, false);
        p.drawLine(QPointF(cx, cy), QPointF(tip.x, tip.y));
    }

    // Range rings + labels, all green phosphor. Rings span only the sector
    // covered by the current point-cloud source. The outermost boundary is
    // drawn once below with the azimuth scale, so it is never doubled up.
    const int startAngle = int(arcStartAngle());
    const int spanAngle = int(arcSpanAngle());
    QFont small = font();
    small.setPointSizeF(7.5);
    p.setFont(small);
    for (double d = 5.0; d <= mRangeLimit + 1e-6; d += 5.0) {
        const double rp = d / mRangeLimit * radiusPx;
        if (d < mRangeLimit - 1e-6) {
            p.setPen(QPen(kRing, 1, Qt::SolidLine));
            p.drawArc(QRectF(cx - rp, cy - rp, 2.0 * rp, 2.0 * rp),
                      startAngle, spanAngle);
        }
        p.setPen(kScale);
        // Lift the range numbers above the 090 radial, which now carries the
        // outer azimuth ticks and the 090 numeral.
        p.drawText(QPointF(cx + rp + 3.0, cy - 14.0),
                   QString::number(int(d)) + "m");
    }

    // Outermost boundary and compass scale share one circle: the bearing
    // radials end on it and the azimuth ticks continue outwards from it, so
    // radials and 30° ticks line up with no gap or doubled arc. Only the
    // source's covered sector is drawn; its no-data wedge stays blank.
    const QRectF scaleRect(cx - radiusPx, cy - radiusPx,
                           2.0 * radiusPx, 2.0 * radiusPx);
    p.setPen(QPen(kRing, 1, Qt::SolidLine));
    p.drawArc(scaleRect, startAngle, spanAngle);

    for (int a = 0; a < 360; a += 5) {
        if (inBlind(a)) {
            continue;
        }
        const bool major = (a % 30 == 0);
        const double len = major ? 9.0 : ((a % 10 == 0) ? 6.0 : 3.5);
        const Coord::ScreenPoint inner =
            Coord::ppiPolarToScreen(a, radiusPx, cx, cy, false);
        const Coord::ScreenPoint outer =
            Coord::ppiPolarToScreen(a, radiusPx + len, cx, cy, false);
        p.setPen(QPen(major ? kScale : kTick, 1.0));
        p.drawLine(QPointF(inner.x, inner.y), QPointF(outer.x, outer.y));
    }

    p.setPen(kScale);
    const double labelR = radiusPx + 28.0;
    for (int a = 0; a < 360; a += 30) {
        if (inBlind(a)) {
            continue;
        }
        const Coord::ScreenPoint label =
            Coord::ppiPolarToScreen(a, labelR, cx, cy, false);
        p.drawText(QRectF(label.x - 16.0, label.y - 8.0, 32.0, 16.0),
                   Qt::AlignCenter,
                   QString::number(a).rightJustified(3, QLatin1Char('0')));
    }

    // Returns: a soft phosphor halo plus a green core graded by SNR.
    // The shared polar->screen helper applies the left-right mirror only to
    // blips (and future overlays); the bearing scale above stays unmirrored.
    // Keep each blip (halo included) inside the outermost range ring. Points
    // farther than the current range limit used to be drawn outside the PPI.
    const double maxRadiusPx = radiusPx - mPointRadius * 2.0;
    int clipped = 0;
    for (const SectorPoint &pt : mPoints) {
        const double rp = pt.rangeM / mRangeLimit * radiusPx;
        if (!std::isfinite(rp) || rp < 0.0 || rp > maxRadiusPx) {
            ++clipped;
            continue;
        }
        const Coord::ScreenPoint sp = Coord::ppiPolarToScreen(
            pt.azimuthDeg, rp, cx, cy, mFlipLeftRight);
        const QPointF pos(sp.x, sp.y);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0, 255, 102, 38));
        p.drawEllipse(pos, mPointRadius * 2.0, mPointRadius * 2.0);
        p.setBrush(snrColor(pt.snr));
        p.drawEllipse(pos, mPointRadius, mPointRadius);
    }
    if (std::getenv("RADAR2D_PPI_DEBUG")) {
        std::fprintf(stderr, "[ppi] range clipped=%d shown=%d limit=%.1fm\n",
                     clipped, int(mPoints.size()) - clipped, mRangeLimit);
    }

    // Origin cross.
    p.setPen(QPen(kPhosphorDim, 1));
    p.drawLine(QPointF(cx - 5.0, cy), QPointF(cx + 5.0, cy));
    p.drawLine(QPointF(cx, cy - 5.0), QPointF(cx, cy + 5.0));

    drawHud(p);
}

void PpiWidget::drawHud(QPainter &p) {
    const HudState &h = mHud;

    QFont mono;
    mono.setFamilies({"Noto Sans Mono CJK SC", "Droid Sans Fallback",
                      "Monospace"});
    mono.setStyleHint(QFont::TypeWriter);
    mono.setPointSizeF(9.0);
    QFont small = mono;
    small.setPointSizeF(7.5);

    p.save();
    p.setFont(mono);
    const auto fm = p.fontMetrics();

    // Keep the card's left edge aligned with the range label in the toolbar.
    const qreal left = mHudAnchor
        ? qreal(mHudAnchor->mapTo(this, QPoint(0, 0)).x())
        : 14.0;
    const qreal rowH = fm.height() + 3.0;
    // Status card sits in the top-left corner, just below the range toolbar.
    const qreal top = 14.0;

    // Status is the first line of the card, left-aligned below the range bar.
    // Frame rate has its own row.
    const QString status = h.working
        ? QString::fromUtf8("当前状态：工作中")
        : QString::fromUtf8("当前状态：未工作");

    const qreal statusY = top + fm.ascent();
    p.setPen(kPhosphor);
    p.drawText(QPointF(left, statusY), status);

    // Info rows start just below the status line; no divider line in between.
    const qreal bodyTop = statusY + fm.descent() + 4.0;

    p.setFont(mono);
    const qreal labelW =
        fm.horizontalAdvance(QString::fromUtf8("丢包率    "));
    constexpr int kRowCount = 11;
    auto cell = [&](int row, const char *label, const QString &value) {
        const qreal y = bodyTop + 6.0 + fm.ascent() + row * rowH;
        p.setPen(kScale);
        p.drawText(QPointF(left, y), QString::fromUtf8(label));
        p.setPen(kPhosphor);
        p.drawText(QPointF(left + labelW, y), value);
    };

    // Single-colour display: abnormal values are flagged with '!'.
    const QString blips = QString::number(h.blips)
        + ((h.live && h.blips == 0) ? "!" : "");

    cell(0, "帧率", QString::number(h.fps, 'f', 1) + " Hz");
    cell(1, "线序号", QString::number(h.token));
    cell(2, "帧号", QString::number(h.frameIndex));
    cell(3, "间隔", QString::number(h.dtMs, 'f', 1) + "ms");
    cell(4, "源", h.source);
    cell(5, "回波", blips);
    cell(6, "点径", QString::number(h.pointRadius, 'f', 1));

    // Packet loss: flagged with '!' once it climbs above a healthy threshold.
    const QString loss = QString::number(h.packetLossPct, 'f', 1) + "%"
        + (h.packetLossPct > 2.0 ? "!" : "");
    cell(7, "丢包率", loss);
    cell(8, "转速", h.rpm > 0.0
         ? QString::number(int(h.rpm + 0.5)) + "rpm"
         : QString::fromUtf8("—"));
    cell(9, "缓冲", QString::number(h.buffered) + " 帧");
    cell(10, "模式", h.mode
         + (h.recording ? QString::fromUtf8(" · 录") : QString()));

    p.setFont(small);
    p.setPen(kPhosphorDim);
    const qreal hintY = bodyTop + 6.0 + fm.ascent() + kRowCount * rowH + 2.0;
    p.drawText(QPointF(left, hintY), "[ ]  点大小");
    p.restore();
}

void PpiWidget::keyPressEvent(QKeyEvent *ev) {
    const QString k = ev->text();
    if (k == "i" || k == "I") {
        setRangeLimit(mRangeLimit - kRangeStepM);
    } else if (k == "o" || k == "O") {
        setRangeLimit(mRangeLimit + kRangeStepM);
    } else if (k == "]" || k == "+" || k == "=") {
        setPointRadius(mPointRadius * 1.25);
    } else if (k == "[" || k == "-") {
        setPointRadius(mPointRadius / 1.25);
    } else {
        QWidget::keyPressEvent(ev);
    }
}
