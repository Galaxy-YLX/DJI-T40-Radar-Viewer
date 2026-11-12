#include "ppiwidget.h"

#include <QKeyEvent>
#include <QPainter>
#include <QPaintEvent>
#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

// Classic monochrome radar: pure black canvas, green phosphor only.
const QColor kGrid(0, 96, 48, 170);      // bearing lines
const QColor kRing(0, 120, 60, 150);     // range rings
const QColor kScale(0, 205, 105);        // ring / bearing labels
const QColor kPhosphor(0, 255, 102);     // primary text, strong returns
const QColor kPhosphorDim(0, 185, 88);   // labels, secondary text
}  // namespace

PpiWidget::PpiWidget(QWidget *parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(420, 420);
}

void PpiWidget::setFrame(const SideFrame &fr) {
    mPoints = fr.points;
    update();
}

void PpiWidget::setHud(const HudState &hud) {
    if (mHud == hud) {
        return;
    }
    mHud = hud;
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
    const double radiusPx = std::min(w, h) * 0.5 - 34.0;  // leave room for labels

    // Bearing lines (every 30 deg).
    p.setPen(QPen(kGrid, 1));
    for (int a = 0; a < 360; a += 30) {
        const double rad = a * kPi / 180.0;
        p.drawLine(QPointF(cx, cy),
                   QPointF(cx + radiusPx * std::sin(rad),
                           cy - radiusPx * std::cos(rad)));
    }

    // Range rings + labels, all green phosphor.
    QFont small = font();
    small.setPointSizeF(7.5);
    p.setFont(small);
    for (double d = 5.0; d <= mRangeLimit + 1e-6; d += 5.0) {
        const double rp = d / mRangeLimit * radiusPx;
        p.setPen(QPen(kRing, 1, Qt::DotLine));
        p.drawEllipse(QPointF(cx, cy), rp, rp);
        p.setPen(kScale);
        p.drawText(QPointF(cx + rp + 3.0, cy), QString::number(int(d)) + "m");
    }

    // Cardinal bearings, just inside the outer ring so they never collide
    // with the range labels drawn along the 090 radial.
    p.setPen(kPhosphorDim);
    for (int a = 0; a < 360; a += 90) {
        const double rad = a * kPi / 180.0;
        const double lx = cx + (radiusPx - 24.0) * std::sin(rad);
        const double ly = cy - (radiusPx - 24.0) * std::cos(rad);
        // Lift the label off the 090/270 radial, where range numbers sit.
        p.drawText(QPointF(lx - 10.0, ly - 12.0),
                   QString::number(a).rightJustified(3, QLatin1Char('0')));
    }

    // Forward marker (green triangle at the top).
    const QPointF tip(cx, cy - radiusPx - 2.0);
    const QPointF b1(cx - 6.0, cy - radiusPx - 14.0);
    const QPointF b2(cx + 6.0, cy - radiusPx - 14.0);
    p.setPen(Qt::NoPen);
    p.setBrush(kPhosphor);
    p.drawPolygon(QPolygonF() << tip << b1 << b2);

    // Rotating sweep arm with a short phosphor afterglow tail.
    if (mHud.live && mHud.fps > 0.0) {
        const double scan = std::fmod(double(mHud.rotA), 360.0);
        constexpr double kTailDeg = 24.0;
        constexpr int kTailSteps = 8;
        const QRectF disc(cx - radiusPx, cy - radiusPx,
                          2.0 * radiusPx, 2.0 * radiusPx);
        p.setPen(Qt::NoPen);
        for (int i = 0; i < kTailSteps; ++i) {
            const double step = kTailDeg / kTailSteps;
            const int alpha = int(70.0 * (1.0 - double(i) / kTailSteps));
            p.setBrush(QColor(0, 255, 102, alpha));
            p.drawPie(disc, int((90.0 - scan + i * step) * 16.0),
                      int(step * 16.0) + 1);
        }
        const double a = scan * kPi / 180.0;
        p.setPen(QPen(QColor(0, 255, 102, 150), 1.5));
        p.drawLine(QPointF(cx, cy),
                   QPointF(cx + radiusPx * std::sin(a),
                           cy - radiusPx * std::cos(a)));
    }

    // Returns: a soft phosphor halo plus a green core graded by SNR.
    for (const SectorPoint &pt : mPoints) {
        const double rad = pt.azimuthDeg * kPi / 180.0;
        const double rp = pt.rangeM / mRangeLimit * radiusPx;
        const double x = cx + rp * std::sin(rad);
        const double y = cy - rp * std::cos(rad);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0, 255, 102, 38));
        p.drawEllipse(QPointF(x, y), mPointRadius * 2.0,
                      mPointRadius * 2.0);
        p.setBrush(snrColor(pt.snr));
        p.drawEllipse(QPointF(x, y), mPointRadius, mPointRadius);
    }

    // Origin cross.
    p.setPen(QPen(kPhosphorDim, 1));
    p.drawLine(QPointF(cx - 5.0, cy), QPointF(cx + 5.0, cy));
    p.drawLine(QPointF(cx, cy - 5.0), QPointF(cx, cy + 5.0));

    drawHud(p);
}

void PpiWidget::drawHud(QPainter &p) {
    const HudState &h = mHud;
    const bool streaming = h.live && h.fps > 0.0;

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

    const qreal left = 14.0;
    const qreal right = 292.0;
    const qreal top = 14.0;
    const qreal rowH = fm.height() + 3.0;

    const QString title = h.demo ? QString("[ 侧向雷达 / 演示 ]")
                                 : QString("[ 侧向雷达 ]");
    const QString badge =
        !h.live ? QString("[ 等待 ]  --.- Hz")
                : (streaming
                       ? QString("[ 实时 ]  %1 Hz").arg(h.fps, 0, 'f', 1)
                       : QString("[ 停滞 ]  0.0 Hz"));

    const qreal titleY = top + fm.ascent();
    p.setPen(kPhosphor);
    p.drawText(QPointF(left, titleY), title);
    p.drawText(QPointF(right - fm.horizontalAdvance(badge), titleY), badge);

    const qreal ruleY = titleY + fm.descent() + 4.0;
    p.setPen(QPen(kPhosphorDim, 1.0));
    p.drawLine(QPointF(left, ruleY), QPointF(right, ruleY));

    p.setFont(mono);
    const qreal labelW =
        fm.horizontalAdvance(QString::fromUtf8("扇区    "));
    const qreal col2 = left + 136.0;
    auto cell = [&](qreal x, int row, const char *label,
                    const QString &value) {
        const qreal y = ruleY + 6.0 + fm.ascent() + row * rowH;
        p.setPen(kScale);
        p.drawText(QPointF(x, y), QString::fromUtf8(label));
        p.setPen(kPhosphor);
        p.drawText(QPointF(x + labelW, y), value);
    };

    // Single-colour display: abnormal values are flagged with '!'.
    const QString sectors = QString("%1/360").arg(h.sectors)
        + ((h.live && h.sectors < 360) ? "!" : "");
    const QString cfar = QString::number(h.cfarError)
        + (h.cfarError != 0 ? "!" : "");
    const QString blips = QString::number(h.blips)
        + ((h.live && h.blips == 0) ? "!" : "");

    cell(left, 0, "帧号", QString::number(h.frames));
    cell(col2, 0, "令牌", QString::number(h.token));
    cell(left, 1, "扇区", sectors);
    cell(col2, 1, "杂波", cfar);
    cell(left, 2, "转子",
         QString("%1/%2/%3").arg(h.rotA).arg(h.rotB).arg(h.rotC));
    cell(col2, 2, "回波", blips);
    cell(left, 3, "量程", QString::number(h.rangeM, 'f', 1) + "m");
    cell(col2, 3, "点径", QString::number(h.pointRadius, 'f', 1));

    p.setFont(small);
    p.setPen(kPhosphorDim);
    p.drawText(QPointF(left, ruleY + 6.0 + fm.ascent() + 4.0 * rowH + 2.0),
               "i / o 键  缩放      [ ]  点大小");
    p.restore();
}

void PpiWidget::keyPressEvent(QKeyEvent *ev) {
    const QString k = ev->text();
    if (k == "i" || k == "I") {
        setRangeLimit(mRangeLimit / 1.1);
    } else if (k == "o" || k == "O") {
        setRangeLimit(mRangeLimit * 1.1);
    } else if (k == "]" || k == "+" || k == "=") {
        setPointRadius(mPointRadius * 1.25);
    } else if (k == "[" || k == "-") {
        setPointRadius(mPointRadius / 1.25);
    } else {
        QWidget::keyPressEvent(ev);
    }
}
