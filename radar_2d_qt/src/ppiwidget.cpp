#include "ppiwidget.h"

#include <QKeyEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;

// viridis colour stops (approximate), interpolated by SNR in 0..255.
struct Stop { float t; int r, g, b; };
constexpr Stop kStops[] = {
    {0.00f,  68,   1,  84},
    {0.13f,  71,  44, 122},
    {0.25f,  59,  81, 139},
    {0.38f,  44, 113, 142},
    {0.50f,  33, 144, 140},
    {0.63f,  59, 173,  97},
    {0.75f, 125, 204,  67},
    {0.88f, 192, 229,  59},
    {1.00f, 253, 231,  37},
};

int lerp(int a, int b, float f) {
    return int(a + (b - a) * f + 0.5f);
}
}  // namespace

PpiWidget::PpiWidget(QWidget *parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(420, 420);
    setStatusText("waiting for SIDE frames (UDP 9877) ...");
}

void PpiWidget::setFrame(const SideFrame &fr) {
    mPoints = fr.points;
    update();
}

void PpiWidget::setStatusText(const QString &text) {
    mStatus = text;
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
    const float t = std::clamp(snr / 255.0f, 0.0f, 1.0f);
    constexpr int n = int(sizeof(kStops) / sizeof(kStops[0]));
    for (int i = 0; i < n - 1; ++i) {
        if (t <= kStops[i + 1].t) {
            const float span = kStops[i + 1].t - kStops[i].t;
            const float f = span > 0.0f ? (t - kStops[i].t) / span : 0.0f;
            return QColor(lerp(kStops[i].r, kStops[i + 1].r, f),
                          lerp(kStops[i].g, kStops[i + 1].g, f),
                          lerp(kStops[i].b, kStops[i + 1].b, f));
        }
    }
    return QColor(kStops[n - 1].r, kStops[n - 1].g, kStops[n - 1].b);
}

void PpiWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.fillRect(rect(), Qt::black);
    p.setRenderHint(QPainter::Antialiasing, true);

    const double w = width(), h = height();
    const double cx = w / 2.0, cy = h / 2.0;
    const double radiusPx = std::min(w, h) * 0.5 - 34.0;  // leave room for labels

    // Bearing lines (every 30 deg).
    p.setPen(QPen(QColor(45, 45, 45, 150), 1));
    for (int a = 0; a < 360; a += 30) {
        const double rad = a * kPi / 180.0;
        p.drawLine(QPointF(cx, cy),
                   QPointF(cx + radiusPx * std::sin(rad),
                           cy - radiusPx * std::cos(rad)));
    }

    // Range rings + labels.
    QFont small = font();
    small.setPointSizeF(7.5);
    p.setFont(small);
    for (double d = 5.0; d <= mRangeLimit + 1e-6; d += 5.0) {
        const double rp = d / mRangeLimit * radiusPx;
        p.setPen(QPen(QColor(60, 60, 60, 130), 1, Qt::DotLine));
        p.drawEllipse(QPointF(cx, cy), rp, rp);
        p.setPen(QColor(150, 160, 150));
        p.drawText(QPointF(cx + rp + 3.0, cy), QString::number(int(d)) + "m");
    }

    // Forward marker (white triangle at the top).
    const QPointF tip(cx, cy - radiusPx - 2.0);
    const QPointF b1(cx - 6.0, cy - radiusPx - 14.0);
    const QPointF b2(cx + 6.0, cy - radiusPx - 14.0);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 255, 255));
    p.drawPolygon(QPolygonF() << tip << b1 << b2);

    // Blips, coloured by SNR.
    for (const SectorPoint &pt : mPoints) {
        const double rad = pt.azimuthDeg * kPi / 180.0;
        const double rp = pt.rangeM / mRangeLimit * radiusPx;
        const double x = cx + rp * std::sin(rad);
        const double y = cy - rp * std::cos(rad);
        p.setBrush(snrColor(pt.snr));
        p.drawEllipse(QPointF(x, y), mPointRadius, mPointRadius);
    }

    // Status text.
    QFont mono("Monospace");
    mono.setStyleHint(QFont::TypeWriter);
    mono.setPointSizeF(8.5);
    p.setFont(mono);
    p.setPen(QColor(142, 255, 176));
    p.drawText(QRectF(8, 8, w - 16, h - 16), Qt::AlignTop | Qt::AlignLeft,
               mStatus);
}

void PpiWidget::wheelEvent(QWheelEvent *ev) {
    const double steps = ev->angleDelta().y() / 120.0;
    if (steps > 0) {
        setRangeLimit(mRangeLimit / 1.1);
    } else if (steps < 0) {
        setRangeLimit(mRangeLimit * 1.1);
    }
    ev->accept();
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
