#include "f0decoder.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <vector>

namespace {
constexpr int kHdr19 = 19;
constexpr int kRec12 = 12;
constexpr float kTwoPi = 6.28319f;  // same constant as the Python decoder
constexpr double kRadToDeg = 180.0 / F0::kPi;

inline uint16_t u16(const uint8_t *p) {
    return uint16_t(p[0] | (p[1] << 8));
}
}  // namespace

namespace F0 {

F0Frame decode(const uint8_t *data, int len) {
    F0Frame f;
    if (!data || len < kHdr19) {
        return f;
    }
    int start = -1;
    for (int i = 0; i + 2 < len; ++i) {
        if (data[i] == 0x03 && data[i + 1] == 0x10 && data[i + 2] == 0x07) {
            start = i;
            break;
        }
    }
    if (start < 0) {
        return f;
    }
    const uint8_t *body = data + start;
    const int avail = len - start;
    if (avail < kHdr19) {
        return f;
    }
    const int nbytes = u16(body + 13);
    if (nbytes == 0 || nbytes % kRec12 != 0 || kHdr19 + nbytes > avail) {
        return f;
    }

    const uint8_t *raw = body + kHdr19;
    int sumRaw = 0;
    for (int i = 0; i < nbytes; ++i) {
        sumRaw += raw[i];
    }
    int sumHdr = 0;
    for (int i = 1; i < 18; ++i) {
        sumHdr += body[i];
    }

    const int n = nbytes / kRec12;
    f.valid = true;
    f.sumOk = (body[17] == uint8_t(sumRaw & 0xFF))
           && (body[18] == uint8_t(sumHdr & 0xFF));
    f.seq = u16(body + 3);
    f.n = n;
    f.az.resize(n);
    f.el.resize(n);
    f.r.resize(n);
    f.c3.resize(n);
    f.c5.resize(n);
    for (int k = 0; k < n; ++k) {
        const uint8_t *e = raw + k * kRec12;
        f.az[k] = u16(e) / 1000.0f;
        f.el[k] = u16(e + 2) / 1000.0f - kTwoPi;
        f.r[k] = u16(e + 4) / 1000.0f;
        f.c3[k] = float(u16(e + 6));
        f.c5[k] = float(u16(e + 10));
    }
    return f;
}

double rpmFromC5(const F0Frame &f) {
    if (!f.valid || f.n < 12) {
        return 0.0;
    }
    // One c5 per azimuth bin, keyed by the firmware's native mrad value.
    std::map<int, float> first;
    for (int k = 0; k < f.n; ++k) {
        const int key = int(f.az[k] * 1000.0f);
        if (first.find(key) == first.end()) {
            first[key] = f.c5[k];
        }
    }
    if (first.size() < 8) {
        return 0.0;
    }

    std::vector<double> ys;
    ys.reserve(first.size());
    double base = 0.0;
    double prev = first.begin()->second;
    for (const auto &kv : first) {
        const double t = kv.second;
        double d = std::fmod(t - prev, 65536.0);
        if (d < 0.0) {
            d += 65536.0;
        }
        if (d > 32768.0) {
            d -= 65536.0;
        }
        base += d;
        prev = t;
        ys.push_back(base);
    }

    double mx = 0.0, my = 0.0;
    size_t i = 0;
    for (const auto &kv : first) {
        mx += kv.first;
        my += ys[i++];
    }
    const double m = double(first.size());
    mx /= m;
    my /= m;
    double num = 0.0, den = 0.0;
    i = 0;
    for (const auto &kv : first) {
        const double dx = double(kv.first) - mx;
        num += dx * (ys[i++] - my);
        den += dx * dx;
    }
    if (den <= 0.0 || num <= 0.0) {
        return 0.0;
    }
    return 60e6 / (num / den * 6283.19);
}

SideFrame toSideFrame(const F0Frame &f, float offsetDeg,
                      int minAmp, float minRangeM) {
    SideFrame fr;
    if (!f.valid) {
        return fr;
    }
    fr.valid = true;
    fr.entryCount = uint16_t(f.n);
    fr.frameToken = uint32_t(f.seq);
    fr.sumOk = f.sumOk;
    fr.rpm = rpmFromC5(f);
    fr.points.reserve(f.n);

    for (int k = 0; k < f.n; ++k) {
        const float amp = f.c3[k];
        if (amp < float(minAmp)) {
            continue;
        }
        // points_xy(): the horizontal range is r*cos(el), azimuth stays
        // clockwise with 0 degrees at the top, matching the PPI convention.
        const float range = float(f.r[k] * std::cos(f.el[k]));
        if (!std::isfinite(range) || range < minRangeM) {
            continue;
        }
        // 0x00F0's c0 is the rotating head angle: its zero sits at the centre
        // of the no-data sector, 180 deg off physical forward (cross-check
        // against 1E61 xyz: best offset = az + pi). Rotate it so the returns
        // land in the PPI's +/-120 deg forward view.
        float azDeg = float(std::fmod(double(f.az[k]) * kRadToDeg
                                      + 180.0 + double(offsetDeg), 360.0));
        if (azDeg < 0.0f) {
            azDeg += 360.0f;
        }
        if (azDeg > kBlindStartDeg && azDeg < kBlindEndDeg) {
            continue;   // rear no-data sector, outside the display range
        }
        SectorPoint p;
        p.azimuthDeg = azDeg;
        p.rangeM = range;
        // The Python view grades c3 by (c3 - threshold)/300 into 16 levels;
        // map the same window onto the PPI's 0..255 brightness scale.
        p.snr = std::clamp((amp - float(minAmp)) * (255.0f / 300.0f),
                           0.0f, 255.0f);
        fr.points.push_back(p);
    }
    return fr;
}

}  // namespace F0
