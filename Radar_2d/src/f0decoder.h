#pragma once

#include <QVector>
#include <cstdint>

#include "sidestream.h"

// Raw 0x00F0 point-cloud frame, wire values only. Ported from
// t40-radar-link/host/t40_cloud_view.py (decode_00f0 / rpm_from_c5).
struct F0Frame {
    bool valid = false;
    bool sumOk = false;
    int seq = 0;
    int n = 0;
    QVector<RawPoint> raw;  // the six u16 words of each 12-byte record
    QVector<float> az;   // azimuth, rad (0 = forward, clockwise)
    QVector<float> el;   // elevation, rad
    QVector<float> r;    // slant range, m
    QVector<float> c3;   // echo amplitude, 0.01 units
    QVector<float> c4;   // second intensity / SNR candidate
    QVector<float> c5;   // detection timestamp, us
};

namespace F0 {

inline constexpr double kPi = 3.14159265358979323846;

// Decode one UDP payload (DUML magic need not be at offset 0).
F0Frame decode(const uint8_t *data, int len);

// Motor rpm inferred from the c5/azimuth slope inside one frame (0 = unknown).
double rpmFromC5(const F0Frame &f);

// Project unfiltered wire records into PPI polar coordinates. Exposed so a
// recorded session can be re-projected with different filter thresholds.
void projectRaw(const QVector<RawPoint> &raw, SideFrame &fr,
                float offsetDeg, int minAmp, float minRangeM);

// Decode + project one frame for the live path.
SideFrame toSideFrame(const F0Frame &f, float offsetDeg,
                      int minAmp, float minRangeM);

}  // namespace F0
