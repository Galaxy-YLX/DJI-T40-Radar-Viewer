#pragma once

#include <cmath>

// Single source of truth for geometry in the radar viewer.
//
// Physical frame (unmirrored):
//   +x = right, +y = forward, +z = up.
//   azimuth = 0 at forward, increasing clockwise, in degrees.
//   SectorPoint::rangeM is the HORIZONTAL range (slant * cos(elevation)).
//
// Screen frames are derived from the physical frame. Mirroring is a display
// option only: it must be applied to blips and overlays, never to the physical
// coordinates used by detection/tracking.
namespace Coord {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// The firmware/Python decoder stores elevation as (el + 2*pi) in mrad with
// this rounded constant. Keep it here so every decoder entry point agrees.
constexpr double kRawElevationWrapRad = 6.28319;

// Unrolled forward sector: 240 -> 360/000 -> 120.
constexpr double kUnrolledStartDeg = 240.0;
constexpr double kUnrolledSpanDeg = 240.0;
constexpr double kElevTopDeg = 45.0;
constexpr double kElevBottomDeg = -45.0;
constexpr double kElevSpanDeg = kElevTopDeg - kElevBottomDeg;

inline double degToRad(double deg) { return deg * kDegToRad; }
inline double radToDeg(double rad) { return rad * kRadToDeg; }

inline double normalizeDeg(double deg) {
    double a = std::fmod(deg, 360.0);
    if (a < 0.0) {
        a += 360.0;
    }
    return a;
}

struct PhysicalPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ScreenPoint {
    double x = 0.0;
    double y = 0.0;
};

// Polar -> physical. rangeM is horizontal; z = rangeM * tan(elevation).
inline PhysicalPoint physicalXYFromPolar(double azDeg, double horizontalRangeM) {
    const double rad = degToRad(azDeg);
    return {horizontalRangeM * std::sin(rad),
            horizontalRangeM * std::cos(rad), 0.0};
}

inline PhysicalPoint physicalFromPolar(double azDeg, double horizontalRangeM,
                                       double elevationDeg) {
    PhysicalPoint p = physicalXYFromPolar(azDeg, horizontalRangeM);
    p.z = horizontalRangeM * std::tan(degToRad(elevationDeg));
    return p;
}

// Physical -> polar (useful when turning a cluster centroid back into a blip).
inline double horizontalRangeFromXY(double x, double y) {
    return std::hypot(x, y);
}

inline double azimuthFromXY(double x, double y) {
    return normalizeDeg(radToDeg(std::atan2(x, y)));
}

inline double elevationFromHorizontal(double z, double horizontalRangeM) {
    return radToDeg(std::atan2(z, horizontalRangeM));
}

// PPI screen mapping. Grid/ticks pass mirrorX=false; blips and overlays pass
// the widget's flip flag. Forward stays up in both cases.
inline ScreenPoint ppiPolarToScreen(double azDeg, double radiusPx,
                                    double cx, double cy, bool mirrorX) {
    const double rad = degToRad(azDeg);
    const double sign = mirrorX ? -1.0 : 1.0;
    return {cx + sign * radiusPx * std::sin(rad),
            cy - radiusPx * std::cos(rad)};
}

// Unrolled-view mapping. `unrolledAz` is in 0..240; >240 is the rear wedge.
inline double unrolledAz(double azDeg) {
    return normalizeDeg(azDeg - kUnrolledStartDeg);
}

inline double flatXFromUnrolledAz(double unrolledDeg, double left, double width,
                                  bool mirrorX) {
    const double a = mirrorX ? kUnrolledSpanDeg - unrolledDeg : unrolledDeg;
    return left + a / kUnrolledSpanDeg * width;
}

inline double flatXFromAz(double azDeg, double left, double width,
                          bool mirrorX) {
    return flatXFromUnrolledAz(unrolledAz(azDeg), left, width, mirrorX);
}

inline double flatYFromElevation(double elevationDeg, double top, double height) {
    return top + (kElevTopDeg - elevationDeg) / kElevSpanDeg * height;
}

}  // namespace Coord
