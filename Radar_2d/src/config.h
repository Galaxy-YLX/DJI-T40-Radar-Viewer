#pragma once

#include <QString>

// Runtime options parsed from the command line, shared by main() and the UI.
struct Config {
    QString host = "192.168.8.2";   // radar host (radar IP)
    quint16 port = 40204;           // DJI 1E61 v2 perception stream UDP port
    double rangeM = 25.0;           // scope range limit (m)
    double offsetDeg = 0.0;         // azimuth rotation (forward -> top, deg)
    int minSnr = 0;                 // drop detections with SNR below this
    float minRangeM = 0.0f;         // drop detections closer than this (m)
    double pointRadius = 0.5;       // blip radius (px)
    bool flipLeftRight = true;      // mirror the point cloud left-right
    bool demo = false;              // synthetic points, no hardware
    bool dump = false;              // decode one frame as text and exit
};
