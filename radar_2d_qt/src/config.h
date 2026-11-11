#pragma once

#include <QString>

// Runtime options parsed from the command line, shared by main() and the UI.
struct Config {
    QString host = "192.168.8.2";   // radar_streamer host (radar IP)
    quint16 port = 9877;            // SIDE streamer UDP port
    double rangeM = 25.0;           // scope range limit (m)
    double offsetDeg = 180.0;       // azimuth offset (forward -> top)
    int minSnr = 0;                 // drop detections with SNR below this
    float minRangeM = 0.0f;         // drop detections closer than this (m)
    double pointRadius = 2.0;       // blip radius (px)
    bool demo = false;              // synthetic points, no hardware
    bool dump = false;              // decode one frame as text and exit
};
