#pragma once

#include <QString>

// Runtime options parsed from the command line, shared by main() and the UI.
// The viewer attaches directly to the radar's perception stream on UDP 40204.
struct Config {
    QString host = "192.168.8.2";   // radar host (radar IP)
    quint16 port = 40204;           // radar perception stream UDP port
    double rangeM = 25.0;           // scope range limit (m)
    double offsetDeg = 0.0;         // azimuth rotation (forward -> top, deg)
    int minSnr = 0;                 // 1E61: drop detections with SNR below this
    int minAmp = 80;                // 0x00F0: drop points with c3 below this
    float minRangeM = 0.0f;         // drop detections closer than this (m)
    double pointRadius = 0.5;       // blip radius (px)
    QString canIface = "can0";      // SocketCAN iface used by the FC bridge
    QString fcClient;               // radar_min_client.py path (empty = auto)
    bool flipLeftRight = true;      // mirror the point cloud left-right
    bool demo = false;              // synthetic points, no hardware
    bool dump = false;              // decode one frame as text and exit
    QString selftest;               // "frames.bin:expected.txt" decoder check
};
