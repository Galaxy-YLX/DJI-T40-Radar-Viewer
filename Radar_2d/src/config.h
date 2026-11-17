#pragma once

#include <QString>

// Runtime options parsed from the command line, shared by main() and the UI.
// The viewer attaches directly to the radar's perception stream on UDP 40204.
struct Config {
    QString host = "192.168.8.2";   // radar host (radar IP)
    quint16 port = 40204;           // radar perception stream UDP port
    double rangeM = 25.0;           // scope range limit (m)
    double offsetDeg = 0.0;         // azimuth rotation (forward -> top, deg)
    int minAmp = 0;                 // 0x00F0: drop points with c3 below this
    float minRangeM = 0.0f;         // drop detections closer than this (m)
    double speedGateMps = 0.0;      // speed gate threshold (m/s, 0 = off)
    int motorRpm = 900;             // radar ESC setpoint (rpm)
    quint16 motorControlPort = 50008; // loopback UDP control port
    double pointRadius = 0.5;       // blip radius (px)
    QString canIface = "can0";      // SocketCAN iface used by the FC bridge
    QString fcClient;               // radar_min_client.py path (empty = auto)
    double fmRampSec = 0.5;         // flight_mode=1 hold before the 1->2 edge
    bool flipLeftRight = true;      // mirror the point cloud left-right
    bool demo = false;              // synthetic points, no hardware
    bool dump = false;              // decode one frame as text and exit
    QString selftest;               // "frames.bin:expected.txt" decoder check
    QString recordPath;             // JSONL frame recorder output
    QString replayPath;             // replay a JSONL instead of UDP
    double speed = 1.0;             // replay speed multiplier
    bool loop = false;              // loop at end of replay file
    int autoExitMs = 0;             // quit after N ms (offscreen regression)
};
