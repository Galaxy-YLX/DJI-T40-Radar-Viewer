#pragma once

#include <QString>

// Which point-cloud source feeds the PPI.
enum class Source {
    Link,  // t40-radar-link board bridge, TCP 46000, 0x00F0 point cloud
    Udp,   // direct radar UDP 40204 (0x00F0 points or legacy 1E61 entries)
};

// Runtime options parsed from the command line, shared by main() and the UI.
struct Config {
    Source source = Source::Link;   // 0x00F0 point cloud over the link by default
    QString linkHost = "192.168.9.1";  // t40_link board address
    quint16 linkPort = 46000;          // T40R forwarding TCP port
    QString host = "192.168.8.2";   // radar host (radar IP, UDP source only)
    quint16 port = 40204;           // radar perception stream UDP port
    double rangeM = 25.0;           // scope range limit (m)
    double offsetDeg = 0.0;         // azimuth rotation (forward -> top, deg)
    int minSnr = 0;                 // 1E61: drop detections with SNR below this
    int minAmp = 80;                // 0x00F0: drop points with c3 below this
    float minRangeM = 0.0f;         // drop detections closer than this (m)
    double pointRadius = 0.5;       // blip radius (px)
    int rpm = 1200;                 // motor rpm sent over the link on power-on
    QString canIface = "can0";      // SocketCAN iface used by the FC bridge
    QString fcClient;               // radar_min_client.py path (empty = auto)
    bool flipLeftRight = true;      // mirror the point cloud left-right
    bool demo = false;              // synthetic points, no hardware
    bool dump = false;              // decode one frame as text and exit
    QString selftest;               // "frames.bin:expected.txt" decoder check
};
