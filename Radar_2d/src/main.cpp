#include <QApplication>
#include <QCommandLineParser>
#include <QEventLoop>
#include <QTimer>
#include <algorithm>
#include <cstdio>

#include "config.h"
#include "mainwindow.h"
#include "sidestream.h"

static Config parseConfig(const QCoreApplication &app) {
    Config cfg;
    QCommandLineParser p;
    p.setApplicationDescription(
        "DJI T40 side radar — 2D PPI (azimuth x range), C++/Qt6");
    p.addHelpOption();
    p.addOptions({
        {"host", "radar host (radar IP)", "host", "192.168.8.2"},
        {"port", "DJI 1E61 v2 perception stream UDP port", "port", "40204"},
        {"range", "scope range limit in metres", "m", "25"},
        {"azimuth-offset", "azimuth rotation in degrees (forward -> top)", "deg",
         "0"},
        {"min-snr", "drop detections with SNR below this", "n", "0"},
        {"min-range", "drop detections closer than this (m)", "m", "0"},
        {"point-size", "blip radius in pixels", "px", "0.5"},
        {"flip", "mirror the point cloud left-right (on by default)"},
        {"no-flip", "disable the left-right mirror"},
        {"demo", "render synthetic points (no radar hardware)"},
        {"dump", "decode one frame as text and exit"},
    });
    p.process(app);

    cfg.host = p.value("host");
    cfg.port = p.value("port").toUShort();
    cfg.rangeM = p.value("range").toDouble();
    cfg.offsetDeg = p.value("azimuth-offset").toDouble();
    cfg.minSnr = p.value("min-snr").toInt();
    cfg.minRangeM = p.value("min-range").toFloat();
    cfg.pointRadius = p.value("point-size").toDouble();
    cfg.demo = p.isSet("demo");
    cfg.dump = p.isSet("dump");
    cfg.flipLeftRight = !p.isSet("no-flip");
    return cfg;
}

static int runDump(const Config &cfg) {
    SideStream stream(cfg.host, cfg.port, float(cfg.offsetDeg), cfg.minSnr,
                      cfg.minRangeM);
    QEventLoop loop;
    bool got = false;
    QObject::connect(&stream, &SideStream::frameReady, [&]() {
        const SideFrame &fr = stream.latest();
        got = true;
        std::printf("=== 2D radar (1E61 one frame) ===\n");
        std::printf("seq=%u subtype=%u entries=%u points=%d\n",
                    fr.frameToken, fr.subtype, fr.entryCount,
                    int(fr.points.size()));
        if (!fr.points.isEmpty()) {
            float rmin = 1e9f, rmax = -1e9f, smin = 1e9f, smax = -1e9f;
            for (const SectorPoint &pt : fr.points) {
                rmin = std::min(rmin, pt.rangeM);
                rmax = std::max(rmax, pt.rangeM);
                smin = std::min(smin, pt.snr);
                smax = std::max(smax, pt.snr);
            }
            std::printf("  range: %.2f .. %.2f m   snr: %.0f .. %.0f\n",
                        rmin, rmax, smin, smax);
        }
        loop.quit();
    });
    QTimer::singleShot(8000, &loop, &QEventLoop::quit);
    stream.start();
    loop.exec();
    if (!got) {
        std::printf("no side frame within 8s — is the streamer running?\n");
    }
    return got ? 0 : 1;
}

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("radar2d");
    const Config cfg = parseConfig(app);

    if (cfg.dump) {
        return runDump(cfg);
    }

    MainWindow w(cfg);
    w.show();
    return app.exec();
}
