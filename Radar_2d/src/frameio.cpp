#include "frameio.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "f0decoder.h"

namespace {

QByteArray compactLine(const QJsonObject &o) {
    return QJsonDocument(o).toJson(QJsonDocument::Compact).trimmed() + '\n';
}

}  // namespace

// ─── FrameMeta ───────────────────────────────────────────────────────────
QJsonObject FrameMeta::toJson() const {
    QJsonObject o;
    o.insert(QStringLiteral("offsetDeg"), offsetDeg);
    o.insert(QStringLiteral("minAmp"), minAmp);
    o.insert(QStringLiteral("minRangeM"), double(minRangeM));
    o.insert(QStringLiteral("rangeM"), rangeM);
    o.insert(QStringLiteral("flipLeftRight"), flipLeftRight);
    o.insert(QStringLiteral("source"), source);
    return o;
}

FrameMeta FrameMeta::fromJson(const QJsonObject &o) {
    FrameMeta m;
    m.offsetDeg = o.value(QStringLiteral("offsetDeg")).toDouble(m.offsetDeg);
    m.minAmp = o.value(QStringLiteral("minAmp")).toInt(m.minAmp);
    m.minRangeM = float(o.value(QStringLiteral("minRangeM"))
                            .toDouble(double(m.minRangeM)));
    m.rangeM = o.value(QStringLiteral("rangeM")).toDouble(m.rangeM);
    m.flipLeftRight = o.value(QStringLiteral("flipLeftRight"))
                          .toBool(m.flipLeftRight);
    m.source = o.value(QStringLiteral("source")).toString(m.source);
    return m;
}

// ─── FrameStore ──────────────────────────────────────────────────────────
FrameStore::FrameStore(int maxFrames) : mMaxFrames(std::max(1, maxFrames)) {}

void FrameStore::append(const SideFrame &fr) {
    mFrames.push_back(fr);
    while (int(mFrames.size()) > mMaxFrames) {
        mFrames.pop_front();
    }
}

void FrameStore::clear() { mFrames.clear(); }

int FrameStore::cursorFor(quint32 frameIndex) const {
    if (mFrames.empty()) {
        return -1;
    }
    // Unsigned subtraction stays correct across a quint32 wrap.
    const quint32 delta = frameIndex - mFrames.front().frameIndex;
    return delta < quint32(mFrames.size()) ? int(delta) : -1;
}

// ─── FrameRecorder ───────────────────────────────────────────────────────
bool FrameRecorder::open(const QString &path, const FrameMeta &meta,
                         QString *err) {
    close();
    mPath = path;
    if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
        if (err) {
            *err = QStringLiteral("cannot create directory for %1").arg(path);
        }
        return false;
    }
    mFile.setFileName(path + QStringLiteral(".part"));
    if (!mFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (err) {
            *err = QStringLiteral("cannot open %1: %2")
                       .arg(mFile.fileName(), mFile.errorString());
        }
        return false;
    }
    QJsonObject o;
    o.insert(QStringLiteral("type"), QStringLiteral("meta"));
    o.insert(QStringLiteral("version"), 1);
    o.insert(QStringLiteral("cfg"), meta.toJson());
    o.insert(QStringLiteral("start_wall_ms"),
             QDateTime::currentMSecsSinceEpoch());
    mFile.write(compactLine(o));
    mBuf.clear();
    mFrames = 0;
    mClock.start();
    return true;
}

void FrameRecorder::write(const SideFrame &fr) {
    if (!mFile.isOpen() || !fr.valid) {
        return;
    }
    QJsonObject o;
    o.insert(QStringLiteral("type"), QStringLiteral("frame"));
    o.insert(QStringLiteral("i"), qint64(fr.frameIndex));
    o.insert(QStringLiteral("token"), qint64(fr.frameToken));
    o.insert(QStringLiteral("ts_us"), fr.recvMonoUs);
    o.insert(QStringLiteral("wall_ms"), fr.tsWallMs);
    o.insert(QStringLiteral("dt_ms"), fr.dtMs);
    o.insert(QStringLiteral("rpm"), fr.rpm);
    o.insert(QStringLiteral("rev_us"), fr.revUs);
    o.insert(QStringLiteral("sum_ok"), fr.sumOk ? 1 : 0);
    QJsonArray pts;
    for (const RawPoint &rp : fr.rawPoints) {
        QJsonArray a;
        a.append(int(rp.azRaw));
        a.append(int(rp.elRaw));
        a.append(int(rp.rRaw));
        a.append(int(rp.c3));
        a.append(int(rp.c4));
        a.append(int(rp.c5Us));
        pts.append(a);
    }
    o.insert(QStringLiteral("raw"), pts);
    mBuf += compactLine(o);
    ++mFrames;
    // Buffered writes: never fsync per frame on the GUI thread.
    if (mBuf.size() >= 65536 || mClock.elapsed() >= 1000) {
        flush();
    }
}

void FrameRecorder::flush() {
    if (!mFile.isOpen() || mBuf.isEmpty()) {
        return;
    }
    mFile.write(mBuf);
    mBuf.clear();
    mClock.restart();
}

void FrameRecorder::close() {
    if (!mFile.isOpen()) {
        return;
    }
    flush();
    const QString part = mFile.fileName();
    mFile.close();
    const QString finalPath = mPath;
    if (!finalPath.isEmpty()) {
        if (QFile::exists(finalPath) && !QFile::remove(finalPath)) {
            std::fprintf(stderr, "[record] cannot replace %s\n",
                         qPrintable(finalPath));
        }
        if (!QFile::rename(part, finalPath)) {
            std::fprintf(stderr, "[record] cannot rename %s -> %s\n",
                         qPrintable(part), qPrintable(finalPath));
        }
    }
    mPath.clear();
}

// ─── ReplaySource ────────────────────────────────────────────────────────
ReplaySource::ReplaySource(const FrameMeta &cfg, QObject *parent)
    : QObject(parent), mCfg(cfg) {
    mTimer.setParent(this);
    mTimer.setSingleShot(true);
    connect(&mTimer, &QTimer::timeout, this, &ReplaySource::onTick);
}

bool ReplaySource::load(const QString &path, QString *err) {
    pause();
    mFrames.clear();
    mCursor = -1;

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (err) {
            *err = QStringLiteral("cannot open %1: %2")
                       .arg(path, f.errorString());
        }
        return false;
    }
    const QList<QByteArray> lines = f.readAll().split('\n');
    bool haveMeta = false;
    int lineNo = 0;
    for (const QByteArray &rawLine : lines) {
        ++lineNo;
        const QByteArray line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        QJsonParseError pe;
        const QJsonDocument doc = QJsonDocument::fromJson(line, &pe);
        if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
            if (err) {
                *err = QStringLiteral("line %1: %2 (truncated file?)")
                           .arg(lineNo)
                           .arg(pe.errorString());
            }
            mFrames.clear();
            return false;
        }
        const QJsonObject o = doc.object();
        const QString type = o.value(QStringLiteral("type")).toString();
        if (type == QLatin1String("meta")) {
            mFileMeta = FrameMeta::fromJson(
                o.value(QStringLiteral("cfg")).toObject());
            haveMeta = true;
            continue;
        }
        if (type != QLatin1String("frame")) {
            continue;
        }
        if (!haveMeta) {
            if (err) {
                *err = QStringLiteral("line %1: frame before meta line")
                           .arg(lineNo);
            }
            mFrames.clear();
            return false;
        }

        SideFrame fr;
        fr.valid = true;
        fr.frameIndex = quint32(mFrames.size());  // local, renumbered
        fr.frameToken = quint32(o.value(QStringLiteral("token")).toInt());
        fr.sumOk = o.value(QStringLiteral("sum_ok")).toInt(1) != 0;
        fr.rpm = o.value(QStringLiteral("rpm")).toDouble();
        fr.recvMonoUs = o.value(QStringLiteral("ts_us")).toInteger();
        fr.tsWallMs = o.value(QStringLiteral("wall_ms")).toInteger();
        fr.dtMs = o.value(QStringLiteral("dt_ms")).toDouble();
        fr.revUs = o.value(QStringLiteral("rev_us")).toDouble();

        const QJsonArray pts = o.value(QStringLiteral("raw")).toArray();
        fr.rawPoints.reserve(pts.size());
        for (const QJsonValue &v : pts) {
            const QJsonArray a = v.toArray();
            if (a.size() != 6) {
                if (err) {
                    *err = QStringLiteral("line %1: malformed raw point")
                               .arg(lineNo);
                }
                mFrames.clear();
                return false;
            }
            RawPoint rp;
            rp.azRaw = quint16(a.at(0).toInt());
            rp.elRaw = quint16(a.at(1).toInt());
            rp.rRaw = quint16(a.at(2).toInt());
            rp.c3 = quint16(a.at(3).toInt());
            rp.c4 = quint16(a.at(4).toInt());
            rp.c5Us = quint16(a.at(5).toInt());
            fr.rawPoints.push_back(rp);
        }
        fr.entryCount = quint16(fr.rawPoints.size());
        // Re-project with the *current* thresholds: offline tuning works
        // without re-recording the radar session.
        F0::projectRaw(fr.rawPoints, fr, float(mCfg.offsetDeg), mCfg.minAmp,
                       mCfg.minRangeM);
        mFrames.push_back(fr);
    }
    if (mFrames.isEmpty()) {
        if (err) {
            *err = QStringLiteral("no frames in %1").arg(path);
        }
        return false;
    }
    std::fprintf(stderr, "[replay] %d frames from %s\n", mFrames.size(),
                 qPrintable(path));
    return true;
}

void ReplaySource::emitCursor() {
    if (mCursor >= 0 && mCursor < mFrames.size()) {
        emit frameReady(mFrames.at(mCursor));
    }
}

void ReplaySource::schedule() {
    mTimer.stop();
    if (!mPlaying || mCursor < 0 || mCursor + 1 >= mFrames.size()) {
        return;
    }
    double dt = mFrames.at(mCursor + 1).dtMs;
    if (!(dt > 0.0) || dt > 1000.0) {
        dt = 50.0;   // first frame / implausible gap: assume 20 Hz
    }
    const int ms = std::max(1, int(std::lround(dt / mSpeed)));
    mTimer.start(ms);
}

void ReplaySource::play() {
    if (mFrames.isEmpty()) {
        return;
    }
    mPlaying = true;
    if (mCursor < 0 || mCursor >= mFrames.size()) {
        mCursor = 0;
    }
    emitCursor();
    schedule();
    emit stateChanged();
}

void ReplaySource::pause() {
    mPlaying = false;
    mTimer.stop();
    emit stateChanged();
}

void ReplaySource::step(int delta) {
    if (mFrames.isEmpty()) {
        return;
    }
    mPlaying = false;
    mTimer.stop();
    seek(mCursor < 0 ? 0 : mCursor + delta);
}

void ReplaySource::seek(int cursor) {
    if (mFrames.isEmpty()) {
        return;
    }
    mCursor = std::clamp<int>(cursor, 0, int(mFrames.size()) - 1);
    emitCursor();
    if (mPlaying) {
        schedule();
    }
    emit stateChanged();
}

void ReplaySource::onTick() {
    if (!mPlaying) {
        return;
    }
    if (mCursor + 1 >= mFrames.size()) {
        if (mLoop) {
            seek(0);
        } else {
            pause();
        }
        return;
    }
    ++mCursor;
    emitCursor();
    schedule();
}
