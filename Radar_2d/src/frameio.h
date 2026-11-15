#pragma once

#include <QByteArray>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QtGlobal>
#include <deque>

#include "sidestream.h"

// Filter/threshold settings that produced a recording. Stored in the meta
// line so an offline replay can be projected under different settings.
struct FrameMeta {
    double offsetDeg = 0.0;
    int minAmp = 80;
    float minRangeM = 0.0f;
    double rangeM = 25.0;
    bool flipLeftRight = true;
    QString source;

    QJsonObject toJson() const;
    static FrameMeta fromJson(const QJsonObject &o);
};

// Bounded ring of recent frames. Live, paused and replay consumers all read
// from this, so they always see exactly the same frame objects.
class FrameStore {
public:
    explicit FrameStore(int maxFrames = kDefaultMaxFrames);

    void append(const SideFrame &fr);
    void clear();
    int size() const { return int(mFrames.size()); }
    int maxFrames() const { return mMaxFrames; }
    bool empty() const { return mFrames.empty(); }
    const SideFrame &at(int cursor) const { return mFrames.at(size_t(cursor)); }
    const SideFrame &back() const { return mFrames.back(); }
    quint32 firstIndex() const {
        return mFrames.empty() ? 0 : mFrames.front().frameIndex;
    }
    // Cursor for a local frameIndex (clamped, -1 when out of range).
    int cursorFor(quint32 frameIndex) const;

    static constexpr int kDefaultMaxFrames = 1200;  // ~20 s at 65 packets/s

private:
    std::deque<SideFrame> mFrames;
    int mMaxFrames;
};

// JSONL recorder: one meta line, then one compact JSON object per frame.
// Writes the unfiltered wire words, never the projected points.
class FrameRecorder {
public:
    ~FrameRecorder() { close(); }

    bool open(const QString &path, const FrameMeta &meta, QString *err);
    void write(const SideFrame &fr);
    void flush();
    void close();
    bool isOpen() const { return mFile.isOpen(); }
    const QString &path() const { return mPath; }
    int frameCount() const { return mFrames; }

private:
    QFile mFile;
    QString mPath;      // final path (we write to .part first)
    QByteArray mBuf;
    QElapsedTimer mClock;
    int mFrames = 0;
};

// File-backed source with the same frameReady(SideFrame) signal as SideStream.
// Frames are re-projected from their raw words using the *current* config,
// so thresholds can be tuned offline without re-recording.
class ReplaySource : public QObject {
    Q_OBJECT
public:
    explicit ReplaySource(const FrameMeta &cfg, QObject *parent = nullptr);

    bool load(const QString &path, QString *err);
    int frameCount() const { return mFrames.size(); }
    const QVector<SideFrame> &frames() const { return mFrames; }
    int cursor() const { return mCursor; }
    const FrameMeta &fileMeta() const { return mFileMeta; }
    bool loaded() const { return !mFrames.isEmpty(); }

    bool playing() const { return mPlaying; }
    double speed() const { return mSpeed; }
    bool loop() const { return mLoop; }
    void setSpeed(double s) { mSpeed = qBound(0.05, s, 16.0); schedule(); }
    void setLoop(bool on) { mLoop = on; }

public slots:
    void play();
    void pause();
    void step(int delta);   // negative = rewind one frame
    void seek(int cursor);

signals:
    void frameReady(const SideFrame &fr);
    void stateChanged();

private slots:
    void onTick();

private:
    void emitCursor();
    void schedule();

    FrameMeta mCfg;             // projection settings (current CLI)
    FrameMeta mFileMeta;        // settings stored in the file
    QVector<SideFrame> mFrames;
    QTimer mTimer;
    int mCursor = -1;
    double mSpeed = 1.0;
    bool mLoop = false;
    bool mPlaying = false;
};
