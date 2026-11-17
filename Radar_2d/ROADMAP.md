# Radar2d 运动目标识别、查看与跟踪 — 阶段路线图与完成情况

> 文档更新日期：2026-11-17
> 对应代码：`Radar_2d/`（C++17 + Qt6 Widgets/Network，`QPainter` 自绘）
> 范围：在已有点云 PPI 与方位-俯仰展开视图之上，分阶段实现运动目标的识别、查看与跟踪。

---

## 1. 背景与现状

`Radar_2d` 原本只做两件事：把 `UDP 40204` 上的 `0x00F0` 点云解码后画成 PPI（方位×距离），以及把同一帧画成展开视图（方位 240→120 横铺，俯仰 +45→−45 纵排，距离用颜色分度）。两者共用 `SideFrame`，通过 `QSplitter` 并排显示。

改造前的关键缺口：

| 缺口 | 表现 |
| --- | --- |
| 无帧序列 | `SideStream::onReadyRead()` 在一批 datagram 中覆盖 `display`，只显示最后一帧；`mLatest` 只存当前帧 |
| 无时基 | `c5` 仅用于反推转速，点级时间未保留；没有帧号、单调时钟、实测 `dt` |
| 无交互 | 无播放/暂停/单步/回放，无录制，无法离线复现一段场景 |
| 无检测/跟踪 | 全仓搜索无 DBSCAN、卡尔曼、聚类、航迹、ID 管理，也无相关 TODO |
| 状态语义混淆 | HUD「当前状态」有 `校验!/等待/实时/停滞` 四态，语义不直观 |

本路线图把这些缺口拆成四个阶段，其中**阶段 0 已完成并验证**，阶段 1/2/3 为已定方案、尚未实现。

---

## 2. 决定方案边界的技术约束

这些约束来自实际解码代码与仓库文档，是所有阶段设计的输入。

| 约束 | 证据 | 对方案的影响 |
| --- | --- | --- |
| 点数据无速度/多普勒/RCS/ID | `SectorPoint` 只有 `azimuthDeg/elevationDeg/rangeM/snr`（阶段 0 补了 `amp/c4/c5Us/azRaw`） | 动/静只能靠跨帧位置变化推断，是几何跟踪，不能承诺静止目标检测 |
| `c5` 是 16 位 µs | 回绕周期 65.536 ms，与 1200 rpm 一圈 50 ms、900 rpm 一圈 66.7 ms 同量级 | 不能当绝对时间戳；阶段 0 只存原值，展开留到阶段 2 |
| `seq`、`c2` 也是 16 位 | `seq` 回绕 65536；`c2` 距离 65.5 m 回绕 | 帧号用本地 `quint32`，差值用无符号环绕减法；量程已 clamp 5–50 m |
| 静态杂波主导 + 闪烁噪声 | 文档记录 ground 点 37% 的 0.5 m 栅格只有 1–2 次命中 | 必须做背景建模 + 时间一致性，不能只对整帧聚类 |
| 一帧约 230–272 点 | `expected.txt` 实测；固件上限约 5000 点/片 | 算法必须 O(点数)，GUI 线程预算 <1 ms/帧 |
| 帧率不唯一 | 文档给出 900 rpm 49.9 UDP/s、1200 rpm 64.8 UDP/s、1800 rpm 97.1 UDP/s | 窗口必须按 `dtMs` 换算成帧数，不能写死 |
| 盲区 120–240° | `kBlindStartDeg/kBlindEndDeg` | 目标穿越盲区必然丢帧，阶段 2 需 coast 外推 |
| `0x1E61` 的 `id` 不可当目标 ID | 文档明确：wire 的 id 是 tracker slot，同一 id 会跳到无关坐标 | 禁止画 per-id 航迹；pcla 的 id=1..22 是方位扇区 |
| `flipLeftRight` 默认镜像 | `PpiWidget` 用 `xSign` 翻转屏幕 x | 检测在物理坐标做，显示叠加必须与点共用同一 `xSign`，否则框点错位 |
| `snr` 不是物理 SNR | 它是 `c3` 减阈值后映射到 0..255 | 统计门限用原始 `amp`(`c3`)，不要用 `snr` |

---

## 3. 总体阶段划分

| 阶段 | 目标 | 状态 | 核心交付 |
| --- | --- | --- | --- |
| 阶段 0 | 帧序列底座：无损管道、时基、录制、回放 | **已完成** | `frameio` 模块、逐帧信号、CLI 录制/回放、两组 selftest |
| 阶段 1 | 运动目标检测：背景减除 + 聚类 + 双视图叠加 | 未开始（方案已定，详见第 5 节） | `MotionDetector`、`DetectionStore`、前景着色与簇叠加、`detect:` 回归 |
| 阶段 2 | 多目标跟踪：关联 + 卡尔曼 + 航迹管理 | 未开始（方案概要见第 6 节） | `TrackProcessor`、ID/速度/航迹、`TrackOverlay` |
| 阶段 3 | 增强与验证：外部源融合、ROI、报警、离线指标 | 未开始（候选清单见第 7 节） | `vel_idx` 验证、1E61 辅助、评估脚本 |

---

## 4. 阶段 0：帧序列底座（已完成）

### 4.1 目标与验收

把「只能看当前一帧」改造成「有可复现、可度量、可回放的帧序列」。阶段 0 不产生任何目标，但决定后续算法能否调、能否验。

| 交付物 | 验收标准 | 结果 |
| --- | --- | --- |
| 无损帧流 | 灌入 `frames.bin` 60 帧，收到 60 帧，帧号连续 | 通过 |
| 统一时基 | 每帧带单调时间、实测 `dt`、线 seq、本地帧号 | 通过 |
| 录制 | `--record` 落盘 JSONL，Python 可读原始字段与采集配置 | 通过 |
| 回放 | 与实时走同一显示路径，支持倍速/循环 | 通过 |
| 回归 | 原 selftest 0 failures，新增管道一致性用例 | 通过 |

### 4.2 工作包完成情况

| 编号 | 内容 | 状态 |
| --- | --- | --- |
| W0.1 | 数据结构扩展：`RawPoint`(6×u16)、`F0Frame::c4`、`SectorPoint` 点级字段、`SideFrame` 帧头 | 完成 |
| W0.2 | 无损帧管道：逐帧 `emit frameReady(fr)`，取消覆盖式丢帧 | 完成 |
| W0.3 | 统一时基：`frameIndex / recvMonoUs / tsWallMs / dtMs / revUs` | 完成 |
| W0.4 | 帧缓冲、JSONL 录制（含 meta 与原子改名） | 完成 |
| W0.5 | 回放数据源 `ReplaySource`，按实测 `dt` 驱动、倍速、循环 | 完成 |
| W0.6 | 回归：原解码器对拍 + 管道/往返/阈值重投影/截断/环形缓冲/控件状态机 | 完成 |

### 4.3 实现清单

| 文件 | 改动 |
| --- | --- |
| `src/sidestream.h/.cpp` | 新增 `RawPoint`；`SectorPoint` 补 `amp/c4/c5Us/azRaw`；`SideFrame` 补帧头；`frameReady(const SideFrame&)` 逐帧发射；`packetLossPct` 按帧统计；`Q_DECLARE_METATYPE(SideFrame)` |
| `src/f0decoder.h/.cpp` | 解码并保留 `c4`(此前丢弃)；`projectRaw()` 抽出为可复用接口，供回放按不同阈值重投影 |
| `src/frameio.h/.cpp`（新增） | `FrameStore`（1200 帧环形缓冲 + `cursorFor()`）、`FrameRecorder`（JSONL meta/frame 两行结构、`.part` 原子改名、1 s/64 KB 缓冲 flush）、`ReplaySource`（加载、seek/step/play/pause、倍速、循环） |
| `src/mainwindow.h/.cpp` | `LIVE/REPLAY` 状态机；`applyFrame()` 统一显示入口；CLI 驱动的录制启动；HUD 缓冲/帧号/模式字段；demo 合成 `RawPoint` 使 demo 也可录制回放 |
| `src/ppiwidget.h/.cpp` | HUD 扩到 12 行（帧号、间隔、缓冲、录制、模式等）；后续改为「工作中/未工作」两态 |
| `src/main.cpp` | 新增 `--record/--replay/--speed/--loop/--auto-exit`；`runPipeSelftest()`；`runDumpWith()` 适配带参信号 |
| `src/config.h` | 新增上述 CLI 配置字段 |
| `CMakeLists.txt` | 登记 `frameio.h/.cpp`（AUTOMOC 需扫描头文件） |
| `run.sh` | 新增 `record`、`replay` 模式；`selftest` 同时跑两组；`demo` 转发额外参数 |

### 4.4 关键数据结构

```cpp
struct RawPoint {                 // 线上 12 字节记录的 6 个 u16，未过滤
    quint16 azRaw, elRaw, rRaw;   // mrad / (el+2π) mrad / mm
    quint16 c3, c4, c5Us;
};

struct SectorPoint {              // 投影后的显示点
    float azimuthDeg, elevationDeg, rangeM, snr;
    float amp, c4;                // 原始幅度与第二强度
    quint16 c5Us, azRaw;          // 供阶段 2 扫描时刻补偿
};

struct SideFrame {
    // 原有：valid / entryCount / frameToken / sumOk / rpm / 盲区 / points
    QVector<RawPoint> rawPoints;  // 录制用，未经过滤
    quint32 frameIndex;           // 本地单调（回放时重编号）
    qint64  recvMonoUs;           // 主时基
    qint64  tsWallMs;             // 仅人对齐
    double  dtMs;                 // 实测帧间隔
    double  revUs;                // 60e6 / rpm
};
```

设计要点：录制点放在 `F0::decode()` 之后、`toSideFrame()` 之前，保存未过滤的原始 6 字段。因此回放可以用**当前 CLI 的 `minAmp/minRangeM/offsetDeg` 重新投影**，不必重录。

### 4.5 JSONL 格式

第一行 meta，后续每行一帧：

```json
{"type":"meta","version":1,"cfg":{"minAmp":80,"minRangeM":0,"offsetDeg":0,"rangeM":25,"flipLeftRight":true,"source":"demo"},"start_wall_ms":1794725519044}
{"type":"frame","i":0,"token":12345,"ts_us":...,"wall_ms":...,"dt_ms":49.8,"rpm":1201.4,"rev_us":49958.0,"sum_ok":1,"raw":[[az,el,r,c3,c4,c5],...]}
```

### 4.6 验证记录

| 检查 | 结果 |
| --- | --- |
| 编译 | Qt 6.10.3 / C++17，`BUILD_RC=0`，0 warning；lint 0 问题 |
| 原解码器对拍 | `selftest: 60 frames, 0 failures` |
| 新管道回归 | `pipe selftest: 60 frames, 0 failures` |
| 端到端录制 | demo 3 s → 38 帧 × 241 点/帧；`dt` 75.5–84.3 ms；`ts` 严格递增；帧号连续 |
| 端到端回放 | 4× 倍速 + 循环驱动 PPI 逐帧绘制（11 次重绘）；`--min-amp 0` 可离线重投影 |
| 2026-11-17 复验 | `bash -n run.sh` 通过；两组 selftest 仍 0 failures |

回归覆盖：逐帧无损、帧头连续、环形缓冲溢出后游标重锚、JSONL 录→放逐字保真、阈值重投影不丢点、截断文件拒绝、播放状态机。

### 4.7 阶段 0 之后的 UI 调整

| 调整 | 说明 |
| --- | --- |
| 删除「帧序列」工具条 | 用户要求移除该行 UI。播放/暂停、单步、进度条、倍速、录制、回到直播的按钮与快捷键全部删除；`LIVE/REPLAY` 两态保留。底层 `FrameStore/FrameRecorder/ReplaySource` 与 CLI 完全保留，阶段 1/2 仍可用 |
| 状态行改两态 | HUD「当前状态」从 `校验!/等待/实时/停滞` 改为 **工作中 / 未工作**。新增 `HudState::working`，由 `refreshHud()` 按「最近 500 ms 内有帧」判定，不依赖 `fps` 估算，避免低帧率闪烁；校验异常不再单独改变状态行 |

当前回放只能按 `--speed` 连续播放（可 `--loop`），无暂停/单步/拖动入口；`ReplaySource::pause/step/seek` 方法本身保留，供 selftest 与后续阶段复用。

### 4.8 已知限制

1. **真实 UDP 链路未做硬件实测**：验证基于 `frames.bin` 与 demo；上机第一步应先跑 `./run.sh dump` 确认。
2. **一包是否等于一整圈尚未实测**：阶段 0 已记录 `dtMs` 与 `revUs`，需用两者交叉校验。
3. `c5` 只存原值、未展开为绝对时间（阶段 2 处理）。
4. 异常终止时录制留在 `xxx.jsonl.part`，手动改名即可恢复；正常关窗自动 rename。
5. 回放帧不写入实时 `FrameStore`，二者共用 `applyFrame` 作为唯一显示入口。

---

## 5. 阶段 1：运动目标检测（方案已定，未开始）

### 5.1 目标与验收

把静态场景从点云里剥掉，输出**无 ID 的逐帧运动点簇**，在 PPI 与展开视图上稳定可见、参数可调。不做跨帧关联、ID、卡尔曼（阶段 2）。

| 验收项 | 标准 |
| --- | --- |
| 静态背景抑制 | 静止场景回放时前景点 ≤ 总点数 5%，不出现持续 >5 帧的簇 |
| 运动簇检出 | 注入的合成匀速目标 warmup 后检出率 ≥ 90%，质心误差 ≤ 0.3 m |
| 闪烁噪声抑制 | 单帧随机噪声点不形成簇 |
| 双视图叠加 | 簇与点云坐标对齐（含 `flip` 镜像） |
| 离线可调 | 一份 JSONL + 参数扫描脚本可出检出率/虚警/质心误差曲线 |

### 5.2 工作包

| 编号 | 内容 | 状态 |
| --- | --- | --- |
| S1.1 | 物理坐标系与栅格（含 `flip` 陷阱、栅格尺寸推导） | 未开始 |
| S1.2 | 双窗背景模型（长窗判静态 + 短窗判一致） | 未开始 |
| S1.3 | 栅格连通域 + 并查集聚类 | 未开始 |
| S1.4 | 俯仰/高度/幅度/量程门控 | 未开始 |
| S1.5 | PPI + 展开视图叠加层、HUD 字段、开关 | 未开始 |
| S1.6 | 回放/暂停/单步下的检测状态一致性 + 结果缓存 | 未开始 |
| S1.7 | CLI 参数、`--export-detections`、`detect:` selftest、离线评估脚本 | 未开始 |

### 5.3 核心设计

**坐标系**：检测在物理笛卡尔坐标做，`x = r·sin(φ)`、`y = r·cos(φ)`、`h = r·tan(θ)`；显示才转屏幕坐标。`PpiWidget` 的 `xSign = flipLeftRight ? -1 : 1` 必须被点与簇共用，否则镜像错位。建议抽 `screenX()` 公共方法。

**栅格尺寸**：波束约 1° → 25 m 处横向 0.436 m，取 `g = 0.6 m`；cell 键用 `qint64` 打包，`QHash` 稀疏存储，不建全图数组。

**双窗背景模型**（核心）：单窗口无法三分类，用长窗判静态、短窗判一致。

| 现象 | 长窗命中率 | 短窗命中率 | 判定 |
| --- | --- | --- | --- |
| 闪烁噪声 | 低 | 低 | 丢弃 |
| 静态地物 | 高 | 高 | 背景，丢弃 |
| 运动目标 | 低（位置在漂） | 高（连续出现） | 前景 |
| 新出现的静止物 | 低→升高 | 高 | 短期误报，靠漂移检验/老化消除 |

实现为两个 `deque<QSet<CellKey>>` + `longHits/shortHits` 计数表，对齐 Python 侧 `PersistenceFilter` 但反用其结论。每帧代价 O(本帧点数 + 过期集合大小)，无全图遍历。

窗口按时间换算：`L = ceil(τ_bg·1000 / median(dtMs))`、`S = ceil(τ_fg·1000 / median(dtMs))`。可检测速度下限 `v_min ≈ g/τ_bg = 0.3 m/s`，属几何方法固有下限，需写入界面说明。冷启动前 `L` 帧只建模型不报簇，HUD 显示「建模中…」；背景按 `τ_forget ≈ 20 s` 老化以吸收场景变化。

**聚类**：前景点 → cell key → 8 邻域（可选 24 邻域）并查集 → 幅度加权质心。不用 DBSCAN：250 点规模下 O(N²) 距离矩阵无必要。过滤：`members < 2` 丢弃、`spanM > 4 m` 丢弃（地面连片）、低置信度丢弃。可选「漂移检验」抑制新静止物误报。

**门控顺序**：`amp/range → el/height → 背景双窗 → 聚类 → 漂移检验`。剔除 `h < -2 m` 的多径与 `|h| < 0.2 m` 的贴地簇；俯仰窗 −25°..+45° 与展开视图对齐。

### 5.4 参数表（建议初值）

| 参数 | 默认 | 依据 |
| --- | --- | --- |
| `gridM` | 0.6 m | 25 m 处 1° 波束 ≈ 0.436 m |
| `tauBg` | 2.0 s | 决定 `v_min ≈ 0.3 m/s` |
| `tauFg` | 0.15 s | 约 3–10 帧，滤闪烁 |
| `longMinHits` | 0.6 × L | 长窗命中率门限 |
| `shortMinHits` | 3 | 短窗一致性 |
| `warmupFrames` | L | 冷启动静默 |
| `tauForget` | 20 s | 背景老化 |
| `elMin/elMax` | −25°/+45° | 对齐展开视图 |
| `hMin` | −2 m | 剔除多径 |
| `minClusterPts` | 2 | 单点不成目标 |
| `maxClusterSpan` | 4 m | 剔除地面连片 |
| `neighborRadius` | 1（8 邻域） | 可切 2 |
| `driftCheck` | off | 可选 |

### 5.5 显示叠加与状态一致性

- **PPI**：在点绘制之后、`drawHud()` 之前加 `drawClusters()`；前景点改橙/红，簇画质心十字 + 包围圆 + 帧内编号；用 `QVector<bool> mForeground` 传入，不改 `SectorPoint`。
- **展开视图**：用 `FlatView::unrolledAz()` 与既有俯仰映射画簇椭圆，并标注 `r` 数值（距离在该视图只用颜色编码）。
- **数据传递**：新增 `DetectionResult { clusters, foreground, bgCells, warmup }`，由 `MainWindow::applyFrame(fr, det)` 同时喂两个视图。
- **状态一致性**：检测器有状态，回退需重建。推荐新增 `DetectionStore`（与 `FrameStore` 按 `frameIndex` 对齐的并行环形数组，约 340 KB），seek 直接取缓存；不做连续重放。只有 LIVE 正向与 REPLAY 首轮构建才 `update()`。
- 簇 `id` 仅是帧内序号，**不得跨帧复用**，界面不要显示成「目标 1/2」以免误解。

### 5.6 离线评估（无标注真值，必须造）

1. 用阶段 0 的 JSONL，在 Python 中注入匀速运动点 6 元组，另存 `truth.jsonl`。
2. 新增 `--export-detections`，输出 `xxx.detections.jsonl`（每行 `{i,ts_us,warmup,fg,bg_cells,clusters:[{cx,cy,r,az,el,n,hits,conf}]}`），不改阶段 0 录制格式。
3. 指标：检出率（最近簇距真值 <0.5 m）、质心 RMSE、虚警簇数/帧、前景抑制率、首次检出延迟。
4. `gridM × tauBg × shortMinHits` 网格扫参，选虚警 <1/帧且检出率 >90% 的组合。
5. `detect:` selftest：构造 60 帧（静止簇 + 匀速簇 + 随机噪声），断言 warmup 内 0 报、静止簇 0 报、噪声 0 报、运动簇 warmup 后每帧检出且误差 <0.3 m。

### 5.7 改动清单

`src/detector.h/.cpp`（新增）、`DetectionStore`、`config.h`/`main.cpp`（参数与导出与 `detect:` selftest）、`mainwindow.*`（检测驱动、工具条开关、HUD）、`ppiwidget.*`（`setDetections`/`drawClusters`/`screenX`）、`flatview.*`（簇叠加）、`CMakeLists.txt`、`tools/eval_detections.py`（新增）。

顺序：`S1.1 → S1.2 → detect: 回归 → S1.3 → S1.4 → S1.6 → S1.5`。先让算法在合成数据上可验证，再做界面，避免靠肉眼调参。

---

## 6. 阶段 2：多目标跟踪（未开始，方案概要）

| 工作包 | 方案 |
| --- | --- |
| 关联 | B1：GNN + 椭圆门限 + α-β/匀速卡尔曼，适合目标 <10；B2：多目标交叉时换匈牙利/JPDA 全局分配 |
| 航迹管理 | B3：M/N 起始（3/5 帧确认）、连续漏检删除、ID 分配、coast 预测外推；输出 `Track{id,x,y,vx,vy,age,miss,snr}` |
| 时间基准 | B4：用 `c5Us/azRaw` 与 `rpm` 补偿旋转扫描时刻，处理 16 位回绕；`dt` 用实测值而非固定 1/fps |
| 坐标系 | 滤波/预测在笛卡尔 `(x,y)`（匀速模型线性），门限/栅格在极坐标 `(az,r)`（方位误差随距离放大） |
| 输入 | `TrackProcessor::update(fr, det)`，直接消费阶段 1 的 `DetectionResult` |
| 显示 | `TrackOverlay`：ID、速度矢量、时间衰减 trail、目标列表；可加鼠标拾取与高亮 |

验收：ID 连续性（穿越盲区后保持）、轨迹质心 RMSE、虚警航迹数、coast 成功率。注意 SORT 的「检测+KF+关联」管线可照搬，但无外观特征，DeepSORT 的 appearance 部分不适用。

---

## 7. 阶段 3：增强与验证（未开始，候选清单）

| 方向 | 说明 |
| --- | --- |
| `vel_idx` 验证与融合 | 侧视 omni 快照每扇区带 `vel_idx`；若确为速度索引，可直接标记动/静，识别质量提升最大，但需先验证语义 |
| `0x1E61` 辅助 | wire/pcla 是固件已聚类的目标级 xyz（约 0.17/1.1 Hz），可作候选补充；**其 `id` 仍不可当航迹 ID** |
| ROI | 框选感兴趣区域，仅在该区域起轨 |
| 越界报警 | 目标进入/离开设定距离或方位区时的提示 |
| 离线指标评估 | 用 `truth.jsonl` 做 ROC/精度召回，形成可复现的调参基线 |
| RTI 对照 | 仓库已有 256 bin × 200 帧 RTI 瀑布图，可与运动检测结果交叉验证 |

---

## 8. 跨阶段风险与未决事项

| 事项 | 状态 | 处理方向 |
| --- | --- | --- |
| 一包是否等于一整圈 | 未实测 | 用阶段 0 的 `dtMs / revUs` 交叉校验；决定阶段 1 窗口与阶段 2 时间补偿模型 |
| `c5` 展开 | 未做 | 结合帧内方位顺序与 rpm，在阶段 2 实现 |
| 真值标注 | 缺失 | 用 JSONL 注入合成目标构造 |
| 慢速目标 | 方法下限 | `v_min ≈ 0.3 m/s` 明确写入界面与文档 |
| `flip` 镜像 | 默认开启 | 叠加层与点共用 `xSign`；selftest 加镜像一致性断言 |
| GUI 线程预算 | <1 ms/帧 | 若 24 邻域 + 长窗 120 帧需实测耗时，必要时分离 `update()` 与绘制 |
| 异常退出录制 | 留在 `.part` | 手动改名恢复；如需自动恢复可在阶段 3 加信号 handler |
| 盲区穿越 | 必然丢帧 | 阶段 2 coast 外推 |
| 硬件实测 | 未做 | 上机先 `./run.sh dump`，再录一段真实回放在 GUI 检查 |

---

## 9. 使用速查

```bash
cd Radar_2d

./run.sh selftest                                    # 解码器 + 帧管道回归
./run.sh dump                                        # 收一帧文本（上机第一步）
./run.sh udp                                         # 实时显示（开机按钮启动雷达）
./run.sh record /tmp/session.jsonl                   # 实时 + JSONL 录制
./run.sh replay /tmp/session.jsonl --speed 2 --loop  # 回放
./run.sh demo --record /tmp/demo.jsonl --auto-exit 3000   # 无硬件自测
```

`--auto-exit <ms>` 是 offscreen 自动化验收选项，正常退出时完成录制文件收尾。

---

## 10. 变更记录

| 日期 | 内容 |
| --- | --- |
| 2026-11-15 | 完成阶段 0：数据结构扩展、无损帧管道、时基、录制/回放、两组 selftest、端到端 demo 录制与回放验证 |
| 2026-11-15 | 按用户要求删除「帧序列」工具条 UI；录制/回放改为纯 CLI 驱动 |
| 2026-11-15 | HUD「当前状态」改为「工作中 / 未工作」两态，判据为最近 500 ms 内有帧 |
| 2026-11-17 | 本文档建立，汇总各阶段方案、完成情况、验证记录与未决事项（复验 selftest 仍 0 failures） |
