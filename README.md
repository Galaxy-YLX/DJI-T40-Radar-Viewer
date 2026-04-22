## Human Says

详情和图片/视频请见这个[blog](https://www.leommxj.com/2026/04/20/dji-t40-RD2484R-radar-reverse-engineering/)

`ar1021x_driver` 目录下是针对ar1021x在6.x kernel中ath6kl驱动的修改,在`6.17.0-20-generic` 经过测试。 `make` then `make load`。

`setup` 目录下是方便启动CAN和`radar_wifi`热点的脚本与配置

`radar_client` 中包含:

- 几个.md: 是对DUML、radar通信协议的分析以及逆向注释，没有完全验证，可能且大概率存在错误。

- 几个脚本: 
  
  - `radar_min_client.py` 基于40204/udp 50007/udp 与CAN总线，可以使雷达进入工作状态，并输出避障业务相关的信息
  
  - `radar_viewer.py` 通过在雷达上部(Digital)运行 `radar_streamer` 程序将app.elf内存中的初阶数据导出并可视化。
  
  - `radar_mem_poke.py` 通过在雷达上部(Digital)运行 `radar_mem_poke` 来检查内存内容，方便分析。
  
  - `duml_can_tool.py` 使用与CAN总线与雷达下部(ESC)交互，可以操作雷达转速、获取雷达运行信息等。

一般的工作流程

```bash
# 首先网卡加载驱动
cd ar1021x_driver
make # 如果没编译
make load

# 启动并连接设备，然后配置CAN与热点
cd ../setup
sudo ./start_can.sh
sudo ./start_ap.sh

cd ../radar_client
pip install -r requirements.txt
# 运行客户端并转发CAN，几秒后应该观察到雷达900rpm旋转，并且绘出部分内容。
python3 radar_min_client.py --can-bridge can0 --plot
# 运行另一个viewer
python radar_viewer.py --deploy 

# 如果要直接与ESC交互
python duml_can_tool.py shell
```

## Human Says (English)

For details, images, and videos, please refer to this [blog]().

The `ar1021x_driver` directory contains modifications to the `ath6kl` driver for the **ar1021x** chipset on **6.x kernels**. It has been tested on `6.17.0-20-generic`. Use `make` followed by `make load` to install and load.

The `setup` directory contains scripts and configurations to conveniently initialize the CAN bus and the `radar_wifi` hotspot.

The `radar_client` directory includes:

* Several .md files: Analysis and reverse-engineering notes on **DUML** and radar communication protocols. These are not fully verified and likely contain errors.
* Several scripts:
  * `radar_min_client.py`: Based on 40204/udp, 50007/udp, and the CAN bus. It enables the radar's operational state and outputs obstacle avoidance data.
  * `radar_viewer.py`: Visualizes raw data exported from the `app.elf` memory by running the `radar_streamer` program on the upper part (Digital) of the radar.
  * `radar_mem_poke.py`: Used to inspect memory contents by running `radar_mem_poke` on the upper part (Digital) of the radar to facilitate analysis.
  * `duml_can_tool.py`: Interacts with the lower part (ESC) of the radar via the CAN bus to control rotation speed, retrieve operational info, etc.

### General Workflow

```bash
# First, load the driver for the network adapter
cd ar1021x_driver
make # If not already compiled
make load

# Power on radar, Then setup CAN & Hotspot
cd ../setup
sudo ./start_can.sh
sudo ./start_ap.sh

# Initialize the client
cd ../radar_client
pip install -r requirements.txt

# Run the client and bridge the CAN bus. 
# After a few seconds, the radar should rotate at 900 RPM and start plotting data.
python3 radar_min_client.py --can-bridge can0 --plot

# Run an additional viewer
python radar_viewer.py --deploy 

# To interact directly with the ESC
python duml_can_tool.py shell
```
