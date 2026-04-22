# AR1021x (ath6kl) Out-of-Tree Driver

Qualcomm AR1021x USB WiFi adapter 的 out-of-tree 内核驱动，基于 Linux v6.17 主线 `ath6kl` 源码修改。

AR1021x 使用 AR6004 hw3.0 芯片，USB ID 为 `0cf3:1022`。

## 编译 & 使用

```bash
# 需要安装内核头文件，例如 Ubuntu:
# sudo apt install linux-headers-$(uname -r)

# 编译
make

# 加载（需要 root，会自动加载 cfg80211 依赖）
sudo make load

# 或安装到系统模块目录（内核更新后需重新编译安装）
sudo make install
```

固件文件需放置于 `/lib/firmware/ath6k/AR6004/hw3.0/`（至少需要 `fw.ram.bin` 和 `bdata.bin`）。

## 基于内核版本

Linux v6.17 (`drivers/net/wireless/ath/ath6kl/`)

## 修改内容

相对于 v6.17 主线 ath6kl 共 6 处修改（涉及 5 个文件）：

### 1. 添加 USB 设备 ID (`usb.c`)

```diff
 static const struct usb_device_id ath6kl_usb_ids[] = {
+	{USB_DEVICE(0x0cf3, 0x1022)},
 	{USB_DEVICE(0x0cf3, 0x9375)},
 	{USB_DEVICE(0x0cf3, 0x9374)},
 	{USB_DEVICE(0x04da, 0x390d)},
```

### 2. 修正 `wmi_connect_event` AP 模式字段顺序 (`wmi.h`)

```diff
 		struct {
-			u8 aid;
 			u8 phymode;
+			u8 aid;
 			u8 mac_addr[ETH_ALEN];
 			u8 auth;
 			u8 keymgmt;
```

AR1021x 固件发送 AP 模式 STA 连接事件时，`phymode` 在前、`aid` 在后，与主线定义相反。
不修正会导致 `ath6kl_connect_ap_mode_sta()` 读到错误的 `aid` 值（实际读到的是 `phymode`），
在 `aid < 1 || aid > AP_MAX_NUM_STA` 检查时被静默丢弃，表现为 AP 模式下客户端能关联但 AP 侧无感知、无法通信。

### 3. 添加 USB flow control 硬件标志 (`core.h`, `init.c`)

```diff
 enum ath6kl_hw_flags {
+	ATH6KL_HW_SINGLE_PIPE_SCHED	= BIT(1),
 	ATH6KL_HW_SDIO_CRC_ERROR_WAR	= BIT(3),
+	ATH6KL_HW_USB_FLOWCTRL		= BIT(4),
 };
```

```diff
 		.uarttx_pin			= 11,
-		.flags				= 0,
+		.flags				= ATH6KL_HW_SINGLE_PIPE_SCHED |
+						  ATH6KL_HW_USB_FLOWCTRL,
```

### 4. 禁用 HTC credit-based flow control (`init.c`)

```diff
 	memset(&connect, 0, sizeof(connect));

+	if (ar->hw.flags & ATH6KL_HW_USB_FLOWCTRL)
+		connect.conn_flags |= HTC_CONN_FLGS_DISABLE_CRED_FLOW_CTRL;
+
 	/* these fields are the same for all service endpoints */
```

AR1021x (AR6004 hw3.0 USB) 固件不使用 HTC credit-based flow control，而是依赖 USB pipe 自身的队列深度进行流控。
主线驱动默认启用 credit flow control，固件不会发送 credit 报告，导致 HTC TX 路径在 credit 耗尽后永久阻塞。
阻塞发生在持有 `tx_lock` 自旋锁的上下文中，进而导致所有网络操作（包括 `ip addr`）因等待 RTNL 锁而挂起，最终系统不可用。

### 5. 移除 USB 断开路径中的 wiphy_lock (`init.c`)

```diff
 	ath6kl_cfg80211_vif_stop(vif, test_bit(WMI_READY, &ar->flag));
 	rtnl_lock();
-	wiphy_lock(ar->wiphy);
 	ath6kl_cfg80211_vif_cleanup(vif);
-	wiphy_unlock(ar->wiphy);
 	rtnl_unlock();
```

### 6. 使用 `unregister_netdevice` 替代 `cfg80211_unregister_netdevice` (`cfg80211.c`)

```diff
-	cfg80211_unregister_netdevice(vif->ndev);
+	unregister_netdevice(vif->ndev);
```

修改 5、6 共同解决 USB 热插拔时的系统死锁问题。
在内核 6.17 中，`cfg80211_unregister_netdevice` 内部会调用 `unregister_netdevice`，
后者触发 `NETDEV_GOING_DOWN` 通知链，cfg80211 的通知处理函数会无条件获取 `wiphy->mtx`。
如果调用者已持有 `wiphy_lock`，则同一线程再次获取同一互斥锁，导致自死锁。
移除外层 `wiphy_lock` 并直接调用 `unregister_netdevice`，让 cfg80211 通知链自行管理锁获取，避免死锁。
