# P4C5 USB 2.0 详细功能规划

- **适用范围**：p4c5 板卡的 USB 2.0 接口（ESP32-P4 OTG_HS），分支 `p4c5_dev`
- **撰写日期**：2026-09-04
- **配套文档**：总路线图见 [p4c5-feature-roadmap_zh.md](p4c5-feature-roadmap_zh.md)（本节对应其中的方向 D）
- **定位一句话**：这条 USB 高速口是 P4 上"屏 / 摄像头"之外的第三条高吞吐通道，能 **Host 也能 Device** —— 既是外设扩展总线，又是一条有线数据通道。对"带 6.2" 大屏 + SD 卡的桌面智能设备"而言，它补足了语音不好使、WiFi 不在线这两类场景。

> ⚠️ 本文是**规划**，不是已实现。所有带 "①" 的段落都是决定方向的**待确认硬件事实**，先确认再投入开发。

---

## 1. 硬件规格（已核实的技术事实）

ESP32-P4 上实际有 **3 套 USB 相关外设**，别混淆：

| 外设 | 速度 | 引脚 | 说明 | 本板可用性 |
|---|---|---|---|---|
| USB-Serial/JTAG | — | 专用 | 烧录 / console，通常已接调试口 | 已用于下载调试（假定） |
| USB OTG **FS**（全速） | 12 Mbps | GPIO matrix 可路由；⚠ 有报告称默认映射在 26/27 | USB1.1 全速 OTG | ⚠ **不推荐**：26/27 已被本板占用（LCD 复位 / 触摸 INT）。除非按原理图用自由引脚重映射，否则避开 |
| **USB OTG HS（即"USB2.0"）** | **480 Mbps**（HS，向下兼容 FS/LS） | **D-/D+ 专用 GPIO49 / GPIO50**，集成 UTMI HS 收发器 | 本规划目标 | 板上 GPIO45–54 未占用，49/50 空闲，**无引脚冲突** |

**控制器能力**（OTG_HS，DWC_OTG 内核）：
- **Host**：16 通道，支持 HS/FS/LS、全部 4 种传输类型（含 isochronous/中断 OUT）；ESP-IDF `usb_host` 驱动 + esp-usb 组件提供现成类驱动：**HID（键盘/鼠标）、CDC-ACM、MSC（U 盘）、UVC（USB 摄像头）**；RNDIS/ECM 网络走 esp-iot-solution 的 `iot_usbh_rndis`（本项目已在依赖里，且有 `main/boards/common/rndis_board.cc` 的现成实现先例）。
- **Device**：EP0 + 15 端点（最多 8 路 IN 同时工作）；用 `esp_tinyusb`（**v1.4.3 起支持 P4 HS，建议 v2.x**）可实现 **CDC-ACM 串口、HID 键盘/鼠标 gadget、MSC、ECM/RNDIS 网络 gadget** 及复合设备（如"串口+键盘"、"串口+U盘"）。UAC 音频（免驱声卡）TinyUSB 提供底层支持，需自行集成验证。
- 两个 OTG 控制器（FS 与 HS）可**各自独立**作 Host/Device。

**必须知道的坑**：
- **HS 的 PHY 是固定的 GPIO49/50**，硬件必须接到这两个脚；接错（比如只接到 FS 或 Serial-JTAG 的脚）设备不枚举。
- **深度睡眠会丢失全部 USB 状态**：Host 唤醒后必须重新安装驱动并重新枚举；light sleep 下 Host 停止 SOF。
- **Hub 支持有局限**：无 Transaction Translator → HS Host 后面挂 Hub 时**不支持 FS/LS 低速设备**；远程唤醒等也不全。不要承诺"USB 分线器随便插"。
- `esp_tinyusb` v2 在 P4 OTG 上存在个别待修问题报告（如 VBUS 监测、某些组合的枚举）——**先跑官方例程验证，再决定版本**。

---

## 2. ① 接口形态待确认（决定下面选 Host 还是 Device）

| # | 问题 | 为什么关键 | 影响 |
|---|---|---|---|
| Q1 | USB 口的物理形态？**Type-C（可 OTG）/ Type-A（只能 Host）/ Type-C 仅 Device** | 决定方向 H（Host）还是 D（Device） | 若 Type-A：砍掉全部 D 系列；若仅 Device：砍掉全部 H 系列 |
| Q2 | 接到的是 **OTG_HS（GPIO49/50）** 还是 FS 路重映射？ | HS 才能跑满 480M 与 UVC/MSC | 接 FS 路则带宽受限，MSC/UVC 体验差 |
| Q3 | **VBUS 5V** 从哪来、能否由 MCU 控制（Host 要主动供 5V）？ID / 角色检测怎么接？ | Host 必须能控 VBUS；自动切角色需要 ID 检测 | 无 VBUS 控制则只能固定一种角色 |
| Q4 | 它和**烧录/调试口是否是同一个口**？ | 若同口，插电脑时既是 Device 又要烧录 | 影响 D3/D5 类"上位机直连"方案 |

> 由于当前**默认可行且无冲突的是 HS（49/50）**，下文按"HS + 可 Host"为主写；若 Q1/Q3 答案不同，按文末"决定矩阵"收缩范围。

---

## 3. 功能候选（按角色分组）

### 3.1 Host 模式（推荐主方向 —— 设备独立工作，往外接外设）

| # | 功能 | 描述 / 价值 | 复用与新增 | 工作量 | 优先级 |
|---|---|---|---|---|---|
| **H1** | **USB 键盘输入**（HID） | 大屏聊天/配网/输入 WiFi 密码/敏感词"打字说"，不开口也能用；免驱 | esp-usb `usb_host_hid`；键盘事件注入聊天输入框 | S–M | **P1** |
| **H2a** | **U 盘数据导出/导入**（MSC） | 把屏上截图、之后录的像/拍的照片、日志、设置备份拷到 U 盘；也支持反向灌入铃声/壁纸 | `usb_host_msc` + `esp_vfs_fat`（SD 已用同栈）；UI 上加"USB 已连接"卡片 | M | **P1** |
| **H2b** | **U 盘离线 OTA** | U 盘放固件包，本地升级，不依赖云端 | 需给 `main/ota.cc` 增加"从文件刷 esp_ota"分支（现为纯 HTTP） | M | P2 |
| **H2d** | **照片/录像一键拷走** | 摄像头（方向 A）就绪后，U 盘替代"拔 SD 卡" | 依赖 A0/A2 + H2a | M | P2（依赖 A） |
| **H3** | **USB 共享网络**（RNDIS/ECM） | 插手机（USB 网络共享）或 4G 模块 → 无 WiFi 也能连服务器；作 WiFi 的第二通道 | `iot_usbh_rndis` 已在依赖；参考 `rndis_board.cc`；需与现有 WiFi 网口做优先级切换 | M–L | P2 |
| **H4** | **USB UVC 摄像头** | 若 CSI 座没引出，USB 摄像头可当视觉输入替代/备份 | esp-usb `usb_host_uvc` | M | P2（仅 CSI 缺失时） |
| **H5** | **USB 鼠标**（HID） | 大屏光标操作/老年友好 | `usb_host_hid` | S | P3（可选） |

**推荐组合（MVP）**：**H1 键盘 + H2a U 盘导出/导入**。理由：① 覆盖"不适合语音"的输入场景；② U 盘让这台"会录像/拍照的智能屏"变成能离线取资料的设备（等方向 A 摄像头落地，H2d 顺理成章）；③ 纯软件+现成组件，不需要额外硬件，风险最低。

### 3.2 Device 模式（把设备插到 PC/手机上 —— 另一种产品姿态）

| # | 功能 | 描述 / 价值 | 复用与新增 | 工作量 | 优先级 |
|---|---|---|---|---|---|
| **D2** | **USB 免驱麦克风+音箱（UAC）** | 插到电脑当"会议麦/音箱"；用 ES8389 的麦和喇叭。补不了蓝牙经典音频（C5 无 BR/EDR）时，USB 是接 PC 的替代通道 | `esp_tinyusb` UAC；PCM 走 USB 端点 ↔ I2S 编解码 | L | P2（需求明确再做） |
| **D1** | **USB 键盘/宏键盘 gadget** | 6.2 屏变身快捷指令/密码面板，插到电脑敲字 | `esp_tinyusb` HID | M | P3（场景待定） |
| **D4** | **USB 上网卡 gadget**（ECM/RNDIS） | PC 借 P4 的 WiFi 上网（反过来当 USB 网卡） | `esp_tinyusb` ECM/RNDIS | M | P3（场景待定） |
| **D3/D5** | 虚拟串口 CDC-ACM / DFU | 上位机直连/调试；若 HS 口与烧录口独立才有增量价值 | `esp_tinyusb` | S–M | P3（视 Q4） |

### 3.3 OTG 角色自动切换（可选）
- 若硬件提供 VBUS/ID 检测：同一接口"插电脑当 Device、平时当 Host"。DWC_OTG 软件可切角色，但要做 VBUS/ID 状态机 + 总线重新枚举，工程量 L。**默认不列入路线图**，除非 Q1/Q3 证明硬件天然支持。

---

## 4. 实现要点与代码锚点

- **依赖增补**（`main/idf_component.yml`）：
  - Host：`espressif/usb_host_hid`、`espressif/usb_host_msc`（UVC 按需 `espressif/usb_host_uvc`）；`espressif/iot_usbh_rndis` **已在依赖**。
  - Device：`espressif/esp_tinyusb`（选 P4 HS 支持的版本）。
- **参考先例**：
  - USB Host 初始化与事件回调写法 → `main/boards/common/rndis_board.cc`（`usbh_cdc_driver_install` → 驱动安装 → netif；事件用 event group 通知）。
  - 本项目 USB 相关代码目前**零引用**，属全新增量，改动可收敛在 `main/boards/p4c5/` 或 `main/boards/common/usb_host_helper.*`。
- **引脚 / PHY 选择**：HS 用 49/50；Kconfig 侧确认 OTG2.0 HS 端口被选中（P4 需在 menuconfig 选对外设/PHY，详见 esp_tinyusb 文档，勿误选 FS 或 Serial/JTAG）。
- **与 WiFi（C5/SDIO）不冲突**：USB 用独立 DWC_OTG 控制器，与 esp_hosted 的 SDIO 是不同外设；只需整体算内存/带宽预算。
- **与现有 UI 联动**（方向 B）：USB 插入/拔出事件 → 屏幕通知卡片（复用 `ShowNotification`）；Home 上加"USB 存储/键盘"状态。
- **睡眠策略**（若以后做低功耗）：进 light/deep sleep 前停 USB Host，唤醒后重装驱动重新枚举；USB 活跃时禁止进入对应睡眠。

---

## 5. 风险与注意事项

1. **HS 引脚固定 49/50** —— 先量原理图确认，焊错脚整条路白做。
2. **FS 路（26/27 默认）与本板 LCD/触摸冲突** —— 不要顺手用了 FS。
3. `esp_tinyusb` v2 在 P4 有零散问题报告 —— Device 系列先跑官方例程（`tusb_serial_device` 等）验证再集成。
4. **Hub 无 TT**：不做"USB 分线器 + 低速外设"的承诺。
5. **睡眠会丢 USB 状态**：涉及低功耗方向时按 §4 处理。
6. MSC/UVC 需要较大缓冲与吞吐，内存占用要并入总预算（PSRAM 前提）。

---

## 6. MVP 验收标准

- **H1**：插入 USB 键盘即用，能在聊天界面输入文字并作为一条用户消息发送；支持中文输入法（若需）。
- **H2a**：插入 U 盘 → 屏上出现"USB 存储"通知 → 可把近期截图/日志导出；UI 提供"安全弹出"，拔出不损坏文件。
- 若走 Device 路线（D2）：插 PC 免驱识别为音箱/麦克风，语音对话音频从 PC 走。

---

## 7. 决定矩阵（根据 Q1/Q3 收缩范围）

| Q1 形态 | Q3 VBUS 可控 | 可做范围 | 推荐 MVP |
|---|---|---|---|
| Type-C OTG | 是 | H 全系 + D 全系（可切角色） | H1 + H2a（再视需要加 D2） |
| Type-A（Host 口） | 是 | **H 全系**（无 D） | H1 + H2a |
| Type-C 仅 Device | 否 | **D 全系**（无 H） | D2 或 D1 |
| 未引出 / 仅烧录 | — | 基本无法用 OTG | 砍掉本方向，USB 仅用于调试 |
