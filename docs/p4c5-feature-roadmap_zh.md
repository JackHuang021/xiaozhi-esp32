# P4C5 板卡功能规划文档

- **适用范围**：`p4c5` 板卡（ESP32-P4 + 板载 ESP32-C5）及本 fork（分支 `p4c5_dev`）
- **撰写日期**：2026-09-04
- **文档目的**：把"语音聊天助手"升级为**类平板设备**——桌面启动器 + 多 App（语音助手/MP3/照相机/文件管理/USB 副屏），在此过程中把 ESP32-P4 的多媒体/算力能力用起来。本文件是功能清单 + 分阶段路线图；**产品形态与各 App 详规见 [p4c5-tablet-plan_zh.md](p4c5-tablet-plan_zh.md)**。

> 语言说明：本文件为本 fork 的私有规划文档，故用中文撰写，文件名遵循仓库 `_zh` 后缀约定。

---

## 1. 现状盘点

### 1.1 硬件

| 器件 | 说明 | 状态 |
|---|---|---|
| ESP32-P4 | 双核 RISC-V 400MHz（HP）+ LP 协处理器；无 WiFi/BT | 主控 |
| ESP32-C5 | 板载，经 SDIO 跑 `esp_hosted`/Wi-Fi Remote，提供 WiFi | 已启用（`config.json`） |
| AXS15260 屏 | 6.2" MIPI-DSI，物理 452×1280 竖屏；LVGL 90° 旋转后 1280×452 横屏，RGB888 | 已启用 |
| 触摸 | AXS15260 内建，I2C INT 事件模式 | 已启用 |
| ES8389 音频 | I2S0，单麦 + 喇叭（PA=GPIO31） | 已启用 |
| AXP2101 PMU | I2C1，电池/充电/LDO | 已启用 |
| MicroSD | SDMMC slot0（专用引脚 39–44），LDO4 供电 | 已启用 |
| ESP32-C5 SDIO | slot1（引脚 14–19/54），4-bit | 已启用 |
| **USB 2.0 OTG_HS** | 高速口（480Mbps，集成 PHY，D+/D- 在 GPIO49/50），可 Host / Device | **未启用**（接口在板上，代码零引用，详见 [p4c5-usb-plan_zh.md](p4c5-usb-plan_zh.md)） |
| 摄像头 | MIPI-CSI | **未启用**（无引脚定义，代码仅注释骨架） |

### 1.2 P4 能力使用情况

| P4 能力 | 是否被本板用到 |
|---|---|
| MIPI-DSI（一路） | ✅ 屏 |
| SDMMC（双口） | ✅ SD 卡 + C5（SDIO） |
| I2S/I2C/UART 等常规外设 | ✅ 部分 |
| **MIPI-CSI 摄像头** | ❌ 未用 |
| **PPA 像素处理加速器**（scale/blend/rotate 专用 DMA） | ❌ 未用 |
| **H.264 / MJPEG 硬件编码器** | ❌ 未用 |
| **LP 低功耗协处理器** | ❌ 未用 |
| 显示算力（60fps + 3 帧缓冲撕裂规避） | ✅ 屏已用满，UI 只用了很小一部分 |
| **USB 2.0 OTG_HS**（Host/Device，480Mbps） | ❌ 未用（主板已引出，代码未初始化） |
| 蓝牙（经 C5） | ❌ 未用（且 C5 仅支持 BLE，无经典蓝牙） |

### 1.3 结论

语音链路 + 屏幕 + 触控 + 存储这套"智能音箱/桌面屏"基座已经完整，且是**平板化的全部地基**。下面 A–D 是"能力方向"，但**主线已升级为平板（见方向 E）**：这些能力会以"App / 系统能力"的形式归位进去，而不是各自独立的 Home。

---

## 2. 功能评估维度

规划每条功能时统一用 4 个维度打分，再决定放哪个阶段：

1. **吃 P4 程度**：是否利用了别家 Wifi MCU 做不到的能力（CSI/AVC/PPA/高算力 UI）。
2. **用户价值**：对"语音助手 / 桌面智能屏"的体验提升。
3. **工作量**：分 S / M / L。
4. **硬件依赖**：无 / 需引出引脚 / 需新增器件（如摄像头模组）。

---

## 3. 功能池

### A. 摄像头 / 视觉（最能吃满 P4 的方向）

> 前置（阻塞项）：PCB 需预留 **MIPI-CSI 摄像头**接口（FPC 座 + I2C + 复位/掉电 GPIO），传感器建议选 `esp_video` 在 P4 上已驱动的型号，如 **OV5647 / SC2336**（Espressif Function-EV-Board 同款）。

**A0 — 拍照识物 / 视觉对话**　`优先级 P0`　`工作量 S`　`硬件依赖: CSI 座 + 摄像头`

- 功能：用户说"帮我看看这个是什么"，设备抓拍一张照片上传到视觉服务，用语音/屏幕返回描述。这是"语音助手 → 语音+视觉助手"的关键一跃。
- **框架层已全通，只差板级驱动**：
  - `self.camera.take_photo` 工具已注册于 `main/mcp_server.cc`（Capture + Explain）；
  - 视觉服务 URL 经 `Camera::SetExplainUrl()` 下发（`main/mcp_server.cc`）；
  - `EspVideo`（`main/boards/common/esp_video.h`）已是现成的 `Camera` 实现，走 `esp_video` V4L2 + JPEG 编码线程，且**支持 P4 的 CSI 路径**；
  - `esp_video` 组件依赖已含 `esp32p4`。
- 板级工作：
  1. `main/boards/p4c5/config.h` 补 `CAMERA_RESET_PIN` / `CAMERA_PWDN_PIN`（I2C 复用 codec 总线或触摸总线）；
  2. `main/boards/p4c5/esp32_p4_c5_board.cc` 取消 `InitializeCamera()`（L364 起）两处注释、`GetCamera()` 返回 `camera_`；
  3. menuconfig 打开对应 sensor 的 esp_video 驱动。
- 验收：语音触发"拍照"能在 3s 内得到视觉回答，画面不花屏。

**A1 — 屏上取景 / 实时预览**　`优先级 P1`　`工作量 M`　`硬件依赖: 同 A0`

- 功能：摄像头画面经 **PPA 缩放**后以低延迟画到屏幕窗口（取景、看护监控画面）。
- 技术路径：`esp_video` 取帧 → PPA 做 RGB/尺寸转换 → 画到 LVGL canvas。不要用 CPU 逐像素搬运。
- 过渡方案：先做"定时抽帧 JPEG → 显示"的低帧率版跑通，再上真预览。

**A2 — 本地录像 / 缩时摄影到 SD**　`优先级 P2`　`工作量 L`　`硬件依赖: 同 A0`

- 功能：用 P4 **硬件 H.264 编码器**做循环覆盖录像 / 缩时摄影 / 事件快照，全部本地存 SD，不占网络。SD 已挂载，天然适合。
- 简化版：定时用现有 `image_to_jpeg` 抓拍 JPEG 存 SD（缩时相册），先不碰 AVC。

### B. 桌面 UI / 交互体验（纯软件，无需新硬件）

**B1 — 待机 Home 桌面**　`优先级 P0`　`工作量 M`　`硬件依赖: 无`

- 现状问题：待机时 6.2" 屏只显示了状态栏里一行 `HH:MM`，大屏基本浪费。
- 功能：`kDeviceStateIdle` 时，在 `content_` 区域渲染一个**全屏 Home**：大字号时钟 + 日期/星期 + 天气卡片 + 快捷状态（音量/勿扰/网络/电量）；进入对话（Listening/Thinking/Speaking）自动切回聊天界面，回到 Idle 再恢复 Home。
- 代码锚点：
  - 显示类继承链：`Display` → `LvglDisplay` → `LcdDisplay` → `P4C5LcdDisplay`；
  - 界面布局与聊天区在 `main/display/lcd_display.cc`（`content_`、`chat_message_label_`、`hide_subtitle_`）；
  - 状态刷新由 `main/application.cc` 的 `MAIN_EVENT_CLOCK_TICK` 驱动 → `LvglDisplay::UpdateStatusBar()`，这是 Home 时钟/天气的刷新入口；
  - 设备状态集合见 `main/device_state.h`（`kDeviceStateIdle` 等）。
- 建议实现位置：先在 `P4C5LcdDisplay`（`main/boards/p4c5/`）内做 Home 容器，稳定后再决定是否上提到 `main/display/` 共享，保持对上游改动最小。

**B2 — 对话动效与表情**　`优先级 P1`　`工作量 M`　`硬件依赖: 无`

- 功能：听/想/说三态的声波动画 + 眼睛表情 + 消息出现过渡；利用 P4 算力把现有聊天体验做"活"。
- 复用：依赖已含 `esp_emote_expression`、LVGL 动画、emoji/gif/jpg 解码、动态字库。
- 注意：所有 LVGL 操作必须走 `DisplayLockGuard`（`esp_lvgl_adapter` 锁）。

**B3 — 触控多页面与息屏时钟（AOD）**　`优先级 P2`　`工作量 M`　`硬件依赖: 无`

- 功能：桌面 ↔ 聊天 ↔ 信息流用滑动翻页（LVGL 9.5 tileview）；待机低亮度常显大时钟而非全黑。
- 复用：`SetPowerSaveMode()` 钩子、背光 `PwmBacklight`。

### C. 蓝牙 / 外设

**C1 — BLE（含风险，先验证再投入）**　`优先级 P3`　`工作量 L`　`硬件依赖: 无`

- **重要事实**：ESP32-C5 **不支持经典蓝牙（BR/EDR）**，所以 A2DP 蓝牙音箱 / 放手机音乐这类**做不了**，不做规划。
- BLE 理论路径：esp_hosted 通过 VHCI 桥接 BLE（`CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE`）。
- **风险**：本板 C5 从固件来自 "Wi-Fi Remote" 组件（esp_hosted Kconfig 注释明确 "ESP32-C5 is Co-processor Target from Wi-Fi Remote Component"），大概率**只有 Wi-Fi RPC、不含 BT RPC**。若要用 BLE，需先确认/自行编译带 BT 的 C5 slave 固件。
- 结论：排到 P3，只做**验证性 POC**（能跑通 BLE 广播/GATT 再谈产品化），不要提前投入。
- 若 POC 通过，可做：手机 App 配网/改名/固件信息/遥控；BLE 通知；作为配网备选（仓库已有 blufi 方案可参考）。

**C2 — 外设扩展（选做，按引脚引出情况挑 1–2 个）**　`优先级 P1`　`工作量 S–M`　`硬件依赖: 需引出空闲引脚/I2C`

| 外设 | 用到的 P4 资源 | 建议场景 | 备注 |
|---|---|---|---|
| RGB LED / 灯带 | RMT（`led_strip` 已在依赖） | 状态呼吸灯、唤醒光效、音量联动 | 需 1 个空闲 GPIO |
| 温湿度传感器 | I2C（与触摸/PMU 共总线） | "房间几度"语音播报 + Home 显示 | 需板上或跳线外接 |
| 光感 | ADC / I2C | 自动背光 | — |
| IMU（BMI270） | I2C | 抬手亮屏、摇晃唤醒/切歌 | ⚠ 依赖 `bmi270_sensor` 的 target 规则**不含 esp32p4**，需改规则或自写驱动 |
| IR 发射 | RMT TX | 语音万能遥控空调/电视 | — |
| 震动马达 | GPIO + 定时器 | 唤醒/低电量触觉反馈 | — |

> 注意：外设落地前需先确认 p4c5 PCB 实际引出了哪些引脚 / I2C 地址冲突（触摸与 PMU 已占 I2C1，codec 占 I2C0）。

### D. USB 2.0（方向详文见 [p4c5-usb-plan_zh.md](p4c5-usb-plan_zh.md)）

- 硬件：ESP32-P4 **OTG_HS**（USB2.0，480Mbps，集成 PHY，GPIO49/50 无冲突）；能 **Host 也能 Device**。
- 主推 Host 三件套：**D-H1 USB 键盘输入**、**D-H2a U 盘数据导出/导入**（照片/录像/日志，离线取资料）；次选 **D-H3 手机/4G USB 共享网络**（`iot_usbh_rndis` 已有依赖与 `rndis_board.cc` 先例）。
- 副线 Device（接 PC）：**D-D2 免驱 USB 麦克风+音箱（UAC）**、键盘/网卡 gadget —— 属另一产品姿态，P2/P3。
- 已核实之坑：FS OTG 默认脚 26/27 与 LCD/触摸冲突勿用；HS 引脚固定 49/50；深度睡眠丢 USB 状态；Hub 无 TT（不支持挂低速设备）。
- **方向 D 依赖 PCB 接口形态**：Type-C 可 OTG / Type-A Host / 仅 Device，决定砍掉哪半（见详文 §2/§7）。

### E. 平板外壳 / 应用启动器（**新主线**，详文见 [p4c5-tablet-plan_zh.md](p4c5-tablet-plan_zh.md)）

> 已拍板：**横屏 1280×452** 平板 UI；**语音助手仅 App 内触发**（非全局唤醒）。

- **壳**：AppManager（生命周期/返回栈）+ **Launcher 桌面**（图标 + 待机时钟）+ 状态栏上移到 `lv_layer_top()` 全局常驻 + 回桌面手势。**纯软件，Phase 0 核心。**
- **App 清单**：`App-1 语音助手`(现有聊天 UI 改造，改"何时运行") · `App-2 MP3`(新增 Helix 解码 + AudioFocus) · `App-3 照相机`(取景+拍照存 DCIM，卡 CSI/UVC 硬件) · `App-4 文件管理`(SD/USB 浏览看图删除) · `App-5 USB 电脑副屏`(USB ECM/RNDIS gadget + PC 端小程序 + esp_new_jpeg 软解 MJPEG，研究性)。
- 既有方向 A–D 全部按 App/系统能力归位（详见平板详文 §5 映射表）。
- 两个硬件确认仍阻塞两条支线：**CSI 摄像头**（相机 App）、**USB 接口形态**（U 盘、副屏传输）。

---

## 4. 分阶段路线图（平板主线，详见 [p4c5-tablet-plan_zh.md](p4c5-tablet-plan_zh.md) §6）

| 阶段 | 内容 | 硬件依赖 | 出口标准 |
|---|---|---|---|
| **Phase 0 壳** | AppManager + Launcher + 状态栏 layer_top + **助手 App 化**（仅 App 内触发改造） | 无 | 开机进桌面；图标进出助手；退出停麦 |
| **Phase 1** | **App-4 文件管理**（浏览/看图/删除 + intent 骨架） | 无 | 可浏览/看图/删除 |
| **Phase 2** | **App-2 MP3**（Helix 解码 + AudioFocus，SD 曲库） | 无 | 选歌播放/暂停/切歌/续播 |
| **Phase 3** | USB 存储进文件管理（原 D-H2a）+ USB 键盘（原 D-H1，可选） | ①USB 为 Host/OTG | U 盘可读可拷 |
| **Phase 4** | **App-3 相机**（取景+拍照存 DCIM；A0 视觉对话入助手） | ①CSI 或 UVC 摄像头 | 取景/拍照/回看可用 |
| **Phase 5 研究** | **App-5 USB 副屏**（USB gadget→MJPEG 软解）· C1 BLE POC · D-H3 USB 共享网络 | ①USB 为 Device/OTG + PC 端程序 | 任一研究项 POC 跑通 |

> 里程碑建议：Phase 0 是其余一切的骨架，先把它跑稳；0–2 全部无新硬件可并行推进；旧方向 A0 视觉对话并入相机阶段、B1/B2/B3 融入壳与桌面、A2/C2 等按需排后。

---

## 5. 待确认项（阻塞决策的问题，需板主确认）

> 已拍板的决策（不必再确认）：平板 **横屏 1280×452**、**语音助手仅 App 内触发**（见 [p4c5-tablet-plan_zh.md](p4c5-tablet-plan_zh.md) §0）。

1. **PCB 是否预留 MIPI-CSI 摄像头接口**？传感器倾向 OV5647 / SC2336 哪个？（决定 App-3 相机走 CSI 还是 UVC）
2. 是否引出**空闲 GPIO / I2C 扩展**？板上已焊哪些器件（RGB LED、马达、光感、温湿度、IR）？（决定 C2 选哪几个）
3. 板上 **PSRAM** 是否确认存在？（当前按"有"规划，媒体解码/相机多帧缓冲依赖它）
4. **USB 2.0 接口形态**（详见 [p4c5-usb-plan_zh.md](p4c5-usb-plan_zh.md) §2/§7）：**Type-C（可 OTG）/ Type-A（仅 Host）/ 仅 Device**？VBUS 5V 是否可控？是否与烧录口同一 USB？—— 决定 U 盘/文件管理（Host）与 App-5 副屏（Device）能不能做。
5. **App-5 USB 电脑副屏**需要 **PC 端一个配套推流小程序**（非 OS 原生即插即用），是否接受并投入 PC 端开发？

---

## 6. 代码锚点速查

| 关注点 | 位置 |
|---|---|
| 摄像头基类 | `main/boards/common/camera.h` |
| P4/S3 摄像头实现（esp_video） | `main/boards/common/esp_video.h` `.cc` |
| 摄像头 MCP 工具（take_photo） | `main/mcp_server.cc`（约 L100/L337） |
| p4c5 摄像头注释骨架 | `main/boards/p4c5/esp32_p4_c5_board.cc`（L364/L470） |
| p4c5 引脚定义 | `main/boards/p4c5/config.h` |
| p4c5 显示适配 | `main/boards/p4c5/p4c5_lcd_display.h` `.cc` |
| 显示基类 / 状态栏时钟 / 通知 | `main/display/display.h`、`main/display/lvgl_display/lvgl_display.cc` |
| 聊天 UI 布局（content_ 等） | `main/display/lcd_display.h` `.cc` |
| 状态驱动（CLOCK_TICK → UpdateStatusBar） | `main/application.cc`（约 L270） |
| 设备状态枚举 | `main/device_state.h` |
| 背光 | `main/boards/p4c5/esp32_p4_c5_board.cc` `GetBacklight()` |
| 平板壳/App（建议新位置） | `main/boards/p4c5/` 下如 `apps/` + `app_manager.*`；规划见 [p4c5-tablet-plan_zh.md](p4c5-tablet-plan_zh.md) |

---

## 7. 风险与注意事项

- **esp_lvgl_adapter 锁**：本板 UI 走 `esp_lvgl_adapter`，任何 LVGL 调用都必须通过 `DisplayLockGuard` / `Lock()/Unlock()`，禁止在其它任务裸调 LVGL API。
- **RGB888 / LV_COLOR_DEPTH_24**：AXS15260 是 24bpp MIPI-DSI 屏，LVGL 必须以 `CONFIG_LV_COLOR_DEPTH_24=y` 编译（config.json 已强制），新增 UI 组件要注意颜色格式。
- **撕裂规避**：适配器用 3 帧部分刷新 + 90° 旋转，Home 若做大面积刷新要考虑刷新时机，避免闪屏。
- **内存**：摄像头多帧缓冲 + JPEG 编码需要 PSRAM；esp_hosted 内存池已配置为优先 SPIRAM，评估时要统一算内存账。
- **IDF 实验特性**：本板依赖 `CONFIG_IDF_EXPERIMENTAL_FEATURES=y`（esp_hosted/Wi-Fi Remote），升级 IDF 或组件版本时要回归验证 WiFi。
- **保持 fork 可合并性**：优先把改动收敛在 `main/boards/p4c5/`；确有必要改共享层（`main/display`、`main/mcp_server.cc`）时，尽量向后兼容，减少与上游冲突。
