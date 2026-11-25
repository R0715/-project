# R08 灰度和超声波台架测试

本测试适用于亚博 YB-MSA01 **STM32F103C8T6** 核心板、GanV005 蓝色探头板配 **MSPM0L1306 红色辅助板**、以及一至三只 HC-SR04。使用现有 R08 引脚和回波中断程序，串口每 250 ms 输出八个灰度电平与左、正前、右三路距离。单独烧录 [sensor_test.hex](sensor_test.hex) 即可测试。电机、驱动板和动力电池都可以留在桌上。

## 无 PCB 接线

断电布线。用面包板或杜邦线把 **核心板 GND、传感器 GND、5 V 电源负极和 ST-LINK GND** 接在一起。核心板用 USB-C 数据线供电，同时通过板载 CH340 看串口。用独立的稳压 5 V 给各传感器供电，两边电源只共地，5 V 正极保持各自独立。通电时先给核心板供电，再打开传感器的 5 V。按模块丝印找针脚。

### GanV005 八路灰度

蓝板和红色辅助板按配套接口对插。红板的 **+5V 接传感器 5 V，GND 接公共地**。从红板数字输出端按下表接线，`KEY`、`Err`、`CLK`、`DAT` 留空。厂家给出的蓝板典型检测高度为 20 mm，可以从这个高度开始。 [感为资料](https://www.ganv.com.cn/No_MCU_grayscale_8ways.html)

| 红板数字输出 | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| STM32 核心板 | PB10 | PB11 | PB8 | PB9 | PB5 | PA5 | PA12 | PA15 |

R08 载板原本会给灰度信号做电平转换。台架上先让红板供电，在黑、白底下分别用万用表量各路对地电压。确认每路是 **0–3.3 V 数字输出**后按表直连。若高电平约为 **5 V**，在每条信号线上用下方的分压接法，或者使用供电为 3.3 V、输入耐 5 V 的单向电平转换器。量到 3.3 V 的信号可直接进 GPIO，套用分压会把高电平压低。 [STM32F103C8 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)

### HC-SR04 超声波

只有一只时先用**正前**那一列。每只的 `VCC` 接传感器 5 V，`GND` 接公共地。`Trig` 直接接 STM32，`Echo` 按 5 V 脉冲处理，经分压后进 STM32。 [HC-SR04 时序与供电资料](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)

| 模块位置 | Trig 直连 | Echo 分压后 |
| --- | --- | --- |
| 正前 F | PB14 | PB15 |
| 左 L | PB12 | PB13 |
| 右 R | PB0 | PB1 |

每只 Echo 各用一组电阻。用面包板把 **Echo → 1 kΩ → 分压节点 → 1.5 kΩ → GND** 串起来，STM32 输入接在 **分压节点**。5 V 输入会变成约 3.0 V。手头只有 1 kΩ 和 2 kΩ 时，可以用 2 kΩ 作上臂，1 kΩ 加 2 kΩ 串联作下臂。如果所购模块的 Echo 在 5 V 供电时经示波器确认只输出 3.3 V 高电平，该路可直连。如果没有示波器且分压后始终无回波，可用输入耐 5 V、输出为 3.3 V 的缓冲器检查，例如 SN74LVC245A。 [STM32F103C8 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf) · [SN74LVC245A 数据手册](https://www.ti.com/lit/ds/symlink/sn74lvc245a.pdf)

### ST-LINK V2 下载线

| ST-LINK V2 | 核心板 SWD 口 |
| --- | --- |
| SWDIO | SWDIO / PA13 |
| SWCLK | SWCLK / PA14 |
| GND | GND |

云舵下载器的 3.3 V / 5 V 供电脚留空，核心板继续用 USB-C 供电。核心板物理排针位置可对照[核心板引脚图](../../车架/修改之前的PCB方案/V12车架_R03载板_亚博核心板引脚图.png)和[R08 引脚图](../../车架/最终方案/R08_ZCMBL_引脚图.pdf)。程序释放了 JTAG 的 PA15 给灰度 CH8，同时保留 SWD。

## 烧录和判定

1. 确保核心板 `BOOT0` 为 0。打开 STM32CubeProgrammer，选 **ST-LINK** 和 **SWD** 后点击 **Connect**。苹果芯片 Mac 可用官方 **STM32CubePrg-McA**。 [ST 下载页](https://www.st.com/en/development-tools/stm32cubeprog.html)
2. 在 **Erasing & Programming** 选择 `sensor_test.hex`，勾选 **Verify programming**，然后点 **Start Programming**。HEX 已带地址，从 `0x08000000` 写入。烧录会替换该区域内的原程序，之后可重新烧录整车固件。完成后断开调试会话，再按核心板复位键。 [ST 烧录手册](https://www.st.com/resource/en/user_manual/dm00403500-stm32cubeprogrammer-software-description-stmicroelectronics.pdf)
3. 串口助手打开核心板 CH340 串口，选 **115200、8N1、无流控**。Mac 可以先用 `ls /dev/cu.*` 找到插入 USB-C 数据线后出现的端口，再用 `screen /dev/cu.wchusbserialXXXX 115200` 查看，把端口替换成实机名称。按复位会看到启动提示，随后循环输出。 [亚博核心板资料](https://github.com/YahboomTechnology/STM32F103C8T6-Core-board)

```text
R08 sensor test | STM32F103C8 | 115200 8N1
GRAY CH1..CH8=11101111 | L=NO_ECHO F=298mm R=NO_ECHO
```

| 传感器 | 验收方法 |
| --- | --- |
| 八路灰度 | 按红板固件流程在同一高度校准黑白底。先放整片白纸，再放整片黑纸，八位应随之改变。随后用黑纸边缘从 CH1 扫到 CH8，确认八个位置都能独立变化。厂家公开版为白高黑低，因此默认全白接近 `11111111`，只有 CH4 见黑时为 `11101111`。以实测记录极性。 |
| 一只超声波 | 正对平整硬板，用尺子量探头前沿到挡板的距离。先放在约 20 cm，再移到约 40 cm，确认 F 从约 200 mm 增至约 400 mm。只接正前时 L、R 显示 `NO_ECHO` 属于预期。 |
| 三只超声波 | 断电补齐 L、R，再分别挡住左、前、右，核对 L、F、R 三个读数与实际探头对应。固定距离读数稳定，移动挡板时相应读数随之变化，即可通过台架检查。 |

`WAIT` 表示首次测量尚未返回，`NO_ECHO` 表示无回波、回波超时或 Echo 线卡高，`>4000mm` 表示收到了超出模块标称 4 m 范围的脉冲。判断模块是否通过时用 20–40 cm 的挡板。若灰度始终全 0 或全 1，先核对红板电源、校准状态、探头高度和公共地。

源码在同一目录。Mac 上安装 Xcode Command Line Tools 和 Python 3 后，在本目录运行 `make` 可以重新生成 HEX。测试程序复用 `line_follow_pack_r03()`、R08 超声波采样和回波中断。构建时用测试适配层承接距离数据，电机和编码器接口无需参与。
