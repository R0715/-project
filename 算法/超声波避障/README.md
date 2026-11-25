# 三超声波避障与终点停车

本模块按 STM32F103C8T6、R08 载板、三只 HC-SR04 和 GanV005 八路灰度编写。避障程序计算左右轮速度目标，电机适配层用编码器闭环换算成 [团队电机接口](../motor.h) 的 PWM 千分比。

| 文件 | 用途 |
| --- | --- |
| `car_control.c` | 循迹时的减速、绕障、找线、终点判定和停机状态机 |
| `car_drive.c` | 调用 `Encoder_GetCounts()`、`Motor_SetPWM()` 和 `Motor_Stop()` |
| `r08_avoidance.c` | R08 三路超声波、EXTI 回波计时与可选的八路灰度读取 |
| `tests/test_avoidance.c` | 用模拟回波、编码器和电机接口验证状态转换 |

## 算法

1. 前、左、前、右依次触发，每槽间隔 45 ms。前方每 90 ms 更新，左右各每 180 ms 更新。Trig 高电平约 12 µs，Echo 由双边沿中断测宽。
2. 前方 530 mm 或侧方 325 mm 以内限速。前方 350 mm 或侧方 230 mm 连续两次检测到障碍时，先停车，再从空间较大的一侧绕行。任一路一次测得不超过 90 mm 时立即请求停机；两侧都没有足够空间时保持停车。
3. 绕障用左右轮编码器的差值估计转角。先转出约 42°，斜向前移，再转回原航向并通过障碍。通过距离由触发距离、车身、障碍尺寸和余量估算。随后朝原线路转回约 32°，遇到连续稳定的窄黑线后恢复循迹。每段都有超时，回线还有行程上限。
4. 起点车头对齐横线时，灰度阵列可能在线后。此时用最多 65 mm 的低速前探找线。驶离起点并达到终点启用里程后，检测同时或先后覆盖两侧的宽黑带。确认终点后锁定当前航向，以 80 mm/s 前移设定距离，再调用 `Motor_Stop()` 并锁存停车。比赛规则要求车头过线、停车时车尾仍未越过。终点位置由灰度阵列判定，计时光电门只用于比赛计时。
5. 任一路回波异常或数据过期时先停车，持续异常会锁存停机。绕障超时、车轮未按命令方向移动、持续丢线或控制循环掉拍时也会锁存停机。复位或重新初始化可开始下一轮。

## R08 接线与工程接入

| 接口 | Trig | Echo 经 U2 接入 |
| --- | --- | --- |
| J1 左 | PB12 | PB13 / EXTI13 |
| J2 前 | PB14 | PB15 / EXTI15 |
| J3 右 | PB0 | PB1 / EXTI1 |

J1–J3 针序为 5 V、Trig、Echo、GND。J4 的 CH1–CH8 对应 PB10、PB11、PB8、PB9、PB5、PA5、PA12、PA15。可选的 `r08_gray_init()` 释放 PA15 的 JTAG 功能并保留 SWD。见 [R08 载板引脚图](../../车架/最终方案/R08_ZCMBL_引脚图.pdf)。

在 STM32CubeIDE、Keil 或已有 STM32F1 工程里加入前三个 `.c` 文件及同目录的头文件，保留 `../motor.h` 的相对位置。工程需要 MCU 对应的 CMSIS 头文件（Cube 通常已添加，器件宏为 `STM32F103xB`）和小组提供的电机实现。初始化放在 `SystemClock_Config()`、其他外设和电机初始化之后，运行期间维持系统主频与 `SystemCoreClock` 一致。

```c
/* main.c */
#include "r08_avoidance.h"
#include "motor.h"

/* 在主循环前的用户代码区 */
CarDriveConfig wheels = CAR_DEFAULT_DRIVE_CONFIG;
/* 台架测试先保留 0。测出实际数值后，左右轮分别填入。 */
/* wheels.left_counts_per_turn = 1040; */
/* wheels.right_counts_per_turn = 1040; */
Motor_Init();
r08_gray_init();
if (!r08_avoidance_init(&CAR_DEFAULT_CONFIG, &wheels, HAL_GetTick())) {
    Error_Handler();
}

/* 在主循环内，建议每 1 ms 或更快调用。 */
r08_avoidance_poll(HAL_GetTick(), r08_gray_mask(true, true));
```

`r08_gray_mask(true, true)` 表示黑色输出高电平、CH1 物理位置最左。用实物确认两个参数。已有灰度读取时，可直接传入其位掩码，约定 bit 0 对应车左、黑色为 1。主循环由本模块统一下发电机命令。超声波轮询只在发 Trig 时短暂忙等约 12 µs，测回波期间可继续执行主循环。

在 `stm32f1xx_it.c` 引入 `r08_avoidance.h`，把下面的调用放到对应中断函数的用户代码区。如工程里还没有对应函数，再添加函数。共享中断其余引脚仍由原处理代码负责。

```c
void EXTI1_IRQHandler(void)
{
    r08_avoidance_echo_irq(1u << 1);
}

void EXTI15_10_IRQHandler(void)
{
    r08_avoidance_echo_irq((1u << 13) | (1u << 15));
    /* 在这里保留其他 EXTI10–15 引脚的处理。 */
}
```

## 台架和实车标定

| 验证项 | 做法 |
| --- | --- |
| 测距与接线 | 保留左右每圈计数为 0，电机保持停止。在调试器看 `r08_avoidance_status()->controller.range`，用卡纸依次遮挡左、中、右探头。有效短脉冲为 `RANGE_ECHO`，有完整脉冲但距离超过 4 m 为 `RANGE_CLEAR`，无上升沿、Echo 卡高或超时为 `RANGE_FAULT`。开阔场地也要测一次，确认所购 HC-SR04 的超量程回波行为。 |
| 电机和编码器 | 车轮架空确认左右轮正命令都对应车身前进，累计计数也都递增。每只轮转一圈，填写对应的实际计数。13 ppr、20:1、四倍频对应约 1040 计数每圈，仅供在确认计数模式时对照。轮周长初值 151 mm，轮距初值 161 mm，按负载下的行程和实车尺寸校正。 |
| 绕行 | 先低速用 60 × 60 mm 和 150 × 100 mm 两种尺寸试左侧、右侧和正前方。V17 图示侧探头朝外约 30°，前探头朝正前。检查车轮外侧和车尾的扫掠余量。横移不足调 `diagonal_mm`，过早回线调 `clearance_run_mm`，触发偏晚按实测先降低 `cruise_mm_s` 或调大探测阈值。 |
| 停车 | 先用比赛同宽的 20 mm 横线试车。调 `finish_min_mm`，使判定在抵达终点以前启用。若停车时车头未过线，增大 `finish_roll_mm`；若车尾越线，减小它并测实际制动距离。验证 `Motor_Stop()` 在 R08 上的实际制动方式，锁存停车效果依赖该接口。|

固定超声波只能采到探头覆盖的区域，车尾和大角度回转侧面有盲区。台架验证完成后还需在实际赛道确认弯中遇障、障碍物紧贴边界和斜过终点横线时的行为。调试时可查看 `r08_avoidance_status()->controller.state` 和 `stop_reason`。

## 主机回归测试

在当前目录运行以下命令。测试文件内含模拟电机函数，仅加入主机测试目标。

```powershell
gcc -std=c11 -Wall -Wextra -Wconversion -Wpedantic -Werror car_control.c car_drive.c tests/test_avoidance.c -o test_avoidance.exe
.\test_avoidance.exe
```

资料见 [比赛规则](../../附件1%20自动循迹避障小车比赛规则与评分标准.pdf)、[V17 车架装配图](../../车架/车架装配.png) 和 [HC-SR04 时序](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)。
