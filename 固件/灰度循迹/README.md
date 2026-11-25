# R08 八路灰度循迹

这部分运行在 STM32F103C8T6 上。GanV005 蓝板和 MSPM0L1306 红色辅助板负责把黑白反射差异变成八路高低电平。STM32 读取这些电平，算出黑线偏左还是偏右，再给出左右轮的前进请求。

## 接到 STM32 工程

1. 把 `line_follow.c` 加入工程源文件，并让工程能找到 `line_follow.h`。两份文件可分别放到 Cube 工程的 `Core/Src`、`Core/Inc`。
2. 按 R08 的 J4 配置 GPIO 输入。J4 前两脚为 GND 和 5 V，红板八路信号经过载板 U1 才到 STM32。R08 延续 R03 的八路脚序，因此可直接调用原来的 `line_follow_pack_r03()`。GPIO 设为普通输入 `GPIO_Input`，使用 `No pull-up and no pull-down`。CubeMX 下 `SYS > Debug` 选 `Serial Wire`，这样 CH8 的 PA15 可以用作普通输入，ST-LINK 仍可通过 PA13、PA14 调试。

| J4 脚号 | 红板信号 | STM32 引脚 |
| --- | --- | --- |
| 3 | CH1 | PB10 |
| 4 | CH2 | PB11 |
| 5 | CH3 | PB8 |
| 6 | CH4 | PB9 |
| 7 | CH5 | PB5 |
| 8 | CH6 | PA5 |
| 9 | CH7 | PA12 |
| 10 | CH8 | PA15 |

红板的 `KEY`、`Err`、`CLK`、`DAT` 留空。PA12 用作灰度输入，Cube 工程为它保留 GPIO 配置。整车调试通过 ST-LINK 接核心板的 SWD 下载口。 [R08 引脚图](https://github.com/R0715/-project/blob/903bba4d0aa49112a4bd8c55ab92ff5b9a667268/%E8%BD%A6%E6%9E%B6/%E6%9C%80%E7%BB%88%E6%96%B9%E6%A1%88/R08_ZCMBL_%E5%BC%95%E8%84%9A%E5%9B%BE.pdf)

3. 在 Cube 工程的 `main.c` 用户代码区引入 `line_follow.h`。下例每 10 ms 更新一次，可以先只观察变量。第二个参数 `true` 表示 CH1 在车身左侧，左右按车辆行驶方向判断。装车后逐路验证，若 CH1 在车身右侧，把它改为 `false`。厂家公开的红板固件输出白色高、黑色低，实测若相反再把第一个参数改为 `true`。 [感为辅助板资料](https://www.ganv.com.cn/No_MCU_grayscale_8ways.html)

```c
/* USER CODE BEGIN Includes */
#include "line_follow.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static LineFollowConfig line_config;
static LineFollowState line_state;
static uint32_t line_last_sample_ms;
volatile uint8_t gray_raw_mask;
volatile LineFollowResult gray_result;
/* USER CODE END PV */

/* 放在 GPIO 初始化之后的 USER CODE BEGIN 2 区。 */
line_config = line_follow_default_config(false, true);
line_follow_reset(&line_state);
line_last_sample_ms = HAL_GetTick();

/* 放在主循环的用户代码区，每圈都调用，不加 HAL_Delay。 */
uint32_t now_ms = HAL_GetTick();
if (now_ms - line_last_sample_ms >= 10u) {
    line_last_sample_ms = now_ms;
    const uint16_t porta = (uint16_t)GPIOA->IDR;
    const uint16_t portb = (uint16_t)GPIOB->IDR;
    const uint8_t raw = line_follow_pack_r03(porta, portb);
    gray_raw_mask = raw;
    gray_result = line_follow_step(&line_state, &line_config, raw, now_ms);
}
```

## 给电机和避障同学的接口

整车工程加上 `line_follow_motor.c` 和 `line_follow_motor.h`，并让编译器能找到 `算法/motor.h` 和电机驱动实现。`line_follow_apply_motor()` 将循迹的 `0–100%` 前进请求换算为电机接口的千分比，例如 `24%` 传 `240`。正常循迹或低速找线时调用 `Motor_SetPWM(left, right)`。丢线超时、读数无法判定或两轮请求均为 0 时调用 `Motor_Stop()`。电机初始化由整车主程序在启动时调用一次 `Motor_Init()`。[小组电机接口](../../算法/README.md)

整车主程序每圈先处理避障和停车，再决定是否把电机控制权交给循迹。联调时引入 `line_follow_motor.h`，用下面的逻辑替换前面示例的定时采样段。`line_follow_has_control` 由整车主程序根据当前模式和避障结果给出。避障或停车接管时，下面的代码只重置循迹状态。控制权恢复后用新采样的结果驱动电机。

```c
uint32_t now_ms = HAL_GetTick();
if (!line_follow_has_control) {
    line_follow_reset(&line_state);
} else if (now_ms - line_last_sample_ms >= 10u) {
    line_last_sample_ms = now_ms;
    const uint16_t porta = (uint16_t)GPIOA->IDR;
    const uint16_t portb = (uint16_t)GPIOB->IDR;
    const uint8_t raw = line_follow_pack_r03(porta, portb);
    const LineFollowResult result = line_follow_step(&line_state, &line_config, raw, now_ms);
    gray_raw_mask = raw;
    gray_result = result;
    line_follow_apply_motor(&result);
}
```

| `status` | 灰度模块的行为 |
| --- | --- |
| `LINE_FOLLOW_TRACKING` | 用 1～3 个连续命中探头的平均位置估计黑线偏差。黑线偏右时左轮请求更大，右轮更小。`error_mm` 在此状态有效，正值代表线在车右。 |
| `LINE_FOLLOW_SEARCHING` | 刚丢线且上次知道偏向哪边，按上一次偏向低速找线。默认从最后一次见线起最多 120 ms。 |
| `LINE_FOLLOW_LOST` | 没有可用的寻线方向，或到达寻线时限。左右轮请求为 0。 |
| `LINE_FOLLOW_AMBIGUOUS` | 连续 4 路以上或有多段命中。基础联调时输出 0，把原始判定留给上层。起点和终点都有横线，整车需结合当前赛段和行驶状态判定。 |

## 上车时怎么验证

1. 先架空轮子或断开电机动力，只观察灰度。红板如果还未校准，按配套固件的流程，在最终安装高度用整片黑底和白底做校准。厂家公开版的红板按键流程是长按约 4 秒进入，松手后先放黑底短按，等指示灯再次慢闪后换白底短按。有过曝提示时先调整光线或安装位置。 [厂家资料与源码下载](https://www.ganv.com.cn/No_MCU_grayscale_8ways.html)
2. 在 CubeIDE 的 Expressions 或 Live Expressions 里看 `gray_raw_mask` 和 `gray_result.black_mask`。逐个探头扫过黑白边界，低电平见黑时全白的原始值应接近 `0xFF`。归一后的 `black_mask` 全白为 `0x00`，只有 CH1 见黑为 `0x01`，只有 CH8 见黑为 `0x80`。先核对八路接线、极性和车身左右。
3. 先在纵向的 20 mm 黑线上测试，避开起点横线。分别检查居中、偏左、偏右、瞬间离线和持续离线。居中时左右轮建议相同。黑线在车右时左轮建议更大。确认实物的左右轮方向、停车指令和超声优先级后，再低速落地。
4. 初始参数为 `base_percent=24`、`max_percent=36`、`kp=0.45`、`kd=0`。从低速和比例控制开始，实测两轮能平稳起转后再调。车子来回摆动时优先降速或减小 `kp`，回线太慢再适当加大。`kd` 留待跑起来后补调。

烧录和调试用 STM32CubeIDE 配 ST-LINK V2。ST 官方安装包在 Apple Silicon Mac 上选 `STM32CubeIDE-McA`。使用 CubeIDE 2.x 新建并配置引脚时，STM32CubeMX 单独运行。 [STM32CubeIDE 安装](https://dev.st.com/stm32cube-docs/stm32cubeide/2.3.0/en/docs/markup/Installation_Guide/Installation_Guide_MacInstal.html) · [ST 的 CubeMX 工作流](https://community.st.com/stm32-software-tools-165/what-s-new-in-stm32cubeide-2-0-0-159478)
