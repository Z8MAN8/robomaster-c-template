# RM_Drives 说明文档。

RM_Drives 为针对单片机片上（On Chip）外设的封装。

RM_Drives 应该提供几种接口。包括初始化接口，一般命名为`XXXRegister()` ( 对于只有一个object的可以叫`XXXInit()`,但建议统一风格都叫register )。

| 外设名称 | 基于 HAL 支持情况 | 基于 LL 支持情况 |
| -------- | ----------------- | ---------------- |
| GPIO     |                   |                  |
| UART     |                   |                  |
| TIM      |                   |                  |
| PWM      |                   |                  |
| I2C      |                   |                  |
| SPI      |                   |                  |
| CAN      |                   |                  |
| USB      |                   |                  |
| ADC      |                   |                  |
| DWT      |                   |                  |

## TODO：

- 考虑移植跃鹿战队，bsp_tools 中通过一个任务统一管理回调函数的实现；
- 各模块的统一初始化；
- 贡献维护指南及示例；