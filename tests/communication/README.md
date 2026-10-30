# 通信主机测试

从云台项目根目录运行：

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/communication/stubs -IUser_File \
  tests/communication/test_communication.cpp \
  User_File/2_Device/Communication/dvc_uart_link.cpp \
  User_File/4_Task/tsk_communication.cpp -o /tmp/rm_communication_test
/tmp/rm_communication_test
```

运行真实通用协议库及当前 板载 USART3 插座（MCU USART1/huart1）导航任务，HAL 使用桩。

覆盖旧通用协议 CRC、队列、拆包和 DMA 发送保护；当前导航固定十六进制向量、负数及缩放、逐字节拆包、连续多帧、错误长度/校验/噪声重同步、半帧间隔超时、200ms 有效期边界、前台延迟不刷新时效、环形队列回绕与溢出恢复、512 字节 DMA 接收边界、毫秒计数回绕、发送饱和与非有限值拒绝、DMA 忙时缓冲保护、中档导航/上档遥控/遥控离线及导航超时清零。确认导航仅注册 huart1，huart3/UART7/USART10 不被导航占用；导航口无旧通用心跳。

HAL 桩不验证硬件引脚、实际波特率、DMA/IRQ 时序；需上板与小电脑联调。
