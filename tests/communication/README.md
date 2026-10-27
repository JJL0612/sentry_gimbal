# 通信主机测试

用 C++17 主机编译器运行真实 `dvc_uart_link.cpp` 和 `tsk_communication.cpp`，测试 HAL 用桩实现。测试文件位于 `User_File` 外，避免被固件 CMake 的递归源文件列表收集。

从项目根目录运行（MinGW bin 目录需在 PATH，确保编译器和运行库可用）：

```powershell
g++ -std=c++17 -Wall -Wextra -Itests/communication/stubs -IUser_File tests/communication/test_communication.cpp User_File/2_Device/Communication/dvc_uart_link.cpp User_File/4_Task/tsk_communication.cpp -o tests/communication/test_communication.exe
./tests/communication/test_communication.exe
```

覆盖 CRC 标准向量、逐字节半帧、粘包、CRC 错误和长度错误后重同步、最大 payload、512 字节接收边界、半帧超时、队列溢出恢复/回绕、时钟回绕、DMA 忙时不覆盖发送缓冲区、默认仅USART1注册、UART7保留给VOFA，及显式启用导航时的USART1/UART7两路注册且不占用底盘USART10、导航解码与 NaN/非法使能拒绝、心跳不延长指令有效期、200ms 超时清零、导航反馈封包。

HAL 桩不能验证硬件引脚、DMA/IRQ 时序或对端协议实现，需要在两端按通信 README 联调。

默认构建测试导航关闭模式；为验证保留的导航功能，再增加编译参数 `-DCOMMUNICATION_NAVIGATION_ENABLE=1` 运行同一测试。此宏仅用于主机测试，不应在UART7已供VOFA使用的固件中开启。
