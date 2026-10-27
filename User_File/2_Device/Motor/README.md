# 电机文件结构

每种电机的底层驱动、控制参数、闭环封装及实现集中在同名的 `.h` / `.cpp` 中。

| 电机 | 文件 | 保留的接口 |
| --- | --- | --- |
| DJI GM6020 | `Motor_DJI/dvc_motor_dji_6020.h/.cpp` | GM6020 驱动、位置闭环控制 |
| DJI M3508 / C620 | `Motor_DJI/dvc_motor_dji_3508.h/.cpp` | C620、M3508 驱动、M3508 速度闭环控制 |
| DJI M2006 / C610 | `Motor_DJI/dvc_motor_dji_2006.h/.cpp` | C610、M2006 驱动、M2006 速度闭环控制 |
| 达妙 DM | `Motor_DM/dvc_motor_dm.h/.cpp` | Normal 与 1-To-4 协议驱动、MIT 位置闭环控制 |

`Motor_DJI/dvc_motor_dji.h/.cpp` 存放 DJI 共用枚举、数据结构、CAN 发送缓冲区分配及功率估算，供各型号共用。

原 `dvc_motor_dji_ctrl.h/.cpp` 和 `dvc_motor_dm_ctrl.h/.cpp` 已合并进对应电机文件。任务层应直接包含对应型号的头文件。类名、函数签名、配置默认值和控制逻辑保持原样，C610/C620 接口也完整保留。

工程使用 CMake 递归收集 `User_File` 源文件。增删文件后需重新运行 CMake 配置，再编译工程。
