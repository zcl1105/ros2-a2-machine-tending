# 真实 ROS 2 验收记录

这些 JSON 与 CSV 来自 GitHub Ubuntu 22.04 / ROS 2 Humble 运行，不是手工构造的示例结果。

- 被测源码提交：`cbb25e5014f5b0a61ddf86b73198c5ddbd5a060a`
- [完整 CI 运行](https://github.com/zcl1105/ros2-a2-machine-tending/actions/runs/37566720357)
- 配置：`motion_seconds=0.7`、`process_seconds=1.5`；异常条件见 `scripts/integration.py`。
- 两个包编译通过，20 项核心与模型测试通过，五场景 ROS 验收通过，终止后复位检查通过。

| 场景 | 实际终止状态 | 经过时间 / 秒 | 完成数 |
| --- | --- | --- | --- |
| normal | COMPLETE | 15.249 | 1 |
| busy | COMPLETE | 19.199 | 1 |
| timeout | FAILED，WAIT_PROCESS 超时 | 10.049 | 0 |
| station_timeout | FAILED，WAIT_READY 超时 | 2.049 | 0 |
| cancel | CANCELED | 3.102 | 0 |

时间是单次实际运行记录，不代表硬实时性能或统计均值。timeout 总时长包括先前取料、上料与退避，不仅是 3 秒加工等待。默认演示参数更慢，与这里的快速验收参数不同。
