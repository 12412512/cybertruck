# 软件测试

当前`test_readonly.py`覆盖协议互通、坏帧恢复、会话/保活/复位、STOP保留应答、核心独占、Web只读性、双浏览器推送及快照失效。`native.c`是主机测试桥，链接正式`frame.c`和`comm.c`，不链接HAL或设备。

安装`tests/requirements.txt`后，在根目录执行`tools/test.ps1`，或`.venv/Scripts/python.exe -B -m unittest discover -s tests -v`。Zig构建原生C库到`output/tests/`；这项验证不是板端UART、电机停止或实物验收。

正式实现可以被测试引用，正式程序不得从本目录导入运行实现。实物测试摘要归 `docs/`，原始日志和录像归 Git 忽略的 `output/`。
