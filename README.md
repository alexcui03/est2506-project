# EST2501 Course Project

## 串口命令
UART串口设置为波特率115200，8位数据，0位校验，1位停止位。

串口命令格式规定：
- 串口命令不区分大小写，即`INIT CLOCK`与`Init cLOck`被视为同一条指令
- 在输入命令中连续的多个空格被视为一个空格，即`GET DATE`与`GET   DATE`被视为同一条指令
- 命令的开头和结尾不能有冗余的空格，即`  GET DATE`与`GET DATE  `是不合法的指令
- 输入均为半角字符
- 每一个指令的输入和返回均以CRLF换行结束

### INIT
**INIT CLOCK**：初始化时钟

### SET
**SET DATE <YYYY/MM/DD>**：将日期设置为YYYY/MM/DD

**SET TIME <HH:MM:SS>**：将时间设置为HH:MM:SS

**SET ALARM <HH:MM:SS>**：设置闹铃时间为HH::MM:SS

### GET
**GET DATE**：获取当前日期

**GET TIME**：获取当前时间

**GET ALARM**：获取闹铃时间

### ?
EST2506 课程大作业 指令帮助
UART串口波特率115200，数据帧8+0+1
    CLOCK INIT          - 初始化时钟到默认状态，包括时间、日期、闹铃
    CLOCK RESTART       - 重新启动时钟
    CLOCK HIB           - 将处理器切入休眠状态
    GET DATE            - 获取当前日期
    GET TIME            - 获取当前时间
    GET ALARM           - 获取闹铃时间
    SET DATE <DATE>     - 设置当前日期，<DATE>为YYYY/MM/DD格式
    SET TIME <TIME>     - 设置当前时间，<TIME>为HH:MM:SS格式
    SET ALARM <TIME>    - 设置闹铃时间，<TIME>为HH:MM:SS格式
    MUTE                - 关闭正在响铃的闹钟
示例：
    SET DATE 2024/06/18
    SET ALARM 13:00:50

