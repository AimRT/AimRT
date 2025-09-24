## pytest_aimrt 使用说明

本目录提供基于 pytest 的 AimRT 端到端示例测试框架与用例。

### 依赖与准备
- 测试框架环境准备： `pip install -r pytest_aimrt/requirements.txt`
- 已编译的 AimRT（建议开启 `AIMRT_BUILD_EXAMPLES` 及所需插件开关，如 `AIMRT_BUILD_ROS2_PLUGIN`、`AIMRT_BUILD_MQTT_PLUGIN`、`AIMRT_BUILD_NET_PLUGIN` 等）
- 构建产物目录中应包含示例脚本与配置，通常位于 `build/`（Linux）

### 运行方式

```bash
export CWD=build/
cd pytest_aimrt/
pytest ./test/
```

也可仅运行某一类用例：

```bash
# 仅运行 net_plugin 用例
pytest -k test_net_plugin_examples

# 仅运行某个具体 YAML
pytest -k plugins_net_pb_chn_http_bench
```

如需在任意目录运行，可在 YAML 的 `config.cwd` 中填写绝对路径到构建产物目录（包含示例脚本与 cfg 的位置），或设置为 `${CWD}` 并在外部导出环境变量 `CWD`。

### 目录结构速览
- `pytest_aimrt/core/`：核心执行与进程编排（`process_manager.py` 等）
- `pytest_aimrt/fixtures/`：pytest 插件/`AimRTTestRunner`）
- `pytest_aimrt/test/`：分插件与语言组织的测试用例与 YAML
  - `test/plugins/<plugin>/`：各插件用例与清单，例如 `net_plugin/`
  - `test/py/...`：Python 端到端示例用例
- `pytest_aimrt/test_reports/`：测试报告与临时运行数据输出目录

### YAML 测试规范
每个 YAML 定义一组进程脚本的编排与判定规则。常见字段如下：

- `name`：用例唯一名
- `description`：简要描述
- `config`
  - `execution_count`：重复执行次数
  - `time_sec`：总超时时间
  - `cwd`：工作目录，通常指向构建产物，支持使用环境变量 `${CWD}`
  - `environment`：为本用例设置的环境变量字典
  - `global_shutdown_patterns`：全局关键字匹配，命中即判定完成
- `input.scripts[]`：进程脚本编排列表（按序）
  - `path`：脚本相对/绝对路径（相对 `cwd`）
  - `args`：脚本参数数组
  - `depends_on`：依赖脚本路径数组（等待依赖启动后再启动）
  - `delay_sec`：启动前延迟秒数
  - `cwd`：单个脚本的工作目录（可覆盖全局）
  - `time_sec`：该脚本的超时时间
  - `monitor`：资源监控开关（`cpu`/`memory`/`disk`）
  - `propagate_shutdown`：命中本脚本关停条件后，是否向其它脚本广播关停
  - `shutdown_patterns`：命中即认为用例完成的关键日志或输出片段

示例（HTTP pb_chn 基准测试，subscriber->publisher）：

```yaml
name: "plugins_net_pb_chn_http_bench"
description: "NET plugin pb_chn over http benchmark (sub->pub)"

config:
  execution_count: 1
  time_sec: 60
  cwd: "${CWD}"

input:
  scripts:
    - path: "./start_examples_plugins_net_plugin_pb_chn_http_benchmark_sub.sh"
      args: []
      depends_on: []
      delay_sec: 0
      cwd: ""
      time_sec: 60
      monitor:
        cpu: true
        memory: true
        disk: true
      shutdown_patterns: ["Benchmark plan 2 completed"]

    - path: "./start_examples_plugins_net_plugin_pb_chn_http_benchmark_pub.sh"
      args: []
      depends_on: ["./start_examples_plugins_net_plugin_pb_chn_http_benchmark_sub.sh"]
      delay_sec: 3
      cwd: ""
      time_sec: 60
      monitor:
        cpu: true
        memory: true
        disk: true
      shutdown_patterns: ["Bench completed."]
```

### 远程主机执行（跨机编排）
支持将脚本分发到多台远程主机执行，通过 `hosts` 声明远程主机别名与连接信息，并在 `input.scripts[]` 中用 `remote` 指定脚本运行在哪个主机别名上；未指定 `remote` 的脚本默认本地执行。

- `hosts`：主机别名到连接信息的映射
  - `host`：远程 IP 或主机名，支持环境变量
  - `ssh_user`：SSH 用户名
  - `ssh_password`：SSH 密码（如使用密钥，可留空，具体以实现支持为准）
  - `ssh_port`：SSH 端口，默认 22
  - `remote_cwd`：远程工作目录（该目录应包含示例脚本与 cfg）
- `input.scripts[].remote`：脚本所在的主机别名（来自 `hosts` 的 key）
- `input.scripts[].environment`：为该脚本（对应主机）设置的环境变量

使用步骤（示例为 x86 与 Orin 两台主机）：
1) 在两台主机各自的 `remote_cwd` 下准备好 AimRT 构建产物与示例脚本
2) 在本地设置连接参数：
   ```bash
   export X86HOST=192.168.1.10 X86USER=ubuntu X86PASS=*** X86PORT=22 X86CWD=/path/to/remote/build
   export ORINHOST=192.168.1.20 ORINUSER=ubuntu ORINPASS=*** ORINPORT=22 ORINCWD=/path/to/remote/build
   ```
3) 运行 pytest：
   ```bash
   cd "$CWD"
   pytest -k chn-ros2-besteffort-cross -q
   ```

完整示例 YAML：

```yaml
name: "chn-ros2-besteffort-cross"
description: "Auto-generated benchmark test for examples_plugins_ros2_plugin_ros2_chn_besteffort_x86_2_orin_multi_topic_msg_size_512"

config:
  execution_count: 1
  time_sec: 60
  cwd: "${CWD}"
  environment:
    LOG_LEVEL: "info"

hosts:
  x86:
    host: "${X86HOST}"
    ssh_user: "${X86USER}"
    ssh_password: "${X86PASS}"
    ssh_port: "${X86PORT}"
    remote_cwd: "${X86CWD}"
  orin:
    host: "${ORINHOST}"
    ssh_user: "${ORINUSER}"
    ssh_password: "${ORINPASS}"
    ssh_port: "${ORINPORT}"
    remote_cwd: "${ORINCWD}"

input:
  scripts:
    - path: "./start_examples_plugins_ros2_plugin_ros2_chn_benchmark_sub_besteffort_x86-2-orin_multi-topic_msg_size_512.sh"
      args: []
      depends_on: []
      delay_sec: 0
      cwd: ""
      time_sec: 60
      monitor:
        cpu: true
        memory: true
        disk: true
      environment:
        ROLE: "server"
      remote: orin
      shutdown_patterns: ["Benchmark plan 0 completed"]

    - path: "./start_examples_plugins_ros2_plugin_ros2_chn_benchmark_pub_besteffort_x86-2-orin_multi-topic_msg_size_512.sh"
      args: []
      depends_on: ["./start_examples_plugins_ros2_plugin_ros2_chn_benchmark_sub_besteffort_x86-2-orin_multi-topic_msg_size_512.sh"]
      delay_sec: 3
      cwd: ""
      time_sec: 60
      monitor:
        cpu: true
        memory: true
        disk: true
      environment:
        ROLE: "client"
      remote: x86
      shutdown_patterns: ["Bench completed."]
```

注意事项：
- 远端需具备与本地一致的运行时依赖（例如 ROS2 环境、插件所需动态库等），可在 `environment` 中补充或在远端 profile 中预置
- 网络链路不可达或凭据错误会导致脚本启动失败

### 如何新增测试用例
1) 在对应目录新增 YAML（例如 `pytest_aimrt/test/plugins/net_plugin/`）：
   - 复制相近用例并修改脚本名/关停条件/超时时间即可
2) 将 YAML 文件名加入对应测试清单 Python 文件中的 `CASES`（例如 `test_plugins_net.py`）
3) 在构建产物 `bin` 目录验证脚本存在且可执行；必要时调整 `cwd`
4) 运行 `pytest -k <test_name>` 验证

### 结果与报告
- 测试执行日志与中间数据会输出到 `pytest_aimrt/test_reports/`
- html 目录可直观浏览各用例日志，json 目录便于自动化比对与结果提取
- 失败排查：
  - 确认 `cwd` 指向包含示例脚本与 cfg 的目录（通常是 `build/`）
  - 检查插件运行前置条件（ROS2 环境、MQTT Broker、端口占用等）



