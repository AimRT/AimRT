

<table style="width: 100%; color: gray; font-size: 14px;">
<tr>
<td style="text-align: left;">NOTE: following tests were done in Mar. 2025, which may not reflect latest status of the package.</td>
</tr>
</table>

# AimRT 0.10.0-cpp &nbsp;Performance Test Report

## Preface
AimRT's communication layer is implemented through plugins. Officially supported communication plugins include iceoryx, ROS2, Zenoh, Http, Grpc, Mqtt, etc., covering common edge and cloud communication scenarios. These plugins provide both `publish-subscribe` and `request-response` communication patterns to enable `intra-host` and `cross-host` multiprocess communication.

## Test Environment
- System Environment:
  - OS: 
  - CPU: 13th Gen Intel(R) Core(TM) i5-1350P
  - Linux 6.1.59-rt16

- Software Environment
  - AimRT Version: 0.10.0

## Test Items
Tests conducted using AimRT-cpp include:

### Intra-Host Performance Tests
- Channel Backend Tests
  - [Impact of Packet Size on Performance](#impact-of-packet-size-on-performance)
  - [Impact of Topic Quantity on Performance](#impact-of-topic-quantity-on-performance)
  - [Impact of Parallelism on Performance](#impact-of-parallelism-on-performance)
- RPC Backend Tests
  - [Packet Size Impact in Bench Mode](#packet-size-impact-in-bench-mode)
  - [Packet Size Impact in Fixed-Freq Mode](#packet-size-impact-in-fixed-freq-mode)
  - [Parallelism Impact in Bench Mode](#parallelism-impact-in-bench-mode)
  - [Parallelism Impact in Fixed-Freq Mode](#parallelism-impact-in-fixed-freq-mode)

### Cross-Machine Performance Tests
- Channel Backend Tests
  - [Impact of Packet Size on Performance](#impact-of-packet-size-on-performance-1)
  - [Impact of Topic Quantity on Performance](#impact-of-topic-quantity-on-performance-1)
  - [Impact of Parallelism on Performance](#impact-of-parallelism-on-performance-1)
- RPC Backend Tests
  - [Packet Size Impact in Bench Mode](#packet-size-impact-in-bench-mode-1)
  - [Packet Size Impact in Fixed-Freq Mode](#packet-size-impact-in-fixed-freq-mode-1)
  - [Parallelism Impact in Bench Mode](#parallelism-impact-in-bench-mode-1)
  - [Parallelism Impact in Fixed-Freq Mode](#parallelism-impact-in-fixed-freq-mode-1)

## Test Results

### Intra-Host Performance Tests (X86)

#### Channel Backend Performance
##### Impact of Packet Size on Performance
- Objective: Test intra-host cross-process Channel backend performance under different packet sizes
- Configuration:
  ```yaml
  channel_frequency: 1 kHz
  pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16, power-of-2 increments)
  topic_number: 1
  parallel_number: 1
  ```
- Results:  
![](./pic/local_chn_cpp_pkgsize.png)

##### Impact of Topic Quantity on Performance
- Objective: Test performance with varying topic quantities
- Configuration:
  ```yaml
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  topic_number: 1 ~ 10
  parallel_number: 1
  ```
- Results:  
![](./pic/local_chn_cpp_topicnumber.png)

##### Impact of Parallelism on Performance
- Objective: Test performance under different parallelism levels
- Configuration:
  ```yaml
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  topic_number: 1
  parallel_number: 1 ~ 10
  ```
- Results:  
![](./pic/local_chn_cpp_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/local_chn_data.csv)'.format(code_site_root_path_url) }}

#### RPC Backend Performance
##### Packet Size Impact in Bench Mode
- Objective: Test RPC performance under bench mode with varying packet sizes
- Configuration:
  ```yaml
  mode: bench
  channel_frequency: 1 kHz
  pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16)
  parallel_number: 1
  ```
- Results:  
![](./pic/local_rpc_cpp_bench_msgsize.png)

##### Packet Size Impact in Fixed-Freq Mode
- Objective: Test RPC performance in fixed-frequency mode
- Configuration:
  ```yaml
  mode: fixed-freq
  channel_frequency: 1 kHz
  pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16)
  parallel_number: 1
  ```
- Results:  
![](./pic/local_rpc_cpp_fixfreq_pkgsize.png)

##### Parallelism Impact in Bench Mode
- Objective: Test parallel performance in bench mode
- Configuration:
  ```yaml
  mode: bench
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  parallel_number: 1 ~ 10
  ```
- Results:  
![](./pic/local_rpc_cpp_bench_parallel.png)

##### Parallelism Impact in Fixed-Freq Mode
- Objective: Test parallel performance in fixed-frequency mode
- Configuration:
  ```yaml
  mode: fixed-freq
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  parallel_number: 1 ~ 10
  ```
- Results:  
![](./pic/local_rpc_cpp_fixfreq_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/local_rpc_data.csv)'.format(code_site_root_path_url) }}

### Cross-Machine Performance Tests

#### Channel Backend Performance
##### Impact of Packet Size on Performance
- Configuration:
  ```yaml
  channel_frequency: 1 kHz
  pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16)
  topic_number: 1
  parallel_number: 1
  ```
- Results:  
![](./pic/cross-machine_chn_cpp_pkgsize.png)

##### Impact of Topic Quantity on Performance
- Configuration:
  ```yaml
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  topic_number: 1 ~ 10
  parallel_number: 1
  ```
- Results:  
![](./pic/cross-machine_chn_cpp_topicnumber.png)

##### Impact of Parallelism on Performance
- Configuration:
  ```yaml
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  topic_number: 1
  parallel_number: 1 ~ 10
  ```
- Results:  
![](./pic/cross-machine_chn_cpp_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/cross-machine_chn_data.csv)'.format(code_site_root_path_url) }}

#### RPC Backend Performance
##### Packet Size Impact in Bench Mode
- Configuration:
  ```yaml
  mode: bench
  channel_frequency: 1 kHz
  pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16)
  parallel_number: 1
  ```
- Results:  
![](./pic/cross-machine_rpc_cpp_bench_msgsize.png)

##### Packet Size Impact in Fixed-Freq Mode
- Configuration:
  ```yaml
  mode: fixed-freq
  channel_frequency: 1 kHz
  pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16)
  parallel_number: 1
  ```
- Results:  
![](./pic/cross-machine_rpc_cpp_fixfreq_pkgsize.png)

##### Parallelism Impact in Bench Mode
- Configuration:
  ```yaml
  mode: bench
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  parallel_number: 1 ~ 10
  ```
- Results:  
![](./pic/cross-machine_rpc_cpp_bench_parallel.png)

##### Parallelism Impact in Fixed-Freq Mode
- Configuration:
  ```yaml
  mode: fixed-freq
  channel_frequency: 1 kHz
  pkg_size: 1024 B
  parallel_number: 1 ~ 10
  ```
- Results:  
![](./pic/cross-machine_rpc_cpp_fixfreq_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/cross-machine_rpc_data.csv)'.format(code_site_root_path_url) }}