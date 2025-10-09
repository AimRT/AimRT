<table style="width: 100%; color: gray; font-size: 14px;">
<tr>
<td style="text-align: left;">NOTE: following tests were done in Mar. 2025, which may not reflect latest status of the package.</td>
</tr>
</table>

# AimRT 0.10.0-cpp &nbsp;Performance Test Report


## Preface
The communication layer of AimRT is implemented via plugins. Officially supported communication plugins include iceoryx, ROS2, Zenoh, Http, Grpc, Mqtt, etc., covering common edge and cloud communication scenarios. These plugins provide the two common communication modes of `publish-subscribe` and `request-response` to enable inter-process communication both `locally` and `across machines`.



## Test Environment
- System Environment:
  - Operating System:
  - CPU: 13th Gen Intel(R) Core(TM) i5-1350P
  - Linux 6.1.59-rt16

- Software Environment
  - AimRT Version: 0.10.0


## Test Items

Tests were conducted using AimRT-cpp; the test items are as follows:
- Single-machine performance test
  - Channel backend performance test
    - Impact of packet size on performance
    - Impact of topic count on performance
    - Impact of concurrency on performance
  - Rpc backend performance test
    - Impact of packet size on performance in bench mode
    - Impact of packet size on performance in fixed_freq mode
    - Impact of concurrency on performance in bench mode
    - Impact of concurrency on performance in fixed_freq mode
- Multi-machine performance test
  - Channel backend performance test
    - Impact of packet size on performance
    - Impact of topic count on performance
    - Impact of concurrency on performance
  - Rpc backend performance test
    - Impact of packet size on performance in bench mode
    - Impact of packet size on performance in fixed_freq mode
    - Impact of concurrency on performance in bench mode
    - Impact of concurrency on performance in fixed_freq mode
  
## Test Results

### Single-machine Performance Test (X86)

#### Channel Backend Performance Test
##### Impact of Packet Size on Performance:
- Test Purpose: Performance test of single-machine cross-process Channel backend under different `packet sizes`
- Test Configuration:
  - channel_frequency: 1 kHz
  - pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16, increasing by powers of 2)
  - topic_number: 1 
  - parallel_number=1
- Test Results:
  
![](./pic/local_chn_cpp_pkgsize.png)

##### Impact of Topic Count on Performance:
- Test Purpose: Performance test of single-machine cross-process Channel backend under different `topic counts`
- Test Configuration:
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B
  - topic_number: 1 ~ 10 
  - parallel_number=1
- Test Results:

![](./pic/local_chn_cpp_topicnumber.png)


##### Impact of Concurrency on Performance:
- Test Purpose: Performance test of single-machine cross-process Channel backend under different `concurrency levels`
- Test Configuration:
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B
  - topic_number: 1 
  - parallel_number=1 ~ 10
- Test Results:
  
![](./pic/local_chn_cpp_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/local_chn_data.csv)'.format(code_site_root_path_url) }}

#### Rpc Backend Performance Test

##### Impact of Packet Size on Performance in Bench Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `packet sizes` in bench mode
- Test Configuration:
  - mode: bench
  - channel_frequency: 1 kHz
  - pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16, increasing by powers of 2)
  - paraller_number: 1
- Test Results:
  
![](./pic/local_rpc_cpp_bench_msgsize.png)

##### Impact of Packet Size on Performance in Fixed-Freq Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `packet sizes` in fixed-freq mode
- Test Configuration:
  - mode: fixed-freq
  - channel_frequency: 1 kHz
  - pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16, increasing by powers of 2)
  - paraller_number: 1
- Test Results:
  
![](./pic/local_rpc_cpp_fixfreq_pkgsize.png)

##### Impact of Concurrency on Performance in Bench Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `concurrency levels` in bench mode
- Test Configuration:
  - mode: bench
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B  
  - paraller_number: 1 ~ 10
- Test Results:

![](./pic/local_rpc_cpp_bench_parallel.png)

##### Impact of Concurrency on Performance in Fixed-Freq Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `concurrency levels` in fixed-freq mode
- Test Configuration:  
  - mode: fixed-freq
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B  
  - paraller_number: 1 ~ 10
- Test Results:

![](./pic/local_rpc_cpp_fixfreq_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/local_rpc_data.csv)'.format(code_site_root_path_url) }}

### Cross-machine Performance Test
#### Channel Backend Performance Test
##### Impact of Packet Size on Performance:
- Test Purpose: Performance test of single-machine cross-process Channel backend under different `packet sizes`
- Test Configuration:
  - channel_frequency: 1 kHz
  - pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16, increasing by powers of 2)
  - topic_number: 1 
  - parallel_number=1
- Test Results:

![](./pic/cross-machine_chn_cpp_pkgsize.png)

##### Impact of Topic Count on Performance:
- Test Purpose: Performance test of single-machine cross-process Channel backend under different `topic counts`
- Test Configuration:
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B
  - topic_number: 1 ~ 10 
  - parallel_number=1
- Test Results:

![](./pic/cross-machine_chn_cpp_topicnumber.png)


##### Impact of Concurrency on Performance:
- Test Purpose: Performance test of single-machine cross-process Channel backend under different `concurrency levels`
- Test Configuration:
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B
  - topic_number: 1 
  - parallel_number=1 ~ 10
- Test Results:
  
![](./pic/cross-machine_chn_cpp_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/cross-machine_chn_data.csv)'.format(code_site_root_path_url) }}

#### Rpc Backend Performance Test

##### Impact of Packet Size on Performance in Bench Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `packet sizes` in bench mode
- Test Configuration:
  - mode: bench
  - channel_frequency: 1 kHz
  - pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16, increasing by powers of 2)
  - paraller_number: 1
- Test Results:

![](./pic/cross-machine_rpc_cpp_bench_msgsize.png)

##### Impact of Packet Size on Performance in Fixed-Freq Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `packet sizes` in fixed-freq mode
- Test Configuration:
  - mode: fixed-freq
  - channel_frequency: 1 kHz
  - pkg_size: 256 B ~ 64 KB (2^8 ~ 2^16, increasing by powers of 2)
  - paraller_number: 1
- Test Results:

![](./pic/cross-machine_rpc_cpp_fixfreq_pkgsize.png)

##### Impact of Concurrency on Performance in Bench Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `concurrency levels` in bench mode
- Test Configuration:
  - mode: bench
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B  
  - paraller_number: 1 ~ 10
- Test Results:

![](./pic/cross-machine_rpc_cpp_bench_parallel.png)

##### Impact of Concurrency on Performance in Fixed-Freq Mode:
- Test Purpose: Performance test of single-machine cross-process Rpc backend under different `concurrency levels` in fixed-freq mode
- Test Configuration:  
  - mode: fixed-freq
  - channel_frequency: 1 kHz
  - pkg_size: 1024 B  
  - paraller_number: 1 ~ 10
- Test Results:

![](./pic/cross-machine_rpc_cpp_fixfreq_parallel.png)

{{ '[Detailed Data]({}/document/sphinx-cn/tutorials/misc/performance_test/0.10.0/cpp/data/cross-machine_rpc_data.csv)'.format(code_site_root_path_url) }}