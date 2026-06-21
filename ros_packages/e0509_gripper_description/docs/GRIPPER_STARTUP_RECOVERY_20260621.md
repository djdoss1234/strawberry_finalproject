# Gripper Startup Recovery - 2026-06-21

## 현재 증상

`dsr_gripper_tcp gripper_service_node` 실행 시 다음 패턴이 반복된다.

```text
Connected to gripper TCP bridge at 110.120.1.66:20002
INITIALIZE attempt N/M failed (gripper): Controller returned error status 3
INITIALIZE transport failed (TCP connection closed while receiving data.)
```

해석:

- DRL TCP server와 PC 사이 TCP 연결은 성공했다.
- 실패 지점은 DRL 내부에서 RH-P12-RN(A) 그리퍼와 플랜지 RS-485/Modbus 통신을
  초기화하는 단계다.
- `status 3`은 `STATUS_IO_ERROR`로, TCP 문제가 아니라 그리퍼 serial/Modbus 응답
  실패에 가깝다.

## 이번에 확인한 핵심

최신 `dsr_gripper_tcp/dsr_gripper_tcp/gripper_tcp_bridge.py` 기준으로
`BridgeConfig.stop_existing_drl` 기본값은 `False`이며, 코드 주석에 다음 취지의
내용이 있다.

```text
drl_stop corrupts controller socket state; never True
```

따라서 평소 재시작 루틴에서 다음 명령처럼 `stop_existing_drl:=true`를 매번 넣는 것은
오히려 컨트롤러 TCP/DRL 상태를 더 불안정하게 만들 수 있다.

```bash
ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 \
  namespace:=dsr01 \
  stop_existing_drl:=true \
  init_attempts:=3 \
  init_timeout_sec:=30.0
```

과거 문서에는 `stop_existing_drl:=true` 성공 사례가 남아 있지만, 이는 당시 코드/상태에서
관찰된 절차다. 현재는 최신 bridge 코드 주석과 launch 기본값을 우선한다.

## 권장 시작 절차

먼저 로봇 bringup은 그리퍼 없이 띄운다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch e0509_gripper_description bringup.launch.py \
  mode:=real host:=110.120.1.66 start_gripper_service:=false
```

그 다음 그리퍼만 별도 실행한다. 기본 루틴은 `stop_existing_drl:=false`다.

```bash
source ~/doosan_ws/install/setup.bash
ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=110.120.1.66 \
  namespace:=dsr01 \
  stop_existing_drl:=false \
  init_attempts:=10 \
  init_timeout_sec:=30.0 \
  init_retry_delay_sec:=2.0 \
  drl_start_retry_count:=5 \
  drl_start_retry_delay_sec:=2.0 \
  post_drl_start_sleep_sec:=2.0
```

동일 명령을 줄이기 위해 wrapper를 추가했다.

```bash
cd ~/doosan_ws/src/e0509_gripper_description
./scripts/start_gripper_service_stable.sh
```

주의: 이 명령은 TCP server가 열릴 때까지 최대 20초 정도 기다린다. 로그에 다음 줄이
몇 번 보인다고 바로 Ctrl-C 하지 않는다.

```text
Waiting for controller TCP server 110.120.1.66:20002 ...
```

## 그리퍼만 청소하는 절차

기존 `scripts/clean_robot_runtime.sh`는 bringup, controller, planner, scan까지 모두
종료한다. 그리퍼 문제만 있을 때는 범위가 너무 넓다.

새로 추가한 좁은 청소 스크립트:

```bash
cd ~/doosan_ws/src/e0509_gripper_description
./scripts/clean_gripper_runtime.sh
```

이 스크립트는 host PC의 `gripper_service_node`/`dsr_gripper_tcp` 프로세스와 ROS daemon만
정리한다. 로봇 bringup, planner, scan 노드는 죽이지 않는다.

정상 종료 확인 기준:

```bash
pgrep -af 'gripper_service_node|dsr_gripper_tcp'
ros2 topic info /gripper_service/state -v
```

`/gripper_service/state`가 `Unknown topic`이면 host-side gripper node는 정리된 것이다.

## 정상 준비 확인 기준

그리퍼 launch 후 다음 3개를 확인한다.

```bash
ros2 topic info /gripper_service/state -v
ros2 topic echo /gripper_service/state --once
ros2 action list -t | grep safe_grasp
```

정상 기준:

```text
Publisher count: 1
ready: true
status_text: ok
/gripper_service/safe_grasp [dsr_gripper_tcp_interfaces/action/SafeGrasp]
```

주의:

- `ready: true`여도 `status_text: timed out`이면 정상 준비 상태가 아니다.
- `Publisher count: 2` 이상이면 고아 `gripper_service_node`가 남아 있을 가능성이 높다.

## 그래도 `status 3`이 반복될 때

다음 순서로 판단한다.

1. `clean_gripper_runtime.sh`로 host-side 고아 프로세스 제거.
2. `start_gripper_service_stable.sh` 또는 위의 `stop_existing_drl:=false` 명령으로 재시작.
3. `DRL already running`인데 `Waiting for controller TCP server ... Connection refused`가
   계속되면 기존 DRL이 PLAY 상태로 남았지만 20002 TCP server는 열지 못한 상태일 수 있다.
   이때만 강제 DRL stop 후 재시작 fallback을 쓴다.

   ```bash
   cd ~/doosan_ws/src/e0509_gripper_description
   ./scripts/restart_gripper_drl_then_start.sh
   ```

4. 그래도 `status 3`이면 그 순간은 ROS graph 문제가 아니라 플랜지 serial/RS-485 쪽 상태
   문제로 본다.
5. 그리퍼 전원/케이블/토크 상태를 확인한다.
6. 계속 반복되면 Doosan controller 재기동이 가장 확실한 회복책이다.

`prime_gripper_serial_drl.py`는 실험용 우회 수단으로 남아 있지만, 현재 기본 절차는 아니다.
해당 primer는 `flange_serial_open()` 자체가 block되는 상태에서는 같이 block될 수 있고,
이후 `stop_existing_drl:=true` 전환이 필요해 최신 기본 절차와 충돌한다.

## 이번 변경 파일

- `scripts/clean_gripper_runtime.sh`
- `scripts/start_gripper_service_stable.sh`
- `scripts/restart_gripper_drl_then_start.sh`
- `CMakeLists.txt` install 목록에 위 두 스크립트 추가
