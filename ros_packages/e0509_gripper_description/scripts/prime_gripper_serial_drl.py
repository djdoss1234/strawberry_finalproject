#!/usr/bin/env python3
"""Experimental serial primer: upload a minimal DRL that opens flange serial.

주의: 컨트롤러 재기동 직후 flange_serial_open() 자체가 block되는 환경에서는
이 primer도 동일하게 block될 수 있다. 검증된 해결책이 아니라 진단용 우회 실험이다.

사용 순서:
  1. ros2 launch e0509_gripper_description bringup.launch.py \
       mode:=real host:=110.120.1.66 start_gripper_service:=false
  2. python3 scripts/prime_gripper_serial_drl.py
     → "Primer DRL uploaded" 출력 확인. timeout이면 더 진행하지 않는다.
  3. ros2 launch dsr_gripper_tcp gripper_service_node.launch.py ...
     (stop_existing_drl:=true → primer DRL 종료 → 자기 DRL 시작 → DRL-to-DRL 전환)

Primer DRL은 port 20002 TCP server를 열지 않음. dsr_gripper_tcp가 stop_existing_drl:=true로
이 DRL을 종료시키고 자신의 DRL을 올림.
"""

import textwrap
import rclpy
from rclpy.node import Node

try:
    from dsr_msgs2.srv import DrlStart, GetDrlState
except ImportError as exc:
    raise SystemExit(f"dsr_msgs2 not found. Run: source ~/doosan_ws/install/setup.bash\n{exc}")


# Minimal DRL: open flange serial and loop forever (dsr_gripper_tcp will stop it)
PRIMER_DRL = textwrap.dedent("""\
    import DR_common
    flange_serial_open(57600, DR_EIGHTBITS, DR_PARITY_NONE, DR_STOPBITS_ONE)
    while True:
        wait(5.0)
""")

DRL_STATE_PLAY = 0


class SerialPrimerNode(Node):
    def __init__(self, namespace: str):
        super().__init__("serial_primer")
        root = f"/{namespace}"
        self._cli_start = self.create_client(DrlStart, f"{root}/drl/drl_start")
        self._cli_state = self.create_client(GetDrlState, f"{root}/drl/get_drl_state")

    def _call(self, client, request, timeout=10.0):
        client.wait_for_service(timeout_sec=timeout)
        f = client.call_async(request)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        return f.result()

    def get_drl_state(self) -> int | None:
        res = self._call(self._cli_state, GetDrlState.Request())
        return int(res.drl_state) if res is not None else None

    def upload_primer(self) -> bool:
        state = self.get_drl_state()
        if state == DRL_STATE_PLAY:
            self.get_logger().info(
                "A DRL is already running. Checking if it's the primer... "
                "If dsr_gripper_tcp's DRL, stop it first."
            )

        req = DrlStart.Request()
        req.robot_system = 0
        req.code = PRIMER_DRL
        res = self._call(self._cli_start, req, timeout=15.0)
        if res is None or not res.success:
            self.get_logger().error(f"DrlStart failed: {res}")
            return False
        self.get_logger().info(
            "Primer DRL uploaded. Flange serial is now in DRL-mode. "
            "Now start dsr_gripper_tcp (stop_existing_drl:=true)."
        )
        return True


def main():
    import argparse
    p = argparse.ArgumentParser(description="Upload primer DRL to put flange serial in DRL-mode")
    p.add_argument("--namespace", default="dsr01")
    args = p.parse_args()

    rclpy.init()
    node = SerialPrimerNode(args.namespace)
    ok = node.upload_primer()
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
