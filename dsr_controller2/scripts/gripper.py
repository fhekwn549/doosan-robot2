#!/usr/bin/env python3
"""
RH-P12-RN-A 그리퍼 제어 모듈
Doosan E0509 로봇의 Tool Flange Serial을 통해 Modbus RTU로 제어

사용법:
    ros2 run dsr_controller2 gripper.py open              # 그리퍼 열기
    ros2 run dsr_controller2 gripper.py close             # 그리퍼 닫기
    ros2 run dsr_controller2 gripper.py pos 350           # 위치 지정 (0~700)
    ros2 run dsr_controller2 gripper.py --ns dsr01 open   # namespace 지정
"""

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import FlangeSerialOpen, FlangeSerialClose, FlangeSerialWrite
from std_msgs.msg import Int32
import argparse
import time


class ModbusRTU:
    """Modbus RTU CRC16 계산 및 프레임 생성"""

    SLAVE_ID = 1
    REG_TORQUE_ENABLE = 256
    REG_GOAL_POSITION = 282

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @classmethod
    def make_frame(cls, data: bytes) -> list:
        """CRC 추가하고 list[int]로 반환"""
        crc = cls.crc16(data)
        frame = data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        return list(frame)

    @classmethod
    def fc06_torque_enable(cls) -> list:
        """토크 활성화 프레임"""
        data = bytes([cls.SLAVE_ID, 0x06, 0x01, 0x00, 0x00, 0x01])
        return cls.make_frame(data)

    @classmethod
    def fc16_position(cls, position: int) -> list:
        """위치 설정 프레임 (FC16)"""
        position = max(0, min(700, position))
        data = bytes([
            cls.SLAVE_ID, 0x10,  # FC16
            0x01, 0x1A,          # Start register 282
            0x00, 0x02,          # Register count 2
            0x04,                # Byte count 4
            (position >> 8) & 0xFF, position & 0xFF,  # Position
            0x00, 0x00           # Reserved
        ])
        return cls.make_frame(data)


class GripperNode(Node):
    """그리퍼 제어 노드"""

    def __init__(self, namespace='dsr01'):
        super().__init__('gripper_node')

        prefix = f'/{namespace}/gripper'
        self.cli_open = self.create_client(FlangeSerialOpen, f'{prefix}/flange_serial_open')
        self.cli_close = self.create_client(FlangeSerialClose, f'{prefix}/flange_serial_close')
        self.cli_write = self.create_client(FlangeSerialWrite, f'{prefix}/flange_serial_write')

        # RViz 시각화용 stroke publisher
        self.stroke_pub = self.create_publisher(Int32, f'{prefix}/stroke', 10)

        self.port = 1
        self.get_logger().info(f'그리퍼 노드 초기화 (namespace: {namespace})')

    def wait_for_services(self, timeout=5.0):
        """서비스 대기"""
        for cli, name in [(self.cli_open, 'open'), (self.cli_close, 'close'), (self.cli_write, 'write')]:
            if not cli.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f'서비스 {name} 없음')
                return False
        return True

    def call_sync(self, client, request):
        """동기 서비스 호출"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def serial_open(self, baudrate=57600):
        """시리얼 포트 열기"""
        req = FlangeSerialOpen.Request()
        req.port = self.port
        req.baudrate = baudrate
        req.bytesize = 8
        req.parity = 0
        req.stopbits = 1

        result = self.call_sync(self.cli_open, req)
        if result and result.success:
            self.get_logger().info('시리얼 포트 열림')
            return True
        self.get_logger().error('시리얼 포트 열기 실패')
        return False

    def serial_close(self):
        """시리얼 포트 닫기"""
        req = FlangeSerialClose.Request()
        req.port = self.port
        result = self.call_sync(self.cli_close, req)
        if result and result.success:
            self.get_logger().info('시리얼 포트 닫힘')
        return result.success if result else False

    def serial_write(self, data: list):
        """데이터 전송"""
        req = FlangeSerialWrite.Request()
        req.port = self.port
        req.data = data

        result = self.call_sync(self.cli_write, req)
        if result and result.success:
            self.get_logger().info(f'전송 성공: {bytes(data).hex()}')
            return True
        self.get_logger().error('전송 실패')
        return False

    def enable_torque(self):
        """토크 활성화"""
        self.get_logger().info('토크 활성화...')
        return self.serial_write(ModbusRTU.fc06_torque_enable())

    def set_position(self, position: int):
        """위치 설정"""
        position = max(0, min(700, position))
        self.get_logger().info(f'위치 설정: {position}')

        # RViz 시각화 업데이트
        stroke_msg = Int32()
        stroke_msg.data = position
        self.stroke_pub.publish(stroke_msg)

        return self.serial_write(ModbusRTU.fc16_position(position))

    def open_gripper(self):
        """그리퍼 열기"""
        self.get_logger().info('그리퍼 열기')
        return self.set_position(0)

    def close_gripper(self):
        """그리퍼 닫기"""
        self.get_logger().info('그리퍼 닫기')
        return self.set_position(700)

    def run_command(self, command: str, value: int = None):
        """명령 실행"""
        if not self.serial_open():
            return False

        try:
            time.sleep(0.1)
            self.enable_torque()
            time.sleep(0.2)

            if command == 'open':
                self.open_gripper()
            elif command == 'close':
                self.close_gripper()
            elif command == 'pos' and value is not None:
                self.set_position(value)

            time.sleep(1.0)  # 그리퍼 동작 대기
            return True
        finally:
            self.serial_close()


def main():
    parser = argparse.ArgumentParser(description='RH-P12-RN-A 그리퍼 제어')
    parser.add_argument('--ns', type=str, default='dsr01', help='로봇 namespace')
    parser.add_argument('command', choices=['open', 'close', 'pos'], help='명령')
    parser.add_argument('value', type=int, nargs='?', help='pos 명령시 위치값 (0~700)')

    args, remaining = parser.parse_known_args()

    if args.command == 'pos' and args.value is None:
        print('pos 명령은 위치값이 필요합니다 (0~700)')
        return

    rclpy.init(args=remaining)
    node = GripperNode(namespace=args.ns)

    if not node.wait_for_services():
        print('서비스를 찾을 수 없습니다. dsr_controller2가 실행 중인지 확인하세요.')
        rclpy.shutdown()
        return

    node.run_command(args.command, args.value)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
