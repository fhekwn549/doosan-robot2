#!/usr/bin/env python3
"""
RH-P12-RN-A 그리퍼 제어 스크립트
Doosan 로봇의 Tool Flange Serial을 통해 Modbus RTU로 그리퍼 제어

사용법:
    ros2 run dsr_controller2 gripper_control.py open                    # 그리퍼 열기
    ros2 run dsr_controller2 gripper_control.py close                   # 그리퍼 닫기
    ros2 run dsr_controller2 gripper_control.py pos 350                 # 위치 설정 (0-700)
    ros2 run dsr_controller2 gripper_control.py init                    # 초기화 (토크 활성화)
    ros2 run dsr_controller2 gripper_control.py --ns dsr01e0509 open    # namespace 지정
"""

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import FlangeSerialOpen, FlangeSerialClose, FlangeSerialWrite, FlangeSerialRead
import sys
import time
import argparse


class ModbusRTU:
    """Modbus RTU 프레임 생성 클래스"""

    SLAVE_ID = 1

    # 레지스터 주소
    REG_TORQUE_ENABLE = 256    # 0x0100
    REG_GOAL_CURRENT = 275     # 0x0113
    REG_GOAL_POSITION = 282    # 0x011A
    REG_PRESENT_POSITION = 281 # 0x0119

    @staticmethod
    def crc16(data: bytes) -> int:
        """Modbus CRC-16 계산"""
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
    def write_single_register(cls, register: int, value: int) -> bytes:
        """FC06: Write Single Register 프레임 생성"""
        frame = bytes([
            cls.SLAVE_ID,
            0x06,  # Function Code
            (register >> 8) & 0xFF,  # Register High
            register & 0xFF,         # Register Low
            (value >> 8) & 0xFF,     # Value High
            value & 0xFF             # Value Low
        ])
        crc = cls.crc16(frame)
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    @classmethod
    def read_holding_registers(cls, start_register: int, quantity: int) -> bytes:
        """FC03: Read Holding Registers 프레임 생성"""
        frame = bytes([
            cls.SLAVE_ID,
            0x03,  # Function Code
            (start_register >> 8) & 0xFF,
            start_register & 0xFF,
            (quantity >> 8) & 0xFF,
            quantity & 0xFF
        ])
        crc = cls.crc16(frame)
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    @classmethod
    def write_multiple_registers(cls, start_register: int, values: list) -> bytes:
        """FC16: Write Multiple Registers 프레임 생성"""
        cnt = len(values)
        frame = bytes([
            cls.SLAVE_ID,
            0x10,  # Function Code 16
            (start_register >> 8) & 0xFF,
            start_register & 0xFF,
            (cnt >> 8) & 0xFF,
            cnt & 0xFF,
            cnt * 2  # Byte count
        ])
        for val in values:
            frame += bytes([(val >> 8) & 0xFF, val & 0xFF])
        crc = cls.crc16(frame)
        return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class GripperController(Node):
    """RH-P12-RN-A 그리퍼 제어 노드"""

    def __init__(self, namespace='dsr01e0509'):
        super().__init__('gripper_controller')

        # 서비스 클라이언트 생성
        srv_prefix = f'/{namespace}/gripper'
        self.cli_open = self.create_client(FlangeSerialOpen, f'{srv_prefix}/flange_serial_open')
        self.cli_close = self.create_client(FlangeSerialClose, f'{srv_prefix}/flange_serial_close')
        self.cli_write = self.create_client(FlangeSerialWrite, f'{srv_prefix}/flange_serial_write')
        self.cli_read = self.create_client(FlangeSerialRead, f'{srv_prefix}/flange_serial_read')

        self.get_logger().info(f'서비스 경로: {srv_prefix}/flange_serial_*')

        self.port = 1  # Tool Flange Serial 포트 번호
        self.is_serial_open = False

    def wait_for_services(self, timeout_sec=5.0):
        """서비스 대기"""
        services = [
            (self.cli_open, 'flange_serial_open'),
            (self.cli_close, 'flange_serial_close'),
            (self.cli_write, 'flange_serial_write'),
            (self.cli_read, 'flange_serial_read')
        ]

        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f'서비스 {name}을(를) 찾을 수 없습니다.')
                return False
        return True

    def open_serial(self, baudrate=57600) -> bool:
        """시리얼 포트 열기 (먼저 닫기 시도)"""
        # 먼저 기존 연결 닫기 시도
        self.close_serial()
        time.sleep(0.1)

        req = FlangeSerialOpen.Request()
        req.port = self.port
        req.baudrate = baudrate
        req.bytesize = 8
        req.parity = 0  # None
        req.stopbits = 1

        future = self.cli_open.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.is_serial_open = future.result().success
            if self.is_serial_open:
                self.get_logger().info('시리얼 포트 열림')
            else:
                self.get_logger().error('시리얼 포트 열기 실패')
            return self.is_serial_open
        return False

    def close_serial(self) -> bool:
        """시리얼 포트 닫기"""
        req = FlangeSerialClose.Request()
        req.port = self.port

        future = self.cli_close.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            self.is_serial_open = False
            return future.result().success
        return False

    def send_modbus(self, data: bytes) -> bool:
        """Modbus 데이터 전송"""
        req = FlangeSerialWrite.Request()
        req.port = self.port
        req.data = list(data)

        future = self.cli_write.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            success = future.result().success
            self.get_logger().info(f'write 결과: {success}')
            return success
        self.get_logger().error('write 응답 없음')
        return False

    def read_modbus(self, timeout: float = 0.5) -> bytes:
        """Modbus 응답 읽기"""
        req = FlangeSerialRead.Request()
        req.port = self.port
        req.timeout = timeout

        future = self.cli_read.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'read 결과: success={result.success}, size={result.size}')
            if result.success:
                return bytes(result.data)
        else:
            self.get_logger().error('read 응답 없음')
        return b''

    def write_register(self, register: int, value: int) -> bool:
        """레지스터에 값 쓰기"""
        frame = ModbusRTU.write_single_register(register, value)
        self.get_logger().info(f'전송: {frame.hex()}')

        if not self.send_modbus(frame):
            self.get_logger().error('전송 실패')
            return False

        time.sleep(0.05)  # 응답 대기
        response = self.read_modbus()

        if response:
            self.get_logger().info(f'응답: {response.hex()}')
            return True
        return False

    def enable_torque(self) -> bool:
        """토크 활성화"""
        self.get_logger().info('토크 활성화...')
        return self.write_register(ModbusRTU.REG_TORQUE_ENABLE, 1)

    def disable_torque(self) -> bool:
        """토크 비활성화"""
        self.get_logger().info('토크 비활성화...')
        return self.write_register(ModbusRTU.REG_TORQUE_ENABLE, 0)

    def set_goal_current(self, current: int = 400) -> bool:
        """목표 전류 설정 (기본값 400)"""
        self.get_logger().info(f'목표 전류 설정: {current}')
        return self.write_register(ModbusRTU.REG_GOAL_CURRENT, current)

    def set_position(self, position: int) -> bool:
        """위치 설정 (0-700, 0=열림, 700=닫힘) - FC16으로 2개 레지스터에 쓰기"""
        position = max(0, min(700, position))
        self.get_logger().info(f'위치 설정: {position}')
        return self.write_multiple_registers(ModbusRTU.REG_GOAL_POSITION, [position, 0])

    def write_multiple_registers(self, start_register: int, values: list) -> bool:
        """여러 레지스터에 값 쓰기 (FC16) - 응답 무시"""
        frame = ModbusRTU.write_multiple_registers(start_register, values)
        self.get_logger().info(f'전송(FC16): {frame.hex()}')

        if not self.send_modbus(frame):
            self.get_logger().error('전송 실패')
            return False

        time.sleep(0.1)  # 그리퍼 처리 시간
        return True

    def init_gripper_with_retry(self, baudrate=57600, max_retries=5) -> bool:
        """그리퍼 초기화 - DART Platform 방식으로 여러 번 시도"""
        for attempt in range(max_retries):
            self.get_logger().info(f'그리퍼 초기화 시도 {attempt + 1}/{max_retries}')

            # 포트 열기
            if not self.open_serial_simple(baudrate):
                self.get_logger().warn('포트 열기 실패, 재시도...')
                time.sleep(0.5)
                continue

            # 토크 활성화 (256, 1)
            self.get_logger().info('토크 활성화...')
            frame = ModbusRTU.write_single_register(256, 1)
            self.get_logger().info(f'전송: {frame.hex()}')
            self.send_modbus(frame)
            time.sleep(0.1)
            response = self.read_modbus(timeout=0.1)

            if response:
                self.get_logger().info(f'응답: {response.hex()}')

            # Goal Current (275, 400)
            self.get_logger().info('Goal Current 설정...')
            frame = ModbusRTU.write_single_register(275, 400)
            self.get_logger().info(f'전송: {frame.hex()}')
            self.send_modbus(frame)
            time.sleep(0.1)
            response = self.read_modbus(timeout=0.1)

            if response:
                self.get_logger().info(f'응답: {response.hex()}')
                self.get_logger().info('그리퍼 초기화 성공!')
                return True
            else:
                self.get_logger().warn('응답 없음, 포트 닫고 재시도...')
                self.close_serial()
                time.sleep(0.5)

        self.get_logger().error('그리퍼 초기화 실패')
        return False

    def open_serial_simple(self, baudrate=57600) -> bool:
        """시리얼 포트 열기 (close 없이)"""
        req = FlangeSerialOpen.Request()
        req.port = self.port
        req.baudrate = baudrate
        req.bytesize = 8
        req.parity = 0
        req.stopbits = 1

        future = self.cli_open.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.is_serial_open = True
            self.get_logger().info('시리얼 포트 열림')
            return True
        return False

    def init_gripper(self) -> bool:
        """그리퍼 초기화 - 토크만 활성화"""
        self.get_logger().info('그리퍼 초기화 중...')

        # 토크 활성화만 (Goal Current는 그리퍼에 이미 설정되어 있음)
        self.enable_torque()
        time.sleep(0.1)

        self.get_logger().info('그리퍼 초기화 완료')
        return True

    def open_gripper(self) -> bool:
        """그리퍼 열기"""
        self.get_logger().info('그리퍼 열기...')
        return self.set_position(0)

    def close_gripper(self) -> bool:
        """그리퍼 닫기"""
        self.get_logger().info('그리퍼 닫기...')
        return self.set_position(700)


def main(args=None):
    # argparse 설정
    parser = argparse.ArgumentParser(description='RH-P12-RN-A 그리퍼 제어')
    parser.add_argument('--ns', type=str, default='dsr01',
                        help='로봇 namespace (예: dsr01, dsr01e0509)')
    parser.add_argument('--baudrate', type=int, default=57600,
                        help='시리얼 통신 속도 (기본값: 57600)')
    parser.add_argument('command', type=str, nargs='?', default='',
                        choices=['open', 'close', 'init', 'pos', 'torque_off', ''],
                        help='실행할 명령')
    parser.add_argument('value', type=int, nargs='?', default=None,
                        help='pos 명령에 대한 위치값 (0-700)')

    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)

    controller = GripperController(namespace=parsed_args.ns)

    if not controller.wait_for_services():
        controller.get_logger().error('서비스를 찾을 수 없습니다. dsr_controller2가 실행 중인지 확인하세요.')
        rclpy.shutdown()
        return

    # 명령 처리
    if not parsed_args.command:
        print(__doc__)
        rclpy.shutdown()
        return

    command = parsed_args.command.lower()

    # 첫 번째 작동했을 때 방식: 응답 기다리지 않고 바로 명령 전송
    controller.get_logger().info(f'baudrate: {parsed_args.baudrate}')

    try:
        # 포트 열기
        if not controller.open_serial_simple(parsed_args.baudrate):
            controller.get_logger().error('포트 열기 실패')
            rclpy.shutdown()
            return

        # 토크 활성화 (응답 무시)
        controller.get_logger().info('토크 활성화...')
        frame = ModbusRTU.write_single_register(256, 1)
        controller.get_logger().info(f'전송: {frame.hex()}')
        controller.send_modbus(frame)
        time.sleep(0.2)  # 그리퍼 처리 시간

        if command == 'init':
            controller.get_logger().info('초기화 완료')
        elif command == 'open':
            controller.open_gripper()
        elif command == 'close':
            controller.close_gripper()
        elif command == 'pos':
            if parsed_args.value is None:
                print('위치값을 지정하세요 (0-700)')
            else:
                controller.set_position(parsed_args.value)
        elif command == 'torque_off':
            controller.disable_torque()
        else:
            print(f'알 수 없는 명령: {command}')
            print(__doc__)

        time.sleep(1)  # 그리퍼 동작 대기
    finally:
        # 시리얼 포트 닫기
        controller.close_serial()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
