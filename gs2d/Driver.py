# ! /usr/bin/env python3
# encoding: utf-8

from abc import ABCMeta, abstractmethod
from .ICommandHandler import ICommandHandler
from .ISerialInterface import ISerialInterface
import logging

try:
    import struct
except:
    # For MicroPython
    import ustruct as struct

# ロガー
logger = logging.getLogger(__name__)


class Driver(metaclass=ABCMeta):
    """
    サーボモータとのデータ送受信管理および各種コントロール関数の抽象クラス
    """

    command_handler = None

    def __init__(self, serial_interface: ISerialInterface, command_handler_class: ICommandHandler = None):
        """初期化
        """

        # command_handler_classが指定されていない場合はデフォルトのコマンドハンドラーにする
        if command_handler_class is None:
            from .DefaultCommandHandler import DefaultCommandHandler
            command_handler_class = DefaultCommandHandler

        self.command_handler = command_handler_class(serial_interface, self.is_complete_response)

    @staticmethod
    def async_wrapper(loop=None):
        """async対応するための関数

        :param loop:
        :return:
        """

        import asyncio

        if loop is None:
            loop = asyncio.get_event_loop()

        f = loop.create_future()

        def callback(result):
            loop.call_soon_threadsafe(
                lambda: f.set_result(result)
            )

        return f, callback

    @staticmethod
    def get_bytes(data, byte_length):
        """intのデータを指定のバイト数のlittle-endianデータに変換

        :param data:
        :param byte_length:
        :return:
        """

        data_hex = struct.pack('<H', int(data) & (2 ** (8 * byte_length) - 1))
        data_bytes = []

        for i in range(byte_length):
            if i < len(data_hex):
                data_bytes.append(int(struct.unpack('<B', data_hex[i:i + 1])[0]))
            else:
                data_bytes.append(0)

        return data_bytes

    @abstractmethod
    def is_complete_response(self, response_data):
        """レスポンスデータをすべて受信できたかチェック

        :param response_data:
        :return:
        """
        raise NotImplementedError()

    @abstractmethod
    def close(self, force=False):
        raise NotImplementedError()

    @abstractmethod
    def ping(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def ping_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def get_torque_enable(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def get_torque_enable_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_torque_enable(self, on_off, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_temperature(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_temperature_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def get_current(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_current_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def get_target_position(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_target_position_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_target_position(self, position, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_current_position(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_current_position_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def get_voltage(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_voltage_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def get_target_time(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_target_time_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_target_time(self, speed_second, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_pid_coefficient(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_pid_coefficient_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_pid_coefficient(self, coef_percent, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_max_torque(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_max_torque_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_max_torque(self, torque_percent, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_speed(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_speed_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_speed(self, dps, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_servo_id(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_servo_id_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_servo_id(self, new_sid, sid):
        raise NotImplementedError()

    @abstractmethod
    def save_rom(self, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_baud_rate(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_baud_rate_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_baud_rate(self, baud_rate_id, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_limit_cw_position(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_limit_cw_position_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_limit_cw_position(self, limit_position, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_limit_ccw_position(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_limit_ccw_position_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_limit_ccw_position(self, limit_position, sid):
        raise NotImplementedError()

    @abstractmethod
    def get_limit_temperature(self, sid, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_limit_temperature_async(self, sid, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def set_limit_temperature(self, limit_temp, sid):
        raise NotImplementedError()

    @abstractmethod
    def set_burst_target_positions(self, sid_target_positions):
        raise NotImplementedError()

    @abstractmethod
    def get_burst_positions(self, sids, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def get_burst_positions_async(self, sids, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def reset_memory(self, sid):
        raise NotImplementedError()

    @abstractmethod
    def read(self, sid, address, length, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def read_async(self, sid, address, length, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def write(self, sid, address, data):
        raise NotImplementedError()

    @abstractmethod
    def burst_read(self, sid_address_length, callback=None):
        raise NotImplementedError()

    @abstractmethod
    def burst_read_async(self, sid_address_length, loop=None):
        raise NotImplementedError()

    @abstractmethod
    def burst_write(self, address, length, sid_data):
        raise NotImplementedError()
