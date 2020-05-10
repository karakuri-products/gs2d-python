from abc import ABCMeta, abstractmethod
from collections import deque
import asyncio
import threading
import time
import logging
import struct

from .Util import CommandBufferOverflowException, NotEnablePollingCommandException, get_printable_hex

# ロガー
logger = logging.getLogger(__name__)

# シリアルポートの送信バッファサイズ
MAX_COMMAND_QUEUE_LENGTH = 1024

# 送信タイムアウト(秒)
SEND_DATA_TIMEOUT_SEC = 2

# 受信タイムアウト(秒)
RECEIVE_DATA_TIMEOUT_SEC = 2


class Driver(metaclass=ABCMeta):
    """
    サーボモータとのデータ送受信管理および各種コントロール関数の抽象クラス
    """

    # シリアルポートインタフェースクラスのオブジェクト
    serial_interface = None

    # コマンドの送信バッファ
    command_queue = None

    command_queue_size = None

    # コマンドバッファを監視し随時送信するスレッド
    polling_thread = None

    # コマンドバッファ監視フラグ
    enable_polling = False

    # クローズ強制フラグ
    close_force = False

    def __init__(self, serial_interface, buffer_size=None):
        """初期化

        :param serial_interface:
        :param buffer_size:
        """

        # バッファサイズ設定
        if buffer_size is None:
            self.command_queue_size = MAX_COMMAND_QUEUE_LENGTH
        else:
            self.command_queue_size = buffer_size

        logger.debug('Buffer size: ' + str(self.command_queue_size))

        # コマンド送信バッファ初期化
        self.command_queue = deque([], self.command_queue_size)
        self.__connect(serial_interface)

        # コマンド送信バッファチェック用スレッドを開始
        self.enable_polling = True
        self.polling_thread = threading.Thread(target=self.__polling_command_queue)
        self.polling_thread.start()

    def __connect(self, serial_interface):
        """シリアルインタフェースに接続

        :param serial_interface:
        :return:
        """

        self.serial_interface = serial_interface

    def __send_command(self, data, recv_callback=None):
        """実際にコマンド送信バッファの中から取り出したコマンドを送信する

        :param data:
        :param recv_callback: 受信データのコールバック。レスポンスがあるリクエストはrecv_callbackを設定する必要あり
        :return:
        """

        byte_data = bytearray(data)

        if self.serial_interface.is_open:
            # データ送信
            self.serial_interface.write(byte_data)

            logger.debug('Sent data: ' + get_printable_hex(byte_data))

            if recv_callback is not None:
                start = time.time()

                # データを受信する
                response = self.serial_interface.readline()

                # データが完全に受信できていないのであれば更に受信する
                while not self.is_complete_response(response):
                    response += self.serial_interface.readline()

                    # タイムアウトチェック
                    elapsed_time = time.time() - start
                    if elapsed_time > RECEIVE_DATA_TIMEOUT_SEC:
                        break

                logger.debug('Response data: ' + get_printable_hex(response))

                # 別スレッドでコールバックを呼ぶ（コールバックでcloseされたりとかもするので）
                threading.Thread(target=recv_callback, args=(response,)).start()

    def __polling_command_queue(self):
        """コマンド送信バッファの監視スレッドで動作する関数
        バッファのサイズをチェックし、コマンドがあればそれを送信する

        :return:
        """

        while self.enable_polling or (not self.close_force and len(self.command_queue) > 0):
            if len(self.command_queue) > 0:
                command = self.command_queue.pop()
                self.__send_command(command['data'], command['recv_callback'])

    @staticmethod
    def async_wrapper(loop=None):
        """async対応するための関数

        :param loop:
        :return:
        """

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
                data_bytes.append(int(struct.unpack('<B', data_hex[i:i+1])[0]))
            else:
                data_bytes.append(0)

        return data_bytes

    def add_command_queue(self, data, recv_callback=None):
        """送信するコマンドを送信バッファに追加する

        :param data:
        :param recv_callback:
        :return:
        """

        try:
            if len(self.command_queue) > MAX_COMMAND_QUEUE_LENGTH:
                raise CommandBufferOverflowException('コマンドバッファの最大サイズ(%d)を超えました' % self.command_queue_size)
            elif not self.enable_polling:
                raise NotEnablePollingCommandException('コマンドバッファのポーリング終了後にコマンド追加はできません')
            command_data = {
                'data': data,
                'recv_callback': recv_callback
            }
            self.command_queue.insert(0, command_data)
            # logger.debug('Command data: ' + str(command_data))
            return True
        except IndexError:
            return False

    def close(self, force=False):
        """接続をクローズする。
        force=False ならバッファにあるコマンドをすべて処理してからクローズ
        force=True なら問答無用ですぐにクローズ

        :return:
        """

        # ポーリングスレッド停止
        if self.enable_polling:
            self.close_force = force
            self.enable_polling = False

            # コマンドバッファポーリングの終了を待つ
            self.polling_thread.join()

    @abstractmethod
    def is_complete_response(self, response_data):
        """レスポンスデータをすべて受信できたかチェック"""
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
