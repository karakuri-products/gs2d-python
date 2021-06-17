import asyncio
import time
import logging

from .Driver import Driver
from .Util import ReceiveDataTimeoutException, NotSupportException, BadInputParametersException, WrongCheckSumException
from .Util import InvalidResponseDataException
from .Util import get_printable_hex

# ロガー
logger = logging.getLogger(__name__)

RECEIVE_DATA_TIMEOUT_SEC = 19

class Futaba(Driver):
    """Futabaのシリアルサーボクラス

    """

    # アドレス空間
    ADDR_MODEL_NUMBER_L = 0  # 0x00
    ADDR_FIRMWARE_VERSION = 2  # 0x02
    ADDR_SERVO_ID = 4  # 0x04
    ADDR_REVERSE = 5  # 0x05
    ADDR_BAUD_RATE = 6  # 0x06
    ADDR_RETURN_DELAY = 7  # 0x07
    ADDR_CW_ANGLE_LIMIT_L = 8  # 0x08
    ADDR_CCW_ANGLE_LIMIT_L = 10  # 0x0A
    ADDR_TEMPERATURE_LIMIT_L = 14  # 0x0E
    ADDR_TORQUE_IN_SILENCE = 22  # 0x16
    ADDR_WARM_UP_TIME = 23  # 0x17
    ADDR_CW_COMPLIANCE_MARGIN = 24  # 0x18
    ADDR_CCW_COMPLIANCE_MARGIN = 25  # 0x19
    ADDR_CW_COMPLIANCE_SLOPE = 26  # 0x1A
    ADDR_CCW_COMPLIANCE_SLOPE = 27  # 0x1B
    ADDR_PUNCH_L = 28  # 0x1C

    ADDR_GOAL_POSITION_L = 30  # 0x1E
    ADDR_GOAL_TIME_L = 32  # 0x20
    ADDR_MAX_TORQUE = 35  # 0x23
    ADDR_TORQUE_ENABLE = 36  # 0x24
    ADDR_PID_COEFFICIENT = 38  # 0x26
    ADDR_PRESENT_POSITION_L = 42  # 0x2A
    ADDR_PRESENT_TIME_L = 44  # 0x2C
    ADDR_PRESENT_SPEED_L = 46  # 0x2E
    ADDR_PRESENT_CURRENT_L = 48  # 0x30
    ADDR_TEMPERATURE_L = 50  # 0x32
    ADDR_VOLTAGE_L = 52  # 0x34

    ADDR_WRITE_FLASH_ROM = 255  # 0xFF
    ADDR_RESET_MEMORY = 255  # 0xFF

    # フラグ
    FLAG4_RESET_MEMORY_MAP = 16  # 0x10
    FLAG30_NO_RETURN_PACKET = 0
    FLAG30_MEM_MAP_ACK = 1
    FLAG30_MEM_MAP_00_29 = 3
    FLAG30_MEM_MAP_30_59 = 5
    FLAG30_MEM_MAP_20_29 = 7
    FLAG30_MEM_MAP_42_59 = 9
    FLAG30_MEM_MAP_30_41 = 11
    FLAG30_MEM_MAP_60_127 = 13
    FLAG30_MEM_MAP_SELECT = 15

    PACKET_DATA_INDEX = 7

    # 通信速度のIDと実際の設定値
    BAUD_RATE_INDEX_9600 = 0x00
    BAUD_RATE_INDEX_14400 = 0x01
    BAUD_RATE_INDEX_19200 = 0x02
    BAUD_RATE_INDEX_28800 = 0x03
    BAUD_RATE_INDEX_38400 = 0x04
    BAUD_RATE_INDEX_57600 = 0x05
    BAUD_RATE_INDEX_76800 = 0x06
    BAUD_RATE_INDEX_115200 = 0x07
    BAUD_RATE_INDEX_153600 = 0x08
    BAUD_RATE_INDEX_230400 = 0x09

    def __init__(self, serial_interface):
        """初期化
        """

        super(Futaba, self).__init__(serial_interface)

    def is_complete_response(self, response_data):
        """レスポンスデータをすべて受信できたかチェック"""
        # print('#########', response_data)

        # Header, ID, Flags, Address, Length, Count, Data, Sum で7バイトは最低限ある
        if len(response_data) < 6:
            return False
        else:
            # カウント取得
            count = response_data[5]
            # print('####', count, len(response_data))
            return len(response_data) == (8 + count)

    @staticmethod
    def __get_checksum(data):
        """チェックサムを生成

        :param data:
        :return:
        """

        checksum = 0
        for i in range(2, len(data)):
            checksum ^= data[i]
        return checksum

    @staticmethod
    def __check_sid(sid):
        """Servo IDのレンジをチェック

        :param sid:
        :return:
        """

        if sid < 1 or sid > 127:
            raise BadInputParametersException('sid: %d がレンジ外です。1から127のIDを設定してください。' % sid)

    def __generate_command(self, sid, addr, data=None, flag=0, count=1, length=None):
        """コマンド生成

        :param sid:  Servo ID
        :param addr:
        :param data:
        :param flag:
        :param count:
        :param length:
        :return:
        """

        # Header:   パケットの先頭を表します。ショートパケットではFAAFに設定します。
        # ID:       サーボのIDです。1~127(01H~7FH)までの値が使用できます。
        #           ID:255を指定すると、全IDのサーボへの共通指令になります(リターンデータは取れません)。
        # Flag:     サーボからのリターンデータ取得やデータ書き込み時の設定をします
        # Address:  メモリーマップ上のアドレスを指定します。
        #           このアドレスから「Length」に指定した長さ分のデータをメモリーマップに書き込みます。
        # Length:   データ1ブロックの長さを指定します。ショートパケットでは Dataのバイト数になります。
        # Count:    サーボの数を表します。ショートパケットでメモリーマップに書き込む時は 1 に設定します。
        # Data:     メモリーマップに書き込むデータです
        # Checksum: 送信データの確認用のチェックサムで、パケットのIDからDataの末尾までを1バイトずつ
        #           XORした値を指定します。

        command = []

        # Header
        command.extend([0xFA, 0xAF])

        # ID
        command.append(sid)

        # Flag
        command.append(flag)

        # Address
        command.append(addr)

        # Length
        if length is not None:
            command.append(length)
        elif data is not None:
            command.append(len(data))
        else:
            command.append(0)

        # Count
        command.append(count)

        # Data
        if data is not None and len(data) > 0:
            command.extend(data)

        # Checksum
        command.append(self.__get_checksum(command))

        return command

    def __generate_burst_command(self, addr, length, vid_data_dict):
        """バーストコマンド生成

        :param addr:
        :param length:
        :param vid_data_dict:
        :return:
        """

        # Header:  パケットの先頭を表します。ショートパケットではFAAFに設定します。
        # ID:      常に00
        # Flag:    常に00
        # Address: メモリーマップ上のアドレスを指定します。
        #          このアドレスから「Length」に指定した長さ分のデータを指定した複数サーボのメモリーマップに書き込みます。
        # Length:  サーボ1つ分のデータ (VID+Data) のバイト数を指定します。
        # Count:   データを送信する対象となるサーボの数を表します。この数分 VID と Data を送信します。
        # VID:     データを送信する個々のサーボの ID を表します。VID と Data が一組でサーボの数分のデータを送信します。
        # Data:    メモリーマップに書き込むサーボ一つ分のデータです。VID と Data が一組でサーボの数分のデータを送信します。
        # Sum:     送信データの確認用のチェックサムで、パケットのIDからDataの末尾までを1バイトずつ
        #          XORした値を指定します。

        command = []

        # Header
        command.extend([0xFA, 0xAF])

        # ID
        command.append(0)

        # Flag
        command.append(0)

        # Address
        command.append(addr)

        # Length
        command.append(length)

        # Count
        command.append(len(vid_data_dict))

        # Data
        for sid, data in vid_data_dict.items():
            command.append(sid)
            if data is not None and len(data) > 0:
                command.extend(data)

        # Checksum
        command.append(self.__get_checksum(command))

        return command

    def __get_function(self, address, flag, length, response_process, sid=1, callback=None):
        """Get系の処理をまとめた関数

        :param address:
        :param flag:
        :param length:
        :param response_process:
        :param sid:
        :param callback:
        :return:
        """

        # データ
        data = None

        # 受信済みフラグ
        is_received = False

        # チェックサム不正エラー
        is_checksum_error = False

        def temp_recv_callback(response):
            nonlocal data
            nonlocal is_received
            nonlocal is_checksum_error

            recv_data = None

            # レスポンスデータのチェックサムが正しいかチェック
            if self.__get_checksum(response[:-1]) != response[-1]:
                logger.debug('Check sum error: ' + get_printable_hex(response))
                is_checksum_error = True
                return

            # 受信済み
            is_received = True

            if len(response) > self.PACKET_DATA_INDEX + length:
                response_data = response[self.PACKET_DATA_INDEX:self.PACKET_DATA_INDEX + length]
                recv_data = response_process(response_data)

            if callback is not None:
                callback(recv_data)
            else:
                data = recv_data

        command = self.__generate_command(sid, address, flag=flag, count=0, length=length)
        self.add_command(command, recv_callback=temp_recv_callback)

        # コールバックが設定できていたら、コールバックに受信データを渡す
        if callback is None:
            # 指定以内にサーボからデータを受信できたかをチェック
            start = time.time()
            while not is_received:
                elapsed_time = time.time() - start
                if elapsed_time > self.RECEIVE_DATA_TIMEOUT_SEC:
                    raise ReceiveDataTimeoutException(str(self.RECEIVE_DATA_TIMEOUT_SEC) + '秒以内にデータ受信できませんでした')
                elif is_checksum_error:
                    raise WrongCheckSumException('受信したデータのチェックサムが不正です')

            return data
        else:
            return True

    def get_torque_enable(self, sid, callback=None):
        """トルクON取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                return response_data[0] == 0x01
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_TORQUE_ENABLE, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
                                   sid=sid, callback=callback)

    def get_torque_enable_async(self, sid, loop=None):
        """トルクON取得async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_torque_enable(sid, callback=callback)
        return f

    def ping(self, sid, callback=None):
        """サーボにPINGを送る
        FutabaにはPINGコマンドはないので、get_servo_idで代用。

        :param sid:
        :param callback:
        :return: {
            'model_no': bytearray (2bytes),
            'version_firmware': int (1byte)
        }
        """

        # TODO: model_noとversion_firmwareをreadで一気に取得する方式に変更

        # サーボIDのチェック
        self.__check_sid(sid)

        if callback:
            def inner_callback(_sid):
                if callback:
                    callback(_sid == sid)

            self.get_servo_id(sid, callback=inner_callback)
        else:
            servo_id = self.get_servo_id(sid)
            return sid == servo_id

    def ping_async(self, sid, loop=None):
        """サーボにPINGを送る async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.ping(sid, callback=callback)
        return f

    def set_torque_enable(self, on_off, sid):
        """トルクON/OFF設定

        :param on_off:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # トルクデータ
        torque_data = 0x01 if on_off else 0x00

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_TORQUE_ENABLE, [torque_data])

        # データ送信バッファに追加
        self.add_command(command)

    def get_temperature(self, sid, callback=None):
        """温度取得（単位: ℃。おおよそ±3℃程度の誤差あり）

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                temperature = int.from_bytes(response_data, 'little', signed=True)
                return temperature
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_TEMPERATURE_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_temperature_async(self, sid, loop=None):
        """温度取得 async版（単位: ℃。おおよそ±3℃程度の誤差あり）

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_temperature(sid, callback=callback)
        return f

    def get_current(self, sid, callback=None):
        """電流(現在の負荷)取得 (単位: mA)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                current = int.from_bytes(response_data, 'little', signed=False)
                return current
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_PRESENT_CURRENT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_current_async(self, sid, loop=None):
        """電流(現在の負荷)取得 async版 (単位: mA)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_current(sid, callback=callback)
        return f

    def get_target_position(self, sid, callback=None):
        """指示位置取得 (単位: 度)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                # 単位は 0.1 度になっているので、度に変換
                position = int.from_bytes(response_data, 'little', signed=True)
                position /= 10
                return position
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_GOAL_POSITION_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_target_position_async(self, sid, loop=None):
        """指示位置取得 async版 (単位: 度)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_target_position(sid, callback=callback)
        return f

    def set_target_position(self, position_degree, sid=1):
        """指示位置設定 (単位: 度。設定可能な範囲は-150.0 度~+150.0 度)

        :param position_degree:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 設定可能な範囲は-150.0 度~+150.0 度
        if position_degree < -150:
            position_degree = -150
        elif position_degree > 150:
            position_degree = 150

        position_hex = format(int(position_degree * 10) & 0xffff, '04x')
        position_hex_h = int(position_hex[0:2], 16)
        position_hex_l = int(position_hex[2:4], 16)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_GOAL_POSITION_L, [position_hex_l, position_hex_h])

        # データ送信バッファに追加
        self.add_command(command)

    def get_current_position(self, sid, callback=None):
        """現在位置取得 (単位: 度)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                # 単位は 0.1 度になっているので、度に変換
                position = int.from_bytes(response_data, 'little', signed=True)
                position /= 10
                return position
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_PRESENT_POSITION_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_current_position_async(self, sid, loop=None):
        """現在位置取得 async版 (単位: 度)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_current_position(sid, callback=callback)
        return f

    def get_voltage(self, sid, callback=None):
        """電圧取得 (単位: V)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                voltage = int.from_bytes(response_data, 'little', signed=True)
                voltage /= 100
                return voltage
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_VOLTAGE_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_voltage_async(self, sid, loop=None):
        """電圧取得 async版 (単位: V)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_voltage(sid, callback=callback)
        return f

    def get_target_time(self, sid, callback=None):
        """目標位置までのサーボ移動時間を取得 (単位: 秒)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                # 単位は 10ms 単位になっているので、秒に変更
                speed = int.from_bytes(response_data, 'little', signed=False)
                speed /= 100
                return speed
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_GOAL_TIME_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_target_time_async(self, sid, loop=None):
        """目標位置までのサーボ移動時間を取得 async版 (単位: 秒)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_speed(sid, callback=callback)
        return f

    def set_target_time(self, speed_second, sid=1):
        """目標位置までのサーボ移動時間を設定 (単位: 秒)

        :param speed_second:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 設定範囲は 0 から 3FFFH。つまり0秒から163830ms=163.83seconds
        if speed_second < 0:
            speed_second = 0
        elif speed_second > 163.83:
            speed_second = 163.83

        # 10ms 単位で設定。この関数のパラメータは秒指定なので*100する
        speed_hex = format(int(speed_second * 100) & 0xffff, '04x')
        speed_hex_h = int(speed_hex[0:2], 16)
        speed_hex_l = int(speed_hex[2:4], 16)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_GOAL_TIME_L, [speed_hex_l, speed_hex_h])

        # データ送信バッファに追加
        self.add_command(command)

    def get_pid_coefficient(self, sid, callback=None):
        """モータの制御係数を取得 (単位: %)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                coef = response_data[0]
                return coef
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_PID_COEFFICIENT, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
                                   sid=sid, callback=callback)

    def get_pid_coefficient_async(self, sid, loop=None):
        """モータの制御係数を取得 async版 (単位: %)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_pid_coefficient(sid, callback=callback)
        return f

    def set_pid_coefficient(self, coef_percent, sid):
        """モータの制御係数を設定 (単位: %)

        :param coef_percent:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 100%のとき設定値は、64H となります。設定範囲は 01H~FFH までです。
        if coef_percent < 1:
            coef_percent = 1
        elif coef_percent == 10:
            # なぜか10に設定すると反応がおかしいので9にシフトさせる
            coef_percent = 9
        elif coef_percent > 255:
            coef_percent = 255

        coef_hex = int(coef_percent)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_PID_COEFFICIENT, [coef_hex])

        # データ送信バッファに追加
        self.add_command(command)

    def get_max_torque(self, sid, callback=None):
        """最大トルク取得 (%)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                max_torque = response_data[0]
                return max_torque
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_MAX_TORQUE, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
                                   sid=sid, callback=callback)

    def get_max_torque_async(self, sid, loop=None):
        """最大トルク取得 async版 (%)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_max_torque(sid, callback=callback)
        return f

    def set_max_torque(self, torque_percent, sid):
        """最大トルク設定 (%)

        :param torque_percent:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 0-100%で設定
        if torque_percent < 0:
            torque_percent = 0
        elif torque_percent > 100:
            torque_percent = 100

        torque_hex = int(torque_percent)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_MAX_TORQUE, [torque_hex])

        # データ送信バッファに追加
        self.add_command(command)

    def get_speed(self, sid, callback=None):
        """現在の回転速度を取得 (deg/s)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                speed = int.from_bytes(response_data, 'little', signed=True)
                return speed
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_PRESENT_SPEED_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_speed_async(self, sid, loop=None):
        """現在の回転速度を取得 async版 (deg/s)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_speed(sid, callback=callback)
        return f

    def set_speed(self, dps, sid):
        """Futabaでは未サポート"""
        raise NotSupportException('Futabaではset_speedに対応していません。set_target_time()で回転スピードを制御してください。')

    def get_servo_id(self, sid, callback=None):
        """サーボIDを取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                servo_id = int.from_bytes(response_data, 'little', signed=False)
                return servo_id
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_SERVO_ID, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
                                   sid=sid, callback=callback)

    def get_servo_id_async(self, sid, loop=None):
        """サーボIDを取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_servo_id(sid, callback=callback)
        return f

    def set_servo_id(self, new_sid, sid):
        """サーボIDを設定

        :param new_sid:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(new_sid)
        self.__check_sid(sid)

        # 0-100%で設定
        if new_sid < 1:
            new_sid = 1
        elif new_sid > 127:
            new_sid = 127

        new_sid_hex = int(new_sid)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_SERVO_ID, [new_sid_hex])

        # データ送信バッファに追加
        self.add_command(command)

    def save_rom(self, sid):
        """フラッシュROMに書き込む

        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_WRITE_FLASH_ROM, flag=0x40, count=0)

        # データ送信バッファに追加
        self.add_command(command)

    def get_baud_rate(self, sid, callback=None):
        """通信速度を取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                baud_rate = int.from_bytes(response_data, 'little', signed=False)
                return baud_rate
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_BAUD_RATE, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
                                   sid=sid, callback=callback)

    def get_baud_rate_async(self, sid, loop=None):
        """通信速度を取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_baud_rate(sid, callback=callback)
        return f

    def set_baud_rate(self, baud_rate_id, sid):
        """通信速度を設定

        :param baud_rate_id:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 通信速度IDのチェック
        if self.BAUD_RATE_INDEX_9600 < baud_rate_id > self.BAUD_RATE_INDEX_230400:
            raise BadInputParametersException('baud_rate_id が不正な値です')

        baud_rate_id_hex = int(baud_rate_id)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_BAUD_RATE, [baud_rate_id_hex])

        # データ送信バッファに追加
        self.add_command(command)

    def get_limit_cw_position(self, sid, callback=None):
        """右(時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                limit_position = int.from_bytes(response_data, 'little', signed=True)
                limit_position /= 10
                return limit_position
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_CW_ANGLE_LIMIT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_limit_cw_position_async(self, sid, loop=None):
        """右(時計回り)リミット角度の取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_limit_cw_position(sid, callback=callback)
        return f

    def set_limit_cw_position(self, limit_position, sid):
        """右(時計回り)リミット角度を設定

        :param limit_position:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # リミット角度のチェック
        if 0 < limit_position > 150:
            raise BadInputParametersException('limit_position が不正な値です。0〜+150を設定してください。')

        limit_position_hex = format(int(limit_position * 10) & 0xffff, '04x')
        limit_position_hex_h = int(limit_position_hex[0:2], 16)
        limit_position_hex_l = int(limit_position_hex[2:4], 16)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_CW_ANGLE_LIMIT_L, [limit_position_hex_l, limit_position_hex_h])

        # データ送信バッファに追加
        self.add_command(command)

    def get_limit_ccw_position(self, sid, callback=None):
        """左(反時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                limit_position = int.from_bytes(response_data, 'little', signed=True)
                limit_position /= 10
                return limit_position
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_CCW_ANGLE_LIMIT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_limit_ccw_position_async(self, sid, loop=None):
        """左(反時計回り)リミット角度の取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_limit_ccw_position(sid, callback=callback)
        return f

    def set_limit_ccw_position(self, limit_position, sid):
        """左(反時計回り)リミット角度を設定

        :param limit_position:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # リミット角度のチェック
        if -150 < limit_position > 0:
            raise BadInputParametersException('limit_position が不正な値です。-150〜0を設定してください。')

        limit_position_hex = format(int(limit_position * 10 & 0xffff), '04x')
        limit_position_hex_h = int(limit_position_hex[0:2], 16)
        limit_position_hex_l = int(limit_position_hex[2:4], 16)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_CCW_ANGLE_LIMIT_L,
                                          [limit_position_hex_l, limit_position_hex_h])

        # データ送信バッファに追加
        self.add_command(command)

    def get_limit_temperature(self, sid, callback=None):
        """温度リミットの取得 (℃)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                limit_temp = int.from_bytes(response_data, 'little', signed=True)
                return limit_temp
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.ADDR_TEMPERATURE_LIMIT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
                                   sid=sid, callback=callback)

    def get_limit_temperature_async(self, sid, loop=None):
        """温度リミットの取得 (℃) async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.get_limit_temperature(sid, callback=callback)
        return f

    def set_limit_temperature(self, limit_temp, sid):
        """Futabaでは未サポート"""
        raise NotSupportException('Futabaではset_limit_temperatureに対応していません。')

    def set_burst_target_positions(self, sid_target_positions):
        """複数のサーボの対象ポジションを一度に設定

        :param sid_target_positions:
        :return:
        """

        # データチェック & コマンドデータ生成
        vid_data = {}
        for sid, position_degree in sid_target_positions.items():
            # サーボIDのチェック
            self.__check_sid(sid)

            # 設定可能な範囲は-150.0 度~+150.0 度
            if position_degree < -150:
                position_degree = -150
            elif position_degree > 150:
                position_degree = 150

            position_hex = format(int(position_degree * 10) & 0xffff, '04x')
            position_hex_h = int(position_hex[0:2], 16)
            position_hex_l = int(position_hex[2:4], 16)

            vid_data[sid] = [position_hex_l, position_hex_h]

        # コマンド生成
        command = self.__generate_burst_command(self.ADDR_GOAL_POSITION_L, 3, vid_data)

        # データ送信バッファに追加
        self.add_command(command)

    def get_burst_positions(self, sids, callback=None):
        """複数のサーボの現在のポジションを一気にリード

        :param sids:
        :param callback:
        :return:
        """
        """Futabaでは未サポート"""
        raise NotSupportException('Futabaではget_burst_positionsに対応していません。')

    def get_burst_positions_async(self, sids, loop=None):
        """複数のサーボの現在のポジションを一気にリード async版

        :param sids:
        :param loop:
        :return:
        """
        """Futabaでは未サポート"""
        raise NotSupportException('Futabaではget_burst_positions_asyncに対応していません。')

    def reset_memory(self, sid):
        """ROMを工場出荷時のものに初期化する

        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_RESET_MEMORY, flag=self.FLAG4_RESET_MEMORY_MAP, count=0)

        # データ送信バッファに追加
        self.add_command(command)

    def read(self, sid, address, length, callback=None):
        """データを読み込む

        :param sid:
        :param address:
        :param length:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == length:
                return response_data
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(address, self.FLAG30_MEM_MAP_SELECT, length, response_process,
                                   sid=sid, callback=callback)

    def read_async(self, sid, address, length, loop=None):
        """データを読み込む async版

        :param sid:
        :param address:
        :param length:
        :param loop:
        :return:
        """

        f, callback = self.__async_wrapper(loop)
        self.read(sid, address, length, callback=callback)
        return f

    def write(self, sid, address, data):
        """データを書き込む

        :param sid:
        :param address:
        :param data:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # コマンド生成
        command = self.__generate_command(sid, address, data)

        # データ送信バッファに追加
        self.add_command(command)

    def burst_read(self, sid_address_length, callback=None):
        """複数サーボから一括でデータ読み取り

        :param sid_address_length:
        :param callback:
        :return:
        """
        """Futabaでは未サポート"""
        raise NotSupportException('Futabaではburst_readに対応していません。')

    def burst_read_async(self, sid_address_length, loop=None):
        """複数サーボから一括でデータ読み取り async版

        :param sid_address_length:
        :param loop:
        :return:
        """
        """Futabaでは未サポート"""
        raise NotSupportException('Futabaではburst_read_asyncに対応していません。')

    def burst_write(self, address, length, sid_data):
        """複数サーボに一括で書き込み

        :param address:
        :param length:
        :param sid_data:
        :return:
        """

        # データチェック & コマンドデータ生成
        vid_data = {}
        for sid, data in sid_data.items():
            # サーボIDのチェック
            self.__check_sid(sid)

            vid_data[sid] = data

        # コマンド生成
        command = self.__generate_burst_command(address, length, vid_data)

        # データ送信バッファに追加
        self.add_command(command)
