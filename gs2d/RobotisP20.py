# ! /usr/bin/env python3
# encoding: utf-8

import time
import logging

from .ICommandHandler import ICommandHandler
from .ISerialInterface import ISerialInterface
from .Driver import Driver
from .Util import ReceiveDataTimeoutException, NotSupportException, BadInputParametersException, WrongCheckSumException
from .Util import InvalidResponseDataException
from .Util import get_printable_hex

# ロガー
logger = logging.getLogger(__name__)

RECEIVE_DATA_TIMEOUT_SEC = 19


class RobotisP20(Driver):
    """Robotis社のP2.0のプロトコルに対応したシリアルサーボクラス

    """

    # インストラクション
    INSTRUCTION_PING = 0x01
    INSTRUCTION_READ = 0x02
    INSTRUCTION_WRITE = 0x03
    INSTRUCTION_REG_WRITE = 0x04
    INSTRUCTION_ACTION = 0x05
    INSTRUCTION_FACTORY_RESET = 0x06
    INSTRUCTION_REBOOT = 0x08
    INSTRUCTION_SYNC_READ = 0x82
    INSTRUCTION_SYNC_WRITE = 0x83
    INSTRUCTION_BULK_READ = 0x92
    INSTRUCTION_BULK_WRITE = 0x93

    # アドレス空間
    ADDR_MODEL_NUMBER = 0
    ADDR_MODEL_INFORMATION = 2
    ADDR_VERSION_FIRMWARE = 6

    ADDR_ID = 7
    ADDR_BAUDRATE = 8
    ADDR_RETURN_DELAY_TIME = 9
    ADDR_DRIVE_MODE = 10
    ADDR_OPERATING_MODE = 11
    ADDR_SECONDARY_ID = 12
    ADDR_PROTOCOL_VERSION = 13
    ADDR_HOMING_OFFSET = 20
    ADDR_MOVING_THRESHOLD = 24
    ADDR_TEMPERATURE_LIMIT = 31
    ADDR_MAX_VOLTAGE_LIMIT = 32
    ADDR_MIN_VOLTAGE_LIMIT = 34
    ADDR_PWM_LIMIT = 36
    ADDR_CURRENT_LIMIT = 38
    ADDR_ACCELERATION_LIMIT = 40
    ADDR_VELOCITY_LIMIT = 44
    ADDR_MAX_POSITION_LIMIT = 48
    ADDR_MIN_POSITION_LIMIT = 52
    ADDR_EXTERNAL_PORT_MODE1 = 56
    ADDR_EXTERNAL_PORT_MODE2 = 57
    ADDR_EXTERNAL_PORT_MODE3 = 58
    ADDR_SHUTDOWN = 63

    ADDR_TORQUE_ENABLE = 64
    ADDR_LED = 65
    ADDR_STATUS_RETURN_LEVEL = 68
    ADDR_REGISTERED_INSTRUCTION = 69
    ADDR_HARDWARE_ERROR_STATUS = 70
    ADDR_VELOCITY_I_GAIN = 76
    ADDR_VELOCITY_P_GAIN = 78
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_FEEDFORWARD_ACCELERATION_GAIN = 88
    ADDR_FEEDFORWARD_VELOCITY_GAIN = 90
    ADDR_BUS_WATCHDOG = 98
    ADDR_GOAL_PWM = 100
    ADDR_GOAL_CURRENT = 102
    ADDR_GOAL_VELOCITY = 104
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = 116

    ADDR_REALTIME_TICK = 120
    ADDR_MOVING = 122
    ADDR_MOVING_STATUS = 123
    ADDR_PRESENT_PWM = 124
    ADDR_PRESENT_CURRENT = 126
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PRESENT_POSITION = 132
    ADDR_VELOCITY_TRAJECTORY = 136
    ADDR_POSITION_TRAJECTORY = 140
    ADDR_PRESENT_INPUT_VOLTAGE = 144
    ADDR_PRESENT_TEMPERATURE = 146

    # 通信速度のIDと実際の設定値
    BAUD_RATE_INDEX_9600 = 0x00
    BAUD_RATE_INDEX_57600 = 0x01
    BAUD_RATE_INDEX_115200 = 0x02
    BAUD_RATE_INDEX_1000000 = 0x03
    BAUD_RATE_INDEX_2000000 = 0x04
    BAUD_RATE_INDEX_3000000 = 0x05
    BAUD_RATE_INDEX_4000000 = 0x06
    BAUD_RATE_INDEX_4500000 = 0x07

    # ステータスパケットのindex
    STATUS_PACKET_LENGTH_INDEX = 5
    STATUS_PACKET_INSTRUCTION_INDEX = 7
    STATUS_PACKET_ERROR_INDEX = 8
    STATUS_PACKET_PARAMETER_INDEX = 9

    STATUS_PACKET_INSTRUCTION = 0x55

    def __init__(self, serial_interface: ISerialInterface, command_handler_class: ICommandHandler = None):
        """初期化
        """

        super(RobotisP20, self).__init__(serial_interface, command_handler_class)

    def is_complete_response(self, response_data):
        """レスポンスデータをすべて受信できたかチェック"""

        # Header, ID, Flags, Address, Length, Count, Data, Sum で7バイトは最低限ある
        if len(response_data) < 6:
            return False
        else:
            # カウント取得
            count = response_data[5]
            # logger.debug('###', count, len(response_data))
            return len(response_data) >= (7 + count)

    def close(self, force=False):
        """閉じる

        :param force:
        :return:
        """

        if self.command_handler:
            self.command_handler.close()

    def __get_checksum(self, data):
        """チェックサム(CRC-16-IBM)を生成

        :param data:
        :return:
        """

        # (X^16+X^15+X^2+1) Polynomial 0x8005
        poly = 0x8005
        n = 16
        g = 1 << n | poly
        crc = 0
        for d in data:
            crc ^= d << (n - 8)
            for _ in range(8):
                crc <<= 1
                if crc & (1 << n):
                    crc ^= g

        # CRCを2bytesに
        crc = self.get_bytes(crc, 2)

        return crc

    @staticmethod
    def __check_sid(sid):
        """Servo IDのレンジをチェック

        :param sid:
        :return:
        """

        # 0~252(0x00~0xFC)の範囲及び254(0xFE)ならOK
        if sid < 0 or sid > 254 or sid == 253:
            raise BadInputParametersException('sid: %d がレンジ外です。0~252(0x00~0xFC)の範囲及び254(0xFE)のIDを設定してください。' % sid)

    def __generate_command(self, sid, instruction, parameters=None, length=None):
        """コマンド生成

        :param sid:
        :param instruction:
        :param parameters:
        :param length:
        :return:
        """

        # Header:      Header,Reserved。先頭4バイトは0xFF,0xFF,0xFD,0x00の固定値。
        # ID:          送信先のデバイスのID。0~252(0x00~0xFC)の範囲及び254(0xFE)。
        #              254のIDは1回のインストラクションパケットで複数のデバイスに対して命令を発効する際に使用。
        # Length:      2byte。Instruction以後の全てのバイト数(16bit幅)。
        # Instruction: PING, READなどのサーボの操作内容を表す。
        # Parameter:   Instructionで要求される追加情報(可変長)。
        # Checksum:    HeaderからParameterまでのチェックサム値(CRC-16-IBM)。
        #              (X^16+X^15+X^2+1) Polynomial 0x8005

        command = []

        # Header
        command.extend([0xFF, 0xFF, 0xFD, 0x00])

        # ID
        command.append(sid)

        # Length
        if length is None:
            if parameters is not None:
                # Instruction: 1byte
                # Parameters: パラメータのデータ長
                # Checksum: 2bytes
                length = 1 + len(parameters) + 2
            else:
                # パラメータなしなので、Instruction(1) + Checksum(2) = 3bytes
                length = 3

        # Lengthを2bytesのデータに変換
        command.extend(self.get_bytes(length, 2))

        # Instruction
        command.append(instruction)

        # Parameters
        if parameters is not None and len(parameters) > 0:
            command.extend(parameters)

        # Header部と一致するデータ列のうしろに0xFDを追加
        # TODO

        # Checksum
        command.extend(self.__get_checksum(command))

        return command

    # def __generate_burst_command(self, addr, length, vid_data_dict):
    #     """バーストコマンド生成
    #
    #     :param addr:
    #     :param length:
    #     :param vid_data_dict:
    #     :return:
    #     """
    #
    #     # Header:  パケットの先頭を表します。ショートパケットではFAAFに設定します。
    #     # ID:      常に00
    #     # Flag:    常に00
    #     # Address: メモリーマップ上のアドレスを指定します。
    #     #          このアドレスから「Length」に指定した長さ分のデータを指定した複数サーボのメモリーマップに書き込みます。
    #     # Length:  サーボ1つ分のデータ (VID+Data) のバイト数を指定します。
    #     # Count:   データを送信する対象となるサーボの数を表します。この数分 VID と Data を送信します。
    #     # VID:     データを送信する個々のサーボの ID を表します。VID と Data が一組でサーボの数分のデータを送信します。
    #     # Data:    メモリーマップに書き込むサーボ一つ分のデータです。VID と Data が一組でサーボの数分のデータを送信します。
    #     # Sum:     送信データの確認用のチェックサムで、パケットのIDからDataの末尾までを1バイトずつ
    #     #          XORした値を指定します。
    #
    #     command = []
    #
    #     # Header
    #     command.extend([0xFA, 0xAF])
    #
    #     # ID
    #     command.append(0)
    #
    #     # Flag
    #     command.append(0)
    #
    #     # Address
    #     command.append(addr)
    #
    #     # Length
    #     command.append(length)
    #
    #     # Count
    #     command.append(len(vid_data_dict))
    #
    #     # Data
    #     for sid, data in vid_data_dict.items():
    #         command.append(sid)
    #         if data is not None and len(data) > 0:
    #             command.extend(data)
    #
    #     # Checksum
    #     command.append(self.__get_checksum(command))
    #
    #     return command

    def __get_function(self, instruction, parameters, response_process=None, sid=1, length=None, callback=None):
        """Get系の処理をまとめた関数

        :param instruction:
        :param parameters:
        :param response_process:
        :param sid:
        :param length:
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

            # ステータスパケットのチェックサムが正しいかチェック
            if len(response) > 2:
                # パラメーターindexまでデータがあるか
                if len(response) <= self.STATUS_PACKET_PARAMETER_INDEX:
                    # TODO: ステータスパケット長エラーexception
                    print('ステータスパケット長エラーexception')
                    return

                # ステータスパケットからInstructionを取得し、0x55かチェック
                status_packet_instruction = response[self.STATUS_PACKET_INSTRUCTION_INDEX]
                if status_packet_instruction != self.STATUS_PACKET_INSTRUCTION:
                    # TODO: ステータスパケット異常exception
                    print('ステータスパケット異常exception ステータスパケットからInstructionを取得し、0x55かチェック')
                    return

                # ステータスパケットからlengthを取得
                status_packet_length = response[self.STATUS_PACKET_LENGTH_INDEX:self.STATUS_PACKET_LENGTH_INDEX + 2]
                status_packet_length = int.from_bytes(status_packet_length, 'little', signed=True)

                if len(response) < self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length:
                    # print('FFFF', len(response), self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length)
                    # TODO: ステータスパケット異常exception
                    print('ステータスパケット異常exception ステータスパケットからlengthを取得')
                    print('########', response)
                    return

                # Errorバイト取得
                status_packet_error = response[self.STATUS_PACKET_ERROR_INDEX]

                if status_packet_error > 0:
                    # TODO: ステータスパケットエラーexception
                    print('ステータスパケットエラーexception')
                    return

                # パラメータ取得
                response_data = response[
                                self.STATUS_PACKET_PARAMETER_INDEX:
                                self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length - 2
                                ]

                # チェックサム検証
                checksum = response[
                           self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length - 2:
                           self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length]
                generated_checksum = self.__get_checksum(
                    response[:self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length - 2]
                )
                if checksum[0] != generated_checksum[0] or checksum[1] != generated_checksum[1]:
                    logger.debug('Check sum error: ' + get_printable_hex(response))
                    is_checksum_error = True
                    return

                # 受信済み
                is_received = True

                # データ処理
                if response_process:
                    recv_data = response_process(response_data)
                else:
                    recv_data = response_data

                if callback is not None:
                    callback(recv_data)
                else:
                    data = recv_data
            else:
                # TODO: 受信エラー
                return

        command = self.__generate_command(sid, instruction, parameters, length=length)
        self.command_handler.add_command(command, recv_callback=temp_recv_callback)

        # コールバックが設定できていたら、コールバックに受信データを渡す
        if callback is None:
            # X秒以内にサーボからデータを受信できたかをチェック
            start = time.time()
            while not is_received:
                elapsed_time = time.time() - start
                if elapsed_time > self.command_handler.RECEIVE_DATA_TIMEOUT_SEC:
                    raise ReceiveDataTimeoutException(
                        str(self.command_handler.RECEIVE_DATA_TIMEOUT_SEC) + '秒以内にデータ受信できませんでした'
                    )
                elif is_checksum_error:
                    raise WrongCheckSumException('受信したデータのチェックサムが不正です')

            return data
        else:
            return True

    def __generate_parameters_read_write(self, start_address, data, data_size):
        """Read/Write系のパラメータを生成する

        :param start_address:
        :param data:
        :param data_size:
        :return:
        """

        params = []
        params.extend(self.get_bytes(start_address, 2))
        params.extend(self.get_bytes(data, data_size))

        return params

    def __callback_write_response(self, response_data):
        """WRITEインストラクション時の返り値ハンドリング

        :param response_data:
        :return:
        """

        # 返り値
        if len(response_data) != 0:
            # TODO: レスポンス以上exception
            print('WRITEインストラクション返り値レスポンス以上exception')

    def ping(self, sid, callback=None):
        """サーボにPINGを送る

        :param sid:
        :param callback:
        :return: {
            'model_no': bytearray (2bytes),
            'version_firmware': int (1byte)
        }
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 3:
                model_no = response_data[0:2]
                version_firmware = response_data[2]
                return {
                    'model_no': model_no,
                    'version_firmware': version_firmware
                }
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self.__get_function(self.INSTRUCTION_PING, None, response_process, sid=sid, length=3, callback=callback)

    def ping_async(self, sid, loop=None):
        """サーボにPINGを送る async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.ping(sid, callback=callback)
        return f

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

        params = self.__generate_parameters_read_write(self.ADDR_TORQUE_ENABLE, 1, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_torque_enable_async(self, sid, loop=None):
        """トルクON取得async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_torque_enable(sid, callback=callback)
        return f

    def set_torque_enable(self, on_off, sid):
        """トルクON/OFF設定

        :param on_off:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                return response_data[0] == 0x01
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        # トルクデータ
        torque_data = 0x01 if on_off else 0x00

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_TORQUE_ENABLE, torque_data, 2)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_temperature(self, sid, callback=None):
        """現在の内部温度（単位: ℃）

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                temperature = int.from_bytes(response_data, 'little', signed=True)
                return temperature
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_PRESENT_TEMPERATURE, 1, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_temperature_async(self, sid, loop=None):
        """温度取得 async版（単位: ℃。おおよそ±3℃程度の誤差あり）

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 2:
        #         current = int.from_bytes(response_data, 'little', signed=False)
        #         return current
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_PRESENT_CURRENT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
        #                            sid=sid, callback=callback)

    def get_current_async(self, sid, loop=None):
        """電流(現在の負荷)取得 async版 (単位: mA)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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
            if response_data is not None and len(response_data) == 4:
                # 単位は 0.1 度になっているので、度に変換
                position = int.from_bytes(response_data, 'little', signed=True)
                position = position * 360 / 4096
                position = position - 180
                return position
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_GOAL_POSITION, 4, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_target_position_async(self, sid, loop=None):
        """指示位置取得 async版 (単位: 度)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_target_position(sid, callback=callback)
        return f

    def set_target_position(self, position_degree, sid=1):
        """指示位置設定 (単位: 度。設定可能な範囲は-180.0 度~+180.0 度)

        :param position_degree:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 位置 (-180から180まで)
        if position_degree < -180:
            position_degree = -180
        elif position_degree > 180:
            position_degree = 180

        # Dynamixelでは0〜360°なので変換
        position_degree += 180

        # データ変換
        position_degree = position_degree * 4096 / 360

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_GOAL_POSITION, position_degree, 4)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_current_position(self, sid, callback=None):
        """現在位置取得 (単位: 度)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 4:
                # 単位は 0.1 度になっているので、度に変換
                position = int.from_bytes(response_data, 'little', signed=True)
                position = position * 360 / 4096
                position = position - 180
                return position
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_PRESENT_POSITION, 4, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_current_position_async(self, sid, loop=None):
        """現在位置取得 async版 (単位: 度)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 2:
        #         voltage = int.from_bytes(response_data, 'little', signed=True)
        #         voltage /= 100
        #         return voltage
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_VOLTAGE_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
        #                            sid=sid, callback=callback)

    def get_voltage_async(self, sid, loop=None):
        """電圧取得 async版 (単位: V)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 2:
        #         # 単位は 10ms 単位になっているので、秒に変更
        #         speed = int.from_bytes(response_data, 'little', signed=False)
        #         speed /= 100
        #         return speed
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_GOAL_TIME_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
        #                            sid=sid, callback=callback)

    def get_target_time_async(self, sid, loop=None):
        """目標位置までのサーボ移動時間を取得 async版 (単位: 秒)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_speed(sid, callback=callback)
        return f

    def set_target_time(self, speed_second, sid):
        """目標位置までのサーボ移動時間を設定 (単位: 秒)

        :param speed_second:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # # 設定範囲は 0 から 3FFFH。つまり0秒から163830ms=163.83seconds
        # if speed_second < 0:
        #     speed_second = 0
        # elif speed_second > 163.83:
        #     speed_second = 163.83
        #
        # # 10ms 単位で設定。この関数のパラメータは秒指定なので*100する
        # speed_hex = format(int(speed_second * 100) & 0xffff, '04x')
        # speed_hex_h = int(speed_hex[0:2], 16)
        # speed_hex_l = int(speed_hex[2:4], 16)
        #
        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_GOAL_TIME_L, [speed_hex_l, speed_hex_h])
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def get_pid_coefficient(self, sid, callback=None):
        """モータの制御係数を取得 (単位: %)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 1:
        #         coef = response_data[0]
        #         return coef
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_PID_COEFFICIENT, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
        #                            sid=sid, callback=callback)

    def get_pid_coefficient_async(self, sid, loop=None):
        """モータの制御係数を取得 async版 (単位: %)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # 100%のとき設定値は、64H となります。設定範囲は 01H~FFH までです。
        # if coef_percent < 1:
        #     coef_percent = 1
        # elif coef_percent == 10:
        #     # なぜか10に設定すると反応がおかしいので9にシフトさせる
        #     coef_percent = 9
        # elif coef_percent > 255:
        #     coef_percent = 255
        #
        # coef_hex = int(coef_percent)
        #
        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_PID_COEFFICIENT, [coef_hex])
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def get_max_torque(self, sid, callback=None):
        """最大トルク取得 (%)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 1:
        #         max_torque = response_data[0]
        #         return max_torque
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_MAX_TORQUE, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
        #                            sid=sid, callback=callback)

    def get_max_torque_async(self, sid, loop=None):
        """最大トルク取得 async版 (%)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # 0-100%で設定
        # if torque_percent < 0:
        #     torque_percent = 0
        # elif torque_percent > 100:
        #     torque_percent = 100
        #
        # torque_hex = int(torque_percent)
        #
        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_MAX_TORQUE, [torque_hex])
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def get_speed(self, sid, callback=None):
        """現在の回転速度を取得 (deg/s)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 2:
        #         speed = int.from_bytes(response_data, 'little', signed=True)
        #         return speed
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_PRESENT_SPEED_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
        #                            sid=sid, callback=callback)

    def get_speed_async(self, sid, loop=None):
        """現在の回転速度を取得 async版 (deg/s)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 1:
        #         servo_id = int.from_bytes(response_data, 'little', signed=False)
        #         return servo_id
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_SERVO_ID, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
        #                            sid=sid, callback=callback)

    def get_servo_id_async(self, sid, loop=None):
        """サーボIDを取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # 0-100%で設定
        # if new_sid < 1:
        #     new_sid = 1
        # elif new_sid > 127:
        #     new_sid = 127
        #
        # new_sid_hex = int(new_sid)
        #
        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_SERVO_ID, [new_sid_hex])
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def save_rom(self, sid):
        """フラッシュROMに書き込む

        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_WRITE_FLASH_ROM, flag=0x40, count=0)
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def get_baud_rate(self, sid, callback=None):
        """通信速度を取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 1:
        #         baud_rate = int.from_bytes(response_data, 'little', signed=False)
        #         return baud_rate
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_BAUD_RATE, self.FLAG30_MEM_MAP_SELECT, 1, response_process,
        #                            sid=sid, callback=callback)

    def get_baud_rate_async(self, sid, loop=None):
        """通信速度を取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # 通信速度IDのチェック
        # if self.BAUD_RATE_INDEX_9600 < baud_rate_id > self.BAUD_RATE_INDEX_230400:
        #     raise BadInputParametersException('baud_rate_id が不正な値です')
        #
        # baud_rate_id_hex = int(baud_rate_id)
        #
        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_BAUD_RATE, [baud_rate_id_hex])
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def get_limit_cw_position(self, sid, callback=None):
        """右(時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 2:
        #         limit_position = int.from_bytes(response_data, 'little', signed=True)
        #         limit_position /= 10
        #         return limit_position
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_CW_ANGLE_LIMIT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
        #                            sid=sid, callback=callback)

    def get_limit_cw_position_async(self, sid, loop=None):
        """右(時計回り)リミット角度の取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # リミット角度のチェック
        # if 0 < limit_position > 150:
        #     raise BadInputParametersException('limit_position が不正な値です。0〜+150を設定してください。')
        #
        # limit_position_hex = format(int(limit_position * 10) & 0xffff, '04x')
        # limit_position_hex_h = int(limit_position_hex[0:2], 16)
        # limit_position_hex_l = int(limit_position_hex[2:4], 16)
        #
        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_CW_ANGLE_LIMIT_L, [limit_position_hex_l, limit_position_hex_h])
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def get_limit_ccw_position(self, sid, callback=None):
        """左(反時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 2:
        #         limit_position = int.from_bytes(response_data, 'little', signed=True)
        #         limit_position /= 10
        #         return limit_position
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_CCW_ANGLE_LIMIT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
        #                            sid=sid, callback=callback)

    def get_limit_ccw_position_async(self, sid, loop=None):
        """左(反時計回り)リミット角度の取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # リミット角度のチェック
        # if -150 < limit_position > 0:
        #     raise BadInputParametersException('limit_position が不正な値です。-150〜0を設定してください。')
        #
        # limit_position_hex = format(int(limit_position * 10 & 0xffff), '04x')
        # limit_position_hex_h = int(limit_position_hex[0:2], 16)
        # limit_position_hex_l = int(limit_position_hex[2:4], 16)
        #
        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_CCW_ANGLE_LIMIT_L,
        #                                   [limit_position_hex_l, limit_position_hex_h])
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

    def get_limit_temperature(self, sid, callback=None):
        """温度リミットの取得 (℃)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == 2:
        #         limit_temp = int.from_bytes(response_data, 'little', signed=True)
        #         return limit_temp
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(self.ADDR_TEMPERATURE_LIMIT_L, self.FLAG30_MEM_MAP_SELECT, 2, response_process,
        #                            sid=sid, callback=callback)

    def get_limit_temperature_async(self, sid, loop=None):
        """温度リミットの取得 (℃) async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # データチェック & コマンドデータ生成
        # vid_data = {}
        # for sid, position_degree in sid_target_positions.items():
        #     # サーボIDのチェック
        #     self.__check_sid(sid)
        #
        #     # 設定可能な範囲は-150.0 度~+150.0 度
        #     if position_degree < -150:
        #         position_degree = -150
        #     elif position_degree > 150:
        #         position_degree = 150
        #
        #     position_hex = format(int(position_degree * 10) & 0xffff, '04x')
        #     position_hex_h = int(position_hex[0:2], 16)
        #     position_hex_l = int(position_hex[2:4], 16)
        #
        #     vid_data[sid] = [position_hex_l, position_hex_h]
        #
        # # コマンド生成
        # command = self.__generate_burst_command(self.ADDR_GOAL_POSITION_L, 3, vid_data)
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)
        pass

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

        # # コマンド生成
        # command = self.__generate_command(sid, self.ADDR_RESET_MEMORY, flag=self.FLAG4_RESET_MEMORY_MAP, count=0)
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

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

        # def response_process(response_data):
        #     if response_data is not None and len(response_data) == length:
        #         return response_data
        #     else:
        #         raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        #
        # return self.__get_function(address, self.FLAG30_MEM_MAP_SELECT, length, response_process,
        #                            sid=sid, callback=callback)

    def read_async(self, sid, address, length, loop=None):
        """データを読み込む async版

        :param sid:
        :param address:
        :param length:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
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

        # # コマンド生成
        # command = self.__generate_command(sid, address, data)
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)

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

        # # データチェック & コマンドデータ生成
        # vid_data = {}
        # for sid, data in sid_data.items():
        #     # サーボIDのチェック
        #     self.__check_sid(sid)
        #
        #     vid_data[sid] = data
        #
        # # コマンド生成
        # command = self.__generate_burst_command(address, length, vid_data)
        #
        # # データ送信バッファに追加
        # self.add_command_queue(command)
        pass
