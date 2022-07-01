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
                    raise InvalidResponseDataException("ステータスパケット長が不正です")
                    return

                # ステータスパケットからInstructionを取得し、0x55かチェック
                status_packet_instruction = response[self.STATUS_PACKET_INSTRUCTION_INDEX]
                if status_packet_instruction != self.STATUS_PACKET_INSTRUCTION:
                    # TODO: ステータスパケット異常exception
                    raise InvalidResponseDataException("ステータスパケットのInstructionが不正です")
                    return

                # ステータスパケットからlengthを取得
                status_packet_length = response[self.STATUS_PACKET_LENGTH_INDEX:self.STATUS_PACKET_LENGTH_INDEX + 2]
                status_packet_length = int.from_bytes(status_packet_length, 'little', signed=True)

                if len(response) < self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length:
                    # print('FFFF', len(response), self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length)
                    # TODO: ステータスパケット異常exception
                    raise InvalidResponseDataException("ステータスパケットの長さが不正です")
                    return

                # Errorバイト取得
                status_packet_error = response[self.STATUS_PACKET_ERROR_INDEX]

                if status_packet_error > 0:
                    # TODO: ステータスパケットエラーexception
                    raise InvalidResponseDataException("ステータスパケットが不正です")
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

    def  __get_function_burstread(self, instruction, parameters, sid, num, response_process=None, length=None, callback=None):
        """Get系の処理をまとめた関数

        :param instruction:
        :param parameters:
        :param sid:
        :param num:
        :param response_process:
        :param length:
        :param callback:
        :return:
        """

        data = None
        data_list = []

        # 受信済みフラグ
        is_received = False
        received_num = 0

        # チェックサム不正エラー
        is_checksum_error = False

        def temp_recv_callback(response):
            nonlocal data
            nonlocal data_list
            nonlocal is_received
            nonlocal received_num
            nonlocal is_checksum_error

            # ステータスパケットのチェックサムが正しいかチェック
            if len(response) > 2:
                # パラメーターindexまでデータがあるか
                if len(response) <= self.STATUS_PACKET_PARAMETER_INDEX:
                    raise InvalidResponseDataException("ステータスパケット長が不正です")
                    return

                # ステータスパケットからInstructionを取得し、0x55かチェック
                status_packet_instruction = response[self.STATUS_PACKET_INSTRUCTION_INDEX]
                if status_packet_instruction != self.STATUS_PACKET_INSTRUCTION:
                    raise InvalidResponseDataException("ステータスパケットが不正です")
                    return

                # ステータスパケットからlengthを取得
                status_packet_length = response[self.STATUS_PACKET_LENGTH_INDEX:self.STATUS_PACKET_LENGTH_INDEX + 2]
                status_packet_length = int.from_bytes(status_packet_length, 'little', signed=True)

                if len(response) < self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length:
                    # print('FFFF', len(response), self.STATUS_PACKET_INSTRUCTION_INDEX + status_packet_length)
                    raise InvalidResponseDataException("ステータスパケットが不正です")
                    return

                # Errorバイト取得
                status_packet_error = response[self.STATUS_PACKET_ERROR_INDEX]

                if status_packet_error > 0:
                    raise InvalidResponseDataException("ステータスパケットが不正です")
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

                # データ処理
                if response_process:
                    recv_data = response_process(response_data)
                else:
                    recv_data = response_data

                data_list.append({'id':response[4], 'data':recv_data })

                received_num = received_num + 1

                if received_num == num:
                    # 受信済み
                    is_received = True

                    if callback is not None:
                        callback(data_list)
                    else:
                        data = data_list
            else:
                # TODO: 受信エラー
                return

        command = self.__generate_command(sid, instruction, parameters, length=length)
        self.command_handler.add_command(command, recv_callback=temp_recv_callback, count=num)

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

    def __generate_parameters_sync_read(self, start_address, length, data_list=None):
        """Sync Readのパラメータを生成する

        :param start_address:
        :param length:
        :param data_list:
        :return:
        """
        params = []
        params.extend(self.get_bytes(start_address, 2))
        params.extend(self.get_bytes(length, 2))
        if data_list is not None:
            params.extend(data_list)
        return params

    def __generate_parameters_sync_write(self, start_address, length, data_list=None):
        """Sync Writeのパラメータを生成する

        :param start_address:
        :param length:
        :param data_list:
        :return:
        """
        params = []
        params.extend(self.get_bytes(start_address, 2))
        params.extend(self.get_bytes(length, 2))
        if data_list is not None:
            for i in range(0, len(data_list), 2):
                params.extend(self.get_bytes(data_list[i], 1))
                params.extend(self.get_bytes(data_list[i + 1], length))
        return params

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
            raise InvalidResponseDataException("Writeの返り値が多すぎます")

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
                model_no = int.from_bytes(response_data[0:2], 'little', signed=True)
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

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                current = int.from_bytes(response_data, 'little', signed = True) * 2.69
                return current
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_PRESENT_CURRENT, 2, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

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

    def get_offset(self, sid, callback=None):
        """オフセット角度取得 (単位: 度)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 4:
                # 単位は 0.1 度になっているので、度に変換
                offset = int.from_bytes(response_data, 'little', signed=True)
                offset = offset * 0.088
                return offset
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_HOMING_OFFSET, 4, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_offset_async(self, sid, loop=None):
        """オフセット角度取得 async版 (単位: 度)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_offset(sid, callback=callback)
        return f

    def set_offset(self, offset, sid):
        """オフセット角度指定 (単位: 度)

        :param offset:
        :param sid:
        :return:
        """
        # サーボIDのチェック
        self.__check_sid(sid)

        # データ変換
        offset = offset / 0.088

        if offset < -1044479:
            offset = -1044479
        elif offset > 1044479:
            offset = 1044479

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_HOMING_OFFSET, offset, 4)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_deadband(self, sid, callback=None):
        raise NotSupportException('RobotisP20ではget_deadbandに対応していません。')

    def get_deadband_async(self, sid, loop=None):
        raise NotSupportException('RobotisP20ではget_deadband_asyncに対応していません。')

    def set_deadband(self, deadband, sid):
        raise NotSupportException('RobotisP20ではset_deadbandに対応していません。')


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
                voltage = voltage / 10
                return voltage
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_PRESENT_INPUT_VOLTAGE, 2, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

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

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                target_time = int.from_bytes(response_data, 'little', signed = False)
                target_time = target_time / 1000.0
                return target_time
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_PROFILE_VELOCITY, 2, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_target_time_async(self, sid, loop=None):
        """目標位置までのサーボ移動時間を取得 async版 (単位: 秒)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_target_time(sid, callback=callback)
        return f

    def set_target_time(self, speed_second, sid):
        """目標位置までのサーボ移動時間を設定 (単位: 秒)

        :param speed_second:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        if speed_second < 0:
            speed_second = 0
        elif speed_second > 32.737:
            speed_second = 32.737

        speed_second *= 1000

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_PROFILE_VELOCITY, speed_second, 4)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_accel_time(self, sid, callback=None):
        """目標位置までのサーボ加速時間を取得 (単位: 秒)

        :param sid:
        :param callback:
        :return:
        """
        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 4:
                speed = int.from_bytes(response_data, 'little', signed = False)
                speed = speed / 1000.0
                return speed
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_PROFILE_ACCELERATION, 4, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_accel_time_async(self, sid, loop=None):
        """目標位置までのサーボ加速時間を取得 async版 (単位: 秒)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_accel_time(sid, callback=callback)
        return f

    def set_accel_time(self, speed_second, sid):
        """目標位置までのサーボ加速時間を設定 (単位: 秒)

        :param speed_second:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        if speed_second < 0:
            speed_second = 0
        elif speed_second > 32.737:
            speed_second = 32.737

        speed_second *= 1000

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_PROFILE_ACCELERATION, speed_second, 4)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_pid_coefficient(self, sid, callback=None):
        """モータの制御係数を取得 (単位: %)

        :param sid:
        :param callback:
        :return:
        """
        raise NotSupportException('RobotisP20ではget_pid_coefficientに対応していません。')

    def get_pid_coefficient_async(self, sid, loop=None):
        """モータの制御係数を取得 async版 (単位: %)

        :param sid:
        :param loop:
        :return:
        """
        raise NotSupportException('RobotisP20ではget_pid_coefficient_asyncに対応していません。')

    def set_pid_coefficient(self, coef_percent, sid):
        """モータの制御係数を設定 (単位: %)

        :param coef_percent:
        :param sid:
        :return:
        """
        raise NotSupportException('RobotisP20ではset_pid_coefficientに対応していません。PIDゲインを個別に設定してください。')

    def get_p_gain(self, sid, callback=None):
        """ pGainの取得（単位無し

        :param sid:
        :param callback
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                pGain = int.from_bytes(response_data, 'little', signed = False)
                return pGain
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_POSITION_P_GAIN, 2, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_p_gain_async(self, sid, loop=None):
        """ pGainの取得（単位無し

        :param sid:
        :param loop:
        :return:
        """
        f, callback = self.async_wrapper(loop)
        self.get_p_gain(sid, callback=callback)
        return f

    def set_p_gain(self, gain, sid=1):
        """ pGainの書き込み

        :param gain:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 位置 (-180から180まで)
        if gain < 0:
            gain = 0
        elif gain > 16383:
            gain = 16383

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_POSITION_P_GAIN, gain, 2)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_i_gain(self, sid, callback=None):
        """ iGainの取得（単位無し

        :param sid:
        :param callback
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                pGain = int.from_bytes(response_data, 'little', signed = False)
                return pGain
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_POSITION_I_GAIN, 2, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_i_gain_async(self, sid, loop=None):
        """ iGainの取得（単位無し

        :param sid:
        :param loop:
        :return:
        """
        f, callback = self.async_wrapper(loop)
        self.get_i_gain(sid, callback=callback)
        return f

    def set_i_gain(self, gain, sid=1):
        """ iGainの書き込み

        :param gain:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 位置 (-180から180まで)
        if gain < 0:
            gain = 0
        elif gain > 16383:
            gain = 16383

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_POSITION_I_GAIN, gain, 2)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_d_gain(self, sid, callback=None):
        """ dGainの取得（単位無し

        :param sid:
        :param callback
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                pGain = int.from_bytes(response_data, 'little', signed = False)
                return pGain
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_POSITION_D_GAIN, 2, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_d_gain_async(self, sid, loop=None):
        """ dGainの取得（単位無し

        :param sid:
        :param loop:
        :return:
        """
        f, callback = self.async_wrapper(loop)
        self.get_d_gain(sid, callback=callback)
        return f

    def set_d_gain(self, gain, sid=1):
        """ dGainの書き込み

        :param gain:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        # 位置 (-180から180まで)
        if gain < 0:
            gain = 0
        elif gain > 16383:
            gain = 16383

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_POSITION_D_GAIN, gain, 2)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_max_torque(self, sid, callback=None):
        """最大トルク取得 (%)

        :param sid:
        :param callback:
        :return:
        """
        raise NotSupportException('RobotisP20ではget_max_torqueに対応していません。')

    def get_max_torque_async(self, sid, loop=None):
        """最大トルク取得 async版 (%)

        :param sid:
        :param loop:
        :return:
        """
        raise NotSupportException('RobotisP20ではget_max_torque_asyncに対応していません。')

    def set_max_torque(self, torque_percent, sid):
        """最大トルク設定 (%)

        :param torque_percent:
        :param sid:
        :return:
        """
        raise NotSupportException('RobotisP20ではset_max_torqueに対応していません。')

    def get_speed(self, sid, callback=None):
        """現在の回転速度を取得 (deg/s)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 4:
                speed = int.from_bytes(response_data, 'little', signed = False)
                dps = speed * 0.229 * 6
                return dps
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_PROFILE_VELOCITY, 4, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)


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
        """回転速度を設定 (deg/s)

        :param dps:
        :param sid:
        :return:
        """
         # サーボIDのチェック
        self.__check_sid(sid)

        rev_min = dps / 0.229 / 6.0

        if rev_min < 0:
            rev_min = 0
        elif rev_min > 32737:
            rev_min = 32737

        # コマンド生成
        params = self.__generate_parameters_read_write(self.ADDR_PROFILE_VELOCITY, rev_min, 4)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)


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
                id = int.from_bytes(response_data, 'little', signed = False)
                return id
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_ID, 1, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)


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

        if new_sid < 0:
            new_sid = 0
        elif new_sid > 252:
            new_sid = 252


        params = self.__generate_parameters_read_write(self.ADDR_ID, new_sid, 1)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)


    def save_rom(self, sid):
        """フラッシュROMに書き込む

        :param sid:
        :return:
        """
        raise NotSupportException('RobotisP20ではsave_romに対応していません。')

    def load_rom(self, sid):
        raise NotSupportException('RobotisP20ではload_romに対応していません。')

    def reset_memory(self, sid):
        """ROMを工場出荷時のものに初期化する

        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        return self.__get_function(self.INSTRUCTION_FACTORY_RESET, sid=sid)

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
                baudrate_list = [9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000]
                baudrate = int.from_bytes(response_data, 'little', signed = False)
                return baudrate_list[baudrate]
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_BAUDRATE, 1, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_baud_rate_async(self, sid, loop=None):
        """通信速度を取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_baud_rate(sid, callback=callback)
        return f

    def set_baud_rate(self, baud_rate, sid):
        """通信速度を設定

        :param baud_rate:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        baud_rate_list = [9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000]
        try:
            baud_rate_id = baud_rate_list.index(baud_rate)
        except:
            raise BadInputParametersException('baud_rate が不正な値です')

        params = self.__generate_parameters_read_write(self.ADDR_BAUDRATE, baud_rate_id, 1)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)


    def get_limit_cw_position(self, sid, callback=None):
        """右(時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 4:
                limit = int.from_bytes(response_data, 'little', signed = False)
                limit = limit * 360 / 4096
                limit = limit - 180
                return limit
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_MIN_POSITION_LIMIT, 4, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)


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

        if limit_position > 0 or limit_position < -180:
             raise BadInputParametersException('limit_position が不正な値です -180~0を設定してください。')

        limit_position = limit_position + 180
        limit_position = limit_position / 360 * 4096

        params = self.__generate_parameters_read_write(self.ADDR_MIN_POSITION_LIMIT, limit_position, 4)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)


    def get_limit_ccw_position(self, sid, callback=None):
        """左(反時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 4:
                limit = int.from_bytes(response_data, 'little', signed = False)
                limit = limit * 360 / 4096
                limit = limit - 180
                return limit
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_MAX_POSITION_LIMIT, 4, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

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

        if limit_position < 0 or limit_position > 180:
             raise BadInputParametersException('limit_position が不正な値です 0~180を設定してください。')

        limit_position = limit_position + 180
        limit_position = limit_position / 360 * 4096

        params = self.__generate_parameters_read_write(self.ADDR_MAX_POSITION_LIMIT, limit_position, 4)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_limit_temperature(self, sid, callback=None):
        """温度リミットの取得 (℃)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                limit = int.from_bytes(response_data, 'little', signed = False)
                return limit
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_TEMPERATURE_LIMIT, 1, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

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
        """温度リミットの設定 (℃) async版

        :param limit_temp:
        :param loop:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        if limit_temp < 0 or limit_temp > 100:
             raise BadInputParametersException('limit_temp が不正な値です 0~100を設定してください。')

        params = self.__generate_parameters_read_write(self.ADDR_TEMPERATURE_LIMIT, limit_temp, 1)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_limit_current(self, sid, callback=None):
        """電流リミットの取得 (mA)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 2:
                limit = int.from_bytes(response_data, 'little', signed = False)
                limit = limit * 2.69
                return limit
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_CURRENT_LIMIT, 2, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_limit_current_async(self, sid, loop=None):
        """電流リミットの取得 (mA) async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_limit_current(sid, callback=callback)
        return f

    def set_limit_current(self, limit_current, sid):
        """電流リミットの書き込み (mA)

        :param limit_current:
        :param sid:
        :return:
        """
        if limit_current < 0 or limit_current > 3210:
             raise BadInputParametersException('limit_current が不正な値です 0~3210を設定してください。')

        limit_current = limit_current / 2.69

        params = self.__generate_parameters_read_write(self.ADDR_CURRENT_LIMIT, limit_current, 2)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def get_drive_mode(self, sid, callback=None):
        """動作モードの取得

        :param sid:
        :param callback:
        :return:
        """
        # サーボIDのチェック
        self.__check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == 1:
                mode = int.from_bytes(response_data, 'little', signed = False)
                return mode
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(self.ADDR_DRIVE_MODE, 1, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

    def get_drive_mode_async(self, sid, loop=None):
        """動作モードの取得 async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_drive_mode(sid, callback=callback)
        return f

    def set_drive_mode(self, drive_mode, sid):
        """動作モードの書き込み (mA)

        :param drive_mode:
        :param sid:
        :return:
        """
        if drive_mode > 5:
             raise BadInputParametersException('limit_current が不正な値です 0~3210を設定してください。')

        params = self.__generate_parameters_read_write(self.ADDR_DRIVE_MODE, drive_mode, 1)

        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)

    def set_burst_target_positions(self, sid_target_positions):
        """複数のサーボの対象ポジションを一度に設定

        :param sid_target_positions:
        :return:
        """
        for sid, position in sid_target_positions.items():
            # 位置 (-180から180まで)
            if position < -180:
                position = -180
            elif position > 180:
                position = 180

            # Dynamixelでは0〜360°なので変換
            position += 180

            # データ変換
            sid_target_positions[sid] = int(position * 4096 / 360)

        self.burst_write(self.ADDR_GOAL_POSITION, 4, sid_target_positions)

    def get_burst_positions(self, sids, callback=None):
        """複数のサーボの現在のポジションを一気にリード

        :param sids:
        :param callback:
        :return:
        """
        for sid in sids:
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

        params = self.__generate_parameters_sync_read(self.ADDR_PRESENT_POSITION, 4, sids)

        return self.__get_function_burstread(self.INSTRUCTION_SYNC_READ, params, 0xFE, len(sids), response_process, callback=callback)


    def get_burst_positions_async(self, sids, loop=None):
        """複数のサーボの現在のポジションを一気にリード async版

        :param sids:
        :param loop:
        :return:
        """
        f, callback = self.async_wrapper(loop)
        self.get_burst_positions(sids, callback=callback)
        return f

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
                data = int.from_bytes(response_data, 'little', signed = False)
                return data
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_read_write(address, length, 2)

        return self.__get_function(self.INSTRUCTION_READ, params, response_process, sid=sid, callback=callback)

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

        params = self.__generate_parameters_read_write(address, int.from_bytes(data, 'little', signed = False), len(data))
        return self.__get_function(self.INSTRUCTION_WRITE, params, sid=sid, callback=self.__callback_write_response)


    def burst_read(self, address, length, sids, callback=None):
        """複数サーボから一括でデータ読み取り

        :param sid_address_length:
        :param callback:
        :return:
        """
        """Futabaでは未サポート"""
        def response_process(response_data):
            if response_data is not None and len(response_data) == length:
                data = int.from_bytes(response_data, 'little', signed = False)
                return data
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        params = self.__generate_parameters_sync_read(address, length, sids)

        return self.__get_function_burstread(self.INSTRUCTION_SYNC_READ, params, 0xFE, num, response_process, callback=callback)


    def burst_read_async(self, address, length, sid_list, num, loop=None):
        """複数サーボから一括でデータ読み取り async版

        :param sid_address_length:
        :param loop:
        :return:
        """
        """Futabaでは未サポート"""
        f, callback = self.async_wrapper(loop)
        self.burst_read(sid, address, length, sid_list, num, callback=callback)
        return f

    def burst_write(self, address, length, sid_data):
        """複数サーボに一括で書き込み

        :param address:
        :param length:
        :param sid_data:
        :return:
        """

        params = []
        for sid, data in sid_data.items():
            # サーボIDのチェック
            self.__check_sid(sid)
            params.append(sid)
            params.append(data)

        params = self.__generate_parameters_sync_write(address, length, params)

        return self.__get_function_burstread(self.INSTRUCTION_SYNC_WRITE, params, 0xFE, 0, callback=self.__callback_write_response)
