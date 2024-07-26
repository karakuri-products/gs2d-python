import time
import logging
import serial
from types import FunctionType
from packaging.version import Version

from .ICommandHandler import ICommandHandler
from .ISerialInterface import ISerialInterface
from .Driver import Driver
from .Util import ReceiveDataTimeoutException, NotSupportException, BadInputParametersException, WrongCheckSumException
from .Util import InvalidResponseDataException
from .Util import get_printable_hex

# ロガー
logger = logging.getLogger(__name__)


class KondoICS(Driver):
    """KONDO ICSのシリアルサーボクラス

    """

    # コマンド
    SHIFT_CMD = 5
    CMD_POSITION = 0b100 << SHIFT_CMD
    CMD_READ = 0b101 << SHIFT_CMD
    CMD_WRITE = 0b110 << SHIFT_CMD
    CMD_ID = 0b111 << SHIFT_CMD
    CMD_RECV = 0b001 << SHIFT_CMD # Not use for servo

    # コマンドヘッダ用マスク
    MASK_CMDHDR_MC = 0b1110_0000 # メインコマンド
    MASK_CMDHDR_ID = 0b0001_1111 # ID
    MASK_CMDHDR_RECV = 0b0111_1111 # 受信用マスク (MBR=0)

    # サブコマンド
    SC_EEPROM = 0x00
    SC_STRC = 0x01
    SC_SPD = 0x02
    SC_CUR = 0x03 # ICS>=3.5
    SC_TMP = 0x04 # ICS>=3.5
    SC_TCH = 0x05 # ICS>=3.6

    SC_ID_READ = 0x00
    SC_ID_WRITE = 0x01

    # EEPROMデータのバイト位置
    # Pythonが0はじまりなので、本来の値-1している
    ROM_STRC = 2 # 偶数値のみ
    ROM_SPD = 4
    ROM_PANCH = 6
    ROM_DEADBAND = 8
    ROM_DUMPING = 10
    ROM_SAFETIM = 12
    ROM_FLAG = 14
    ROM_PULSELIM_UP = 16 # 4bit x 4byte
    ROM_PULSELIM_DOWN = 20 # 4bit x 4byte
    ROM_BAUDRATE = 26 # 0x00, 0x01, 0x10
    ROM_TEMPLIM = 28
    ROM_CURLIM = 30
    ROM_OFFSET = 52
    ROM_ID = 56
    ROM_CHARA1 = 58
    ROM_CHARA2 = 60
    ROM_CHARA3 = 62

    # 通信速度のIDと実際の設定値
    BAUD_RATE_INDEX_1250000 = 0x00
    BAUD_RATE_INDEX_625000  = 0x01
    BAUD_RATE_INDEX_115200  = 0x10

    def __init__(self, serial_interface: ISerialInterface, command_handler_class: ICommandHandler = None,  version: str = "3.6", loopback: bool = True, slave:bool=False):
        """初期化
        """

        super(KondoICS, self).__init__(serial_interface, command_handler_class)
        self.loopback:bool = bool(loopback)
        self.slave:bool = bool(slave)
        self.baudrate = 115200
        if isinstance(self.command_handler.serial_interface.ser, serial.Serial):
            self.baudrate = self.command_handler.serial_interface.ser.baudrate
            self.command_handler.serial_interface.ser.parity = serial.PARITY_EVEN

        if version == "3.6":
            self.version = Version("3.6")
        elif version == "3.5":
            self.version = Version("3.5")
        elif version == "3.0":
            self.version = Version("3.0")
        elif version == "2.0":
            self.version = Version("2.0")
        else:
            raise BadInputParametersException("ics_version: 2.0、3.0、3.5、3.6 のみ対応です。入力値:", version)

    def is_complete_response(self, response_data):
        """レスポンスデータをすべて受信できたかチェック"""
        # print('#########', response_data)

        # パケット長のデータがないため、パケット解析して返却
        if response_data is None or len(response_data) == 0:
            return False
        elif self.loopback: # 送信データも受信
            if (response_data[0] & self.MASK_CMDHDR_MC) == self.CMD_POSITION:
                return len(response_data) == 6
            elif (response_data[0] & self.MASK_CMDHDR_MC) == self.CMD_READ:
                if len(response_data) < 2:
                    return False
                elif response_data[1] == self.SC_EEPROM:
                    return len(response_data) == 68
                elif response_data[1] == self.SC_STRC \
                    or response_data[1] == self.SC_SPD \
                    or response_data[1] == self.SC_CUR \
                    or response_data[1] == self.SC_TMP:
                    return len(response_data) == 5
                elif response_data[1] == self.SC_TCH:
                    return len(response_data) == 6
            elif (response_data[0] & self.MASK_CMDHDR_MC) == self.CMD_WRITE:
                if len(response_data) < 2:
                    return False
                elif response_data[1] == self.SC_EEPROM:
                    return len(response_data) == 68
                elif response_data[1] == self.SC_STRC \
                    or response_data[1] == self.SC_SPD \
                    or response_data[1] == self.SC_CUR \
                    or response_data[1] == self.SC_TMP:
                    return len(response_data) == 6
            elif (response_data[0] & self.MASK_CMDHDR_MC) == self.CMD_ID:
                return len(response_data) == 5
        else: # 受信データのみ
            if (response_data[0] & self.MASK_CMDHDR_MC) == (self.CMD_POSITION & self.MASK_CMDHDR_RECV) \
                or ((self.baudrate == 115_200 and (response_data[0] & self.MASK_CMDHDR_ID) == 0 and (response_data[0] & self.MASK_CMDHDR_MC) == self.CMD_POSITION)): # 従来のICS2.0との互換性のため、ID0で通信速度が115.2Kの場合、RXのMSBは1になります。
                return len(response_data) == (6-3)

            elif (response_data[0] & self.MASK_CMDHDR_MC) == (self.CMD_READ & self.MASK_CMDHDR_RECV):
                if len(response_data) < 2:
                    return False
                elif response_data[1] == self.SC_EEPROM:
                    return len(response_data) == (68-2)
                elif response_data[1] == self.SC_STRC \
                    or response_data[1] == self.SC_SPD \
                    or response_data[1] == self.SC_CUR \
                    or response_data[1] == self.SC_TMP:
                    return len(response_data) == (5-2)
                elif response_data[1] == self.SC_TCH:
                    return len(response_data) == (6-2)
            elif (response_data[0] & self.MASK_CMDHDR_MC) == (self.CMD_WRITE & self.MASK_CMDHDR_RECV):
                if len(response_data) < 2:
                    return False
                elif response_data[1] == self.SC_EEPROM:
                    return len(response_data) == (68-66)
                elif response_data[1] == self.SC_STRC \
                    or response_data[1] == self.SC_SPD \
                    or response_data[1] == self.SC_CUR \
                    or response_data[1] == self.SC_TMP:
                    return len(response_data) == (6-3)
            # IDコマンドのみサーボからの返事でもMSBのマスクはありません。
            elif (response_data[0] & self.MASK_CMDHDR_MC) == self.CMD_ID:
                return len(response_data) == (5-4)

        # TODO
        # raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')
        return False

    @staticmethod
    def _check_sid(sid):
        """Servo IDのレンジをチェック

        :param sid:
        :return:
        """

        if sid != 0xFF:
            if sid < 0 or sid > 31:
                raise BadInputParametersException('sid: %d がレンジ外です。1から31のIDを設定してください。' % sid)

    @staticmethod
    def _calc_par2deg(data):
        hex = ((data[0] & 0x7F) << 7) | (data[1] & 0x7F)
        deg = (int(hex) - 7500) * 135 / 4000
        return deg

    @staticmethod
    def _calc_deg2par(deg):
        pos_hex = int(deg * 4000 / 135 + 7500) & 0x3FFF
        pos_hex_h = (pos_hex >> 7) & 0x7F
        pos_hex_l = pos_hex & 0x7F
        return [pos_hex_h, pos_hex_l]

    @staticmethod
    def _get_hex_from_2x4bit(data):
        hex = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F)
        return hex

    @staticmethod
    def _set_hex_to_2x4bit(hex):
        data = [hex & 0xF0, hex & 0x0F]
        return data

    def _send_recv(self, sid, mc, sc=None, send_data=None, response_process=None, callback=None):
        """Send -> Receiveする関数
        KondoICSはスレーブモード以外応答が返る

        :param sid: サーボID
        :param mc: メインコマンド
        :param sc: サブコマンド
        :param response_process: 受信データ処理関数
        :param callback: ユーザー指定の受信データ処理関数
        :return:
        """

        # データ
        ret_data = None

        # 受信済みフラグ
        is_received = False

        # 引数チェック
        if response_process is not None:
            if not isinstance(response_process, FunctionType):
                raise BadInputParametersException("response_processには関数を指定してください")
        if callback is not None:
            if not isinstance(callback, FunctionType):
                raise BadInputParametersException("callbackには関数を指定してください")
            if self.slave:
                raise BadInputParametersException("スレーブモードではデータを受信しないため、callbackは使えません。")

        if mc == self.CMD_POSITION:
            if send_data is None or len(send_data) != 2 \
                or (send_data[0] & 0b1000_0000) != 0 or (send_data[1] & 0b1000_0000) != 0:
                raise BadInputParametersException("角度指定時の送信データが不正です。")
        elif mc == self.CMD_READ:
            if sc not in (self.SC_EEPROM, self.SC_STRC, self.SC_SPD, self.SC_CUR, self.SC_TMP, self.SC_TCH):
                raise BadInputParametersException("sc: サブコマンド %x が不正です。" % sc)
            if sc == self.SC_TCH and self.version != Version("3.6"):
                raise BadInputParametersException("sc: サブコマンド %x は使用できません。" % sc)
        elif mc == self.CMD_WRITE:
            if sc not in (self.SC_EEPROM, self.SC_STRC, self.SC_SPD, self.SC_CUR, self.SC_TMP, self.SC_TCH):
                raise BadInputParametersException("sc: サブコマンド %x が不正です。" % sc)
            elif sc == self.SC_EEPROM:
                if send_data is None or len(send_data) != 64:
                    raise BadInputParametersException("送信データが不正です。")
            elif sc in (self.SC_STRC, self.SC_SPD, self.SC_CUR, self.SC_TMP):
                if sc == self.SC_CUR and self.version < Version("3.0") :
                    raise BadInputParametersException("sc: サブコマンド %x は使用できません。" % sc)
                if sc == self.SC_TMP and self.version < Version("3.5") :
                    raise BadInputParametersException("sc: サブコマンド %x は使用できません。" % sc)
                if send_data is None or len(send_data) != 1:
                    raise BadInputParametersException("送信データが不正です。")
            elif sc == self.SC_TCH:
                if self.version < Version("3.6") :
                    raise BadInputParametersException("sc: サブコマンド %x は使用できません。" % sc)
                if send_data is None or len(send_data) != 2:
                    raise BadInputParametersException("送信データが不正です。")
        elif mc == self.CMD_ID:
            if sc != self.SC_ID_READ and sc != self.SC_ID_WRITE:
                raise BadInputParametersException("sc: サブコマンド %x が不正です。" % sc)
        else:
            raise BadInputParametersException("mc: メインコマンド %x が不正です。" % mc)


        # 送信コマンド生成
        send_command = []

        send_command.append(sid | mc)
        if mc == self.CMD_POSITION:
            send_command.extend(send_data)
        elif mc == self.CMD_READ:
            send_command.append(sc)
        elif mc == self.CMD_WRITE:
            send_command.append(sc)
            send_command.extend(send_data)
        elif mc == self.CMD_ID:
            send_command.append(sc)
            send_command.append(sc)
            send_command.append(sc)

        print("send_command:", send_command)

        # 受信処理
        def temp_recv_callback(response):
            nonlocal ret_data
            nonlocal is_received
            nonlocal send_command

            recv_byte_list = list(bytes(response))

            offset = 0
            if self.loopback:
                offset = len(send_command)
                if recv_byte_list[:offset] != send_command:
                    raise InvalidResponseDataException("送信コマンドを正しくループバックできていません。")
            recv_command = recv_byte_list[offset:]

            print("recv_command:", recv_command)

            send_mc = send_command[0] & self.MASK_CMDHDR_MC
            recv_mc = recv_command[0] & self.MASK_CMDHDR_MC
            # if (send_mc == self.CMD_ID and send_mc != recv_mc) or (send_mc & self.MASK_CMDHDR_RECV != recv_mc):
            #     raise InvalidResponseDataException("受信コマンドヘッダが不正です。")

            send_id = send_command[0] & self.MASK_CMDHDR_ID
            recv_id = recv_command[0] & self.MASK_CMDHDR_ID
            if send_id != 0xFF and send_id != recv_id:
                raise InvalidResponseDataException("受信したサーボIDが不正です。")

            if callback is None:
                if response_process is None:
                    ret_data = recv_command
                else:
                    ret_data = response_process(recv_command)
            else:
                callback(recv_command)

            # 受信済み
            is_received = True

        # コマンド処理キューに追加
        if self.slave:
            # No waiting data receive
            self.command_handler.add_command(send_command)
            return True
        else:
            self.command_handler.add_command(send_command, recv_callback=temp_recv_callback)

            # コールバックが設定できていたら、コールバックに受信データを渡す
            if callback is None:
                # 指定以内にサーボからデータを受信できたかをチェック
                start = time.time()
                while not is_received:
                    elapsed_time = time.time() - start
                    if elapsed_time > self.command_handler.RECEIVE_DATA_TIMEOUT_SEC:
                        raise ReceiveDataTimeoutException(str(self.command_handler.RECEIVE_DATA_TIMEOUT_SEC) + '秒以内にデータ受信できませんでした')
                return ret_data
            else:
                return True

    def get_torque_enable(self, sid, callback=None):
        """トルクON取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            data = response_data[2:]
            # EEPROMのFREEの有効ビットを確認し、反対の結果を返す(トルク有効ならFREE==0)
            return (data[self.ROM_FLAG+1] & 0b0000_0010) != 0b0000_0010

        return self._send_recv(sid, self.CMD_READ, self.SC_EEPROM, response_process=response_process)

    def get_torque_enable_async(self, sid, loop=None):
        """トルクON取得async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_torque_enable(sid, callback=callback)
        return f

    def close(self, force=False):
        """閉じる

        :param force:
        :return:
        """

        if self.command_handler:
            self.command_handler.close()

    def ping(self, sid=0xFF, callback=None):
        """サーボにPINGを送る
        KondoICSにはPINGコマンドはないので、IDの取得で代用。

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            servo_id = int(response_data[0] & self.MASK_CMDHDR_ID)
            try:
                self._check_sid(servo_id)
            except BadInputParametersException as e:
                return False
            return True

        return self._send_recv(sid, self.CMD_ID, response_process, callback=callback)

    def ping_async(self, sid, loop=None):
        """サーボにPINGを送る async版

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.ping(sid, callback=callback)
        return f

    def set_torque_enable(self, on_off, sid):
        """トルクON/OFF設定

        :param on_off:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        if on_off:
            # KondoICSではトルクONコマンドが存在しないため、現在の位置を指定しなおす
            pos = self.get_current_position(sid)
            self.set_target_position(pos, sid=sid)
        else:
            # Free状態を送信
            self._send_recv(sid, self.CMD_POSITION, data=[0x00, 0x00])

    def get_temperature(self, sid, callback=None):
        """温度取得 (単位: ℃。おおよそ±3℃程度の誤差あり)

        :param sid:
        :param callback:
        :return:
        """

        # バージョンのチェック
        if self.version < Version("3.5"):
            raise NotSupportException('KondoICSのバージョン3.5未満ではget_temperatureに対応していません。')

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            # 温度パラメーターは０から１２７までの値で、小さいほど温度が高いことを示します。
            # 目安として、パラメーター６０で温度が約８０度、パラメーター３０で約１００度です。
            data = response_data[2]
            if data < 0 or data > 127:
                raise InvalidResponseDataException("受信データが不正です。温度パラメータが %d で0~127の範囲外です。" % data)
            return data * (80-100)/(60-30) + 120

        return self._send_recv(sid, self.CMD_READ, self.SC_TMP, response_process=response_process)

    def get_temperature_async(self, sid, loop=None):
        """温度取得 async版 (単位: ℃。おおよそ±3℃程度の誤差あり)

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

        # バージョンのチェック
        if self.version < Version("3.0"):
            raise NotSupportException('KondoICSのバージョン3.0未満ではget_currentに対応していません。')

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            # 設定範囲 正転 (Low) 0 ～ 63 (High)
            # 逆転 (Low) 64 ～ 127 (High)
            # 電流値 0.1A = 設定値 1
            data = response_data[2]
            if data < 0 or data > 127:
                raise InvalidResponseDataException("受信データが不正です。電流パラメータが %d で0~127の範囲外です。" % data)
            if data < 64:
                return 1000 * data / 10
            else:
                return -1000 * (data-64) / 10

        return self._send_recv(sid, self.CMD_READ, self.SC_TMP, response_process=response_process)

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
        raise NotSupportException('KondoICSではget_target_positionに対応していません。')

    def get_target_position_async(self, sid, loop=None):
        raise NotSupportException('KondoICSではget_target_position_asyncに対応していません。')

    def set_target_position(self, position_degree, sid=1):
        """指示位置設定 (単位: 度。設定可能な範囲は-135.0 度~135.0 度)

        :param position_degree:
        :param sid:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        # 設定可能な範囲は-135.0 度~135.0 度
        if position_degree < -135:
            position_degree = -135
        elif position_degree > 135:
            position_degree = 135

        position_data = self._calc_deg2par(position_degree)

        self._send_recv(sid, self.CMD_POSITION, send_data=position_data)

    def get_current_position(self, sid, callback=None):
        """現在位置取得 (単位: 度)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        if self.version == Version("3.6"):
            def response_process(response_data):
                return self._calc_par2deg(response_data[2:])

            return self._send_recv(sid, self.CMD_READ, self.SC_TCH, response_process=response_process, callback=callback)
        else:
            def response_process(response_data):
                return self._calc_par2deg(response_data[1:])

            is_torque_on = self.get_torque_enable(sid)
            pos = self._send_recv(sid, self.CMD_POSITION, send_data=[0x00, 0x00], response_process=response_process, callback=callback)

            if is_torque_on:
                self.set_target_position(pos)

            return pos

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
        self._check_sid(sid)

        def response_process(response_data):
            rom_data = response_data[2:]
            data = rom_data[self.ROM_OFFSET:self.ROM_OFFSET+2]
            pos_hex = self._get_hex_from_2x4bit(data)

            # パラメータ範囲は-127~127
            deg = int(pos_hex & 0x7F) * 135 / 4000
            # 負の値
            if pos_hex & 0b1000_0000:
                deg = -deg
            return deg

        return self._send_recv(sid, self.CMD_READ, self.SC_EEPROM, response_process=response_process)

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

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        # 設定可能なパラメータ範囲は-127~127
        offset_abs = 0x7f * 135 / 4000
        if offset > offset_abs:
            offset_hex = 0x7f
        elif offset < -offset_abs:
            offset_hex = 0xff
        else:
            offset_hex = int(offset * 4000 / 135)
            if offset < 0:
                # 負の値
                offset_hex |= 0b1000_0000

        rom_data = self.load_rom(sid)
        data = self._set_hex_to_2x4bit(offset_hex)
        rom_data[self.ROM_OFFSET] = data[0]
        rom_data[self.ROM_OFFSET+1] = data[1]

        self._send_recv(sid, self.CMD_WRITE, self.SC_EEPROM, send_data=rom_data)

    def get_deadband(self, sid, callback=None):
        """デッドバンド角度取得 (単位: 度)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            rom_data = response_data[2:]
            data = rom_data[self.ROM_DEADBAND:self.ROM_DEADBAND+2]

            # TODO おそらくデッドバンド幅の片側の値のため2倍して返却
            pos_hex = self._get_hex_from_2x4bit(data)
            deg = int(pos_hex) * 2 * 135 / 4000
            return deg

        return self._send_recv(sid, self.CMD_READ, self.SC_EEPROM, response_process=response_process)

    def get_deadband_async(self, sid, loop=None):
        """デッドバンド角度取得 async版 (単位: 度)

        :param sid:
        :param loop:
        :return:
        """

        f, callback = self.async_wrapper(loop)
        self.get_deadband(sid, callback=callback)
        return f

    def set_deadband(self, deadband, sid):
        """デッドバンド角度指定 (単位: 度)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        # TODO おそらくデッドバンド幅の片側の値のため1/2して設定
        deadband_abs = abs(deadband)
        deadband_hex = int(deadband_abs * 135 / 4000 / 2)

        if deadband_hex > 0x05:
            deadband_hex = 0x05

        rom_data = self.load_rom(sid)
        data = self._set_hex_to_2x4bit(deadband_hex)
        rom_data[self.ROM_DEADBAND] = data[0]
        rom_data[self.ROM_DEADBAND+1] = data[1]

        self._send_recv(sid, self.CMD_WRITE, self.SC_EEPROM, send_data=rom_data)

    def get_voltage(self, sid, callback=None):
        """電圧取得 (単位: V)

        :param sid:
        :param callback:
        :return:
        """
        raise NotSupportException('KondoICSではget_voltageに対応していません。')

    def get_voltage_async(self, sid, loop=None):
        """電圧取得 async版 (単位: V)

        :param sid:
        :param loop:
        :return:
        """
        raise NotSupportException('KondoICSではget_voltageに対応していません。')

    def get_target_time(self, sid, callback=None):
        raise NotSupportException('KondoICSではget_target_timeに対応していません。')

    def get_target_time_async(self, sid, loop=None):
        raise NotSupportException('KondoICSではget_target_time_asyncに対応していません。')

    def set_target_time(self, speed_second, sid=1):
        raise NotSupportException('KondoICSではset_target_timeに対応していません。')

    def get_accel_time(self, sid, callback=None):
        raise NotSupportException('KondoICSではget_accel_timeに対応していません。')

    def get_accel_time_async(self, sid, loop=None):
        raise NotSupportException('KondoICSではget_accel_time_asyncに対応していません。')

    def set_accel_time(self, speed_second, sid):
        raise NotSupportException('KondoICSではset_accel_timeに対応していません。')

    def get_pid_coefficient(self, sid, callback=None):
        raise NotSupportException('KondoICSではget_pid_coefficientに対応していません。')

    def get_pid_coefficient_async(self, sid, loop=None):
        raise NotSupportException('KondoICSではget_pid_coefficient_asyncに対応していません。')

    def set_pid_coefficient(self, coef_percent, sid):
        raise NotSupportException('KondoICSではset_pid_coefficientに対応していません。')

    def get_p_gain(self, sid, callback=None):
        """ pGainの取得 (単位無し)

        :param sid:
        :param callback
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_p_gainに対応していません。')

    def get_p_gain_async(self, sid, loop=None):
        """ pGainの取得 (単位無し async版)

        :param sid:
        :param loop:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_p_gain_asyncに対応していません。')

    def set_p_gain(self, gain, sid=1):
        """ pGainの書き込み

        :param gain:
        :param sid:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではset_p_gainに対応していません。')


    def get_i_gain(self, sid, callback=None):
        """ iGainの取得 (単位無し)

        :param sid:
        :param callback
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_i_gainに対応していません。')


    def get_i_gain_async(self, sid, loop=None):
        """ iGainの取得 (単位無し) async版

        :param sid:
        :param loop:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_i_gain_asyncに対応していません。')

    def set_i_gain(self, gain, sid=1):
        """ iGainの書き込み

        :param gain:
        :param sid:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではset_i_gainに対応していません。')


    def get_d_gain(self, sid, callback=None):
        """ dGainの取得 (単位無し)

        :param sid:
        :param callback
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_d_gainに対応していません。')


    def get_d_gain_async(self, sid, loop=None):
        """ dGainの取得 (単位無し) async版

        :param sid:
        :param loop:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_d_gain_asyncに対応していません。')

    def set_d_gain(self, gain, sid=1):
        """ dGainの書き込み

        :param gain:
        :param sid:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではset_d_gainに対応していません。')

    def get_max_torque(self, sid, callback=None):
        """最大トルク取得 (%)

        :param sid:
        :param callback:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_max_torqueに対応していません。')

    def get_max_torque_async(self, sid, loop=None):
        """最大トルク取得 async版 (%)

        :param sid:
        :param loop:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_max_torque_asyncに対応していません。')

    def set_max_torque(self, torque_percent, sid):
        """最大トルク設定 (%)

        :param torque_percent:
        :param sid:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではset_max_torqueに対応していません。')

    def get_speed(self, sid, callback=None):
        """現在の回転速度を取得 (deg/s)

        :param sid:
        :param callback:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_speedに対応していません。')

    def get_speed_async(self, sid, loop=None):
        """現在の回転速度を取得 async版 (deg/s)

        :param sid:
        :param loop:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_speed_asyncに対応していません。')

    def set_speed(self, dps, sid):
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではset_speedに対応していません。')

    def get_servo_id(self, sid, callback=None):
        """サーボIDを取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            recv_id = int(response_data[0] & self.MASK_CMDHDR_ID)
            if recv_id == 0xFF:
                raise InvalidResponseDataException("受信IDが不正です。受信IDは %d で0~127の範囲外です。" % recv_id)
            try:
                self._check_sid(recv_id)
            except BadInputParametersException as e:
                raise InvalidResponseDataException("受信IDが不正です。受信IDは %d で0~127の範囲外です。" % recv_id)
            return recv_id

        return self._send_recv(sid, self.CMD_ID, response_process, callback=callback)

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
        if sid == 0xFF:
            raise BadInputParametersException("指定したサーボIDは %d で0~31の範囲外です。" % sid)
        self._check_sid(sid)

        if new_sid == 0xFF:
            raise BadInputParametersException("新たに指定したサーボIDは %d で0~31の範囲外です。" % sid)
        self._check_sid(new_sid)

        new_sid_hex = int(new_sid)

        # コマンド生成
        rom_data = self.load_rom(sid)
        data = self._set_hex_to_2x4bit(new_sid)
        rom_data[self.ROM_ID] = data[0]
        rom_data[self.ROM_ID+1] = data[1]

        self._send_recv(sid, self.CMD_ID, self.SC_EEPROM, send_data=rom_data)

    def save_rom(self, sid):
        """フラッシュROMに書き込む

        :param sid:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではsave_romに対応していません。')

    def load_rom(self, sid):
        """フラッシュROMを読み込む

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            return response_data[2:]

        return self._send_recv(sid, self.CMD_READ, self.SC_EEPROM, response_process=response_process)

    def get_baud_rate(self, sid, callback=None):
        """通信速度を取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            data = response_data[2:]
            baud_rate_hex = self._get_hex_from_2x4bit(data)
            if baud_rate_hex in (0x00,0x01,0x10):
                baud_rate_list = [1_250_000, 625_000, 115_200]
                baud_rate_id = int(baud_rate_hex)
                return baud_rate_list[baud_rate_id]
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self._send_recv(sid, self.CMD_READ, self.SC_EEPROM, response_process=response_process)

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
        self._check_sid(sid)

        # 通信速度IDのチェック
        baud_rate_list = [1_250_000, 625_000, 115_200]
        try:
            baud_rate_id = baud_rate_list.index(baud_rate)
        except:
            raise BadInputParametersException('baud_rate が不正な値です')

        baud_rate_id_hex = int(baud_rate_id)

        rom_data = self.load_rom(sid)
        data = self._set_hex_to_2x4bit(baud_rate_id_hex)
        rom_data[self.ROM_BAUDRATE] = data[0]
        rom_data[self.ROM_BAUDRATE+1] = data[1]

        self._send_recv(sid, self.CMD_WRITE, self.SC_EEPROM, send_data=rom_data)

    def get_limit_cw_position(self, sid, callback=None):
        """右(時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        pass # TODO

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
        self._check_sid(sid)

        pass # TODO

    def get_limit_ccw_position(self, sid, callback=None):
        """左(反時計回り)リミット角度の取得

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        pass # TODO

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
        self._check_sid(sid)

        pass # TODO

    def get_limit_temperature(self, sid, callback=None):
        """温度リミットの取得 (℃)

        :param sid:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        pass # TODO

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
        pass # TODO

    def get_limit_current(self, sid, callback=None):
        pass # TODO

    def get_limit_current_async(self, sid, loop=None):
        pass # TODO

    def set_limit_current(self, limit_current, sid):
        pass # TODO

    def get_drive_mode(self, sid, callback=None):
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_drive_modeに対応していません。')
    def get_drive_mode_async(self, sid, loop=None):
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_drive_mode_asyncに対応していません。')
    def set_drive_mode(self, drive_mode, sid):
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではset_drive_modeに対応していません。')


    def set_burst_target_positions(self, sid_target_positions):
        """複数のサーボの対象ポジションを一度に設定

        :param sid_target_positions:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではset_burst_target_positionsに対応していません。')

    def get_burst_positions(self, sids, callback=None):
        """複数のサーボの現在のポジションを一気にリード

        :param sids:
        :param callback:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_burst_positionsに対応していません。')

    def get_burst_positions_async(self, sids, loop=None):
        """複数のサーボの現在のポジションを一気にリード async版

        :param sids:
        :param loop:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではget_burst_positions_asyncに対応していません。')

    def reset_memory(self, sid):
        """ROMを工場出荷時のものに初期化する

        :param sid:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        # コマンド生成
        command = self.__generate_command(sid, self.ADDR_RESET_MEMORY, flag=self.FLAG4_RESET_MEMORY_MAP, count=0)

        # データ送信バッファに追加
        self.command_handler.add_command(command)

    def read(self, sid, address, length, callback=None):
        """データを読み込む

        :param sid:
        :param address:
        :param length:
        :param callback:
        :return:
        """

        # サーボIDのチェック
        self._check_sid(sid)

        def response_process(response_data):
            if response_data is not None and len(response_data) == length:
                return response_data
            else:
                raise InvalidResponseDataException('サーボからのレスポンスデータが不正です')

        return self._send_recv(address, self.FLAG30_MEM_MAP_SELECT, length, response_process,
                                   sid=sid, callback=callback)

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
        self._check_sid(sid)

        # コマンド生成
        command = self.__generate_command(sid, address, data)

        # データ送信バッファに追加
        self.command_handler.add_command(command)

    def burst_read(self, address, length, sids, callback=None):
        """複数サーボから一括でデータ読み取り

        :param address:
        :param length:
        :param sids:
        :param callback:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではburst_readに対応していません。')

    def burst_read_async(self, address, length, sids, loop=None):
        """複数サーボから一括でデータ読み取り async版

        :param address:
        :param length:
        :param sids:
        :param loop:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではburst_read_asyncに対応していません。')

    def burst_write(self, address, length, sid_data):
        """複数サーボに一括で書き込み

        :param address:
        :param length:
        :param sid_data:
        :return:
        """
        """KondoICSでは未サポート"""
        raise NotSupportException('KondoICSではburst_writeに対応していません。')
