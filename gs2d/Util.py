class SerialServoDriverException(IOError):
    """Exceptionのベースクラス"""


class NotSupportException(IOError):
    """機能の未サポートException (サーボごとに実装されていない機能がある)"""


class SerialDeviceNotFoundException(SerialServoDriverException):
    """シリアルデバイスが見つからないException"""


class ReceiveDataTimeoutException(SerialServoDriverException):
    """データ受信タイムアウトException"""


class CloseTimeoutException(SerialServoDriverException):
    """クローズタイムアウトException"""


class CommandBufferOverflowException(SerialServoDriverException):
    """コマンド用バッファのオーバーフローException"""


class NotEnablePollingCommandException(SerialServoDriverException):
    """ポーリング無効時にコマンド追加したときのException"""


class BadInputParametersException(SerialServoDriverException):
    """入力パラメータがよくないException"""


class InvalidResponseDataException(SerialServoDriverException):
    """サーボからのレスポンスデータが不正なデータだったException"""


class WrongCheckSumException(SerialServoDriverException):
    """チェックサムが間違ってるException"""


def get_printable_hex(byte_data):
    """
    bytearrayのデータを見やすい16進数表現の文字列に変換する
    :param byte_data:
    :return:
    """
    data_hex_string = byte_data.hex()
    return '[' + ' '.join([data_hex_string[i: i + 2].upper() for i in range(0, len(data_hex_string), 2)]) + ']'
