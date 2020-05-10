# ! /usr/bin/env python3
# encoding: utf-8

import platform
import glob
import serial
import logging

from .ISerialInterface import ISerialInterface
from .Util import SerialDeviceNotFoundException

# ロガー
logger = logging.getLogger(__name__)

# デフォルトのボーレート
DEFAULT_BAUDRATE = 115200


class SerialInterface(ISerialInterface):
    __ser = None

    def __init__(self, device=None, baudrate=None):
        """シリアルインタフェース初期化

        :param device:
        :param baudrate:
        """

        # デバイス設定
        if device is None:
            pf = platform.system()
            base_path = None
            if pf == 'Windows':
                base_path = None
            elif pf == 'Darwin':
                base_path = '/dev/tty.usbserial-*'
            elif pf == 'Linux':
                base_path = '/dev/tty.usbserial-*'
            if base_path:
                devices = glob.glob(base_path)

                if len(devices) > 0:
                    device = devices[0]
                else:
                    raise SerialDeviceNotFoundException('シリアルデバイスを設定してください')
            else:
                raise SerialDeviceNotFoundException('シリアルデバイスを設定してください')

        # ボーレート設定
        if baudrate is None:
            baudrate = DEFAULT_BAUDRATE

        logger.debug('Device name: {}, Baudrate: {}'.format(device, baudrate))

        # シリアルデバイス生成
        self.__ser = serial.Serial(device, baudrate, timeout=0.1)

    def write(self, data):
        """データ送信

        :param data:
        :return:
        """

        return self.__ser.write(data)

    def read(self):
        """サーボからのデータ1文字受信

        :return:
        """

        return self.__ser.read()

    def is_open(self):
        """シリアルインタフェースがオープンされているかチェック

        :return:
        """

        return self.__ser.is_open

    def close(self):
        """シリアルインタフェースをクローズ

        :return:
        """

        self.__ser.close()
