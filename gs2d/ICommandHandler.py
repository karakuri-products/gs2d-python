# ! /usr/bin/env python3
# encoding: utf-8

from abc import ABCMeta, abstractmethod


class ICommandHandler(metaclass=ABCMeta):
    """
    データ送受信管理の抽象クラス
    """

    # 受信タイムアウト(秒)
    RECEIVE_DATA_TIMEOUT_SEC = 2

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

    @abstractmethod
    def add_command(self, data, recv_callback=None):
        raise NotImplementedError()

    @abstractmethod
    def close(self, force=False):
        raise NotImplementedError()
