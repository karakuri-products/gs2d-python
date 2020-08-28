# ! /usr/bin/env python3
# encoding: utf-8

import threading
import time
import logging
from collections import deque
from .ICommandHandler import ICommandHandler
from .ISerialInterface import ISerialInterface
from .Util import CommandBufferOverflowException, NotEnablePollingCommandException, get_printable_hex

# ロガー
logger = logging.getLogger(__name__)

# シリアルポートの送信バッファサイズ
MAX_COMMAND_QUEUE_LENGTH = 1024


class DefaultCommandHandler(ICommandHandler):
    """
    Python3環境でのデータ送受信管理クラス
    別スレッドでコマンドバッファの監視およびデータ送信を行っています
    """

    # シリアルポートインタフェースクラスのオブジェクト
    serial_interface = None

    # コマンドの送信バッファ
    command_queue = None

    # 送信バッファサイズ
    command_queue_size = None

    # コマンドバッファを監視し随時送信するスレッド
    polling_thread = None

    # コマンドバッファ監視フラグ
    enable_polling = False

    # クローズ強制フラグ
    close_force = False

    # レスポンスデータを受信完了したかをチェックする関数
    function_is_complete_response = None

    def __init__(self, serial_interface: ISerialInterface, function_is_complete_response, buffer_size=1024):
        """初期化

        :param serial_interface:
        :param function_is_complete_response:
        :param buffer_size:
        """

        # レスポンスデータを受信完了チェック関数を設定
        self.function_is_complete_response = function_is_complete_response

        # バッファサイズ設定
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
                response = self.serial_interface.read()

                # データが完全に受信できていないのであれば更に受信する
                while not self.function_is_complete_response(response):
                    response += self.serial_interface.read()

                    # タイムアウトチェック
                    elapsed_time = time.time() - start
                    if elapsed_time > self.RECEIVE_DATA_TIMEOUT_SEC:
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

    def add_command(self, data, recv_callback=None):
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
