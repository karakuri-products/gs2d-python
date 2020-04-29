#! /usr/bin/env python3
# encoding: utf-8

import sys
import time
import logging

sys.path.insert(0, '../..')
from gs2d import SerialInterface, Futaba

# ログ設定
logging.basicConfig()
logging.getLogger('gs2d').setLevel(level=logging.DEBUG)

try:
    # 初期化
    si = SerialInterface()
    futaba = Futaba(si)

    # バーストトルクON
    # enable: 1
    sid_data = {
        1: [1]
    }
    # Length: サーボ一つ分のデータ(VID+Data)のバイト数を指定。
    # Length = VID(1) + Data(1) = 2
    futaba.burst_write(Futaba.ADDR_TORQUE_ENABLE, 2, sid_data)

    # バーストポジション設定
    for position_degree in [0, 50, 0, -50, 0]:
        # ADDR_GOAL_POSITION_L 30 (0x1E), ADDR_GOAL_POSITION_H 31 (0x1F) なので
        # AddressにはADDR_GOAL_POSITION_Lを指定してDataを2バイト書き込む
        position_hex = format(int(position_degree * 10) & 0xffff, '04x')
        position_hex_h = int(position_hex[0:2], 16)
        position_hex_l = int(position_hex[2:4], 16)
        sid_data = {
            # サーボID: データ
            1: [position_hex_l, position_hex_h]
        }
        # Length = VID(1) + Data(2) = 3
        futaba.burst_write(Futaba.ADDR_GOAL_POSITION_L, 3, sid_data)

        # 1秒待機
        time.sleep(1.0)

    # クローズ
    futaba.close()
    si.close()

except Exception as e:
    print('Error', e)
