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

    # 色んな角度にバースト設定
    for position_degree in [0, 50, 0, -50, 0]:
        # バーストポジション設定
        sid_positions = {
            # サーボID: ポジション
            1: position_degree
        }
        futaba.set_burst_target_positions(sid_positions)

        # 1秒待機
        time.sleep(1.0)

    # クローズ
    futaba.close()
    si.close()

except Exception as e:
    print('Error', e)
