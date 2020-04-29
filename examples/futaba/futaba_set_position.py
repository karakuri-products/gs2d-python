#! /usr/bin/env python3
# encoding: utf-8

import time
import sys
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

    # トルクON
    futaba.set_torque_enable(True, sid=1)

    # 0.5秒ごとにサーボを動かす
    for i in range(11):
        angle = i * 20 - 100
        print('Angle:', angle, 'deg')
        futaba.set_target_position(angle, sid=1)
        time.sleep(0.5)

    # トルクOFF
    futaba.set_torque_enable(False, sid=1)

    # クローズ
    futaba.close()
    si.close()

except Exception as e:
    print('Error', e)
