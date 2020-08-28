#! /usr/bin/env python3
# encoding: utf-8

import sys
import logging

sys.path.insert(0, '../..')
from gs2d import SerialInterface, RobotisP20

# ログ設定
logging.basicConfig()
logging.getLogger('gs2d').setLevel(level=logging.DEBUG)

try:
    # 初期化
    si = SerialInterface(baudrate=3000000)
    robotis = RobotisP20(si)

    sid = 1

    # 電圧取得
    temperature = robotis.get_temperature(sid=sid)
    print('Temperature: %.2f(degC)' % temperature)

    # クローズ
    robotis.close()
    si.close()

except Exception as e:
    print('Error', e)
