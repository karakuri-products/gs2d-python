#! /usr/bin/env python3
# encoding: utf-8

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

    # 電圧取得
    v = futaba.get_voltage(sid=1)
    print('Voltage: %.2f(V)' % v)

    # クローズ
    futaba.close()
    si.close()

except Exception as e:
    print('Error', e)
