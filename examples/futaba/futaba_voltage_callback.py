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
    def voltage_callback(voltage):
        """電圧取得できたときに呼ばれる"""
        print('Voltage: %.2f(V)' % voltage)

        # クローズ
        futaba.close()
        si.close()


    # 初期化
    si = SerialInterface()
    futaba = Futaba(si)

    # コールバックつきで電圧取得
    futaba.get_voltage(sid=1, callback=voltage_callback)

except Exception as e:
    print('Error', e)
