#! /usr/bin/env python3
# encoding: utf-8

import sys
import traceback
import asyncio
import logging
import time

sys.path.insert(0, '..')
from gs2d import SerialInterface, RobotisP20

# ログ設定
logging.basicConfig()
logging.getLogger('gs2d').setLevel(level=logging.DEBUG)


async def main(loop):
    # 初期化
    si = SerialInterface(baudrate=3000000)
    dm = RobotisP20(si)

    print(dm.ping(1))

    dm.set_torque_enable(True, sid=1)

    # 0.5秒ごとにサーボを動かす
    for i in range(11):
        angle = i * 20 - 100
        print('Angle:', angle, 'deg')
        dm.set_target_position(angle, sid=1)
        time.sleep(0.5)

    # クローズ
    dm.close(force=False)
    si.close()


# Initialize event loop
lp = asyncio.get_event_loop()
lp.run_until_complete(main(lp))
lp.close()
